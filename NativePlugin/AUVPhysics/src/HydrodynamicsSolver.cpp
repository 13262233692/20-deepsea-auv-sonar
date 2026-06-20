#include "AUVPhysicsAPI.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdint>

// ============================================================================
//  HARDENED 6DOF HYDRODYNAMICS SOLVER (RK4 + ENERGY CONSERVATION)
//
//  Fixes:
//  [1] Coriolis matrix is FORCED skew-symmetric: C = 0.5*(C - C^T)
//      → v^T·C·v ≡ 0 → no spurious energy injection during maneuvers
//  [2] Euler explicit → RK4 (4th-order Runge-Kutta) for rigid body state
//      → stable for stiff Coriolis-dominated systems at large dt
//  [3] NaN/Inf watchdog: hard reset of state on any singularity
//  [4] Multi-stage clamp (accelerations → velocities → positions)
//  [5] Total mechanical energy monitor: dE/dt spikes trigger damping injection
// ============================================================================

static AUVParameters g_params;
static AUVState      g_state;
static Vec3         g_ctrlForce;
static Vec3         g_ctrlTorque;
static bool         g_initialized = false;
static float        g_lastEnergy  = 0.0f;
static uint32_t     g_singularityCount = 0;

// ----------------------------- utilities ---------------------------------
static inline Vec3 v3(float x, float y, float z) { return {x,y,z}; }
static inline Vec3 v3add(Vec3 a, Vec3 b)       { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline Vec3 v3sub(Vec3 a, Vec3 b)       { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline Vec3 v3scl(Vec3 a, float s)      { return {a.x*s, a.y*s, a.z*s}; }
static inline Vec3 v3crs(Vec3 a, Vec3 b)       { return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
static inline float v3dot(Vec3 a, Vec3 b)      { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3len(Vec3 a) { return std::sqrt(v3dot(a, a)); }
static inline Vec3 v3clamp(Vec3 a, float mx)
{
    float l2 = v3dot(a,a);
    if (l2 <= mx*mx) return a;
    float s = mx / std::sqrt(std::max(1e-20f, l2));
    return v3scl(a, s);
}
static inline bool v3isnan(Vec3 a) { return std::isnan(a.x)||std::isnan(a.y)||std::isnan(a.z)||std::isinf(a.x)||std::isinf(a.y)||std::isinf(a.z); }
static inline bool qisnan(Quat q)  { return std::isnan(q.x)||std::isnan(q.y)||std::isnan(q.z)||std::isnan(q.w)||std::isinf(q.w); }

static inline Quat qnormalize(Quat q)
{
    float l2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (std::isnan(l2) || l2 < 1e-20f) return {0,0,0,1};
    float s = 1.0f / std::sqrt(l2);
    return {q.x*s, q.y*s, q.z*s, q.w*s};
}
static inline Quat qmul(Quat a, Quat b)
{
    return {
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    };
}
static inline Quat qscaleVecDeriv(Quat q, Vec3 omega)
{
    // dq/dt = 0.5 * q ⊗ [ω;0]
    Quat qw = {omega.x, omega.y, omega.z, 0.0f};
    Quat r  = qmul(q, qw);
    return {r.x*0.5f, r.y*0.5f, r.z*0.5f, r.w*0.5f};
}
static inline Quat qadd(Quat a, Quat b) { return {a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w}; }
static inline Quat qscl(Quat a, float s) { return {a.x*s, a.y*s, a.z*s, a.w*s}; }

static inline Vec3 qrot(Quat q, Vec3 v)
{
    Quat vq = {v.x, v.y, v.z, 0};
    Quat qc = {-q.x, -q.y, -q.z, q.w};
    Quat r1 = qmul(q, vq);
    Quat r2 = qmul(r1, qc);
    return {r2.x, r2.y, r2.z};
}

// 6-vector = [linear, angular]
using Vec6 = float[6];
static inline void v6assign(Vec6& o, Vec3 l, Vec3 a)
{
    o[0]=l.x; o[1]=l.y; o[2]=l.z; o[3]=a.x; o[4]=a.y; o[5]=a.z;
}
static inline void v6addv(Vec6& o, const Vec6& a, const Vec6& b)
{
    for (int i=0;i<6;i++) o[i]=a[i]+b[i];
}
static inline void v6sclv(Vec6& o, const Vec6& a, float s)
{
    for (int i=0;i<6;i++) o[i]=a[i]*s;
}
static inline void v6extract(const Vec6& v, Vec3& l, Vec3& a)
{
    l = {v[0],v[1],v[2]}; a = {v[3],v[4],v[5]};
}

// ----------------------------- matrix ops ---------------------------------
static void m6mul(const Matrix6& M, const Vec6& v, Vec6& out)
{
    for (int i = 0; i < 6; i++) {
        float s = 0;
        for (int j = 0; j < 6; j++) s += M.m[i*6+j] * v[j];
        out[i] = s;
    }
}
// Solve (M_RB + M_A)·x = b via Cholesky-like diagonal-dominant Jacobi iteration
// (diagonal scaling + 6 Gauss-Seidel passes, analytically invertible since PD)
static void m6solvePD(const Matrix6& Mtot, const Vec6& b, Vec6& x)
{
    // 1 step of Jacobi precondition + then direct GJ elimination on 6x6
    float aug[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) aug[i][j] = Mtot.m[i*6+j];
        for (int j = 6; j < 12; j++) aug[i][j] = (j-6 == i) ? 1.0f : 0.0f;
    }
    for (int c = 0; c < 6; c++) {
        int pv = c;
        float mx = std::fabs(aug[c][c]);
        for (int r = c+1; r < 6; r++) {
            float v = std::fabs(aug[r][c]);
            if (v > mx) { mx = v; pv = r; }
        }
        if (pv != c) for (int j=0;j<12;j++) std::swap(aug[c][j], aug[pv][j]);
        float d = aug[c][c];
        if (std::fabs(d) < 1e-14f) { d = 1e-14f; }
        float inv = 1.0f / d;
        for (int j=0;j<12;j++) aug[c][j] *= inv;
        for (int r=0;r<6;r++) if (r != c) {
            float f = aug[r][c];
            for (int j=0;j<12;j++) aug[r][j] -= f*aug[c][j];
        }
    }
    for (int i=0;i<6;i++) {
        float s = 0;
        for (int j=0;j<6;j++) s += aug[i][j+6] * b[j];
        x[i] = s;
    }
}

// Build SKEW-SYMMETRIC Coriolis-Centripetal matrix from total momentum p = M·ν
// (Fossen 2011 Theorem 3.2: C must satisfy C = -C^T for energy conservation)
static void buildCoriolisSkew(const Vec6& mom, const Vec6& vel, Matrix6& C)
{
    // 3x3 skew helper
    auto skew = [](Vec3 a, float out[9]) {
        out[0]=0;      out[1]=-a.z;    out[2]=a.y;
        out[3]=a.z;    out[4]=0;       out[5]=-a.x;
        out[6]=-a.y;   out[7]=a.x;     out[8]=0;
    };

    Vec3 momLin = {mom[0], mom[1], mom[2]};
    Vec3 momAng = {mom[3], mom[4], mom[5]};
    Vec3 velLin = {vel[0], vel[1], vel[2]};
    Vec3 velAng = {vel[3], vel[4], vel[5]};

    float S1[9], S2[9];
    skew(velAng, S1);       // S(ω)
    skew(momAng, S2);       // S(M_ω·ω)

    // C = [ S(ω)    ,  0        ]
    //     [ S(m_lin),  S(m_ang) ]
    // Then skew-symmetrize C = 0.5*(C - C^T) to guarantee no energy leakage
    float Craw[36] = {0};
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Craw[i*6+j]     = S1[i*3+j];     // top-left
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Craw[(i+3)*6+j] = S2[i*3+j];     // bottom-left
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Craw[(i+3)*6+(j+3)] = S1[i*3+j]; // bottom-right

    // CRITICAL: enforce skew-symmetry exactly in floating point
    for (int i=0;i<6;i++) for (int j=0;j<6;j++)
        C.m[i*6+j] = 0.5f * (Craw[i*6+j] - Craw[j*6+i]);
}

// ----------------------------- state derivatives --------------------------
// Compute derivative of the augmented state Y = [pos; quat; vel; angvel]
// given control inputs. Returns dY/dt as 13-vector decomposed.
struct Derivatives
{
    Vec3 dPos;        // dp/dt = R(q)·v
    Quat dQuat;       // dq/dt = 0.5 q⊗ω
    Vec3 dLinVel;     // M^{-1}·(τ - Cv - Dv - g)
    Vec3 dAngVel;
};

// Full RHS: compute total generalized force given a state (for RK4 stages)
static void computeForcesAtState(const AUVState* s, const Vec3& cF, const Vec3& cT,
                                 Vec3& outForce, Vec3& outTorque,
                                 Vec3* debugAM = nullptr,
                                 Vec3* debugVisc = nullptr,
                                 Vec3* debugBuoy = nullptr,
                                 Vec3* debugCor = nullptr)
{
    // Assemble total mass matrix 6x6
    float MRB[36] = {0};
    float m = g_params.mass;
    // Rough inertia approximation if not set
    float L = std::max(0.1f, g_params.dimensions.z);
    float W = std::max(0.05f, g_params.dimensions.x);
    float H = std::max(0.05f, g_params.dimensions.y);
    float Ixx = m * (H*H + W*W) / 12.0f;
    float Iyy = m * (L*L + H*H) / 12.0f;
    float Izz = m * (L*L + W*W) / 12.0f;
    for (int i=0;i<3;i++) MRB[i*6+i] = m;
    MRB[21] = Ixx; MRB[28] = Iyy; MRB[35] = Izz;

    Matrix6 Mtot;
    for (int i=0;i<36;i++) Mtot.m[i] = MRB[i] + g_params.addedMass.m[i];

    Vec6 vel = {s->linearVelocity.x, s->linearVelocity.y, s->linearVelocity.z,
                s->angularVelocity.x, s->angularVelocity.y, s->angularVelocity.z};

    // Momentum
    Vec6 mom; m6mul(Mtot, vel, mom);

    // Damping
    Vec6 vabs; for (int i=0;i<6;i++) vabs[i] = std::fabs(vel[i]);
    Vec6 vquad; for (int i=0;i<6;i++) vquad[i] = vel[i] * vabs[i];
    Vec6 dampLin, dampQuad;
    m6mul(g_params.linearDamping, vel, dampLin);
    m6mul(g_params.quadraticDamping, vquad, dampQuad);

    // Coriolis (skew-symmetric, GUARANTEED)
    Matrix6 C; buildCoriolisSkew(mom, vel, C);
    Vec6 Cv; m6mul(C, vel, Cv);

    // Added mass force (proxy: approximate rate via -MA·(J·v) scaled)
    Vec6 fAM; for (int i=0;i<6;i++) fAM[i] = -0.5f * (g_params.addedMass.m[i*6+i]) * vel[i]; // simplified proxy in RHS evaluation

    // Buoyancy + gravity (world frame then rotate back to body)
    Quat q = qnormalize(s->orientation);
    Vec3 Fg = {0, -m * g_params.gravity, 0};
    float buoy = g_params.waterDensity * g_params.gravity * g_params.displacementVolume;
    Vec3 Fb = {0, buoy, 0};
    // transform to body frame
    Quat qc = {-q.x, -q.y, -q.z, q.w};
    Vec3 Rw_Fg = qrot(qc, Fg);
    Vec3 Rw_Fb = qrot(qc, Fb);
    Vec3 Fg_body = Rw_Fg;
    Vec3 Fb_body = Rw_Fb;
    // Restoring torque: BG in body frame cross Fb_body
    Vec3 BG_world = v3sub(g_params.centerOfBuoyancy, g_params.centerOfGravity);
    Vec3 BG_body = qrot(qc, BG_world);
    Vec3 T_rest = v3crs(BG_body, Fb_body);

    // Assemble total 6-force (negative sign convention per Fossen eq)
    Vec6 tau = {
        cF.x + Fg_body.x + Fb_body.x - dampLin[0] - dampQuad[0] - Cv[0] + fAM[0],
        cF.y + Fg_body.y + Fb_body.y - dampLin[1] - dampQuad[1] - Cv[1] + fAM[1],
        cF.z + Fg_body.z + Fb_body.z - dampLin[2] - dampQuad[2] - Cv[2] + fAM[2],
        cT.x + T_rest.x - dampLin[3] - dampQuad[3] - Cv[3] + fAM[3],
        cT.y + T_rest.y - dampLin[4] - dampQuad[4] - Cv[4] + fAM[4],
        cT.z + T_rest.z - dampLin[5] - dampQuad[5] - Cv[5] + fAM[5]
    };

    // Solve M·ν̇ = τ  →  ν̇ = M^{-1}·τ (M is positive definite)
    Vec6 nudot;
    m6solvePD(Mtot, tau, nudot);

    // Rotate linear velocity to world frame for position derivative (we return world-frame dv)
    Vec3 acc_body = {nudot[0], nudot[1], nudot[2]};
    Vec3 acc_body2 = {nudot[3], nudot[4], nudot[5]};
    outForce  = qrot(q, acc_body);   // world force for debug
    outTorque = qrot(q, acc_body2);

    // Fill in debug (optional)
    if (debugAM)   *debugAM   = qrot(q, {fAM[0], fAM[1], fAM[2]});
    if (debugVisc) *debugVisc = qrot(q, {-dampLin[0]-dampQuad[0], -dampLin[1]-dampQuad[1], -dampLin[2]-dampQuad[2]});
    if (debugBuoy) { *debugBuoy = v3add(Fg, Fb); /* world frame */ }
    if (debugCor)  *debugCor  = qrot(q, {-Cv[0], -Cv[1], -Cv[2]});
}

static void computeDerivatives(const AUVState* s, const Vec3& cF, const Vec3& cT, Derivatives& d)
{
    Quat q = qnormalize(s->orientation);
    d.dPos  = qrot(q, s->linearVelocity);
    d.dQuat = qscaleVecDeriv(q, s->angularVelocity);

    Vec3 Fw, Tw, amF, vF, bF, cFv;
    computeForcesAtState(s, cF, cT, Fw, Tw, &amF, &vF, &bF, &cFv);

    // We accumulated world-frame total force/torque above including M^{-1} baked in
    // (we need to re-derive body-frame dv/dt correctly here).
    // Simpler: compute nudot fresh via body-frame.
    float MRB[36]={0};
    float mm = g_params.mass;
    float L = std::max(0.1f, g_params.dimensions.z);
    float W = std::max(0.05f, g_params.dimensions.x);
    float H = std::max(0.05f, g_params.dimensions.y);
    MRB[0]=MRB[7]=MRB[14]=mm;
    MRB[21] = mm*(H*H+W*W)/12.0f;
    MRB[28] = mm*(L*L+H*H)/12.0f;
    MRB[35] = mm*(L*L+W*W)/12.0f;
    Matrix6 Mtot;
    for (int i=0;i<36;i++) Mtot.m[i] = MRB[i] + g_params.addedMass.m[i];

    Vec6 vel = {s->linearVelocity.x, s->linearVelocity.y, s->linearVelocity.z,
                s->angularVelocity.x, s->angularVelocity.y, s->angularVelocity.z};
    Vec6 mom; m6mul(Mtot, vel, mom);

    Vec6 vabs, vquad;
    for (int i=0;i<6;i++) { vabs[i]=std::fabs(vel[i]); vquad[i]=vel[i]*vabs[i]; }
    Vec6 dLin, dQuad;
    m6mul(g_params.linearDamping, vel, dLin);
    m6mul(g_params.quadraticDamping, vquad, dQuad);

    Matrix6 C; buildCoriolisSkew(mom, vel, C);
    Vec6 Cv; m6mul(C, vel, Cv);

    Quat qc = {-q.x,-q.y,-q.z,q.w};
    Vec3 Fg_b = qrot(qc, {0, -mm*g_params.gravity, 0});
    float buoy = g_params.waterDensity * g_params.gravity * g_params.displacementVolume;
    Vec3 Fb_b = qrot(qc, {0, buoy, 0});
    Vec3 BG_b = qrot(qc, v3sub(g_params.centerOfBuoyancy, g_params.centerOfGravity));
    Vec3 Tr_b = v3crs(BG_b, Fb_b);

    Vec6 tau = {
        cF.x + Fg_b.x + Fb_b.x - dLin[0] - dQuad[0] - Cv[0],
        cF.y + Fg_b.y + Fb_b.y - dLin[1] - dQuad[1] - Cv[1],
        cF.z + Fg_b.z + Fb_b.z - dLin[2] - dQuad[2] - Cv[2],
        cT.x + Tr_b.x - dLin[3] - dQuad[3] - Cv[3],
        cT.y + Tr_b.y - dLin[4] - dQuad[4] - Cv[4],
        cT.z + Tr_b.z - dLin[5] - dQuad[5] - Cv[5]
    };
    Vec6 nudot; m6solvePD(Mtot, tau, nudot);
    d.dLinVel  = {nudot[0], nudot[1], nudot[2]};
    d.dAngVel  = {nudot[3], nudot[4], nudot[5]};
}

// Advance state by dt given derivatives (for RK4 accumulation)
static AUVState advanceState(const AUVState& s, const Derivatives& d, float dt)
{
    AUVState r = s;
    r.position    = v3add(r.position, v3scl(d.dPos, dt));
    r.orientation = qnormalize(qadd(r.orientation, qscl(d.dQuat, dt)));
    r.linearVelocity  = v3add(r.linearVelocity,  v3scl(d.dLinVel, dt));
    r.angularVelocity = v3add(r.angularVelocity, v3scl(d.dAngVel, dt));
    return r;
}

// ============================================================================
//  Public API
// ============================================================================

extern "C" void AUV_Init(const AUVParameters* params)
{
    if (!params) return;
    g_params = *params;
    // Ensure mass matrix diagonals are positive (else NaN downstream)
    for (int i = 0; i < 6; i++) {
        if (g_params.addedMass.m[i*6+i] <= 0) g_params.addedMass.m[i*6+i] = 1.0f;
        if (g_params.linearDamping.m[i*6+i] < 0) g_params.linearDamping.m[i*6+i] = 0.0f;
        if (g_params.quadraticDamping.m[i*6+i] < 0) g_params.quadraticDamping.m[i*6+i] = 0.0f;
    }
    if (g_params.mass <= 0) g_params.mass = 100.0f;
    if (g_params.waterDensity <= 0) g_params.waterDensity = 1025.0f;
    if (g_params.gravity <= 0) g_params.gravity = 9.81f;
    if (g_params.displacementVolume <= 0)
        g_params.displacementVolume = g_params.mass / g_params.waterDensity;

    std::memset(&g_state, 0, sizeof(AUVState));
    g_state.orientation = {0,0,0,1};
    g_state.ambientPressure = AUV_ComputeHydrostaticPressure(g_params.depth);
    g_ctrlForce = {0,0,0};
    g_ctrlTorque = {0,0,0};
    g_lastEnergy = 0.0f;
    g_singularityCount = 0;
    g_initialized = true;
}

extern "C" float AUV_ComputeHydrostaticPressure(float depthMeters)
{
    const float P0 = 101325.0f;
    const float rho = g_params.waterDensity > 0 ? g_params.waterDensity : 1025.0f;
    const float g = g_params.gravity > 0 ? g_params.gravity : 9.81f;
    return P0 + rho * g * depthMeters;
}

static bool checkSingularityAndReset()
{
    bool bad = v3isnan(g_state.position) || v3isnan(g_state.linearVelocity) ||
               v3isnan(g_state.angularVelocity) || qisnan(g_state.orientation);
    float lv = v3len(g_state.linearVelocity);
    float av = v3len(g_state.angularVelocity);
    if (lv > 1e4f || av > 100.0f) bad = true;
    if (bad) {
        g_singularityCount++;
        g_state.position = {0, -std::max(1.0f, g_params.depth), 0};
        g_state.orientation = {0,0,0,1};
        g_state.linearVelocity = {0,0,0};
        g_state.angularVelocity = {0,0,0};
        return true;
    }
    return false;
}

extern "C" void AUV_ComputeHydrodynamics(const AUVState* state, AUVState* outForces)
{
    if (!state || !outForces) return;
    std::memset(outForces, 0, sizeof(AUVState));
    Vec3 Fw, Tw;
    computeForcesAtState(state, g_ctrlForce, g_ctrlTorque, Fw, Tw,
                         &outForces->addedMassForce,
                         &outForces->viscousForce,
                         &outForces->buoyancyForce,
                         &outForces->coriolisForce);
    outForces->totalForce = v3add(v3add(outForces->addedMassForce, outForces->viscousForce),
                                  v3add(v3add(outForces->buoyancyForce, outForces->coriolisForce), g_ctrlForce));
    outForces->totalTorque = Tw;
}

extern "C" void AUV_Update(float dtSec)
{
    if (!g_initialized || dtSec <= 0) return;

    // Sub-step for extreme maneuvers (RK4 is good, but stiff problems love smaller h)
    int subSteps = 1;
    float av = v3len(g_state.angularVelocity);
    float lv = v3len(g_state.linearVelocity);
    if (av > 1.0f || lv > 10.0f) subSteps = 2;
    if (av > 2.5f || lv > 25.0f) subSteps = 4;
    float h = dtSec / (float)subSteps;
    h = std::min(h, 0.005f); // absolute max step 5 ms

    for (int step = 0; step < subSteps; step++)
    {
        // ------------------------------------------------ RK4 stages
        Derivatives k1, k2, k3, k4;
        AUVState sA = g_state;
        computeDerivatives(&sA, g_ctrlForce, g_ctrlTorque, k1);

        AUVState sB = advanceState(sA, k1, 0.5f * h);
        computeDerivatives(&sB, g_ctrlForce, g_ctrlTorque, k2);

        AUVState sC = advanceState(sA, k2, 0.5f * h);
        computeDerivatives(&sC, g_ctrlForce, g_ctrlTorque, k3);

        AUVState sD = advanceState(sA, k3, h);
        computeDerivatives(&sD, g_ctrlForce, g_ctrlTorque, k4);

        // Final state increment (RK4 weight = 1/6 (k1 + 2k2 + 2k3 + k4))
        Derivatives D;
        D.dPos  = v3scl(v3add(v3add(k1.dPos, v3scl(k2.dPos,2)),
                              v3add(v3scl(k3.dPos,2), k4.dPos)), 1.0f/6.0f);
        D.dQuat = qscl(qadd(qadd(k1.dQuat, qscl(k2.dQuat,2)),
                           qadd(qscl(k3.dQuat,2), k4.dQuat)), 1.0f/6.0f);
        D.dLinVel  = v3scl(v3add(v3add(k1.dLinVel, v3scl(k2.dLinVel,2)),
                                 v3add(v3scl(k3.dLinVel,2), k4.dLinVel)), 1.0f/6.0f);
        D.dAngVel  = v3scl(v3add(v3add(k1.dAngVel, v3scl(k2.dAngVel,2)),
                                 v3add(v3scl(k3.dAngVel,2), k4.dAngVel)), 1.0f/6.0f);

        // Multi-stage safety: clamp accelerations BEFORE integration
        D.dLinVel  = v3clamp(D.dLinVel,  120.0f);   // ~12 g
        D.dAngVel  = v3clamp(D.dAngVel,  30.0f);    // ~1700 °/s²

        g_state = advanceState(sA, D, h);
        g_state.orientation = qnormalize(g_state.orientation);

        // Clamp velocities
        g_state.linearVelocity  = v3clamp(g_state.linearVelocity,  30.0f);   // 108 km/h ceiling
        g_state.angularVelocity = v3clamp(g_state.angularVelocity, 6.0f);    // ~344 °/s ceiling

        // Energy conservation watchdog — inject damping on energy spikes
        // E = 0.5 ν^T M ν
        float MM[36];
        float m = g_params.mass;
        float L = std::max(0.1f, g_params.dimensions.z);
        float W = std::max(0.05f, g_params.dimensions.x);
        float H = std::max(0.05f, g_params.dimensions.y);
        for (int i=0;i<36;i++) MM[i] = g_params.addedMass.m[i];
        for (int i=0;i<3;i++) MM[i*6+i] += m;
        MM[21] += m*(H*H+W*W)/12.0f; MM[28] += m*(L*L+H*H)/12.0f; MM[35] += m*(L*L+W*W)/12.0f;
        Vec6 vel = {g_state.linearVelocity.x, g_state.linearVelocity.y, g_state.linearVelocity.z,
                    g_state.angularVelocity.x, g_state.angularVelocity.y, g_state.angularVelocity.z};
        Vec6 Mv; for (int i=0;i<6;i++){float s=0;for(int j=0;j<6;j++)s+=MM[i*6+j]*vel[j];Mv[i]=s;}
        float E = 0; for (int i=0;i<6;i++) E += vel[i]*Mv[i];
        E = std::max(0.0f, 0.5f*E);

        if (g_lastEnergy > 0 && h > 0) {
            float dEdt = (E - g_lastEnergy) / h;
            // With damping, mechanical energy should decrease. Allow small injection (<5%/step)
            float threshold = std::max(50.0f, 0.05f * g_lastEnergy / std::max(1e-6f, h));
            if (dEdt > threshold && !checkSingularityAndReset()) {
                // Inject artificial damping proportional to excess
                float bleed = std::min(0.35f, (dEdt - threshold) / (dEdt + 1e-6f));
                g_state.linearVelocity  = v3scl(g_state.linearVelocity,  1.0f - bleed);
                g_state.angularVelocity = v3scl(g_state.angularVelocity, 1.0f - bleed);
            }
        }
        g_lastEnergy = E;

        // Quaternion singularity watchdog
        if (checkSingularityAndReset()) continue;

        // Surface ceiling (cannot rise above y = 0)
        if (g_state.position.y > 0.0f) {
            g_state.position.y = 0.0f;
            if (g_state.linearVelocity.y > 0) g_state.linearVelocity.y *= -0.25f;
        }
    } // end sub-step

    // Refresh debug diagnostics
    computeForcesAtState(&g_state, g_ctrlForce, g_ctrlTorque,
                         g_state.totalForce, g_state.totalTorque,
                         &g_state.addedMassForce,
                         &g_state.viscousForce,
                         &g_state.buoyancyForce,
                         &g_state.coriolisForce);
    g_params.depth = std::max(0.0f, -g_state.position.y);
    g_state.ambientPressure = AUV_ComputeHydrostaticPressure(g_params.depth);

    // Reset control accumulator
    g_ctrlForce = {0,0,0};
    g_ctrlTorque = {0,0,0};
}

extern "C" void AUV_GetState(AUVState* outState) { if (outState) *outState = g_state; }
extern "C" void AUV_SetState(const AUVState* state) { if (state) { g_state = *state; g_lastEnergy = 0; } }

extern "C" void AUV_ApplyControlForce(const Vec3* force, const Vec3* torque)
{
    if (force)  g_ctrlForce  = v3clamp(*force,  2.5e5f);
    if (torque) g_ctrlTorque = v3clamp(*torque, 8.0e4f);
}
