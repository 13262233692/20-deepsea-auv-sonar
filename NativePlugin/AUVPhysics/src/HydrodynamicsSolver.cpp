#include "AUVPhysicsAPI.h"
#include <cmath>
#include <cstring>
#include <algorithm>

static AUVParameters g_params;
static AUVState      g_state;
static Vec3         g_ctrlForce;
static Vec3         g_ctrlTorque;
static bool         g_initialized = false;

static inline Vec3 vec3_add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline Vec3 vec3_sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline Vec3 vec3_scale(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}
static inline float vec3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline float vec3_length(Vec3 a) { return std::sqrt(vec3_dot(a, a)); }

static inline Quat quat_normalize(Quat q) {
    float l = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (l < 1e-12f) return {0,0,0,1};
    return {q.x/l, q.y/l, q.z/l, q.w/l};
}

static inline Quat quat_multiply(Quat a, Quat b) {
    return {
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    };
}

static Vec3 quat_rotate_vector(Quat q, Vec3 v) {
    Quat vq = {v.x, v.y, v.z, 0};
    Quat qConj = {-q.x, -q.y, -q.z, q.w};
    Quat r1 = quat_multiply(q, vq);
    Quat r2 = quat_multiply(r1, qConj);
    return {r2.x, r2.y, r2.z};
}

static void matrix6_multiply_vec(const Matrix6& M, const float v[6], float out[6]) {
    for (int i = 0; i < 6; i++) {
        out[i] = 0.0f;
        for (int j = 0; j < 6; j++) {
            out[i] += M.m[i * 6 + j] * v[j];
        }
    }
}

static void matrix6_inverse_diag_dominant(const Matrix6& M, Matrix6& out) {
    float aug[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            aug[i][j] = M.m[i * 6 + j];
        }
        for (int j = 6; j < 12; j++) {
            aug[i][j] = (i == (j - 6)) ? 1.0f : 0.0f;
        }
    }
    for (int col = 0; col < 6; col++) {
        int pivot = col;
        float maxVal = std::fabs(aug[col][col]);
        for (int row = col + 1; row < 6; row++) {
            if (std::fabs(aug[row][col]) > maxVal) {
                maxVal = std::fabs(aug[row][col]);
                pivot = row;
            }
        }
        if (pivot != col) {
            for (int j = 0; j < 12; j++) {
                std::swap(aug[col][j], aug[pivot][j]);
            }
        }
        float div = aug[col][col];
        if (std::fabs(div) < 1e-12f) continue;
        for (int j = 0; j < 12; j++) aug[col][j] /= div;
        for (int row = 0; row < 6; row++) {
            if (row != col) {
                float factor = aug[row][col];
                for (int j = 0; j < 12; j++) {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            out.m[i * 6 + j] = aug[i][j + 6];
        }
    }
}

extern "C" void AUV_Init(const AUVParameters* params) {
    if (!params) return;
    g_params = *params;
    std::memset(&g_state, 0, sizeof(AUVState));
    g_state.orientation = {0,0,0,1};
    g_state.ambientPressure = AUV_ComputeHydrostaticPressure(g_params.depth);
    g_ctrlForce = {0,0,0};
    g_ctrlTorque = {0,0,0};
    g_initialized = true;
}

extern "C" float AUV_ComputeHydrostaticPressure(float depthMeters) {
    const float P0 = 101325.0f;
    const float rho = g_params.waterDensity > 0 ? g_params.waterDensity : 1025.0f;
    const float g = g_params.gravity > 0 ? g_params.gravity : 9.81f;
    return P0 + rho * g * depthMeters;
}

extern "C" void AUV_ComputeHydrodynamics(const AUVState* state, AUVState* outForces) {
    if (!state || !outForces) return;
    std::memset(outForces, 0, sizeof(AUVState));

    Vec3 vel = state->linearVelocity;
    Vec3 ang = state->angularVelocity;
    float v[6] = {vel.x, vel.y, vel.z, ang.x, ang.y, ang.z};

    float amResult[6];
    matrix6_multiply_vec(g_params.addedMass, v, amResult);
    outForces->addedMassForce  = {-amResult[0], -amResult[1], -amResult[2]};
    outForces->addedMassTorque = {-amResult[3], -amResult[4], -amResult[5]};

    float dampResult[6];
    matrix6_multiply_vec(g_params.linearDamping, v, dampResult);
    Vec3 linForce = {-dampResult[0], -dampResult[1], -dampResult[2]};
    Vec3 linTorque = {-dampResult[3], -dampResult[4], -dampResult[5]};

    float vMag[6];
    for (int i = 0; i < 6; i++) vMag[i] = std::fabs(v[i]);
    float quadV[6];
    for (int i = 0; i < 6; i++) quadV[i] = v[i] * vMag[i];
    float quadResult[6];
    matrix6_multiply_vec(g_params.quadraticDamping, quadV, quadResult);
    Vec3 quadForce = {-quadResult[0], -quadResult[1], -quadResult[2]};
    Vec3 quadTorque = {-quadResult[3], -quadResult[4], -quadResult[5]};

    outForces->viscousForce  = vec3_add(linForce, quadForce);
    outForces->viscousTorque = vec3_add(linTorque, quadTorque);

    Vec3 gravForce = {0, -g_params.mass * g_params.gravity, 0};
    float buoyancy = g_params.waterDensity * g_params.gravity * g_params.displacementVolume;
    Vec3 buoyForce = {0, buoyancy, 0};
    outForces->buoyancyForce = vec3_add(gravForce, buoyForce);

    Quat q = quat_normalize(state->orientation);
    Vec3 r = vec3_sub(g_params.centerOfBuoyancy, g_params.centerOfGravity);
    Vec3 rWorld = quat_rotate_vector(q, r);
    outForces->buoyancyTorque = vec3_cross(rWorld, buoyForce);

    Vec3 gravWorld = quat_rotate_vector(q, g_params.centerOfGravity);
    outForces->buoyancyTorque = vec3_add(outForces->buoyancyTorque, vec3_cross(gravWorld, gravForce));

    float m = g_params.mass;
    Matrix6 MRB;
    std::memset(&MRB, 0, sizeof(MRB));
    for (int i = 0; i < 3; i++) MRB.m[i*6+i] = m;
    for (int i = 3; i < 6; i++) MRB.m[i*6+i] = m * 0.05f;

    float MA[36];
    for (int i = 0; i < 36; i++) MA[i] = MRB.m[i] + g_params.addedMass.m[i];
    Matrix6 Mtot;
    std::memcpy(Mtot.m, MA, 36 * sizeof(float));

    Matrix6 Minv;
    matrix6_inverse_diag_dominant(Mtot, Minv);

    float mom[6] = {
        MA[0]*v[0] + MA[1]*v[1] + MA[2]*v[2] + MA[3]*v[3] + MA[4]*v[4] + MA[5]*v[5],
        MA[6]*v[0] + MA[7]*v[1] + MA[8]*v[2] + MA[9]*v[3] + MA[10]*v[4] + MA[11]*v[5],
        MA[12]*v[0] + MA[13]*v[1] + MA[14]*v[2] + MA[15]*v[3] + MA[16]*v[4] + MA[17]*v[5],
        MA[18]*v[0] + MA[19]*v[1] + MA[20]*v[2] + MA[21]*v[3] + MA[22]*v[4] + MA[23]*v[5],
        MA[24]*v[0] + MA[25]*v[1] + MA[26]*v[2] + MA[27]*v[3] + MA[28]*v[4] + MA[29]*v[5],
        MA[30]*v[0] + MA[31]*v[1] + MA[32]*v[2] + MA[33]*v[3] + MA[34]*v[4] + MA[35]*v[5]
    };

    Vec3 crbForce = {
        ang.y * mom[2] - ang.z * mom[1],
        ang.z * mom[0] - ang.x * mom[2],
        ang.x * mom[1] - ang.y * mom[0]
    };
    Vec3 crbTorque = {
        ang.y * mom[5] - ang.z * mom[4] + vel.y * mom[2] - vel.z * mom[1],
        ang.z * mom[3] - ang.x * mom[5] + vel.z * mom[0] - vel.x * mom[2],
        ang.x * mom[4] - ang.y * mom[3] + vel.x * mom[1] - vel.y * mom[0]
    };
    outForces->coriolisForce  = vec3_scale(crbForce, -g_params.coriolisFactor);
    outForces->coriolisTorque = vec3_scale(crbTorque, -g_params.coriolisFactor);

    outForces->totalForce = vec3_add(
        vec3_add(outForces->addedMassForce, outForces->viscousForce),
        vec3_add(vec3_add(outForces->buoyancyForce, outForces->coriolisForce), g_ctrlForce)
    );
    outForces->totalTorque = vec3_add(
        vec3_add(outForces->addedMassTorque, outForces->viscousTorque),
        vec3_add(vec3_add(outForces->buoyancyTorque, outForces->coriolisTorque), g_ctrlTorque)
    );
}

extern "C" void AUV_Update(float dtSec) {
    if (!g_initialized || dtSec <= 0) return;

    AUVState forces;
    AUV_ComputeHydrodynamics(&g_state, &forces);

    g_state.totalForce  = forces.totalForce;
    g_state.totalTorque = forces.totalTorque;
    g_state.addedMassForce   = forces.addedMassForce;
    g_state.addedMassTorque  = forces.addedMassTorque;
    g_state.viscousForce     = forces.viscousForce;
    g_state.viscousTorque    = forces.viscousTorque;
    g_state.buoyancyForce    = forces.buoyancyForce;
    g_state.buoyancyTorque   = forces.buoyancyTorque;
    g_state.coriolisForce    = forces.coriolisForce;
    g_state.coriolisTorque   = forces.coriolisTorque;

    float m = g_params.mass;
    float Ixx = m * 0.05f, Iyy = m * 0.05f, Izz = m * 0.05f;
    for (int i = 0; i < 3; i++) {
        g_params.addedMass.m[i*6+i] = std::max(g_params.addedMass.m[i*6+i], m * 0.1f);
    }
    Vec3 acc = {
        forces.totalForce.x  / (m + g_params.addedMass.m[0]),
        forces.totalForce.y  / (m + g_params.addedMass.m[7]),
        forces.totalForce.z  / (m + g_params.addedMass.m[14])
    };
    Vec3 accAng = {
        forces.totalTorque.x / (Ixx + g_params.addedMass.m[21]),
        forces.totalTorque.y / (Iyy + g_params.addedMass.m[28]),
        forces.totalTorque.z / (Izz + g_params.addedMass.m[35])
    };

    g_state.linearVelocity  = vec3_add(g_state.linearVelocity,  vec3_scale(acc, dtSec));
    g_state.angularVelocity = vec3_add(g_state.angularVelocity, vec3_scale(accAng, dtSec));

    float maxV = 20.0f;
    float lv = vec3_length(g_state.linearVelocity);
    if (lv > maxV) g_state.linearVelocity = vec3_scale(g_state.linearVelocity, maxV / lv);
    float av = vec3_length(g_state.angularVelocity);
    if (av > 3.0f) g_state.angularVelocity = vec3_scale(g_state.angularVelocity, 3.0f / av);

    Quat q = quat_normalize(g_state.orientation);
    Vec3 av_half = vec3_scale(g_state.angularVelocity, dtSec * 0.5f);
    Quat dq = {av_half.x, av_half.y, av_half.z, 1.0f};
    g_state.orientation = quat_normalize(quat_multiply(q, dq));

    Vec3 posDelta = quat_rotate_vector(g_state.orientation, vec3_scale(g_state.linearVelocity, dtSec));
    g_state.position = vec3_add(g_state.position, posDelta);

    if (g_state.position.y > 0.0f) {
        g_state.position.y = 0.0f;
        if (g_state.linearVelocity.y > 0) g_state.linearVelocity.y *= -0.3f;
    }

    g_params.depth = -g_state.position.y;
    g_state.ambientPressure = AUV_ComputeHydrostaticPressure(g_params.depth);

    g_ctrlForce  = {0,0,0};
    g_ctrlTorque = {0,0,0};
}

extern "C" void AUV_GetState(AUVState* outState) {
    if (outState) *outState = g_state;
}

extern "C" void AUV_SetState(const AUVState* state) {
    if (state) g_state = *state;
}

extern "C" void AUV_ApplyControlForce(const Vec3* force, const Vec3* torque) {
    if (force)  g_ctrlForce  = *force;
    if (torque) g_ctrlTorque = *torque;
}
