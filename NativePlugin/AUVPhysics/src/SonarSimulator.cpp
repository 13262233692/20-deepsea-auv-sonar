#include "AUVPhysicsAPI.h"
#include <cmath>
#include <cstdint>
#include <algorithm>

static SonarConfig g_sonarCfg;
static Vec3        g_sonarPos;
static Quat        g_sonarRot;
static bool        g_sonarInit = false;

static inline Quat quat_normalize(Quat q) {
    float l = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (l < 1e-12f) return {0,0,0,1};
    return {q.x/l, q.y/l, q.z/l, q.w/l};
}

static Vec3 quat_rotate(Quat q, Vec3 v) {
    Quat qv = {v.x, v.y, v.z, 0};
    Quat qc = {-q.x, -q.y, -q.z, q.w};
    Quat r1 = {
        q.w*qv.x + q.x*qv.w + q.y*qv.z - q.z*qv.y,
        q.w*qv.y - q.x*qv.z + q.y*qv.w + q.z*qv.x,
        q.w*qv.z + q.x*qv.y - q.y*qv.x + q.z*qv.w,
        q.w*qv.w - q.x*qv.x - q.y*qv.y - q.z*qv.z
    };
    Quat r2 = {
        r1.w*qc.x + r1.x*qc.w + r1.y*qc.z - r1.z*qc.y,
        r1.w*qc.y - r1.x*qc.z + r1.y*qc.w + r1.z*qc.x,
        r1.w*qc.z + r1.x*qc.y - r1.y*qc.x + r1.z*qc.w,
        r1.w*qc.w - r1.x*qc.x - r1.y*qc.y - r1.z*qc.z
    };
    return {r2.x, r2.y, r2.z};
}

extern "C" void Sonar_Init(const SonarConfig* config) {
    if (!config) return;
    g_sonarCfg = *config;
    g_sonarRot = {0,0,0,1};
    g_sonarPos = {0,0,0};
    g_sonarInit = true;
}

extern "C" void Sonar_SetPose(const Vec3* pos, const Quat* quat) {
    if (pos)  g_sonarPos = *pos;
    if (quat) g_sonarRot = quat_normalize(*quat);
}

extern "C" uint32_t Sonar_GetBeamCount() {
    return g_sonarInit ? g_sonarCfg.numBeams : 0u;
}

extern "C" void Sonar_GeneratePingDirections(Vec3* outDirections, uint32_t count) {
    if (!outDirections || !g_sonarInit) return;
    uint32_t N = (count < g_sonarCfg.numBeams) ? count : g_sonarCfg.numBeams;
    float halfSwath = g_sonarCfg.swathAngleRad * 0.5f;

    for (uint32_t i = 0; i < N; i++) {
        float t = (N == 1) ? 0.0f : (float)i / (float)(N - 1);
        float azimuth = -halfSwath + 2.0f * halfSwath * t;
        float pitch = 0.0f;

        float cx = std::cos(pitch) * std::sin(azimuth);
        float cy = -std::sin(pitch);
        float cz = std::cos(pitch) * std::cos(azimuth);

        Vec3 local = {cx, cy, cz};
        outDirections[i] = quat_rotate(g_sonarRot, local);
    }
}

extern "C" float Sonar_ComputeIntensity(float rangeMeters, float incidentAngleCos, float targetStrengthDb) {
    if (!g_sonarInit || rangeMeters <= 0.0f || rangeMeters > g_sonarCfg.maxRange) return -120.0f;

    float spreadingLoss = 20.0f * std::log10(std::max(rangeMeters, 0.01f));
    float absorptionLoss = g_sonarCfg.absorptionCoeff * rangeMeters * 2.0f;
    float beamWeight = std::max(0.0f, incidentAngleCos);
    float beamLoss = -3.0f * (1.0f - beamWeight);

    float twoWay = -2.0f * spreadingLoss - absorptionLoss + beamLoss + targetStrengthDb;
    if (twoWay < g_sonarCfg.noiseFloorDb) return g_sonarCfg.noiseFloorDb;
    return twoWay;
}
