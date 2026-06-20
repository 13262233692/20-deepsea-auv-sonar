#pragma once

#ifdef _WIN32
    #ifdef AUVPHYSICS_EXPORTS
        #define AUVPHYSICS_API __declspec(dllexport)
    #else
        #define AUVPHYSICS_API __declspec(dllimport)
    #endif
#else
    #define AUVPHYSICS_API __attribute__((visibility("default")))
#endif

#include <cstdint>

extern "C" {

struct Vec3 {
    float x, y, z;
};

struct Quat {
    float x, y, z, w;
};

struct Matrix6 {
    float m[36];
};

struct AUVParameters {
    float mass;
    float displacementVolume;
    float waterDensity;
    float gravity;
    float depth;
    float pressure;
    Vec3  centerOfBuoyancy;
    Vec3  centerOfGravity;
    Vec3  dimensions;
    Matrix6 addedMass;
    Matrix6 linearDamping;
    Matrix6 quadraticDamping;
    Matrix6 restoringCoeffs;
    float coriolisFactor;
};

struct AUVState {
    Vec3  position;
    Quat  orientation;
    Vec3  linearVelocity;
    Vec3  angularVelocity;
    Vec3  totalForce;
    Vec3  totalTorque;
    Vec3  addedMassForce;
    Vec3  addedMassTorque;
    Vec3  viscousForce;
    Vec3  viscousTorque;
    Vec3  buoyancyForce;
    Vec3  buoyancyTorque;
    Vec3  coriolisForce;
    Vec3  coriolisTorque;
    float ambientPressure;
};

struct SonarConfig {
    uint32_t numBeams;
    uint32_t numPings;
    float    maxRange;
    float    minRange;
    float    beamWidthRad;
    float    swathAngleRad;
    float    soundVelocity;
    float    frequency;
    float    absorptionCoeff;
    float    noiseFloorDb;
};

struct SonarPing {
    float    depth;
    float    intensity;
    float    range;
    uint32_t beamId;
    uint8_t  hit;
    uint8_t  _pad[3];
};

AUVPHYSICS_API void  AUV_Init(const AUVParameters* params);
AUVPHYSICS_API void  AUV_Update(float dtSec);
AUVPHYSICS_API void  AUV_GetState(AUVState* outState);
AUVPHYSICS_API void  AUV_SetState(const AUVState* state);
AUVPHYSICS_API void  AUV_ApplyControlForce(const Vec3* force, const Vec3* torque);
AUVPHYSICS_API void  AUV_ComputeHydrodynamics(const AUVState* state, AUVState* outForces);
AUVPHYSICS_API float AUV_ComputeHydrostaticPressure(float depthMeters);

AUVPHYSICS_API void  Sonar_Init(const SonarConfig* config);
AUVPHYSICS_API void  Sonar_SetPose(const Vec3* pos, const Quat* quat);
AUVPHYSICS_API void  Sonar_GeneratePingDirections(Vec3* outDirections, uint32_t count);
AUVPHYSICS_API float Sonar_ComputeIntensity(float rangeMeters, float incidentAngleCos, float targetStrengthDb);
AUVPHYSICS_API uint32_t Sonar_GetBeamCount();

}
