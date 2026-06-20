using System.Runtime.InteropServices;
using UnityEngine;

namespace DeepseaAUV.Native
{
    [StructLayout(LayoutKind.Sequential)]
    public struct NativeVec3
    {
        public float x, y, z;

        public NativeVec3(Vector3 v)
        {
            x = v.x; y = v.y; z = -v.z;
        }

        public Vector3 ToUnity()
        {
            return new Vector3(x, y, -z);
        }

        public static NativeVec3 FromUnity(Vector3 v)
        {
            return new NativeVec3 { x = v.x, y = v.y, z = -v.z };
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct NativeQuat
    {
        public float x, y, z, w;

        public static NativeQuat FromUnity(Quaternion q)
        {
            return new NativeQuat { x = q.x, y = q.y, z = -q.z, w = q.w };
        }

        public Quaternion ToUnity()
        {
            return new Quaternion(x, y, -z, w);
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct NativeMatrix6
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
        public float[] m;

        public static NativeMatrix6 IdentityScaled(float linearScale, float angularScale)
        {
            var mat = new NativeMatrix6 { m = new float[36] };
            for (int i = 0; i < 3; i++) mat.m[i * 6 + i] = linearScale;
            for (int i = 3; i < 6; i++) mat.m[i * 6 + i] = angularScale;
            return mat;
        }

        public static NativeMatrix6 Diagonal(float m00, float m11, float m22, float m33, float m44, float m55)
        {
            var mat = new NativeMatrix6 { m = new float[36] };
            mat.m[0] = m00; mat.m[7] = m11; mat.m[14] = m22;
            mat.m[21] = m33; mat.m[28] = m44; mat.m[35] = m55;
            return mat;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AUVParametersNative
    {
        public float mass;
        public float displacementVolume;
        public float waterDensity;
        public float gravity;
        public float depth;
        public float pressure;
        public NativeVec3 centerOfBuoyancy;
        public NativeVec3 centerOfGravity;
        public NativeVec3 dimensions;
        public NativeMatrix6 addedMass;
        public NativeMatrix6 linearDamping;
        public NativeMatrix6 quadraticDamping;
        public NativeMatrix6 restoringCoeffs;
        public float coriolisFactor;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct AUVStateNative
    {
        public NativeVec3 position;
        public NativeQuat orientation;
        public NativeVec3 linearVelocity;
        public NativeVec3 angularVelocity;
        public NativeVec3 totalForce;
        public NativeVec3 totalTorque;
        public NativeVec3 addedMassForce;
        public NativeVec3 addedMassTorque;
        public NativeVec3 viscousForce;
        public NativeVec3 viscousTorque;
        public NativeVec3 buoyancyForce;
        public NativeVec3 buoyancyTorque;
        public NativeVec3 coriolisForce;
        public NativeVec3 coriolisTorque;
        public float ambientPressure;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SonarConfigNative
    {
        public uint numBeams;
        public uint numPings;
        public float maxRange;
        public float minRange;
        public float beamWidthRad;
        public float swathAngleRad;
        public float soundVelocity;
        public float frequency;
        public float absorptionCoeff;
        public float noiseFloorDb;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SonarPingNative
    {
        public float depth;
        public float intensity;
        public float range;
        public uint beamId;
        public byte hit;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public byte[] _pad;
    }

    public static class HydrodynamicsBridge
    {
        private const string DLL = "AUVPhysics";

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AUV_Init(ref AUVParametersNative p);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AUV_Update(float dtSec);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AUV_GetState(out AUVStateNative s);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AUV_SetState(ref AUVStateNative s);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AUV_ApplyControlForce(ref NativeVec3 force, ref NativeVec3 torque);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void AUV_ComputeHydrodynamics(ref AUVStateNative s, out AUVStateNative outForces);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern float AUV_ComputeHydrostaticPressure(float depthMeters);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Sonar_Init(ref SonarConfigNative cfg);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Sonar_SetPose(ref NativeVec3 pos, ref NativeQuat quat);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Sonar_GeneratePingDirections([In, Out] NativeVec3[] outDirs, uint count);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern float Sonar_ComputeIntensity(float range, float cosIncident, float targetStrengthDb);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint Sonar_GetBeamCount();
    }
}
