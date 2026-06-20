using UnityEngine;
using DeepseaAUV.Native;

namespace DeepseaAUV
{
    [DefaultExecutionOrder(-100)]
    public class AUVController : MonoBehaviour
    {
        [Header("Mass / Displacement")]
        [SerializeField] private float _mass = 1500f;
        [SerializeField] private float _displacementM3 = 1.47f;
        [SerializeField] private float _waterDensity = 1025f;
        [SerializeField] private float _gravity = 9.81f;
        [SerializeField] private Vector3 _centerOfGravity = Vector3.zero;
        [SerializeField] private Vector3 _centerOfBuoyancy = new Vector3(0, 0.15f, 0);
        [SerializeField] private Vector3 _dimensions = new Vector3(2.5f, 1.0f, 6.0f);

        [Header("Added Mass (added mass matrix diagonal X,Y,Z,K,M,N)")]
        [SerializeField] private Vector3 _addedMassLin = new Vector3(800f, 1500f, 2200f);
        [SerializeField] private Vector3 _addedMassAng = new Vector3(400f, 600f, 5000f);

        [Header("Linear Damping")]
        [SerializeField] private Vector3 _linDampLin = new Vector3(60f, 700f, 900f);
        [SerializeField] private Vector3 _linDampAng = new Vector3(200f, 500f, 600f);

        [Header("Quadratic Damping")]
        [SerializeField] private Vector3 _quadDampLin = new Vector3(180f, 8000f, 12000f);
        [SerializeField] private Vector3 _quadDampAng = new Vector3(800f, 2000f, 3000f);

        [Header("Coriolis / Hydrostatic")]
        [SerializeField] [Range(0f, 1f)] private float _coriolisFactor = 0.95f;

        [Header("Control Inputs (runtime)")]
        public Vector3 ControlForce;
        public Vector3 ControlTorque;

        [Header("Physics Stats")]
        public Vector3 DebugTotalForce;
        public Vector3 DebugTotalTorque;
        public Vector3 DebugBuoyancyForce;
        public Vector3 DebugBuoyancyTorque;
        public Vector3 DebugViscousForce;
        public Vector3 DebugAddedMassForce;
        public float   DebugPressureKPa;
        public float   DebugDepthM;

        public Vector3  LinearVelocity  { get; private set; }
        public Vector3  AngularVelocity { get; private set; }

        private bool _initialized;

        private void Awake()
        {
            InitializeNative();
        }

        private void InitializeNative()
        {
            var p = new AUVParametersNative
            {
                mass               = _mass,
                displacementVolume = _displacementM3,
                waterDensity       = _waterDensity,
                gravity            = _gravity,
                depth              = Mathf.Max(0, -transform.position.y),
                pressure           = 0f,
                centerOfGravity    = NativeVec3.FromUnity(_centerOfGravity),
                centerOfBuoyancy   = NativeVec3.FromUnity(_centerOfBuoyancy),
                dimensions         = NativeVec3.FromUnity(_dimensions),
                addedMass          = NativeMatrix6.Diagonal(
                    _addedMassLin.x, _addedMassLin.y, _addedMassLin.z,
                    _addedMassAng.x, _addedMassAng.y, _addedMassAng.z),
                linearDamping      = NativeMatrix6.Diagonal(
                    _linDampLin.x, _linDampLin.y, _linDampLin.z,
                    _linDampAng.x, _linDampAng.y, _linDampAng.z),
                quadraticDamping   = NativeMatrix6.Diagonal(
                    _quadDampLin.x, _quadDampLin.y, _quadDampLin.z,
                    _quadDampAng.x, _quadDampAng.y, _quadDampAng.z),
                restoringCoeffs    = NativeMatrix6.Diagonal(0,0,0,0,0,0),
                coriolisFactor     = _coriolisFactor
            };
            HydrodynamicsBridge.AUV_Init(ref p);
            SyncTransformToNative();
            _initialized = true;
        }

        private void SyncTransformToNative()
        {
            var ns = new AUVStateNative
            {
                position       = NativeVec3.FromUnity(transform.position),
                orientation    = NativeQuat.FromUnity(transform.rotation),
                linearVelocity = NativeVec3.FromUnity(Vector3.zero),
                angularVelocity = NativeVec3.FromUnity(Vector3.zero)
            };
            HydrodynamicsBridge.AUV_SetState(ref ns);
        }

        private void FixedUpdate()
        {
            if (!_initialized) return;

            var f = NativeVec3.FromUnity(ControlForce);
            var t = NativeVec3.FromUnity(ControlTorque);
            HydrodynamicsBridge.AUV_ApplyControlForce(ref f, ref t);

            float dt = Mathf.Clamp(Time.fixedDeltaTime, 1e-4f, 0.05f);
            HydrodynamicsBridge.AUV_Update(dt);

            HydrodynamicsBridge.AUV_GetState(out var s);

            transform.position    = s.position.ToUnity();
            transform.rotation    = s.orientation.ToUnity();
            LinearVelocity        = s.linearVelocity.ToUnity();
            AngularVelocity       = s.angularVelocity.ToUnity();

            DebugTotalForce       = s.totalForce.ToUnity();
            DebugTotalTorque      = s.totalTorque.ToUnity();
            DebugBuoyancyForce    = s.buoyancyForce.ToUnity();
            DebugBuoyancyTorque   = s.buoyancyTorque.ToUnity();
            DebugViscousForce     = s.viscousForce.ToUnity();
            DebugAddedMassForce   = s.addedMassForce.ToUnity();
            DebugPressureKPa      = s.ambientPressure / 1000f;
            DebugDepthM           = Mathf.Max(0, -transform.position.y);
        }

        public void AddImpulse(Vector3 forceWorld)
        {
            ControlForce += forceWorld;
        }

        public void ResetPose(Vector3 pos, Quaternion rot)
        {
            transform.SetPositionAndRotation(pos, rot);
            SyncTransformToNative();
            ControlForce = Vector3.zero;
            ControlTorque = Vector3.zero;
        }
    }
}
