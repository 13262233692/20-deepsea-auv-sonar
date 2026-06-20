using UnityEngine;

namespace DeepseaAUV
{
    [RequireComponent(typeof(AUVController))]
    public class AUVKeyboardInput : MonoBehaviour
    {
        [Header("Force / Torque Scales")]
        [SerializeField] private Vector3 _forceScale  = new Vector3(800, 4000, 12000);
        [SerializeField] private Vector3 _torqueScale = new Vector3(300, 900, 500);

        [Header("Auto-Trim")]
        [SerializeField] private bool  _autoLevel       = true;
        [SerializeField] private float _levelGain       = 400f;
        [SerializeField] private float _autoDepthTarget = -60f;
        [SerializeField] private bool  _holdDepth       = true;
        [SerializeField] private float _depthGain       = 6000f;

        [Header("Boost")]
        [SerializeField] private KeyCode _boostKey = KeyCode.LeftShift;
        [SerializeField] private float   _boostMul = 2.5f;

        private AUVController _auv;

        private void Awake()
        {
            _auv = GetComponent<AUVController>();
        }

        private void Update()
        {
            Vector3 F = Vector3.zero;
            Vector3 T = Vector3.zero;
            float boost = Input.GetKey(_boostKey) ? _boostMul : 1f;

            float surge   = Input.GetAxisRaw("Vertical");
            float sway    = 0;
            if (Input.GetKey(KeyCode.A)) sway -= 1;
            if (Input.GetKey(KeyCode.D)) sway += 1;
            float heave   = 0;
            if (Input.GetKey(KeyCode.Q)) heave -= 1;
            if (Input.GetKey(KeyCode.E)) heave += 1;

            float roll = 0;
            float pitch = Input.GetAxisRaw("Mouse Y");
            float yaw = Input.GetAxisRaw("Mouse X");
            if (Input.GetKey(KeyCode.Keypad4)) roll  -= 1;
            if (Input.GetKey(KeyCode.Keypad6)) roll  += 1;

            // vehicle frame -> world frame
            Quaternion q = transform.rotation;
            Vector3 fwd   = q * Vector3.forward;
            Vector3 right = q * Vector3.right;
            Vector3 up    = Vector3.up;

            F += fwd   * surge   * _forceScale.z * boost;
            F += right * sway    * _forceScale.x * boost;
            F += up    * heave   * _forceScale.y * boost;

            // torque in world frame (approx)
            Vector3 angVel = _auv.AngularVelocity;
            T += Vector3.right   * pitch * _torqueScale.y;
            T += Vector3.up      * yaw   * _torqueScale.z;
            T += Vector3.forward * roll  * _torqueScale.x;
            T -= angVel * 60f; // damping term

            if (_autoLevel)
            {
                Vector3 fwdProj = Vector3.ProjectOnPlane(fwd, Vector3.up).normalized;
                Vector3 rightProj = Vector3.ProjectOnPlane(right, Vector3.up).normalized;
                float pitchErr = Vector3.SignedAngle(fwdProj, fwd, right);
                float rollErr  = Vector3.SignedAngle(rightProj, right, fwd);
                T += right * (-pitchErr) * _levelGain;
                T += fwd   * (-rollErr)  * _levelGain;
            }

            if (_holdDepth)
            {
                float err = _autoDepthTarget - transform.position.y;
                float vY  = _auv.LinearVelocity.y;
                float targetVY = Mathf.Clamp(err * 1.2f, -2.5f, 2.5f);
                F += Vector3.up * (targetVY - vY) * _depthGain;
            }

            if (Input.GetKey(KeyCode.R))
            {
                _auv.ResetPose(new Vector3(0, -60, 0), Quaternion.identity);
            }

            _auv.ControlForce  = F;
            _auv.ControlTorque = T;
        }
    }
}
