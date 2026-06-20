using UnityEngine;

namespace DeepseaAUV.AI
{
    /// <summary>
    /// 纯追踪控制器 (Pure Pursuit + Lookahead)
    /// 给定一条 B-spline 平滑路径，反向计算所需推力器合力/合力矩，
    /// 驱动物体跟踪路径。支持纵向速度控制 + 姿态 + 深度跟踪。
    /// </summary>
    public class PathTracker
    {
        public float LookaheadDistance = 8.0f;   // 前视距离 (m)
        public float CruiseSpeed      = 2.5f;    // 巡航速度 (m/s)
        public float MaxSpeed         = 5.0f;    // 最大速度

        public float PositionGain     = 1.2f;    // 位置跟踪增益
        public float VelocityGain     = 0.8f;    // 速度阻尼增益
        public float AttitudeGain     = 4.0f;    // 姿态角增益
        public float AttitudeRateGain = 1.5f;    // 姿态角速度增益

        public float DepthGain        = 8000f;   // 深度控制推力 (N)
        public float SurgeForceMax    = 12000f;  // 纵向推力上限
        public float YawTorqueMax     = 600f;    // 偏航力矩上限
        public float PitchTorqueMax   = 800f;    // 俯仰力矩上限

        private Vector3[] _path;
        private int       _pathIndex;
        private float     _pathProgress;         // 累积弧长
        private float     _totalLength;

        public Vector3[] Path       => _path;
        public float     Progress   => _pathProgress;
        public float     TotalLength => _totalLength;
        public bool      IsFinished => _path != null && _pathProgress >= _totalLength * 0.98f;

        public void SetPath(Vector3[] path)
        {
            _path = path;
            _pathIndex = 0;
            _pathProgress = 0f;
            _totalLength = BSplineSmoother.TotalLength(path);
        }

        /// <summary>
        /// 计算控制指令 (世界坐标系力 + 力矩)
        /// </summary>
        public void ComputeControl(AUVController auv, out Vector3 forceWorld, out Vector3 torqueWorld)
        {
            forceWorld = Vector3.zero;
            torqueWorld = Vector3.zero;

            if (_path == null || _path.Length < 2) return;

            Vector3 pos = auv.transform.position;
            Vector3 vel = auv.LinearVelocity;

            // ---- 1. Find lookahead point on path ----
            UpdateProgress(pos);
            Vector3 targetPos = GetLookaheadPoint();
            Vector3 pathTangent = GetPathTangent(_pathIndex);

            // ---- 2. Surge speed control (longitudinal) ----
            float targetSpeed = CruiseSpeed;
            // Slow down on sharp curves
            float curv = EstimateCurvature(_pathIndex);
            targetSpeed = Mathf.Lerp(CruiseSpeed, MaxSpeed * 0.4f, Mathf.Clamp01(curv * 4f));

            Vector3 fwd = pathTangent.normalized;
            float speedAlong = Vector3.Dot(vel, fwd);
            float speedError = targetSpeed - speedAlong;
            float surgeForce = speedError * 4000f; // crude gain
            surgeForce = Mathf.Clamp(surgeForce, -SurgeForceMax * 0.4f, SurgeForceMax);

            // ---- 3. Lateral position correction (cross-track error) ----
            Vector3 pathPos = GetPathPoint(_pathIndex);
            Vector3 crossErr = pos - pathPos;
            // Remove along-track component, keep only lateral
            crossErr -= fwd * Vector3.Dot(crossErr, fwd);
            Vector3 latForce = -crossErr * PositionGain * 600f;
            latForce -= Vector3.Project(vel - fwd * speedAlong, -crossErr.normalized) * VelocityGain * 300f;

            // ---- 4. Depth control (heave) ----
            float depthErr = targetPos.y - pos.y;
            float heaveForce = depthErr * DepthGain;
            heaveForce = Mathf.Clamp(heaveForce, -15000f, 15000f);

            // ---- 5. Attitude control (yaw + pitch) ----
            Vector3 desiredForward = fwd;
            Vector3 currentForward = auv.transform.forward;
            Vector3 currentUp = auv.transform.up;

            // yaw error (rotation around world Y)
            Vector3 fwdProjXZ = Vector3.ProjectOnPlane(desiredForward, Vector3.up).normalized;
            Vector3 curFwdXZ = Vector3.ProjectOnPlane(currentForward, Vector3.up).normalized;
            float yawErr = Vector3.SignedAngle(curFwdXZ, fwdProjXZ, Vector3.up);

            // pitch error
            float pitchErr = Vector3.SignedAngle(currentForward, desiredForward, -auv.transform.right);

            // roll error (keep level, world up = desired up)
            Vector3 desiredUp = Vector3.up;
            float rollErr = Vector3.SignedAngle(currentUp, desiredUp, currentForward);

            // angular velocity damping
            Vector3 angVel = auv.AngularVelocity;

            Vector3 torque = new Vector3(
                -rollErr  * AttitudeGain * 120f  - angVel.x * AttitudeRateGain * 60f,
                 pitchErr * AttitudeGain * 80f   - angVel.y * AttitudeRateGain * 40f,
                 yawErr   * AttitudeGain * 100f  - angVel.z * AttitudeRateGain * 50f);

            torque.x = Mathf.Clamp(torque.x, -400f, 400f);
            torque.y = Mathf.Clamp(torque.y, -PitchTorqueMax, PitchTorqueMax);
            torque.z = Mathf.Clamp(torque.z, -YawTorqueMax, YawTorqueMax);

            // Assemble world-frame force
            forceWorld = fwd * surgeForce + latForce + Vector3.up * heaveForce;

            // Convert body torque to world (approx)
            torqueWorld = auv.transform.rotation * torque;
        }

        private void UpdateProgress(Vector3 pos)
        {
            if (_path == null) return;
            // Advance along path until nearest point is found
            float minDist = float.MaxValue;
            int bestIdx = _pathIndex;
            int searchAhead = Mathf.Min(40, _path.Length - _pathIndex - 2);
            for (int i = 0; i <= searchAhead; i++)
            {
                int idx = _pathIndex + i;
                if (idx >= _path.Length - 1) break;
                float d = DistanceToSegment(pos, _path[idx], _path[idx + 1]);
                if (d < minDist) { minDist = d; bestIdx = idx; }
            }
            _pathIndex = bestIdx;

            // Update arc-length progress
            float s = 0;
            for (int i = 0; i < _pathIndex && i < _path.Length - 1; i++)
                s += Vector3.Distance(_path[i], _path[i + 1]);
            // add fractional segment
            float segLen = Vector3.Distance(_path[_pathIndex], _path[_pathIndex + 1]);
            Vector3 closest = ClosestPointOnSegment(pos, _path[_pathIndex], _path[_pathIndex + 1]);
            s += Vector3.Distance(_path[_pathIndex], closest);
            _pathProgress = s;
        }

        public Vector3 GetLookaheadPoint()
        {
            float targetS = _pathProgress + LookaheadDistance;
            targetS = Mathf.Min(targetS, _totalLength);
            float acc = 0;
            for (int i = 0; i < _path.Length - 1; i++)
            {
                float seg = Vector3.Distance(_path[i], _path[i + 1]);
                if (acc + seg >= targetS)
                {
                    float t = (targetS - acc) / Mathf.Max(1e-6f, seg);
                    return Vector3.Lerp(_path[i], _path[i + 1], t);
                }
                acc += seg;
            }
            return _path[_path.Length - 1];
        }

        private Vector3 GetPathPoint(int idx)
        {
            idx = Mathf.Clamp(idx, 0, _path.Length - 1);
            return _path[idx];
        }

        private Vector3 GetPathTangent(int idx)
        {
            int i0 = Mathf.Max(0, idx - 1);
            int i1 = Mathf.Min(_path.Length - 1, idx + 2);
            return (_path[i1] - _path[i0]).normalized;
        }

        private float EstimateCurvature(int idx)
        {
            if (idx < 1 || idx > _path.Length - 3) return 0;
            Vector3 p0 = _path[idx - 1];
            Vector3 p1 = _path[idx];
            Vector3 p2 = _path[idx + 1];
            Vector3 d1 = p1 - p0;
            Vector3 d2 = p2 - p1;
            float a = d1.magnitude;
            float b = d2.magnitude;
            if (a < 1e-4f || b < 1e-4f) return 0;
            float angle = Vector3.Angle(d1, d2) * Mathf.Deg2Rad;
            return Mathf.Abs(angle / (0.5f * (a + b)));
        }

        private static float DistanceToSegment(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float t = Mathf.Clamp01(Vector3.Dot(p - a, ab) / Mathf.Max(1e-6f, ab.sqrMagnitude));
            Vector3 cp = a + ab * t;
            return Vector3.Distance(p, cp);
        }

        private static Vector3 ClosestPointOnSegment(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float t = Mathf.Clamp01(Vector3.Dot(p - a, ab) / Mathf.Max(1e-6f, ab.sqrMagnitude));
            return a + ab * t;
        }
    }
}
