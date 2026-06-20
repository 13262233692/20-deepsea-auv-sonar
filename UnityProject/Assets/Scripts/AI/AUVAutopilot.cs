using UnityEngine;
using System.Collections.Generic;

namespace DeepseaAUV.AI
{
    public enum AUVMode { Manual, AutoCruise, ObstacleAvoid, SurveyPattern }

    /// <summary>
    /// 顶层 AI 自治控制器（大脑）：
    ///  - 接收声呐点云 → 梯度检测障碍物
    ///  - 状态机切换：巡航 / 避障
    ///  - 启动 RRT* 规划器生成绕行路径
    ///  - B-spline 平滑 → 纯追踪跟踪 → 控制指令输出
    ///  - 可切换 Manual/Auto 模式
    /// </summary>
    [RequireComponent(typeof(AUVController))]
    public class AUVAutopilot : MonoBehaviour
    {
        [Header("Mode")]
        [SerializeField] private AUVMode _mode = AUVMode.AutoCruise;
        [SerializeField] private bool   _drawDebug = true;

        [Header("Waypoints (Cruise)")]
        [SerializeField] private List<Vector3> _waypoints = new List<Vector3>();
        [SerializeField] private int           _currentWaypoint = 0;
        [SerializeField] private float         _waypointReachRadius = 6f;

        [Header("Mapping & Planning")]
        [SerializeField] private ObstacleMapper _mapper;
        [SerializeField] private int   _maxRRTIterations = 1500;
        [SerializeField] private float _rrtTimeBudgetMs = 350f;
        [SerializeField] private float _safetyMargin = 3.5f;
        [SerializeField] private float _replanInterval = 1.2f;  // 重新规划周期

        [Header("Tracking")]
        [SerializeField] private float _cruiseSpeed = 2.5f;
        [SerializeField] private float _lookahead = 9f;

        [Header("Avoidance Triggers")]
        [SerializeField] private float _dangerLookahead = 25f;   // 前方探测距离
        [SerializeField] private int   _dangerClusterThreshold = 2;  // 超过 N 个危险簇才触发

        [Header("Stats")]
        public string DebugMode = "Idle";
        public string DebugPlan  = "N/A";
        public float  DebugPlanTimeMs;
        public int    DebugNodesExpanded;

        private AUVController   _auv;
        private RRTStarPlanner  _planner;
        private PathTracker     _tracker;

        private Vector3[]       _currentPath;
        private Vector3[]       _smoothPath;
        private float           _replanTimer;
        private AUVMode         _prevMode;

        // ---- properties ----
        public AUVMode Mode
        {
            get => _mode;
            set { _mode = value; if (value != _prevMode) OnModeChanged(); _prevMode = value; }
        }
        public ObstacleMapper Mapper => _mapper;
        public PathTracker    Tracker => _tracker;
        public Vector3[]      Path  => _smoothPath;

        private void Awake()
        {
            _auv = GetComponent<AUVController>();
            _tracker = new PathTracker();
            _tracker.CruiseSpeed = _cruiseSpeed;
            _tracker.LookaheadDistance = _lookahead;

            _planner = new RRTStarPlanner(2048);
            _planner.StepSize = 4.5f;
            _planner.NearRadius = 10f;
            _planner.SafetyMargin = _safetyMargin;
            _planner.MinAltitude = -200f;
            _planner.MaxAltitude = -5f;

            if (_waypoints.Count == 0)
            {
                GenerateDefaultWaypoints();
            }

            _replanTimer = 0;
            _prevMode = _mode;
        }

        private void Start()
        {
            // mapper is injected by SceneBootstrap via reflection between Awake and Start
            if (_mapper != null)
            {
                _planner.Obstacles = _mapper;
                _mapper.OnMapUpdated += OnMapUpdated;
            }
            OnModeChanged();
        }

        private void GenerateDefaultWaypoints()
        {
            float y = -65f;
            _waypoints.Add(new Vector3(0,    y, -80f));
            _waypoints.Add(new Vector3(120f, y, -40f));
            _waypoints.Add(new Vector3(180f, y,  80f));
            _waypoints.Add(new Vector3(40f,  y,  150f));
            _waypoints.Add(new Vector3(-100f,y,  100f));
            _waypoints.Add(new Vector3(-150f,y,  -50f));
            _waypoints.Add(new Vector3(-60f, y,  -120f));
        }

        private void OnEnable()
        {
            if (_mapper != null) _mapper.OnMapUpdated += OnMapUpdated;
        }
        private void OnDisable()
        {
            if (_mapper != null) _mapper.OnMapUpdated -= OnMapUpdated;
        }

        private void OnMapUpdated()
        {
            // 新的声呐数据到来 → 评估是否需要避障
            if (_mode == AUVMode.AutoCruise || _mode == AUVMode.ObstacleAvoid)
            {
                EvaluateDangerAndMaybeReplan();
            }
        }

        private void Update()
        {
            if (_mode == AUVMode.Manual) return;

            _replanTimer -= Time.deltaTime;

            switch (_mode)
            {
                case AUVMode.AutoCruise: UpdateCruise(); break;
                case AUVMode.ObstacleAvoid: UpdateAvoidance(); break;
                case AUVMode.SurveyPattern: break;
            }

            // Apply computed control if not manual
            if (_mode != AUVMode.Manual && _tracker != null)
            {
                _tracker.ComputeControl(_auv, out Vector3 F, out Vector3 T);
                _auv.ControlForce  = F;
                _auv.ControlTorque = T;
            }
        }

        private void OnModeChanged()
        {
            DebugMode = _mode.ToString();
            if (_mode == AUVMode.AutoCruise)
            {
                PlanToWaypoint(_currentWaypoint);
            }
        }

        // ---------------- Cruise ----------------
        private void UpdateCruise()
        {
            // Check if reached current waypoint
            if (_waypoints.Count == 0) return;
            Vector3 target = _waypoints[_currentWaypoint];
            float dist = Vector3.Distance(transform.position, target);

            if (dist < _waypointReachRadius)
            {
                _currentWaypoint = (_currentWaypoint + 1) % _waypoints.Count;
                PlanToWaypoint(_currentWaypoint);
            }

            // Periodic replanning if obstacles shift
            if (_replanTimer <= 0f && _mapper != null && _mapper.Clusters.Count > 0)
            {
                _replanTimer = _replanInterval;
                PlanToWaypoint(_currentWaypoint);
            }
        }

        private void UpdateAvoidance()
        {
            // Similar to cruise but we already have a detour path
            if (_tracker.IsFinished)
            {
                // back to cruise
                _mode = AUVMode.AutoCruise;
                PlanToWaypoint(_currentWaypoint);
                return;
            }

            if (_replanTimer <= 0f)
            {
                _replanTimer = _replanInterval * 0.6f;
                PlanToWaypoint(_currentWaypoint); // re-optimize as we learn more
            }
        }

        // ---------------- Planning ----------------
        private void EvaluateDangerAndMaybeReplan()
        {
            if (_mapper == null) return;
            var clusters = _mapper.Clusters;

            // Count dangerous clusters in forward sector
            int forwardDanger = 0;
            Vector3 fwd = transform.forward;
            Vector3 pos = transform.position;
            foreach (var c in clusters)
            {
                Vector3 toC = c.Centroid - pos;
                float along = Vector3.Dot(toC, fwd);
                if (along < 0 || along > _dangerLookahead) continue;
                float cross = Vector3.Cross(toC, fwd).magnitude;
                if (cross < c.Size.x * 0.5f + 6f) forwardDanger++;
            }

            bool needsAvoid = forwardDanger >= _dangerClusterThreshold;
            if (needsAvoid && _mode == AUVMode.AutoCruise)
            {
                _mode = AUVMode.ObstacleAvoid;
                DebugMode = "OBSTACLE AVOID";
                PlanToWaypoint(_currentWaypoint);
            }
        }

        private void PlanToWaypoint(int wpIdx)
        {
            if (wpIdx < 0 || wpIdx >= _waypoints.Count || _planner == null) return;
            Vector3 goal = _waypoints[wpIdx];

            var result = _planner.Plan(transform.position, goal, _maxRRTIterations, _rrtTimeBudgetMs);
            DebugPlanTimeMs = (float)result.ComputeTimeMs;
            DebugNodesExpanded = result.IterationsUsed;
            DebugPlan = result.Success ? $"OK {result.PathCost:F1}m" : $"FAIL (near)";

            if (result.Success)
            {
                _currentPath = result.Path;
                _smoothPath = BSplineSmoother.Smooth(_currentPath, 6);
                _tracker.SetPath(_smoothPath);
            }
            else
            {
                // Fallback: straight-line path; tracker will try its best
                _currentPath = new[] { transform.position, goal };
                _smoothPath = _currentPath;
                _tracker.SetPath(_smoothPath);
            }
        }

        // ---------------- Gizmos ----------------
        private void OnDrawGizmosSelected()
        {
            if (!_drawDebug) return;

            // Waypoints
            if (_waypoints != null && _waypoints.Count > 0)
            {
                for (int i = 0; i < _waypoints.Count; i++)
                {
                    bool active = i == _currentWaypoint;
                    Gizmos.color = active ? Color.magenta : new Color(0.5f, 0.3f, 1f, 0.6f);
                    Gizmos.DrawSphere(_waypoints[i], active ? 2.5f : 1.5f);
                    int next = (i + 1) % _waypoints.Count;
                    Gizmos.color = new Color(0.5f, 0.3f, 1f, 0.3f);
                    Gizmos.DrawLine(_waypoints[i], _waypoints[next]);
                }
            }

            // Smooth path
            if (_smoothPath != null && _smoothPath.Length > 1)
            {
                Gizmos.color = Color.cyan;
                for (int i = 0; i < _smoothPath.Length - 1; i++)
                    Gizmos.DrawLine(_smoothPath[i], _smoothPath[i+1]);

                // Lookahead point
                if (_tracker != null)
                {
                    // approximate
                    float s = _tracker.Progress + _tracker.LookaheadDistance;
                    float acc = 0;
                    Vector3 look = _smoothPath[0];
                    for (int i = 0; i < _smoothPath.Length - 1; i++)
                    {
                        float seg = Vector3.Distance(_smoothPath[i], _smoothPath[i+1]);
                        if (acc + seg >= s)
                        {
                            float t = (s - acc) / Mathf.Max(1e-6f, seg);
                            look = Vector3.Lerp(_smoothPath[i], _smoothPath[i+1], t);
                            break;
                        }
                        acc += seg;
                    }
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawSphere(look, 0.8f);
                }
            }

            // Obstacle clusters
            if (_mapper != null)
            {
                Gizmos.color = new Color(1f, 0.2f, 0.2f, 0.6f);
                foreach (var c in _mapper.Clusters)
                {
                    Gizmos.DrawWireCube(c.Centroid, c.Size + Vector3.one * 2f);
                }
            }
        }

        // ---- Public API for external control ----
        public void AddWaypoint(Vector3 wp) { _waypoints.Add(wp); }
        public void ClearWaypoints() { _waypoints.Clear(); _currentWaypoint = 0; }
        public void SetWaypoints(IEnumerable<Vector3> wps)
        {
            _waypoints.Clear();
            _waypoints.AddRange(wps);
            _currentWaypoint = 0;
            if (_mode == AUVMode.AutoCruise) PlanToWaypoint(0);
        }
    }
}
