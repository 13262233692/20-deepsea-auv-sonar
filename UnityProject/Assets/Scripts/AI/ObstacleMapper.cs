using System;
using UnityEngine;

namespace DeepseaAUV.AI
{
    /// <summary>
    /// 从多波束声呐 ping 数据构造局部 xz 平面深度网格，
    /// 用 Sobel 算子计算深度梯度，提取高危障碍物点云簇。
    /// 输出：障碍物集合 + 危险区域代价图。
    /// </summary>
    public class ObstacleMapper : MonoBehaviour
    {
        [Header("Grid Config")]
        [SerializeField] private int   _gridSize = 128;      // 128×128 cells
        [SerializeField] private float _cellSize = 1.5f;     // 1.5 m / cell
        [SerializeField] private float _dangerThreshold = 3.0f; // 梯度 > 3 视为悬崖

        [Header("Obstacle Clustering")]
        [SerializeField] private int   _minClusterSize = 8;
        [SerializeField] private float _clusterMergeDist = 6.0f;

        [Header("Carriage")]
        [SerializeField] private MultibeamSonar _sonar;
        [SerializeField] private Transform     _auv;

        // Depth grid in vehicle-body frame (xz plane)
        private float[,] _depthGrid;    // height (y-coordinate) of seafloor at each cell; +Inf = no data
        private float[,] _gradMagGrid;  // gradient magnitude
        private bool[,]  _dangerMask;

        private Vector2 _gridCenter;     // world xz coordinates of grid center

        public int    GridSize       => _gridSize;
        public float  CellSize       => _cellSize;
        public float  DangerThreshold => _dangerThreshold;
        public float[,] DepthGrid   => _depthGrid;
        public bool[,]  DangerMask  => _dangerMask;
        public Vector2   GridCenterWorld => _gridCenter;

        public struct ObstacleCluster
        {
            public Vector3 Centroid;
            public Vector3 Size;      // axis-aligned bounding box
            public float   MaxGradient;
            public int     PointCount;
        }

        private System.Collections.Generic.List<ObstacleCluster> _clusters
            = new System.Collections.Generic.List<ObstacleCluster>();
        public System.Collections.Generic.List<ObstacleCluster> Clusters => _clusters;

        public event System.Action OnMapUpdated;

        private bool _initialized;

        private void Awake()
        {
            // Grid allocation deferred to Start() so that SceneBootstrap can
            // inject parameters via reflection between Awake and Start.
        }

        private void Start()
        {
            EnsureInitialized();
        }

        private void EnsureInitialized()
        {
            if (_initialized) return;
            _depthGrid   = new float[_gridSize, _gridSize];
            _gradMagGrid = new float[_gridSize, _gridSize];
            _dangerMask  = new bool[_gridSize, _gridSize];
            ClearGrid();

            if (_sonar != null && enabled)
            {
                _sonar.PingFired += OnPingFired;
            }
            _initialized = true;
        }

        private void OnEnable()
        {
            if (_initialized && _sonar != null)
                _sonar.PingFired += OnPingFired;
        }
        private void OnDisable()
        {
            if (_initialized && _sonar != null)
                _sonar.PingFired -= OnPingFired;
        }

        public void ClearGrid()
        {
            for (int z = 0; z < _gridSize; z++)
                for (int x = 0; x < _gridSize; x++)
                {
                    _depthGrid[x, z] = float.NaN;
                    _gradMagGrid[x, z] = 0;
                    _dangerMask[x, z] = false;
                }
        }

        private void OnPingFired(MultibeamSonar.PingData[] data, int hits)
        {
            if (_auv == null) return;
            UpdateGridWithPing(data);
            ComputeGradient();
            FindDangerClusters();
            OnMapUpdated?.Invoke();
        }

        private void UpdateGridWithPing(MultibeamSonar.PingData[] data)
        {
            _gridCenter = new Vector2(_auv.position.x, _auv.position.z);
            float halfWorld = _gridSize * _cellSize * 0.5f;

            // Fade old data by shifting + averaging (simulate memory)
            for (int z = 0; z < _gridSize; z++)
                for (int x = 0; x < _gridSize; x++)
                {
                    if (!float.IsNaN(_depthGrid[x, z]))
                    {
                        // exponential forgetting
                        _depthGrid[x, z] = float.NaN; // for now: clear and rewrite each ping (simpler)
                    }
                }

            for (int i = 0; i < data.Length; i++)
            {
                ref var d = ref data[i];
                if (!d.Hit) continue;
                Vector3 p = d.WorldPos;
                float dx = p.x - _gridCenter.x;
                float dz = p.z - _gridCenter.y;
                if (Mathf.Abs(dx) > halfWorld || Mathf.Abs(dz) > halfWorld) continue;

                int gx = Mathf.Clamp(Mathf.FloorToInt((dx + halfWorld) / _cellSize), 0, _gridSize - 1);
                int gz = Mathf.Clamp(Mathf.FloorToInt((dz + halfWorld) / _cellSize), 0, _gridSize - 1);

                // simple average if multiple hits per cell
                if (float.IsNaN(_depthGrid[gx, gz]))
                    _depthGrid[gx, gz] = p.y;
                else
                    _depthGrid[gx, gz] = Mathf.Lerp(_depthGrid[gx, gz], p.y, 0.5f);
            }
        }

        private void ComputeGradient()
        {
            // Sobel operator on depth grid (3x3 kernel)
            float[,] gx = new float[_gridSize, _gridSize];
            float[,] gz = new float[_gridSize, _gridSize];

            for (int z = 1; z < _gridSize - 1; z++)
                for (int x = 1; x < _gridSize - 1; x++)
                {
                    float d00 = SafeDepth(x - 1, z - 1);
                    float d10 = SafeDepth(x,     z - 1);
                    float d20 = SafeDepth(x + 1, z - 1);
                    float d01 = SafeDepth(x - 1, z);
                    float d21 = SafeDepth(x + 1, z);
                    float d02 = SafeDepth(x - 1, z + 1);
                    float d12 = SafeDepth(x,     z + 1);
                    float d22 = SafeDepth(x + 1, z + 1);

                    float gxV = (d20 + 2*d21 + d22) - (d00 + 2*d01 + d02);
                    float gzV = (d02 + 2*d12 + d22) - (d00 + 2*d10 + d20);
                    gx[x, z] = gxV / (8.0f * _cellSize);
                    gz[x, z] = gzV / (8.0f * _cellSize);
                    _gradMagGrid[x, z] = Mathf.Sqrt(gxV*gxV + gzV*gzV) / (8.0f * _cellSize);
                    _dangerMask[x, z] = _gradMagGrid[x, z] > _dangerThreshold;
                }
        }

        private float SafeDepth(int x, int z)
        {
            if (x < 0 || x >= _gridSize || z < 0 || z >= _gridSize) return 0f;
            float d = _depthGrid[x, z];
            return float.IsNaN(d) ? 0f : d;
        }

        // Find clusters of dangerous cells via flood-fill (BFS)
        private void FindDangerClusters()
        {
            _clusters.Clear();
            bool[,] visited = new bool[_gridSize, _gridSize];
            var queue = new System.Collections.Generic.Queue<Vector2Int>();

            for (int z = 0; z < _gridSize; z++)
                for (int x = 0; x < _gridSize; x++)
                {
                    if (!_dangerMask[x, z] || visited[x, z]) continue;
                    queue.Clear();
                    queue.Enqueue(new Vector2Int(x, z));
                    visited[x, z] = true;

                    int count = 0;
                    float minY = float.MaxValue, maxY = float.MinValue;
                    float sumX = 0, sumZ = 0, sumY = 0;
                    float maxGrad = 0;
                    Vector2Int minCell = new Vector2Int(x, z);
                    Vector2Int maxCell = new Vector2Int(x, z);

                    while (queue.Count > 0)
                    {
                        var c = queue.Dequeue();
                        count++;
                        float d = SafeDepth(c.x, c.y);
                        sumX += c.x; sumZ += c.y; sumY += d;
                        minY = Mathf.Min(minY, d);
                        maxY = Mathf.Max(maxY, d);
                        maxGrad = Mathf.Max(maxGrad, _gradMagGrid[c.x, c.y]);
                        minCell = Vector2Int.Min(minCell, c);
                        maxCell = Vector2Int.Max(maxCell, c);

                        for (int dz = -1; dz <= 1; dz++)
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                if (dx == 0 && dz == 0) continue;
                                int nx = c.x + dx, nz = c.y + dz;
                                if (nx < 0 || nx >= _gridSize || nz < 0 || nz >= _gridSize) continue;
                                if (visited[nx, nz] || !_dangerMask[nx, nz]) continue;
                                visited[nx, nz] = true;
                                queue.Enqueue(new Vector2Int(nx, nz));
                            }
                    }

                    if (count >= _minClusterSize)
                    {
                        float avgX = sumX / count;
                        float avgZ = sumZ / count;
                        float avgY = sumY / count;
                        float halfWorld = _gridSize * _cellSize * 0.5f;
                        Vector3 worldCentroid = new Vector3(
                            _gridCenter.x - halfWorld + avgX * _cellSize,
                            avgY,
                            _gridCenter.y - halfWorld + avgZ * _cellSize);
                        Vector3 size = new Vector3(
                            (maxCell.x - minCell.x + 1) * _cellSize,
                            maxY - minY,
                            (maxCell.y - minCell.y + 1) * _cellSize);
                        _clusters.Add(new ObstacleCluster
                        {
                            Centroid = worldCentroid,
                            Size = size,
                            MaxGradient = maxGrad,
                            PointCount = count
                        });
                    }
                }

            // Merge nearby clusters
            MergeClusters();
        }

        private void MergeClusters()
        {
            bool changed;
            do
            {
                changed = false;
                for (int i = 0; i < _clusters.Count; i++)
                    for (int j = i + 1; j < _clusters.Count; j++)
                    {
                        float d = Vector3.Distance(_clusters[i].Centroid, _clusters[j].Centroid);
                        if (d < _clusterMergeDist)
                        {
                            var a = _clusters[i];
                            var b = _clusters[j];
                            int totalPts = a.PointCount + b.PointCount;
                            Vector3 centroid = (a.Centroid * a.PointCount + b.Centroid * b.PointCount) / totalPts;
                            Vector3 size = Vector3.Max(a.Size, b.Size);
                            var merged = new ObstacleCluster
                            {
                                Centroid = centroid,
                                Size = size,
                                MaxGradient = Mathf.Max(a.MaxGradient, b.MaxGradient),
                                PointCount = totalPts
                            };
                            _clusters.RemoveAt(j);
                            _clusters[i] = merged;
                            changed = true;
                            goto endLoop;
                        }
                    }
                endLoop:;
            } while (changed);
        }

        // World-to-grid utility for planner
        public bool WorldToGrid(Vector3 worldPos, out Vector2Int cell)
        {
            float halfWorld = _gridSize * _cellSize * 0.5f;
            float dx = worldPos.x - _gridCenter.x;
            float dz = worldPos.z - _gridCenter.y;
            if (Mathf.Abs(dx) > halfWorld || Mathf.Abs(dz) > halfWorld) { cell = default; return false; }
            int gx = Mathf.Clamp(Mathf.FloorToInt((dx + halfWorld) / _cellSize), 0, _gridSize - 1);
            int gz = Mathf.Clamp(Mathf.FloorToInt((dz + halfWorld) / _cellSize), 0, _gridSize - 1);
            cell = new Vector2Int(gx, gz);
            return true;
        }

        public bool IsDangerous(Vector3 worldPos, float safetyRadius)
        {
            if (!_initialized || _dangerMask == null) return false;
            if (!WorldToGrid(worldPos, out var c)) return false;
            int radiusCells = Mathf.CeilToInt(safetyRadius / _cellSize);
            for (int dz = -radiusCells; dz <= radiusCells; dz++)
                for (int dx = -radiusCells; dx <= radiusCells; dx++)
                {
                    int nx = c.x + dx, nz = c.y + dz;
                    if (nx < 0 || nx >= _gridSize || nz < 0 || nz >= _gridSize) continue;
                    float dist = new Vector2(dx, dz).magnitude * _cellSize;
                    if (dist <= safetyRadius && _dangerMask[nx, nz]) return true;
                }
            return false;
        }
    }
}
