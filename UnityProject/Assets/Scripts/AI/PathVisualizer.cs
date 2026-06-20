using UnityEngine;
using System.Collections.Generic;
using DeepseaAUV.AI;

namespace DeepseaAUV
{
    /// <summary>
    /// 路径可视化：用 LineRenderer 渲染 B-spline 路径 + 危险簇包围盒
    /// 挂在 AUV 上，由 AUVAutopilot 驱动
    /// </summary>
    [RequireComponent(typeof(AUVAutopilot))]
    public class PathVisualizer : MonoBehaviour
    {
        [Header("Path Render")]
        [SerializeField] private Color _pathColor = new Color(0f, 1f, 0.8f, 0.7f);
        [SerializeField] private float _pathWidth = 0.15f;
        [SerializeField] private bool  _showWaypoints = true;

        [Header("Obstacles")]
        [SerializeField] private Color _obstacleColor = new Color(1f, 0.2f, 0.2f, 0.5f);

        [Header("Lookahead")]
        [SerializeField] private Color _lookaheadColor = Color.yellow;
        [SerializeField] private float _lookaheadSphereRadius = 0.6f;

        private LineRenderer      _pathLine;
        private GameObject        _lookaheadMarker;
        private AUVAutopilot      _autopilot;
        private List<GameObject>  _clusterMarkers = new List<GameObject>();

        private void Awake()
        {
            _autopilot = GetComponent<AUVAutopilot>();

            // Path line
            var lineGO = new GameObject("PathLine");
            lineGO.transform.SetParent(transform, false);
            _pathLine = lineGO.AddComponent<LineRenderer>();
            _pathLine.material = new Material(Shader.Find("Unlit/Color"));
            _pathLine.material.color = _pathColor;
            _pathLine.startWidth = _pathWidth;
            _pathLine.endWidth = _pathWidth;
            _pathLine.positionCount = 0;
            _pathLine.useWorldSpace = true;
            _pathLine.numCapVertices = 4;

            // Lookahead marker
            _lookaheadMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _lookaheadMarker.name = "LookaheadMarker";
            Destroy(_lookaheadMarker.GetComponent<Collider>());
            _lookaheadMarker.transform.localScale = Vector3.one * _lookaheadSphereRadius * 2f;
            var mr = _lookaheadMarker.GetComponent<MeshRenderer>();
            mr.material = new Material(Shader.Find("Unlit/Color"));
            mr.material.color = _lookaheadColor;
        }

        private void LateUpdate()
        {
            UpdatePath();
            UpdateObstacles();
            UpdateLookahead();
        }

        private void UpdatePath()
        {
            if (_autopilot == null || _autopilot.Path == null || _autopilot.Path.Length < 2)
            {
                _pathLine.positionCount = 0;
                return;
            }
            var path = _autopilot.Path;
            if (_pathLine.positionCount != path.Length)
                _pathLine.positionCount = path.Length;
            _pathLine.SetPositions(path);
        }

        private void UpdateObstacles()
        {
            if (_autopilot == null || _autopilot.Mapper == null) return;
            var clusters = _autopilot.Mapper.Clusters;

            // Grow pool as needed
            while (_clusterMarkers.Count < clusters.Count)
            {
                var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                go.name = "ClusterMarker";
                Destroy(go.GetComponent<Collider>());
                var mr = go.GetComponent<MeshRenderer>();
                var mat = new Material(Shader.Find("Unlit/Color"));
                mat.color = _obstacleColor;
                mr.material = mat;
                _clusterMarkers.Add(go);
            }

            // Activate only needed
            for (int i = 0; i < _clusterMarkers.Count; i++)
            {
                bool active = i < clusters.Count;
                _clusterMarkers[i].SetActive(active);
                if (active)
                {
                    _clusterMarkers[i].transform.position = clusters[i].Centroid;
                    _clusterMarkers[i].transform.localScale = clusters[i].Size + Vector3.one * 2f;
                }
            }
        }

        private void UpdateLookahead()
        {
            if (_autopilot == null || _autopilot.Tracker == null || _autopilot.Path == null)
            {
                _lookaheadMarker.SetActive(false);
                return;
            }
            _lookaheadMarker.SetActive(true);
            var tracker = _autopilot.Tracker;
            Vector3 look = tracker.GetLookaheadPoint();
            _lookaheadMarker.transform.position = look;
        }
    }
}
