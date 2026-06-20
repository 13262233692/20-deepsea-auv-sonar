using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace DeepseaAUV
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class SonarPointCloud : MonoBehaviour
    {
        [Header("Target Sonar")]
        [SerializeField] private MultibeamSonar _sonar;

        [Header("Buffer Capacity")]
        [SerializeField] private int  _maxPointsPerStripe = 65536;
        [SerializeField] private int  _stripeHistory = 16;
        [SerializeField] private float _pointSize = 0.25f;
        [SerializeField] [Range(0f, 1f)] private float _globalOpacity = 1f;

        [Header("Depth Color Scale")]
        [SerializeField] private float _minDepth = 10f;
        [SerializeField] private float _maxDepth = 200f;
        [SerializeField] private Gradient _depthColor = DefaultBathymetryGradient();

        [Header("Intensity Blending")]
        [SerializeField] [Range(-100f, 0f)] private float _minIntensityDb = -80f;
        [SerializeField] [Range(-100f, 0f)] private float _maxIntensityDb = -10f;

        [Header("Rendering")]
        [SerializeField] private Shader _pointShader;
        [SerializeField] private bool   _castShadows = false;
        [SerializeField] private bool   _receiveShadows = false;

        [StructLayout(LayoutKind.Sequential)]
        private struct Point
        {
            public Vector3 Pos;
            public Color   Color;
            public float   Size;
            public float   _pad;
        }

        private Mesh       _mesh;
        private Material   _mat;
        private List<Point>[] _stripes;
        private int        _writeHead;
        private int        _pointCount;
        private ComputeBuffer _cbPoints;
        private int        _bufferSlot;
        private MaterialPropertyBlock _mpb;
        private int[]      _indices;

        private static Gradient DefaultBathymetryGradient()
        {
            var g = new Gradient();
            var ck = new GradientColorKey[]
            {
                new GradientColorKey(new Color(0.10f, 0.05f, 0.35f), 0.00f), // deep violet
                new GradientColorKey(new Color(0.00f, 0.35f, 0.70f), 0.25f), // blue
                new GradientColorKey(new Color(0.00f, 0.75f, 0.65f), 0.50f), // cyan
                new GradientColorKey(new Color(0.95f, 0.85f, 0.20f), 0.75f), // yellow
                new GradientColorKey(new Color(0.95f, 0.30f, 0.10f), 1.00f), // red (shallow)
            };
            var ak = new GradientAlphaKey[] { new GradientAlphaKey(1, 0), new GradientAlphaKey(1, 1) };
            g.SetKeys(ck, ak);
            g.mode = GradientMode.Blend;
            return g;
        }

        private void Awake()
        {
            _stripes = new List<Point>[_stripeHistory];
            for (int i = 0; i < _stripeHistory; i++) _stripes[i] = new List<Point>(_maxPointsPerStripe);
            _mpb = new MaterialPropertyBlock();

            if (_pointShader == null) _pointShader = Shader.Find("DeepseaAUV/BathymetryPointCloud");
            if (_pointShader == null) _pointShader = Shader.Find("Standard");

            BuildMesh();
            SetupMaterial();
        }

        private void BuildMesh()
        {
            int total = _maxPointsPerStripe * _stripeHistory;
            _mesh = new Mesh { name = "SonarPointCloud", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            var verts = new Vector3[total];
            var uvs   = new Vector2[total];
            var cols  = new Color[total];
            for (int i = 0; i < total; i++) { verts[i] = Vector3.zero; cols[i] = new Color(0,0,0,0); uvs[i] = Vector2.zero; }

            int triCount = total;
            _indices = new int[triCount];
            for (int i = 0; i < triCount; i++) _indices[i] = i;

            _mesh.vertices = verts;
            _mesh.colors   = cols;
            _mesh.uv       = uvs;
            _mesh.SetIndices(_indices, MeshTopology.Points, 0);
            _mesh.bounds = new Bounds(Vector3.zero, new Vector3(10000, 10000, 10000));
            GetComponent<MeshFilter>().sharedMesh = _mesh;
            _pointCount = 0;
        }

        private void SetupMaterial()
        {
            _mat = new Material(_pointShader) { name = "SonarPointCloudMat" };
            _mat.SetFloat("_PointSize", _pointSize);
            _mat.SetFloat("_Opacity", _globalOpacity);
            var r = GetComponent<MeshRenderer>();
            r.sharedMaterial = _mat;
            r.shadowCastingMode = _castShadows ? UnityEngine.Rendering.ShadowCastingMode.On : UnityEngine.Rendering.ShadowCastingMode.Off;
            r.receiveShadows = _receiveShadows;
            r.allowOcclusionWhenDynamic = false;
        }

        private void OnEnable()
        {
            if (_sonar != null) _sonar.PingFired += OnPingFired;
        }

        private void OnDisable()
        {
            if (_sonar != null) _sonar.PingFired -= OnPingFired;
        }

        private void OnPingFired(MultibeamSonar.PingData[] data, int hits)
        {
            AppendStripe(data);
            UpdateMeshBuffer();
        }

        private void AppendStripe(MultibeamSonar.PingData[] data)
        {
            var list = _stripes[_writeHead];
            list.Clear();

            float iRange = Mathf.Max(1e-6f, _maxIntensityDb - _minIntensityDb);
            float dRange = Mathf.Max(1e-6f, _maxDepth - _minDepth);

            for (int i = 0; i < data.Length; i++)
            {
                ref var d = ref data[i];
                if (!d.Hit) continue;

                float depthT = Mathf.Clamp01((d.Depth - _minDepth) / dRange);
                Color depthColor = _depthColor.Evaluate(depthT);

                float intenT = Mathf.Clamp01((d.IntensityDb - _minIntensityDb) / iRange);
                Color c = Color.Lerp(new Color(0.02f, 0.02f, 0.05f, 0f), depthColor, intenT);
                c.a = Mathf.Clamp01(intenT * 1.2f) * _globalOpacity;

                list.Add(new Point { Pos = d.WorldPos, Color = c, Size = _pointSize });
            }

            _writeHead = (_writeHead + 1) % _stripeHistory;
            _bufferSlot = (_bufferSlot + 1) % 2;
        }

        private Vector3[] _vertCache;
        private Color[]   _colCache;
        private Vector2[] _uvCache;

        private void UpdateMeshBuffer()
        {
            int total = _maxPointsPerStripe * _stripeHistory;
            if (_vertCache == null || _vertCache.Length != total)
            {
                _vertCache = new Vector3[total];
                _colCache  = new Color[total];
                _uvCache   = new Vector2[total];
            }

            int cursor = 0;
            int count = 0;
            for (int s = 0; s < _stripeHistory; s++)
            {
                int idx = (_writeHead + s) % _stripeHistory;
                var list = _stripes[idx];
                float ageAlpha = s / (float)(_stripeHistory - 1);
                for (int i = 0; i < list.Count && cursor < total; i++)
                {
                    ref var p = ref list[i];
                    _vertCache[cursor] = p.Pos;
                    Color c = p.Color;
                    c.a *= Mathf.Lerp(1f, 0.25f, ageAlpha);
                    _colCache[cursor] = c;
                    _uvCache[cursor] = new Vector2(p.Size, 0);
                    cursor++;
                }
                count = cursor;
            }

            _mesh.vertices = _vertCache;
            _mesh.colors   = _colCache;
            _mesh.uv       = _uvCache;
            _mesh.UploadMeshData(false);
            _pointCount = count;
        }

        public void Clear()
        {
            for (int i = 0; i < _stripeHistory; i++) _stripes[i].Clear();
            _writeHead = 0;
            UpdateMeshBuffer();
        }

        public int  PointCount       => _pointCount;
        public int  MaxPointsPerStripe => _maxPointsPerStripe;
    }
}
