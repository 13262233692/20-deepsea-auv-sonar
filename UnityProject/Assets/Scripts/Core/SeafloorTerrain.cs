using UnityEngine;

namespace DeepseaAUV
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(MeshCollider))]
    [DefaultExecutionOrder(-50)]
    public class SeafloorTerrain : MonoBehaviour
    {
        [Header("Terrain Dimensions")]
        [SerializeField] private int   _resolution = 512;
        [SerializeField] private float _sizeX = 1000f;
        [SerializeField] private float _sizeZ = 1000f;
        [SerializeField] private float _baseDepth = 80f;
        [SerializeField] private float _amplitude = 35f;

        [Header("Procedural Noise")]
        [SerializeField] private int   _seed = 20241109;
        [SerializeField] private float _frequency = 0.004f;
        [SerializeField] private int   _octaves = 6;
        [SerializeField] [Range(0f, 1f)] private float _persistence = 0.55f;
        [SerializeField] private float _lacunarity = 2.1f;

        [Header("Features")]
        [SerializeField] private Vector2[] _ridgeCenters = new Vector2[]
        {
            new Vector2(0, 0),
            new Vector2(250, -150),
            new Vector2(-200, 180)
        };
        [SerializeField] private float[] _ridgeHeights = new float[] { 20f, 12f, 16f };
        [SerializeField] private float[] _ridgeWidths  = new float[] { 90f, 60f, 70f };

        public int   Resolution  => _resolution;
        public float SizeX       => _sizeX;
        public float SizeZ       => _sizeZ;
        public float BaseDepth   => _baseDepth;
        public Vector2 WorldOrigin => new Vector2(transform.position.x - _sizeX * 0.5f,
                                                  transform.position.z - _sizeZ * 0.5f);

        private float[] _heightLookup;
        private Mesh    _mesh;

        private void Awake()
        {
            Generate();
        }

        public void Generate()
        {
            if (_resolution < 2) _resolution = 2;
            _heightLookup = new float[_resolution * _resolution];

            var verts  = new Vector3[_resolution * _resolution];
            var uvs    = new Vector2[_resolution * _resolution];
            var tris   = new int[(_resolution - 1) * (_resolution - 1) * 6];

            Random.State prevState = Random.state;
            Random.InitState(_seed);
            float phaseX = Random.value * 1000f;
            float phaseY = Random.value * 1000f;
            Random.state = prevState;

            int triIdx = 0;
            for (int z = 0; z < _resolution; z++)
            {
                for (int x = 0; x < _resolution; x++)
                {
                    int idx = z * _resolution + x;
                    float u = (float)x / (_resolution - 1);
                    float v = (float)z / (_resolution - 1);
                    float wx = (u - 0.5f) * _sizeX;
                    float wz = (v - 0.5f) * _sizeZ;

                    float h = FractalNoise(wx + phaseX, wz + phaseY) * _amplitude;

                    for (int r = 0; r < _ridgeCenters.Length; r++)
                    {
                        float dx = wx - _ridgeCenters[r].x;
                        float dz = wz - _ridgeCenters[r].y;
                        float d2 = dx * dx + dz * dz;
                        float w  = _ridgeWidths[r];
                        h += _ridgeHeights[r] * Mathf.Exp(-d2 / (w * w));
                    }

                    float edgeX = Mathf.Min(u, 1f - u) * 2f;
                    float edgeZ = Mathf.Min(v, 1f - v) * 2f;
                    float edgeFade = Mathf.SmoothStep(0f, 1f, Mathf.Min(edgeX, edgeZ));
                    h *= edgeFade;

                    _heightLookup[idx] = h;
                    verts[idx] = new Vector3(wx, -_baseDepth + h, wz);
                    uvs[idx]   = new Vector2(u, v);

                    if (x < _resolution - 1 && z < _resolution - 1)
                    {
                        int a = idx;
                        int b = idx + 1;
                        int c = idx + _resolution;
                        int d = c + 1;
                        tris[triIdx++] = a;
                        tris[triIdx++] = c;
                        tris[triIdx++] = b;
                        tris[triIdx++] = b;
                        tris[triIdx++] = c;
                        tris[triIdx++] = d;
                    }
                }
            }

            if (_mesh == null) _mesh = new Mesh { indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            else _mesh.Clear();
            _mesh.vertices  = verts;
            _mesh.uv        = uvs;
            _mesh.triangles = tris;
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();

            GetComponent<MeshFilter>().sharedMesh = _mesh;
            GetComponent<MeshCollider>().sharedMesh = null;
            GetComponent<MeshCollider>().sharedMesh = _mesh;
        }

        private float FractalNoise(float x, float y)
        {
            float total = 0f;
            float amp = 1f;
            float freq = _frequency;
            float norm = 0f;
            for (int o = 0; o < _octaves; o++)
            {
                total += Mathf.PerlinNoise(x * freq, y * freq) * amp;
                norm  += amp;
                amp   *= _persistence;
                freq  *= _lacunarity;
            }
            return (total / Mathf.Max(0.0001f, norm)) * 2f - 1f;
        }

        public float SampleHeightWorld(float worldX, float worldZ)
        {
            Vector2 o = WorldOrigin;
            float u = Mathf.Clamp01((worldX - o.x) / _sizeX);
            float v = Mathf.Clamp01((worldZ - o.y) / _sizeZ);
            float fx = u * (_resolution - 1);
            float fz = v * (_resolution - 1);
            int x0 = Mathf.FloorToInt(fx); int x1 = Mathf.Min(x0 + 1, _resolution - 1);
            int z0 = Mathf.FloorToInt(fz); int z1 = Mathf.Min(z0 + 1, _resolution - 1);
            float sx = fx - x0; float sz = fz - z0;
            float h00 = _heightLookup[z0 * _resolution + x0];
            float h10 = _heightLookup[z0 * _resolution + x1];
            float h01 = _heightLookup[z1 * _resolution + x0];
            float h11 = _heightLookup[z1 * _resolution + x1];
            float hx0 = Mathf.Lerp(h00, h10, sx);
            float hx1 = Mathf.Lerp(h01, h11, sx);
            return -_baseDepth + Mathf.Lerp(hx0, hx1, sz);
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = new Color(0, 0.6f, 1f, 0.15f);
            Gizmos.DrawWireCube(
                transform.position + new Vector3(0, -_baseDepth, 0),
                new Vector3(_sizeX, _amplitude * 2f + 5f, _sizeZ));
        }
    }
}
