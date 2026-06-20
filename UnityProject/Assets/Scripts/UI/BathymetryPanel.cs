using UnityEngine;
using UnityEngine.UI;

namespace DeepseaAUV.UI
{
    [RequireComponent(typeof(RawImage))]
    public class BathymetryPanel : MonoBehaviour
    {
        [Header("Panel Config")]
        [SerializeField] private int _panelWidth  = 1024;
        [SerializeField] private int _panelHeight = 512;
        [SerializeField] private MultibeamSonar _sonar;
        [SerializeField] private AUVController   _auv;
        [SerializeField] private SeafloorTerrain _terrain;

        [Header("Bathymetric Colormap")]
        [SerializeField] private Gradient _colormap = DefaultRamp();
        [SerializeField] private float _minDepth = 10f;
        [SerializeField] private float _maxDepth = 200f;

        [Header("History")]
        [SerializeField] private int   _swathHistory  = 128;
        [SerializeField] private float _decayPerSec   = 0.05f;

        [Header("UI")]
        [SerializeField] private Text   _statusLabel;
        [SerializeField] private Text   _depthLabel;
        [SerializeField] private Text   _hitsLabel;
        [SerializeField] private Text   _posLabel;
        [SerializeField] private Text   _pressureLabel;

        private Texture2D _panelTex;
        private Color[]   _panelPixels;
        private RawImage  _image;

        // Swath ring buffer: each swath row = 1 ping across-beam profile
        private struct Swath
        {
            public float[] Depth;
            public float[] Intensity;
            public byte[]  Hit;
            public float   Age;
            public float   SonarY;
        }
        private Swath[] _swaths;
        private int     _swathWrite;

        private static Gradient DefaultRamp()
        {
            var g = new Gradient();
            g.SetKeys(new GradientColorKey[]
            {
                new GradientColorKey(new Color(0.10f, 0.05f, 0.35f), 0.00f),
                new GradientColorKey(new Color(0.00f, 0.35f, 0.70f), 0.25f),
                new GradientColorKey(new Color(0.00f, 0.75f, 0.65f), 0.50f),
                new GradientColorKey(new Color(0.95f, 0.85f, 0.20f), 0.75f),
                new GradientColorKey(new Color(0.95f, 0.30f, 0.10f), 1.00f),
            },
            new GradientAlphaKey[] { new GradientAlphaKey(1,0), new GradientAlphaKey(1,1) });
            return g;
        }

        private void Awake()
        {
            _image = GetComponent<RawImage>();
            BuildTexture();
            BuildSwathBuffer();
        }

        private void BuildTexture()
        {
            _panelTex = new Texture2D(_panelWidth, _panelHeight, TextureFormat.RGBA32, false);
            _panelTex.filterMode = FilterMode.Point;
            _panelTex.wrapMode = TextureWrapMode.Clamp;
            _panelPixels = new Color[_panelWidth * _panelHeight];
            ClearPixels();
            _image.texture = _panelTex;
        }

        private void BuildSwathBuffer()
        {
            _swaths = new Swath[_swathHistory];
            for (int i = 0; i < _swathHistory; i++)
            {
                int N = _sonar ? _sonar.BeamCount : 1024;
                _swaths[i] = new Swath
                {
                    Depth     = new float[N],
                    Intensity = new float[N],
                    Hit       = new byte[N],
                    Age       = 1e6f,
                    SonarY    = 0
                };
            }
            _swathWrite = 0;
        }

        private void ClearPixels()
        {
            for (int i = 0; i < _panelPixels.Length; i++) _panelPixels[i] = new Color(0.01f, 0.01f, 0.04f, 1);
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
            PushSwath(data);
        }

        private void PushSwath(MultibeamSonar.PingData[] data)
        {
            var sw = _swaths[_swathWrite];
            int N = Mathf.Min(data.Length, sw.Depth.Length);
            for (int i = 0; i < N; i++)
            {
                sw.Depth[i]     = data[i].Depth;
                sw.Intensity[i] = data[i].IntensityDb;
                sw.Hit[i]       = data[i].Hit ? (byte)1 : (byte)0;
            }
            sw.Age    = 0;
            sw.SonarY = _sonar ? _sonar.transform.position.y : 0;
            _swathWrite = (_swathWrite + 1) % _swathHistory;
        }

        private void Update()
        {
            float dt = Time.deltaTime;
            for (int i = 0; i < _swathHistory; i++) _swaths[i].Age += dt;

            RenderBathymetry();
            UpdateLabels();
        }

        private void RenderBathymetry()
        {
            ClearPixels();

            int W = _panelWidth, H = _panelHeight;
            int N = _sonar ? Mathf.Min(_sonar.BeamCount, W) : W;
            float dRange = Mathf.Max(1, _maxDepth - _minDepth);

            for (int sIdx = 0; sIdx < _swathHistory; sIdx++)
            {
                int s = (_swathWrite + _swathHistory - 1 - sIdx + _swathHistory * 10) % _swathHistory;
                var sw = _swaths[s];
                if (sw.Age > 1e5f) continue;

                float ageT = Mathf.Clamp01(sw.Age / Mathf.Max(0.5f, _swathHistory * 0.12f));
                float alpha = Mathf.Pow(1f - ageT, 0.6f);
                if (alpha < 0.03f) continue;

                float rowFrac = sIdx / (float)_swathHistory;
                int rowCenter = Mathf.RoundToInt(rowFrac * (H - 1));

                for (int b = 0; b < N; b++)
                {
                    float u = b / (float)(N - 1);
                    int px = Mathf.Clamp(Mathf.RoundToInt(u * (W - 1)), 0, W - 1);
                    float depth = sw.Depth[b];
                    float inten = sw.Intensity[b];
                    byte  hit   = sw.Hit[b];

                    float depthT = Mathf.Clamp01((depth - _minDepth) / dRange);
                    Color c;
                    if (hit == 0)
                    {
                        c = new Color(0.01f, 0.01f, 0.04f, 1);
                    }
                    else
                    {
                        c = _colormap.Evaluate(depthT);
                        float intT = Mathf.Clamp01((inten + 80f) / 70f);
                        c.r = Mathf.Lerp(0.02f, c.r, intT);
                        c.g = Mathf.Lerp(0.02f, c.g, intT);
                        c.b = Mathf.Lerp(0.04f, c.b, intT);
                        c.a = Mathf.Clamp01(intT * alpha);
                    }

                    int stripH = Mathf.Max(1, Mathf.RoundToInt(H / (float)_swathHistory * 0.9f));
                    for (int dh = -stripH / 2; dh <= stripH / 2; dh++)
                    {
                        int py = Mathf.Clamp(rowCenter + dh, 0, H - 1);
                        int idx = py * W + px;
                        Color prev = _panelPixels[idx];
                        float aw = c.a;
                        _panelPixels[idx] = new Color(
                            prev.r * (1 - aw) + c.r * aw,
                            prev.g * (1 - aw) + c.g * aw,
                            prev.b * (1 - aw) + c.b * aw,
                            1);
                    }
                }
            }

            DrawGridLines();
            DrawAUVTrack();

            _panelTex.SetPixels(_panelPixels);
            _panelTex.Apply(false);
        }

        private void DrawGridLines()
        {
            int W = _panelWidth, H = _panelHeight;
            Color grid = new Color(0, 0.45f, 0.55f, 0.4f);
            for (int i = 1; i < 10; i++)
            {
                int x = i * (W / 10);
                for (int y = 0; y < H; y += 2) _panelPixels[y * W + x] = Color.Lerp(_panelPixels[y * W + x], grid, 0.6f);
                int y2 = i * (H / 10);
                for (int x2 = 0; x2 < W; x2 += 2) _panelPixels[y2 * W + x2] = Color.Lerp(_panelPixels[y2 * W + x2], grid, 0.6f);
            }
        }

        private Vector3 _lastTrackPos;
        private void DrawAUVTrack()
        {
            if (!_auv || !_terrain) return;
            int W = _panelWidth, H = _panelHeight;
            Vector3 p = _auv.transform.position;
            Vector2 o = _terrain.WorldOrigin;
            float u = Mathf.Clamp01((p.x - o.x) / _terrain.SizeX);
            float v = Mathf.Clamp01((p.z - o.y) / _terrain.SizeZ);
            int px = Mathf.RoundToInt(u * (W - 1));
            int py = Mathf.RoundToInt(v * (H - 1));
            px = Mathf.Clamp(px, 2, W - 3);
            py = Mathf.Clamp(py, 2, H - 3);
            for (int dy = -2; dy <= 2; dy++)
                for (int dx = -2; dx <= 2; dx++)
                {
                    if (Mathf.Abs(dx) + Mathf.Abs(dy) > 3) continue;
                    _panelPixels[(py + dy) * W + (px + dx)] = new Color(1f, 0.3f, 0.5f, 1);
                }
        }

        private void UpdateLabels()
        {
            if (_depthLabel && _sonar)
                _depthLabel.text = $"Depth: {_sonar.MeanDepth,6:F1} m";
            if (_hitsLabel && _sonar)
                _hitsLabel.text  = $"Hits:  {_sonar.HitsLastPing}/{_sonar.BeamCount}";
            if (_posLabel && _auv)
                _posLabel.text   = $"Pos: {_auv.transform.position.XZ():F0} | Alt: {_auv.transform.position.y:F1}";
            if (_pressureLabel && _auv)
                _pressureLabel.text = $"Pressure: {_auv.DebugPressureKPa:F1} kPa";
            if (_statusLabel && _sonar)
                _statusLabel.text   = $"Sonar: {(_sonar.enabled ? "ACTIVE" : "PAUSED")}  |  Mean Intensity: {_sonar.MeanIntensityDb:F1} dB";
        }
    }

    internal static class PanelExt
    {
        public static string XZ(this Vector3 v) => $"({v.x:F0}, {v.z:F0})";
    }
}
