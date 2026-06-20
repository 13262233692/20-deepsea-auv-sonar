using UnityEngine;
using DeepseaAUV.Native;
using System.Runtime.InteropServices;

namespace DeepseaAUV
{
    [DefaultExecutionOrder(10)]
    public class MultibeamSonar : MonoBehaviour
    {
        public enum EmitMode { FixedRate, OnDemand }

        [Header("Hardware Config")]
        [SerializeField] private uint    _numBeams         = 1024;
        [SerializeField] private float   _maxRange         = 150f;
        [SerializeField] private float   _minRange         = 0.5f;
        [SerializeField] private float   _swathAngleDeg    = 120f;
        [SerializeField] private float   _beamWidthDeg     = 0.5f;
        [SerializeField] private float   _soundVelocity    = 1500f;
        [SerializeField] private float   _frequencyKhz     = 300f;
        [SerializeField] private float   _absorptionDbPM   = 0.08f;
        [SerializeField] private float   _noiseFloorDb     = -80f;
        [SerializeField] private float   _targetStrengthDb = -25f;

        [Header("Emission")]
        [SerializeField] private EmitMode _mode = EmitMode.FixedRate;
        [SerializeField] private float   _pingRateHz       = 8f;
        [SerializeField] private bool    _useGPU           = true;

        [Header("References")]
        [SerializeField] private Transform _auvTransform;
        [SerializeField] private SeafloorTerrain _terrain;
        [SerializeField] private ComputeShader _raycastCS;

        [Header("Runtime Diagnostics")]
        public  int     HitsLastPing;
        public  float   MeanDepth;
        public  float   MeanIntensityDb;

        public struct PingData
        {
            public Vector3 WorldPos;
            public float   Depth;
            public float   IntensityDb;
            public float   Range;
            public uint    BeamId;
            public bool    Hit;
        }

        private PingData[]   _lastPingData;
        public  PingData[]   LastPingData => _lastPingData;
        public  int          BeamCount    => (int)_numBeams;

        // GPU resources
        private ComputeBuffer _cbDirections;
        private ComputeBuffer _cbOrigins;
        private ComputeBuffer _cbResults;
        private ComputeBuffer _cbHeights;
        private int           _kernelMain;
        private Vector3[]     _dirCache;

        private float         _pingAccumulator;
        private bool          _nativeInit;
        private NativeVec3[]  _nativeDirs;

        private void OnEnable()
        {
            InitNative();
            InitGPU();
        }

        private void OnDisable()
        {
            ReleaseGPU();
        }

        private void InitNative()
        {
            var cfg = new SonarConfigNative
            {
                numBeams        = _numBeams,
                numPings        = 1,
                maxRange        = _maxRange,
                minRange        = _minRange,
                beamWidthRad    = _beamWidthDeg * Mathf.Deg2Rad,
                swathAngleRad   = _swathAngleDeg * Mathf.Deg2Rad,
                soundVelocity   = _soundVelocity,
                frequency       = _frequencyKhz * 1000f,
                absorptionCoeff = _absorptionDbPM,
                noiseFloorDb    = _noiseFloorDb
            };
            HydrodynamicsBridge.Sonar_Init(ref cfg);
            _nativeDirs = new NativeVec3[_numBeams];
            _lastPingData = new PingData[_numBeams];
            _dirCache = new Vector3[_numBeams];
            _nativeInit = true;
        }

        private void InitGPU()
        {
            if (!_useGPU || _raycastCS == null || _terrain == null) return;
            _kernelMain = _raycastCS.FindKernel("CSMain");
            int strideDir = Marshal.SizeOf(typeof(Vector3));
            int strideRes = 16 + 4 + 4 + 4 + 1; // 16 pos + 4 depth + 4 intensity + 4 range + 1 hit
            strideRes = Mathf.CeilToInt(strideRes / 16f) * 16; // align

            _cbDirections = new ComputeBuffer((int)_numBeams, strideDir);
            _cbOrigins    = new ComputeBuffer((int)_numBeams, strideDir);
            _cbResults    = new ComputeBuffer((int)_numBeams, 32); // Vector3 + 5 floats = 32 bytes

            int res = _terrain.Resolution;
            _cbHeights = new ComputeBuffer(res * res, sizeof(float));
            UploadHeightsToGPU();

            _raycastCS.SetBuffer(_kernelMain, "_Directions", _cbDirections);
            _raycastCS.SetBuffer(_kernelMain, "_Origins",    _cbOrigins);
            _raycastCS.SetBuffer(_kernelMain, "_Results",    _cbResults);
            _raycastCS.SetBuffer(_kernelMain, "_Heightmap",  _cbHeights);
            _raycastCS.SetInt("_Resolution",    res);
            _raycastCS.SetFloat("_SizeX",       _terrain.SizeX);
            _raycastCS.SetFloat("_SizeZ",       _terrain.SizeZ);
            _raycastCS.SetFloat("_BaseDepth",   _terrain.BaseDepth);
            _raycastCS.SetVector("_Origin", new Vector4(_terrain.WorldOrigin.x, 0, _terrain.WorldOrigin.y, 0));
            _raycastCS.SetFloat("_MaxRange",    _maxRange);
            _raycastCS.SetFloat("_MinRange",    _minRange);
        }

        private void UploadHeightsToGPU()
        {
            if (_terrain == null) return;
            int res = _terrain.Resolution;
            var arr = new float[res * res];
            for (int z = 0; z < res; z++)
                for (int x = 0; x < res; x++)
                    arr[z * res + x] = _terrain.SampleHeightWorld(
                        _terrain.WorldOrigin.x + _terrain.SizeX * (x / (float)(res - 1)),
                        _terrain.WorldOrigin.y + _terrain.SizeZ * (z / (float)(res - 1)));
            _cbHeights?.SetData(arr);
        }

        private void ReleaseGPU()
        {
            _cbDirections?.Release();
            _cbOrigins?.Release();
            _cbResults?.Release();
            _cbHeights?.Release();
            _cbDirections = _cbOrigins = _cbResults = _cbHeights = null;
        }

        private void LateUpdate()
        {
            if (!_nativeInit) return;

            if (_mode == EmitMode.FixedRate)
            {
                _pingAccumulator += Time.deltaTime;
                float interval = Mathf.Max(0.001f, 1f / Mathf.Max(0.1f, _pingRateHz));
                while (_pingAccumulator >= interval)
                {
                    _pingAccumulator -= interval;
                    FirePing();
                }
            }
        }

        public void RequestPing()
        {
            if (_nativeInit) FirePing();
        }

        private void FirePing()
        {
            Transform t = _auvTransform ? _auvTransform : transform;
            var pos  = NativeVec3.FromUnity(t.position);
            var rot  = NativeQuat.FromUnity(t.rotation);
            HydrodynamicsBridge.Sonar_SetPose(ref pos, ref rot);
            HydrodynamicsBridge.Sonar_GeneratePingDirections(_nativeDirs, _numBeams);

            for (int i = 0; i < _numBeams; i++) _dirCache[i] = _nativeDirs[i].ToUnity();

            if (_useGPU && _raycastCS != null && _cbResults != null)
            {
                DispatchGPU(t.position);
                ReadbackGPUResults(t.position);
            }
            else
            {
                RaycastCPU(t.position);
            }

            ComputeStats();
            PingFired?.Invoke(_lastPingData, HitsLastPing);
        }

        private void DispatchGPU(Vector3 origin)
        {
            for (int i = 0; i < _numBeams; i++) _dirCache[i] = _dirCache[i].normalized;
            _cbDirections.SetData(_dirCache);
            var origins = new Vector3[_numBeams];
            for (int i = 0; i < _numBeams; i++) origins[i] = origin;
            _cbOrigins.SetData(origins);

            uint tx, ty, tz;
            _raycastCS.GetKernelThreadGroupSizes(_kernelMain, out tx, out ty, out tz);
            int groups = Mathf.CeilToInt(_numBeams / (float)tx);
            _raycastCS.Dispatch(_kernelMain, groups, 1, 1);
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct GPUResult
        {
            public Vector3 Pos;
            public float   Depth;
            public float   Intensity;
            public float   Range;
            public uint    BeamId;
            public uint    Hit;
            public Vector2 _pad;
        }

        private void ReadbackGPUResults(Vector3 origin)
        {
            var results = new GPUResult[_numBeams];
            _cbResults.GetData(results);

            for (int i = 0; i < _numBeams; i++)
            {
                ref var d = ref results[i];
                float intensityDb = d.Intensity;
                float incidentCos = Mathf.Max(0.05f, Vector3.Dot(-_dirCache[i].normalized, Vector3.up));
                intensityDb = HydrodynamicsBridge.Sonar_ComputeIntensity(d.Range, incidentCos, _targetStrengthDb);

                _lastPingData[i] = new PingData
                {
                    WorldPos    = d.Pos,
                    Depth       = d.Depth,
                    IntensityDb = intensityDb,
                    Range       = d.Range,
                    BeamId      = (uint)i,
                    Hit         = d.Hit != 0
                };
            }
        }

        private void RaycastCPU(Vector3 origin)
        {
            var hits = new RaycastHit[1];
            int hitsTotal = 0;
            for (int i = 0; i < _numBeams; i++)
            {
                Vector3 dir = _dirCache[i];
                bool got = Physics.RaycastNonAlloc(origin, dir, hits, _maxRange, ~0, QueryTriggerInteraction.Ignore) > 0;
                RaycastHit h = hits[0];
                float range   = got ? h.distance : _maxRange;
                float depth   = got ? origin.y - h.point.y : 0f;
                float incident = got ? Mathf.Max(0.01f, Vector3.Dot(-dir.normalized, h.normal)) : 0.01f;
                float inten = HydrodynamicsBridge.Sonar_ComputeIntensity(range, incident, _targetStrengthDb);
                _lastPingData[i] = new PingData
                {
                    WorldPos    = got ? h.point : origin + dir * _maxRange,
                    Depth       = depth,
                    IntensityDb = inten,
                    Range       = range,
                    BeamId      = (uint)i,
                    Hit         = got
                };
                if (got) hitsTotal++;
            }
            HitsLastPing = hitsTotal;
        }

        private void ComputeStats()
        {
            int h = 0;
            float d = 0, it = 0;
            for (int i = 0; i < _lastPingData.Length; i++)
            {
                if (_lastPingData[i].Hit)
                {
                    h++;
                    d  += _lastPingData[i].Depth;
                    it += _lastPingData[i].IntensityDb;
                }
            }
            HitsLastPing    = h;
            MeanDepth       = h > 0 ? d / h : 0f;
            MeanIntensityDb = h > 0 ? it / h : _noiseFloorDb;
        }

        public event System.Action<PingData[], int> PingFired;

        private void OnDrawGizmosSelected()
        {
            if (_dirCache == null) return;
            Transform t = _auvTransform ? _auvTransform : transform;
            Gizmos.color = Color.cyan;
            int step = Mathf.Max(1, (int)_numBeams / 64);
            for (int i = 0; i < _numBeams; i += step)
                Gizmos.DrawRay(t.position, _dirCache[i] * _maxRange * 0.1f);
        }
    }
}
