using UnityEngine;
using UnityEngine.UI;

namespace DeepseaAUV
{
    /// <summary>
    /// Runtime scene bootstrap. Drops this component on an empty GameObject
    /// in an empty Unity scene, hit Play, and the entire AUV simulation +
    /// sonar pipeline + UI is constructed procedurally.
    /// </summary>
    [DefaultExecutionOrder(-1000)]
    public class SceneBootstrap : MonoBehaviour
    {
        [Header("Terrain")]
        [SerializeField] private int   _terrainResolution = 512;
        [SerializeField] private float _terrainSize = 1000f;
        [SerializeField] private float _baseDepth = 80f;
        [SerializeField] private int   _terrainSeed = 20241109;

        [Header("Sonar")]
        [SerializeField] private uint  _numBeams = 1024;
        [SerializeField] private float _pingRate = 10f;
        [SerializeField] private float _maxRange = 200f;
        [SerializeField] private float _swathAngleDeg = 120f;
        [SerializeField] private bool  _useGPU = true;

        [Header("Environment")]
        [SerializeField] private Color _ambientColor = new Color(0.015f, 0.025f, 0.05f, 1);
        [SerializeField] private Color _fogColor     = new Color(0.005f, 0.010f, 0.020f, 1);
        [SerializeField] private float _fogStart = 20f;
        [SerializeField] private float _fogEnd   = 250f;

        public static SceneBootstrap Instance { get; private set; }

        public AUVController    AUV     { get; private set; }
        public MultibeamSonar   Sonar   { get; private set; }
        public SeafloorTerrain  Terrain { get; private set; }
        public SonarPointCloud  PointCloud { get; private set; }
        public Camera           MainCam { get; private set; }

        private void Awake()
        {
            Instance = this;
            SetupRenderSettings();
            BuildTerrain();
            BuildAUV();
            BuildSonar();
            BuildAI();
            BuildPointCloud();
            BuildCameraAndLights();
            BuildCanvasUI();
        }

        private void SetupRenderSettings()
        {
            RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
            RenderSettings.ambientLight = _ambientColor;
            RenderSettings.ambientEquatorColor = _ambientColor * 0.6f;
            RenderSettings.ambientSkyColor = new Color(0.005f, 0.008f, 0.02f, 1);
            RenderSettings.ambientGroundColor = new Color(0.01f, 0.005f, 0.003f, 1);
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.Linear;
            RenderSettings.fogColor = _fogColor;
            RenderSettings.fogStartDistance = _fogStart;
            RenderSettings.fogEndDistance = _fogEnd;
            RenderSettings.reflectionIntensity = 0.02f;
            QualitySettings.vSyncCount = 1;
            QualitySettings.shadows = ShadowQuality.HardOnly;
            QualitySettings.shadowDistance = 150f;
        }

        private void BuildTerrain()
        {
            var go = new GameObject("SeafloorTerrain");
            go.transform.SetParent(transform);
            go.layer = LayerMask.NameToLayer("Default");
            Terrain = go.AddComponent<SeafloorTerrain>();

            // Inject via reflection-like public fields manually (since fields are serialized)
            SetPrivate(Terrain, "_resolution", _terrainResolution);
            SetPrivate(Terrain, "_sizeX", _terrainSize);
            SetPrivate(Terrain, "_sizeZ", _terrainSize);
            SetPrivate(Terrain, "_baseDepth", _baseDepth);
            SetPrivate(Terrain, "_seed", _terrainSeed);
            Terrain.Generate();

            // Assign seafloor shader material
            var shader = Shader.Find("DeepseaAUV/SeafloorTerrain");
            if (shader == null) shader = Shader.Find("Standard");
            var mat = new Material(shader) { name = "SeafloorMat" };
            if (shader.name.Contains("Seafloor"))
            {
                mat.SetColor("_BaseColor",  new Color(0.05f, 0.07f, 0.10f, 1));
                mat.SetColor("_SlopeColor", new Color(0.08f, 0.045f, 0.03f, 1));
                mat.SetColor("_DepthRim",   new Color(0.02f, 0.25f, 0.45f, 1));
                mat.SetFloat("_Glossiness", 0.05f);
                mat.SetFloat("_Metallic", 0.0f);
                mat.SetFloat("_NoiseScale", 4.0f);
                mat.SetFloat("_NoiseAmount", 0.4f);
            }
            else
            {
                mat.color = new Color(0.03f, 0.05f, 0.08f, 1);
            }
            go.GetComponent<MeshRenderer>().sharedMaterial = mat;
        }

        private void BuildAUV()
        {
            var auvGO = new GameObject("AUV");
            auvGO.transform.SetParent(transform);
            auvGO.transform.SetPositionAndRotation(new Vector3(0, -_baseDepth + 20, 0), Quaternion.identity);

            BuildAUVVisual(auvGO.transform);

            AUV = auvGO.AddComponent<AUVController>();
            auvGO.AddComponent<AUVKeyboardInput>();
        }

        private AI.AUVAutopilot    Autopilot       { get; set; }
        private AI.ObstacleMapper  ObstacleMapper  { get; set; }

        private void BuildAI()
        {
            // ObstacleMapper (child of AUV)
            var mapperGO = new GameObject("ObstacleMapper");
            mapperGO.transform.SetParent(AUV.transform, false);
            mapperGO.transform.localPosition = Vector3.zero;
            var mapper = mapperGO.AddComponent<AI.ObstacleMapper>();
            SetPrivate(mapper, "_gridSize", 96);
            SetPrivate(mapper, "_cellSize", 1.6f);
            SetPrivate(mapper, "_dangerThreshold", 2.8f);
            SetPrivate(mapper, "_sonar", Sonar);
            SetPrivate(mapper, "_auv", AUV.transform);
            ObstacleMapper = mapper;

            // Autopilot (on AUV root)
            var autopilot = AUV.gameObject.AddComponent<AI.AUVAutopilot>();
            SetPrivate(autopilot, "_mapper", mapper);
            SetPrivate(autopilot, "_mode", AI.AUVMode.AutoCruise);
            SetPrivate(autopilot, "_cruiseSpeed", 2.2f);
            SetPrivate(autopilot, "_lookahead", 9f);
            SetPrivate(autopilot, "_maxRRTIterations", 1200);
            SetPrivate(autopilot, "_rrtTimeBudgetMs", 350f);
            SetPrivate(autopilot, "_safetyMargin", 3.5f);
            SetPrivate(autopilot, "_replanInterval", 1.5f);
            SetPrivate(autopilot, "_drawDebug", true);
            Autopilot = autopilot;

            // Path visualizer
            AUV.gameObject.AddComponent<PathVisualizer>();
        }

        private static void BuildAUVVisual(Transform parent)
        {
            var hull = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            hull.name = "Hull";
            hull.transform.SetParent(parent, false);
            hull.transform.localPosition = Vector3.zero;
            hull.transform.localRotation = Quaternion.Euler(0, 0, 90);
            hull.transform.localScale = new Vector3(0.5f, 3.0f, 0.5f);
            Destroy(hull.GetComponent<CapsuleCollider>());
            var hm = hull.GetComponent<MeshRenderer>();
            var hullMat = new Material(Shader.Find("Standard"));
            hullMat.color = new Color(0.85f, 0.65f, 0.15f, 1);
            hullMat.SetFloat("_Metallic", 0.7f);
            hullMat.SetFloat("_Glossiness", 0.4f);
            hm.sharedMaterial = hullMat;

            var nose = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            nose.name = "NoseFairing";
            nose.transform.SetParent(parent, false);
            nose.transform.localPosition = new Vector3(0, 0, 3.1f);
            nose.transform.localScale = new Vector3(0.9f, 0.9f, 0.9f);
            Destroy(nose.GetComponent<SphereCollider>());
            nose.GetComponent<MeshRenderer>().sharedMaterial = hullMat;

            for (int i = 0; i < 4; i++)
            {
                var fin = GameObject.CreatePrimitive(PrimitiveType.Cube);
                fin.name = $"Fin_{i}";
                fin.transform.SetParent(parent, false);
                Destroy(fin.GetComponent<BoxCollider>());
                var fm = new Material(Shader.Find("Standard"));
                fm.color = new Color(0.2f, 0.2f, 0.25f, 1);
                fm.SetFloat("_Metallic", 0.5f);
                fin.GetComponent<MeshRenderer>().sharedMaterial = fm;
                if (i == 0) { fin.transform.localPosition = new Vector3(0, 0.55f, -2.4f); fin.transform.localScale = new Vector3(1.2f, 0.9f, 0.08f); }
                if (i == 1) { fin.transform.localPosition = new Vector3(0, -0.55f, -2.4f); fin.transform.localScale = new Vector3(1.2f, 0.9f, 0.08f); }
                if (i == 2) { fin.transform.localPosition = new Vector3(0.55f, 0, -2.4f); fin.transform.localScale = new Vector3(0.08f, 1.0f, 1.1f); }
                if (i == 3) { fin.transform.localPosition = new Vector3(-0.55f, 0, -2.4f); fin.transform.localScale = new Vector3(0.08f, 1.0f, 1.1f); }
            }

            var prop = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            prop.name = "Propeller";
            prop.transform.SetParent(parent, false);
            prop.transform.localPosition = new Vector3(0, 0, -3.3f);
            prop.transform.localScale = new Vector3(0.6f, 0.08f, 0.6f);
            Destroy(prop.GetComponent<CapsuleCollider>());
            prop.GetComponent<MeshRenderer>().sharedMaterial = hullMat;

            var sonarDome = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sonarDome.name = "SonarDome";
            sonarDome.transform.SetParent(parent, false);
            sonarDome.transform.localPosition = new Vector3(0, -0.4f, 1.8f);
            sonarDome.transform.localScale = new Vector3(0.7f, 0.25f, 0.9f);
            Destroy(sonarDome.GetComponent<SphereCollider>());
            var sm = new Material(Shader.Find("Standard"));
            sm.color = new Color(0.05f, 0.15f, 0.25f, 1);
            sm.SetFloat("_Metallic", 0.9f);
            sm.SetFloat("_Glossiness", 0.8f);
            sonarDome.GetComponent<MeshRenderer>().sharedMaterial = sm;
        }

        private void BuildSonar()
        {
            var sonarGO = new GameObject("MultibeamSonar");
            sonarGO.transform.SetParent(AUV.transform, false);
            sonarGO.transform.localPosition = new Vector3(0, -0.4f, 1.8f);
            sonarGO.transform.localRotation = Quaternion.Euler(-15, 0, 0);

            Sonar = sonarGO.AddComponent<MultibeamSonar>();
            SetPrivate(Sonar, "_auvTransform", AUV.transform);
            SetPrivate(Sonar, "_terrain", Terrain);
            SetPrivate(Sonar, "_numBeams", _numBeams);
            SetPrivate(Sonar, "_maxRange", _maxRange);
            SetPrivate(Sonar, "_swathAngleDeg", _swathAngleDeg);
            SetPrivate(Sonar, "_pingRateHz", _pingRate);
            SetPrivate(Sonar, "_useGPU", _useGPU);

            var cs = Resources.Load<ComputeShader>("MultibeamSonarRaycast");
            if (cs != null) SetPrivate(Sonar, "_raycastCS", cs);
        }

        private void BuildPointCloud()
        {
            var pgo = new GameObject("SonarPointCloud");
            pgo.transform.SetParent(transform);
            pgo.AddComponent<MeshFilter>();
            var mr = pgo.AddComponent<MeshRenderer>();
            PointCloud = pgo.AddComponent<SonarPointCloud>();
            SetPrivate(PointCloud, "_sonar", Sonar);
            SetPrivate(PointCloud, "_maxPointsPerStripe", (int)_numBeams);
            SetPrivate(PointCloud, "_stripeHistory", 64);
            SetPrivate(PointCloud, "_pointSize", 0.3f);

            var shader = Shader.Find("DeepseaAUV/BathymetryPointCloud");
            if (shader != null) SetPrivate(PointCloud, "_pointShader", shader);
            mr.allowOcclusionWhenDynamic = false;
        }

        private void BuildCameraAndLights()
        {
            var camGO = new GameObject("MainCamera");
            camGO.tag = "MainCamera";
            camGO.transform.SetParent(AUV.transform, false);
            camGO.transform.localPosition = new Vector3(0, 3.5f, -9f);
            camGO.transform.localRotation = Quaternion.Euler(10, 0, 0);
            MainCam = camGO.AddComponent<Camera>();
            MainCam.clearFlags = CameraClearFlags.SolidColor;
            MainCam.backgroundColor = _fogColor;
            MainCam.nearClipPlane = 0.1f;
            MainCam.farClipPlane = 1500f;
            MainCam.fieldOfView = 60f;
            camGO.AddComponent<AudioListener>();
            var follow = camGO.AddComponent<CameraFollow>();
            follow.target = AUV.transform;

            // Subsea ambient light
            var ambient = new GameObject("AmbientLight");
            ambient.transform.SetParent(transform, false);
            var al = ambient.AddComponent<Light>();
            al.type = LightType.Directional;
            al.color = new Color(0.25f, 0.35f, 0.6f, 1);
            al.intensity = 0.35f;
            al.shadows = LightShadows.Hard;
            al.transform.rotation = Quaternion.Euler(65, 35, 0);

            // AUV headlight (conical)
            var hlGO = new GameObject("Headlight");
            hlGO.transform.SetParent(AUV.transform, false);
            hlGO.transform.localPosition = new Vector3(0, 0, 3.2f);
            hlGO.transform.localRotation = Quaternion.Euler(0, 0, 0);
            var hl = hlGO.AddComponent<Light>();
            hl.type = LightType.Spot;
            hl.color = new Color(0.6f, 0.75f, 1f, 1);
            hl.intensity = 3.5f;
            hl.range = 120f;
            hl.spotAngle = 55f;
            hl.shadows = LightShadows.Hard;
        }

        private void BuildCanvasUI()
        {
            var canvasGO = new GameObject("HUDCanvas");
            canvasGO.transform.SetParent(transform, false);
            var canvas = canvasGO.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder = 100;
            var scaler = canvasGO.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.matchWidthOrHeight = 0.5f;
            canvasGO.AddComponent<GraphicRaycaster>();

            // Title header
            var titleGO = new GameObject("Title");
            titleGO.transform.SetParent(canvasGO.transform, false);
            var title = titleGO.AddComponent<Text>();
            title.text = "DEEPSEA AUV :: Multibeam Sonar Simulator";
            title.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            title.fontSize = 20;
            title.alignment = TextAnchor.MiddleCenter;
            title.color = new Color(0.4f, 0.9f, 1f, 1);
            var tr = title.GetComponent<RectTransform>();
            tr.anchorMin = new Vector2(0, 1);
            tr.anchorMax = new Vector2(1, 1);
            tr.pivot = new Vector2(0.5f, 1);
            tr.anchoredPosition = new Vector2(0, -6);
            tr.sizeDelta = new Vector2(0, 32);
            var outline = titleGO.AddComponent<Shadow>();
            outline.effectColor = Color.black;

            // Control hint
            var hintGO = new GameObject("Controls");
            hintGO.transform.SetParent(canvasGO.transform, false);
            var hint = hintGO.AddComponent<Text>();
            hint.text = "W/S:Surge  A/D:Sway  Q/E:Heave  Mouse:Pitch/Yaw  Keypad4/6:Roll  Shift:Boost  R:Reset";
            hint.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            hint.fontSize = 12;
            hint.alignment = TextAnchor.MiddleLeft;
            hint.color = new Color(0.7f, 0.9f, 1f, 0.85f);
            var hr = hint.GetComponent<RectTransform>();
            hr.anchorMin = new Vector2(0, 0);
            hr.anchorMax = new Vector2(1, 0);
            hr.pivot = new Vector2(0.5f, 0);
            hr.anchoredPosition = new Vector2(0, 6);
            hr.sizeDelta = new Vector2(0, 20);

            // Bathymetry panel
            var panelGO = new GameObject("BathymetryPanel");
            panelGO.transform.SetParent(canvasGO.transform, false);
            var ri = panelGO.AddComponent<RawImage>();
            ri.color = Color.white;
            var pr = panelGO.GetComponent<RectTransform>();
            pr.anchorMin = new Vector2(1, 1);
            pr.anchorMax = new Vector2(1, 1);
            pr.pivot = new Vector2(1, 1);
            pr.anchoredPosition = new Vector2(-16, -56);
            pr.sizeDelta = new Vector2(640, 320);

            var panel = panelGO.AddComponent<UI.BathymetryPanel>();
            SetPrivate(panel, "_sonar", Sonar);
            SetPrivate(panel, "_auv", AUV);
            SetPrivate(panel, "_terrain", Terrain);
            SetPrivate(panel, "_panelWidth", 1024);
            SetPrivate(panel, "_panelHeight", 512);

            // Info labels (top-left)
            string[] names = { "DepthLabel", "HitsLabel", "PosLabel", "PressureLabel", "StatusLabel" };
            Vector2[] positions = {
                new Vector2(16, -56), new Vector2(16, -80), new Vector2(16, -104),
                new Vector2(16, -128), new Vector2(16, -152)
            };
            Text[] labels = new Text[names.Length];
            for (int i = 0; i < names.Length; i++)
            {
                var go = new GameObject(names[i]);
                go.transform.SetParent(canvasGO.transform, false);
                var t = go.AddComponent<Text>();
                t.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
                t.fontSize = 14;
                t.alignment = TextAnchor.MiddleLeft;
                t.color = new Color(0.55f, 0.95f, 1f, 1);
                t.text = "---";
                var r = t.GetComponent<RectTransform>();
                r.anchorMin = new Vector2(0, 1);
                r.anchorMax = new Vector2(0, 1);
                r.pivot = new Vector2(0, 1);
                r.anchoredPosition = positions[i];
                r.sizeDelta = new Vector2(520, 20);
                labels[i] = t;
            }
            SetPrivate(panel, "_depthLabel", labels[0]);
            SetPrivate(panel, "_hitsLabel", labels[1]);
            SetPrivate(panel, "_posLabel", labels[2]);
            SetPrivate(panel, "_pressureLabel", labels[3]);
            SetPrivate(panel, "_statusLabel", labels[4]);

            // ---- AI Status Panel ----
            var aiGO = new GameObject("AIStatusPanel");
            aiGO.transform.SetParent(canvasGO.transform, false);
            var aiPanel = aiGO.AddComponent<UI.AIStatusPanel>();

            // AI labels (below the main info labels)
            string[] aiNames = { "AIModeLabel", "AIPlanLabel", "AIClusterLabel", "AISpeedLabel", "AIHintLabel" };
            Vector2[] aiPositions = {
                new Vector2(16, -180), new Vector2(16, -204), new Vector2(16, -228),
                new Vector2(16, -252), new Vector2(16, -280)
            };
            Text[] aiLabels = new Text[aiNames.Length];
            for (int i = 0; i < aiNames.Length; i++)
            {
                var go = new GameObject(aiNames[i]);
                go.transform.SetParent(aiGO.transform, false);
                var t = go.AddComponent<Text>();
                t.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
                t.fontSize = i == 4 ? 11 : 13;
                t.alignment = TextAnchor.MiddleLeft;
                t.color = i == 0 ? new Color(0.3f, 1f, 0.6f, 1f) : new Color(0.55f, 0.85f, 1f, 1f);
                if (i == 4) t.color = new Color(0.6f, 0.75f, 0.9f, 0.8f);
                t.text = "AI: ---";
                var r = t.GetComponent<RectTransform>();
                r.anchorMin = new Vector2(0, 1);
                r.anchorMax = new Vector2(0, 1);
                r.pivot = new Vector2(0, 1);
                r.anchoredPosition = aiPositions[i];
                r.sizeDelta = new Vector2(520, 22);
                aiLabels[i] = t;
            }
            SetPrivate(aiPanel, "_modeLabel",    aiLabels[0]);
            SetPrivate(aiPanel, "_planLabel",    aiLabels[1]);
            SetPrivate(aiPanel, "_clusterLabel", aiLabels[2]);
            SetPrivate(aiPanel, "_speedLabel",   aiLabels[3]);
            SetPrivate(aiPanel, "_hintLabel",    aiLabels[4]);

            if (Autopilot != null && ObstacleMapper != null)
            {
                aiPanel.Setup(Autopilot, ObstacleMapper);
            }
        }

        private static void SetPrivate<T>(object target, string fieldName, T value)
        {
            var f = target.GetType().GetField(fieldName,
                System.Reflection.BindingFlags.Instance |
                System.Reflection.BindingFlags.NonPublic);
            if (f == null)
            {
                Debug.LogWarning($"[SceneBootstrap] Field '{fieldName}' not found on {target.GetType().Name}");
                return;
            }
            f.SetValue(target, value);
        }
    }

    public class CameraFollow : MonoBehaviour
    {
        public Transform target;
        public Vector3 localOffset = new Vector3(0, 3.5f, -9f);
        [Range(0f, 1f)] public float positionLerp = 0.15f;
        [Range(0f, 1f)] public float rotationLerp = 0.08f;

        private void LateUpdate()
        {
            if (!target) return;
            Vector3 desiredPos = target.TransformPoint(localOffset);
            transform.position = Vector3.Lerp(transform.position, desiredPos, positionLerp);
            Quaternion desiredRot = Quaternion.LookRotation(
                (target.position + target.up * 1.2f - transform.position).normalized, target.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRot, rotationLerp);
        }
    }
}
