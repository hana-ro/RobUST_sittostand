using UnityEngine;
using Unity.Mathematics;
using System;

public class RobotVisualizer : MonoBehaviour
{
    [Header("General Settings")]
    [Tooltip("Toggle to enable/disable Robot visualization.")]
    public bool isActive = true;

    [Header("Tracker Prefabs")]
    public Transform comTrackerVisual;
    public Transform endEffectorVisual;
    public Transform frameTrackerVisual;

    [Header("Body Markers (meters)")]
    [Min(0.01f)] public float chestMarkerHeight = 0.25f;
    // [Tooltip("Chest width, height, and depth in meters. Applied at startup.")]
    // public Vector3 chestMarkerSize = new Vector3(0.28f, 0.18f, 0.20f);
    [Tooltip("Pelvis width, height, and depth in meters. Applied at startup.")]
    public Vector3 pelvisMarkerSize = new Vector3(0.28f, 0.18f, 0.20f);
    public Vector3 chestMarkerSize = new Vector3(0.28f, 0.18f, 0.20f);
    public Color chestMarkerColor = new Color(0.2f, 0.65f, 1f, BodyMarkerAlpha);
    public Color pelvisMarkerColor = new Color(1f, 0.5f, 0.15f, BodyMarkerAlpha);
    private const float BodyMarkerAlpha = 0.35f;

    [Header("Trajectory Markers")]
    public Color targetMarkerColor = new Color(0.1f, 0.85f, 0.2f, BodyMarkerAlpha);
    public float targetChestOffset = 0.15f;
    [Tooltip("Toggle visibility of the target chest marker.")]
    public bool showTargetChestMarker = true;
    [Tooltip("Toggle visibility of the target pelvis marker.")]
    public bool showTargetPelvisMarker = true;
    [Tooltip("Toggle visibility of the green pelvis trajectory lookahead horizon spheres.")]
    public bool showPelvisHorizon = true;
    [Tooltip("Toggle visibility of the cyan trunk trajectory lookahead horizon spheres.")]
    public bool showTrunkHorizon = true;

    private Mesh bodyMarkerMesh;
    private Material chestMarkerMaterial, pelvisMarkerMaterial;
    private GameObject chestMarker, pelvisMarker;
    private Material targetChestMaterial, targetPelvisMaterial;
    private GameObject targetChestMarker, targetPelvisMarker;
    
    [Header("Multi-Camera Setup")]
    [Tooltip("Top-Down view (Top Left). Manually placed.")]
    public Camera topViewCamera;
    [Tooltip("Side view (Left Half). Manually placed.")]
    public Camera sideViewCamera;
    [Tooltip("Back view view (Right Half). This one will track the robot center.")]
    [UnityEngine.Serialization.FormerlySerializedAs("perspectiveCamera")]
    public Camera backViewCamera;
    public Camera perspectiveCamera { get => backViewCamera; set => backViewCamera = value; }

    [Header("Force Plate Visual")]
    [Tooltip("Toggle display of the ground reaction forces.")]
    public bool showForceVisuals = true;

    private float metersPerNewton = 0.002f;
    private float forceCapsuleRadius = 0.01f;

    // -- Goal Trajectory Constants --
    private const int GoalTrajSteps = 10;
    private const float GoalSphereDiameter = 0.1f;
    private readonly Color GoalTrajColor = new Color(0.0f, 1.0f, 0.0f, 0.4f); // Transparent Green (Pelvis)
    private readonly Color TrunkTrajColor = new Color(0.1f, 0.65f, 1.0f, 0.4f); // Transparent Cyan (Trunk)

    // -- Internal State --
    private RobUSTDescription robot;
    private bool isInitialized = false;

    private Transform[] pulleySpheres;
    private Transform grf0Capsule;
    private Transform grf1Capsule;
    private Transform[] goalTrajectorySpheres;
    private Transform[] trunkTrajectorySpheres;

    // -- Thread Safety --
    private readonly object dataLock = new object();

    // -- CACHED DATA --
    private double4x4 _comPose_Robot;
    private double4x4 _eePose_Robot;
    private double3 _cop0_Robot;
    private double3 _grf0_Robot;
    private double3 _cop1_Robot;
    private double3 _grf1_Robot;

    // Trajectory Caches
    private double3[] _goalTrajectoryCache = new double3[GoalTrajSteps];
    private double3[] _trunkTrajectoryCache = new double3[GoalTrajSteps];
    private double _pelvisTunnelDeviation = -1.0;
    private double _trunkTunnelDeviation = -1.0;

    public bool Initialize(RobUSTDescription robotDescription)
    {
        // Validation
        if (comTrackerVisual == null ||
            endEffectorVisual == null ||
            frameTrackerVisual == null ||
            (backViewCamera == null && perspectiveCamera == null) ||
            topViewCamera == null ||
            sideViewCamera == null)
        {
            Debug.LogError("RobotVisualizer: Please assign all modules.");
            return false;
        }
        robot = robotDescription;
        StripPhysics(comTrackerVisual);
        StripPhysics(endEffectorVisual);
        StripPhysics(frameTrackerVisual);
        StripPhysics(backViewCamera.transform);
        StripPhysics(topViewCamera.transform);
        StripPhysics(sideViewCamera.transform);
        Destroy(topViewCamera.GetComponent<AudioListener>());
        Destroy(sideViewCamera.GetComponent<AudioListener>());
        Destroy(backViewCamera.GetComponent<AudioListener>());

        var root = new GameObject("Generated_Visuals").transform;
        root.SetParent(this.transform);

        grf0Capsule = CreateCapsule("Vis_GRF0", Color.cyan, root);
        grf1Capsule = CreateCapsule("Vis_GRF1", Color.cyan, root);
        GeneratePulleyVisuals(root);
        GenerateForcePlateVisual(root);
        GenerateGoalTrajectoryVisuals(root);
        GenerateBodyMarkers(root);

        ConfigureSplitScreen();

        showTargetChestMarker = true;
        showTargetPelvisMarker = true;

        isInitialized = true;
        return true;
    }

    private void GenerateBodyMarkers(Transform root)
    {
        var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Standard");
        if (shader == null)
        {
            Debug.LogError("RobotVisualizer: No supported body marker shader found.", this);
            return;
        }
        bodyMarkerMesh = CreateMarshmallowMesh();
        Vector3 pelvisSize = new Vector3((float)robot.PelvisMLDistance, pelvisMarkerSize.y, (float)robot.PelvisAPDistance);
        Vector3 trunkSize = new Vector3((float)robot.TrunkMLDistance, chestMarkerHeight, (float)robot.TrunkAPDistance);

        chestMarker = CreateBodyMarker("Chest_Body", endEffectorVisual,
            trunkSize, shader, chestMarkerColor, out chestMarkerMaterial);
        pelvisMarker = CreateBodyMarker("Pelvis_Body", comTrackerVisual,
            pelvisSize, shader, pelvisMarkerColor, out pelvisMarkerMaterial);

        targetChestMarker = CreateBodyMarker("Target_Chest_Body", root,
            trunkSize * 1.08f, shader, GoalTrajColor, out targetChestMaterial);
        targetPelvisMarker = CreateBodyMarker("Target_Pelvis_Body", root,
            pelvisSize * 1.08f, shader, GoalTrajColor, out targetPelvisMaterial);
    }

    private GameObject CreateBodyMarker(string name, Transform tracker, Vector3 size,
        Shader shader, Color color, out Material material)
    {
        var marker = new GameObject(name);
        marker.layer = tracker.gameObject.layer;
        marker.transform.SetParent(tracker, false);
        marker.transform.localScale = new Vector3(
            Mathf.Max(0.01f, size.x), Mathf.Max(0.01f, size.y), Mathf.Max(0.01f, size.z));
        marker.AddComponent<MeshFilter>().sharedMesh = bodyMarkerMesh;

        material = new Material(shader);

        // Configure transparency
        material.SetFloat("_Surface", 1); // Transparent in URP
        material.SetFloat("_Mode", 3);    // Transparent in Standard
        material.SetFloat("_Blend", 0);   // Alpha blend
        material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        material.SetInt("_ZWrite", 0);
        material.EnableKeyword("_ALPHABLEND_ON");
        material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        material.renderQueue = 3000;

        // Hardcode fixed translucent opacity
        color.a = BodyMarkerAlpha;
        material.color = color;
        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color")) material.SetColor("_Color", color);

        marker.AddComponent<MeshRenderer>().sharedMaterial = material;
        return marker;
    }

    private static Mesh CreateMarshmallowMesh()
    {
        // Flat ends and rounded rims, centered on the origin with local Y as height.
        const int sides = 48, bevelSteps = 6;
        const float bevel = 0.12f;
        const int rings = 2 * (bevelSteps + 1) + 2;
        var vertices = new Vector3[rings * (sides + 1)];
        var normals = new Vector3[vertices.Length];
        var triangles = new System.Collections.Generic.List<int>();
        for (int ring = 0; ring < rings; ring++)
        {
            float radius, height, radialNormal, verticalNormal;
            if (ring == 0 || ring == rings - 1)
            {
                radius = 0f;
                height = ring == 0 ? -0.5f : 0.5f;
                radialNormal = 0f;
                verticalNormal = ring == 0 ? -1f : 1f;
            }
            else
            {
                bool bottom = ring <= bevelSteps + 1;
                int step = bottom ? ring - 1 : ring - bevelSteps - 2;
                float angle = (bottom ? -Mathf.PI / 2f : 0f) + step * Mathf.PI / (2f * bevelSteps);
                radialNormal = Mathf.Cos(angle);
                verticalNormal = Mathf.Sin(angle);
                radius = 0.5f - bevel + bevel * radialNormal;
                height = (bottom ? -0.5f + bevel : 0.5f - bevel) + bevel * verticalNormal;
            }
            for (int side = 0; side <= sides; side++)
            {
                float angle = side * 2f * Mathf.PI / sides;
                int index = ring * (sides + 1) + side;
                vertices[index] = new Vector3(radius * Mathf.Cos(angle), height, radius * Mathf.Sin(angle));
                normals[index] = new Vector3(radialNormal * Mathf.Cos(angle), verticalNormal, radialNormal * Mathf.Sin(angle));
            }
        }
        for (int ring = 0; ring < rings - 1; ring++)
        for (int side = 0; side < sides; side++)
        {
            int a = ring * (sides + 1) + side;
            int b = a + sides + 1;
            if (ring != 0)
            {
                triangles.Add(a); triangles.Add(b); triangles.Add(a + 1);
            }
            if (ring != rings - 2)
            {
                triangles.Add(a + 1); triangles.Add(b); triangles.Add(b + 1);
            }
        }
        var mesh = new Mesh { name = "Marshmallow Body Marker" };
        mesh.vertices = vertices;
        mesh.normals = normals;
        mesh.SetTriangles(triangles, 0);
        mesh.RecalculateBounds();
        return mesh;
    }

    private void OnDestroy()
    {
        if (chestMarker != null) Destroy(chestMarker);
        if (pelvisMarker != null) Destroy(pelvisMarker);
        if (targetChestMarker != null) Destroy(targetChestMarker);
        if (targetPelvisMarker != null) Destroy(targetPelvisMarker);
        if (bodyMarkerMesh != null) Destroy(bodyMarkerMesh);
        if (chestMarkerMaterial != null) Destroy(chestMarkerMaterial);
        if (pelvisMarkerMaterial != null) Destroy(pelvisMarkerMaterial);
        if (targetChestMaterial != null) Destroy(targetChestMaterial);
        if (targetPelvisMaterial != null) Destroy(targetPelvisMaterial);
    }

    private void ConfigureSplitScreen()
    {
        // Screen Layout:
        // ----------------------+----------------------
        // |                      |                     |
        // | Side View from Left  |    Back View        |
        // |       (Left)         |     (Right)         |
        // |                      |                     |
        // |                      |                     |
        // ----------------------+----------------------
        float3 robust_center = new float3(0f, 0.7f, 0.2f);

        if (sideViewCamera != null)
        {
            // sideViewCamera.orthographic = true;
            // sideViewCamera.orthographicSize = 2.25f;
            // Left half of screen
            sideViewCamera.rect = new Rect(0.0f, 0.0f, 0.5f, 1.0f);
            float3 sidePos = robust_center + new float3(0f, -3.3f, 0f);

            sideViewCamera.transform.position = (Vector3)RobotToUnityPos(sidePos);
            sideViewCamera.transform.LookAt((Vector3)RobotToUnityPos(robust_center));
        }

        //
        // RIGHT PANEL: BACK VIEW
        //
        if (backViewCamera != null)
        {
            backViewCamera.orthographic = true;
            backViewCamera.orthographicSize = 2.25f;
            backViewCamera.rect = new Rect(0.5f, 0.0f, 0.5f, 1.0f);

            // Back view along robot X axis
            float3 backPos = robust_center + new float3(4.0f, 0f, 0f);
            float3 backViewCenter = robust_center + new float3(0f, 0.5f, 0f);

            backViewCamera.transform.position = (Vector3)RobotToUnityPos(backPos);
            backViewCamera.transform.LookAt((Vector3)RobotToUnityPos(backViewCenter), Vector3.up);
        }
    }

    /// <summary>
    /// Updated API: Accepts double4x4 directly.
    /// </summary>
    public void PushState(in double4x4 comPose, in double4x4 eePose, in ForcePlateData fp0, in ForcePlateData fp1)
    {
        lock (dataLock)
        {
            _comPose_Robot = comPose;
            _eePose_Robot = eePose;
            _cop0_Robot = fp0.CoP;
            _grf0_Robot = fp0.Force;
            _cop1_Robot = fp1.CoP;
            _grf1_Robot = fp1.Force;
        }
    }



    /// <summary>
    /// Pushes both pelvis and trunk goal trajectories to the visualizer.
    /// </summary>
    public void PushGoalTrajectories(ReadOnlySpan<RBState> pelvisTrajectory, ReadOnlySpan<RBState> trunkTrajectory)
    {
        lock (dataLock)
        {
            CopyTrajectoryToCache(pelvisTrajectory, _goalTrajectoryCache);
            CopyTrajectoryToCache(trunkTrajectory, _trunkTrajectoryCache);
        }
    }

    /// <summary>
    /// Pushes each segment's tunnel deviation (1 = at the wall or the edge of the timing band).
    /// Negative when that segment has no active tunnel.
    /// </summary>
    public void PushTunnelDeviation(double pelvisDeviation, double trunkDeviation)
    {
        lock (dataLock)
        {
            _pelvisTunnelDeviation = pelvisDeviation;
            _trunkTunnelDeviation = trunkDeviation;
        }
    }

    /// <summary>
    /// Overload for backwards compatibility (pushes pelvis trajectory only).
    /// </summary>
    public void PushGoalTrajectory(ReadOnlySpan<RBState> trajectory)
    {
        PushGoalTrajectories(trajectory, ReadOnlySpan<RBState>.Empty);
    }

    private static void CopyTrajectoryToCache(ReadOnlySpan<RBState> source, double3[] cache)
    {
        if (source.IsEmpty)
        {
            Array.Clear(cache, 0, GoalTrajSteps);
            return;
        }

        int len = source.Length;
        int copyCount = (len < GoalTrajSteps) ? len : GoalTrajSteps;

        for (int i = 0; i < copyCount; i++)
            cache[i] = source[i].p;

        if (len < GoalTrajSteps)
        {
            double3 lastP = source[len - 1].p;
            for (int i = len; i < GoalTrajSteps; i++)
                cache[i] = lastP;
        }
    }

    private void Update()
    {
        if (!isInitialized || !isActive) return;

        // Local cache vars
        double4x4 comMat, eeMat;
        double3 cop0, grf0, cop1, grf1;
        double pelvisTunnelDeviation, trunkTunnelDeviation;
        Span<double3> goalSnapshot = stackalloc double3[GoalTrajSteps];
        Span<double3> trunkSnapshot = stackalloc double3[GoalTrajSteps];

        lock (dataLock)
        {
            // Simple snapshotting - dump latest state
            comMat = _comPose_Robot;
            eeMat = _eePose_Robot;
            cop0 = _cop0_Robot;
            grf0 = _grf0_Robot;
            cop1 = _cop1_Robot;
            grf1 = _grf1_Robot;
            pelvisTunnelDeviation = _pelvisTunnelDeviation;
            trunkTunnelDeviation = _trunkTunnelDeviation;
            for (int i = 0; i < GoalTrajSteps; i++)
            {
                goalSnapshot[i] = _goalTrajectoryCache[i];
                trunkSnapshot[i] = _trunkTrajectoryCache[i];
            }
        }

        // 1. Cast to float4x4 once (Visuals don't need double precision)
        float4x4 comF = (float4x4)comMat;
        float4x4 eeF = (float4x4)eeMat;
        
        comTrackerVisual.position = (Vector3)RobotToUnityPos(comF.c3.xyz);
        comTrackerVisual.rotation = (Quaternion)RobotToUnityRot(new quaternion(comF));

        endEffectorVisual.position = (Vector3)RobotToUnityPos(eeF.c3.xyz);
        endEffectorVisual.rotation = (Quaternion)RobotToUnityRot(new quaternion(eeF));
        
        // Frame (Always Origin)
        frameTrackerVisual.localPosition = Vector3.zero; 
        frameTrackerVisual.localRotation = Quaternion.identity;

        if (showForceVisuals)
        {
            UpdateForceCapsule(grf0Capsule, RobotToUnityPos((float3)cop0), RobotToUnityPos((float3)grf0));
            UpdateForceCapsule(grf1Capsule, RobotToUnityPos((float3)cop1), RobotToUnityPos((float3)grf1));
        }
        else
        {
            if (grf0Capsule != null) grf0Capsule.localScale = Vector3.zero;
            if (grf1Capsule != null) grf1Capsule.localScale = Vector3.zero;
        }

        // Update Goal Trajectories (Pelvis: Green, Trunk: Cyan)
        bool hasPelvisGoal = math.lengthsq(goalSnapshot[0]) > 0.001f;
        bool hasTrunkGoal = math.lengthsq(trunkSnapshot[0]) > 0.001f;

        for (int i = 0; i < GoalTrajSteps; i++)
        {
            goalTrajectorySpheres[i].position = (Vector3)RobotToUnityPos((float3)goalSnapshot[i]);
            goalTrajectorySpheres[i].gameObject.SetActive(hasPelvisGoal && showPelvisHorizon);

            if (trunkTrajectorySpheres != null && i < trunkTrajectorySpheres.Length)
            {
                trunkTrajectorySpheres[i].position = (Vector3)RobotToUnityPos((float3)trunkSnapshot[i]);
                trunkTrajectorySpheres[i].gameObject.SetActive(hasTrunkGoal && showTrunkHorizon);
            }
        }

        if (targetPelvisMarker != null)
        {
            bool showP = hasPelvisGoal && showTargetPelvisMarker;
            targetPelvisMarker.SetActive(showP);
            if (showP)
            {
                targetPelvisMarker.transform.position = (Vector3)RobotToUnityPos((float3)goalSnapshot[0]);
                targetPelvisMarker.transform.rotation = comTrackerVisual.rotation;
                SetMarkerColor(targetPelvisMaterial, pelvisTunnelDeviation >= 0.0 ? DeviationColor(pelvisTunnelDeviation) : GoalTrajColor);
            }
        }
        if (targetChestMarker != null)
        {
            bool showT = hasTrunkGoal && showTargetChestMarker;
            targetChestMarker.SetActive(showT);
            if (showT)
            {
                targetChestMarker.transform.position = (Vector3)RobotToUnityPos((float3)trunkSnapshot[0]);
                targetChestMarker.transform.rotation = (endEffectorVisual != null && math.lengthsq(eeF.c3.xyz) > 0.001f)
                    ? endEffectorVisual.rotation
                    : comTrackerVisual.rotation;
                SetMarkerColor(targetChestMaterial, trunkTunnelDeviation >= 0.0 ? DeviationColor(trunkTunnelDeviation) : GoalTrajColor);
            }
        }

    }

    private void OnGUI()
    {
        if (!isActive) return;

        GUIStyle buttonStyle = new GUIStyle(GUI.skin.button)
        {
            fontSize = 32,
            fontStyle = FontStyle.Bold
        };

        if (GUI.Button(new Rect(12f, 12f, 320f, 48f), showForceVisuals ? "Hide GRFs" : "Display GRFs", buttonStyle))
        {
            SetForceVisualsVisible(!showForceVisuals);
        }

        DrawViewLabel(sideViewCamera, "Side View");
        DrawViewLabel(backViewCamera, "Back View");
    }

    // Labels a split-screen panel at its bottom left, above the trajectory progress bar.
    private static void DrawViewLabel(Camera viewCamera, string text)
    {
        if (viewCamera == null) return;

        GUIStyle labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 30,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter,
            normal = { textColor = Color.white }
        };

        Rect panel = viewCamera.pixelRect; // origin at bottom left, GUI origin is top left
        Vector2 size = labelStyle.CalcSize(new GUIContent(text)) + new Vector2(16f, 8f);
        Rect box = new Rect(panel.x + 12f, Screen.height - panel.y - size.y - 72f, size.x, size.y);

        Color prevColor = GUI.color;
        GUI.color = new Color(0f, 0f, 0f, 0.6f);
        GUI.DrawTexture(box, Texture2D.whiteTexture);
        GUI.color = prevColor;
        GUI.Label(box, text, labelStyle);
    }

    // =========================================================
    // Helpers
    // =========================================================

    // Continuous hue sweep: green on the path, yellow at the tunnel wall, red at twice the wall distance.
    private static Color DeviationColor(double deviation)
    {
        float hue = Mathf.Lerp(1f / 3f, 0f, Mathf.Clamp01((float)deviation / 2f));
        return Color.HSVToRGB(hue, 1f, 1f);
    }

    private static void SetMarkerColor(Material material, Color color)
    {
        color.a = BodyMarkerAlpha;
        material.color = color;
        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
    }

    private static float3 RobotToUnityPos(float3 v)
    {
        return new float3(v.x, v.z, v.y);
    }

    private static quaternion RobotToUnityRot(quaternion q)
    {
        float3 fwd_r = math.rotate(q, new float3(0, 1, 0)); 
        float3 up_r  = math.rotate(q, new float3(0, 0, 1));

        float3 fwd_u = RobotToUnityPos(fwd_r);
        float3 up_u  = RobotToUnityPos(up_r);

        if (math.lengthsq(fwd_u) < 0.001f) return quaternion.identity;
        return quaternion.LookRotation(fwd_u, up_u);
    }

    public void SetForceVisualsVisible(bool visible)
    {
        showForceVisuals = visible;
        if (!visible)
        {
            if (grf0Capsule != null) grf0Capsule.localScale = Vector3.zero;
            if (grf1Capsule != null) grf1Capsule.localScale = Vector3.zero;
        }
    }

    private void UpdateForceCapsule(Transform capsule, float3 anchor, float3 forceVector)
    {
        float magnitude = math.length(forceVector);
        if (magnitude < 0.1f)
        {
            capsule.localScale = Vector3.zero;
            return;
        }

        float3 dir = forceVector / magnitude;
        float length = magnitude * metersPerNewton;

        capsule.rotation = Quaternion.FromToRotation(Vector3.up, (Vector3)dir);
        capsule.position = (Vector3)(anchor + (dir * (length * 0.5f)));
        capsule.localScale = new Vector3(forceCapsuleRadius * 2, length * 0.5f, forceCapsuleRadius * 2);
    }

    private Transform CreateCapsule(string name, Color c, Transform parent)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        go.name = name;
        go.transform.SetParent(parent);
        Destroy(go.GetComponent<Collider>());
        go.GetComponent<Renderer>().material.color = c;
        go.transform.localScale = Vector3.zero;
        return go.transform;
    }

    private void GeneratePulleyVisuals(Transform root)
    {
        // Trunk pulleys are drawn only once their real positions are entered
        int numTrunk = robot.HasTrunkCables ? robot.NumTrunkCables : 0;
        int numPelvis = robot.FramePulleyPositions.Length;
        pulleySpheres = new Transform[numPelvis + numTrunk];

        for (int i = 0; i < pulleySpheres.Length; i++)
        {
            bool isTrunk = i >= numPelvis;
            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = isTrunk ? $"Trunk_Pulley_{i - numPelvis}" : $"Pulley_{i}";
            sphere.transform.SetParent(root);
            sphere.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);

            Destroy(sphere.GetComponent<Collider>());
            sphere.GetComponent<Renderer>().material.color = isTrunk ? TrunkTrajColor : Color.gray;

            double3 position = isTrunk ? robot.TrunkFramePulleyPositions[i - numPelvis] : robot.FramePulleyPositions[i];
            sphere.transform.position = (Vector3)RobotToUnityPos((float3)position);
            pulleySpheres[i] = sphere.transform;
        }
    }
    
    private void GenerateForcePlateVisual(Transform root)
    {
        // Calculate center and orientation from corners in Description
        var pBL = robot.FP_BackLeft;
        var pBR = robot.FP_BackRight;
        var pFL = robot.FP_FrontLeft;
        var pFR = robot.FP_FrontRight;

        double3 center = (pBL + pBR + pFL + pFR) * 0.25;
        
        // Approximate basis vectors for visualization
        double3 forward = math.normalize((pFL - pBL) + (pFR - pBR)); // Approx Y usually
        double3 right = math.normalize((pBR - pBL) + (pFR - pFL));   // Approx X usually
        double3 up = math.cross(forward, right); // Z?
        
        // Dimensions
        float length = (float)math.length((pFL - pBL) + (pFR - pBR)) * 0.5f;
        float width = (float)math.length((pBR - pBL) + (pFR - pFL)) * 0.5f;
        float thickness = 0.05f;

        var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = "ForcePlate_Surface";
        cube.transform.SetParent(root);
        Destroy(cube.GetComponent<Collider>());
        
        cube.transform.position = (Vector3)RobotToUnityPos((float3)center - (float3)up * (thickness * 0.5f)); // Shift down so top surface is at 0
        quaternion rotRobot = quaternion.LookRotation((float3)up, (float3)forward); 
        cube.transform.rotation = (Quaternion)RobotToUnityRot(rotRobot);

        cube.transform.localScale = new Vector3(width, thickness, length);

        var ren = cube.GetComponent<Renderer>();
        ren.material.color = new Color(0.3f, 0.3f, 0.3f, 0.5f); // Transparent Grey
    }

    private void GenerateGoalTrajectoryVisuals(Transform root)
    {
        goalTrajectorySpheres = new Transform[GoalTrajSteps];
        trunkTrajectorySpheres = new Transform[GoalTrajSteps];
        var trajRoot = new GameObject("Goal_Trajectories").transform;
        trajRoot.SetParent(root);

        // Attempt URP Unlit (fastest/cleanest), fallback to Standard
        var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Standard");

        var pelvisMat = CreateTrajectoryMaterial(shader, GoalTrajColor);
        var trunkMat = CreateTrajectoryMaterial(shader, TrunkTrajColor);

        for (int i = 0; i < GoalTrajSteps; i++)
        {
            // Pelvis Sphere (Green)
            var pSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            pSphere.name = $"Pelvis_Goal_Step_{i}";
            pSphere.transform.SetParent(trajRoot);
            Destroy(pSphere.GetComponent<Collider>());
            pSphere.GetComponent<Renderer>().sharedMaterial = pelvisMat;
            pSphere.transform.localScale = Vector3.one * GoalSphereDiameter;
            pSphere.transform.position = Vector3.zero;
            pSphere.SetActive(false);
            goalTrajectorySpheres[i] = pSphere.transform;

            // Trunk Sphere (Cyan)
            var tSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            tSphere.name = $"Trunk_Goal_Step_{i}";
            tSphere.transform.SetParent(trajRoot);
            Destroy(tSphere.GetComponent<Collider>());
            tSphere.GetComponent<Renderer>().sharedMaterial = trunkMat;
            tSphere.transform.localScale = Vector3.one * GoalSphereDiameter;
            tSphere.transform.position = Vector3.zero;
            tSphere.SetActive(false);
            trunkTrajectorySpheres[i] = tSphere.transform;
        }
    }

    private static Material CreateTrajectoryMaterial(Shader shader, Color color)
    {
        var mat = new Material(shader);
        mat.SetFloat("_Surface", 1); // URP
        mat.SetFloat("_Mode", 3);    // Standard
        mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.SetInt("_ZWrite", 0);
        mat.EnableKeyword("_ALPHABLEND_ON");
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        mat.renderQueue = 3000;
        mat.SetColor("_BaseColor", color);
        mat.SetColor("_Color", color);
        return mat;
    }

    private void StripPhysics(Transform root)
    {
        // Remove Rigidbodies (Start from root, usually only one RB exists)
        foreach (var rb in root.GetComponentsInChildren<Rigidbody>())
        {
            Destroy(rb);
        }

        // Remove Colliders (Recursively)
        foreach (var col in root.GetComponentsInChildren<Collider>())
        {
            Destroy(col);
        }
    }
}
