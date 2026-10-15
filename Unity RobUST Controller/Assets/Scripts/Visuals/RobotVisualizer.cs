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
    
    [Header("Multi-Camera Setup")]
    [Tooltip("Top-Down view (Top Left). Manually placed.")]
    public Camera topViewCamera;
    [Tooltip("Side view (Top Right). Manually placed.")]
    public Camera sideViewCamera;
    [Tooltip("Perspective view (Bottom Half). This one will track the robot center.")]
    public Camera perspectiveCamera;
    
    private float metersPerNewton = 0.002f;
    private float forceCapsuleRadius = 0.01f;

    // -- Goal Trajectory Constants --
    private const int GoalTrajSteps = 10;
    private const float GoalSphereDiameter = 0.1f;
    private readonly Color GoalTrajColor = new Color(0.0f, 1.0f, 0.0f, 0.4f); // Transparent Green

    // -- Internal State --
    private RobUSTDescription robot;
    private bool isInitialized = false;

    private Transform[] pulleySpheres;
    private Transform grf0Capsule;
    private Transform grf1Capsule;
    private Transform[] goalTrajectorySpheres;

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

    public bool Initialize(RobUSTDescription robotDescription)
    {
        // Validation
        if (comTrackerVisual == null ||
            endEffectorVisual == null ||
            frameTrackerVisual == null || 
            perspectiveCamera == null ||
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
        StripPhysics(perspectiveCamera.transform);
        StripPhysics(topViewCamera.transform);
        StripPhysics(sideViewCamera.transform);
        Destroy(topViewCamera.GetComponent<AudioListener>());
        Destroy(sideViewCamera.GetComponent<AudioListener>());
        Destroy(perspectiveCamera.GetComponent<AudioListener>());

        var root = new GameObject("Generated_Visuals").transform;
        root.SetParent(this.transform);

        grf0Capsule = CreateCapsule("Vis_GRF0", Color.cyan, root);
        grf1Capsule = CreateCapsule("Vis_GRF1", Color.cyan, root);
        GeneratePulleyVisuals(root);
        GenerateForcePlateVisual(root);
        GenerateGoalTrajectoryVisuals(root);

        ConfigureSplitScreen();

        isInitialized = true;
        return true;
    }

    private void ConfigureSplitScreen()
    {
        // Screen Layout:
        // |              |              |
        // |   Top View   |   Side View  |
        // |  (Top Left)  |  (Top Right) |
        // |______________|______________|
        // |                             |
        // |        Perspective          |
        // |       (Bottom Half)         |
        // |_____________________________|
        float3 robust_center = new float3(0f, 0.7f, 0.2f);

        if (topViewCamera != null)
        {
            topViewCamera.rect = new Rect(0.0f, 0.5f, 0.5f, 0.5f);
            topViewCamera.transform.position = (Vector3)RobotToUnityPos(robust_center + new float3(0f, 0f, 1.5f));
            topViewCamera.transform.LookAt((Vector3)RobotToUnityPos(robust_center), -Vector3.right);
        }

        if (sideViewCamera != null)
        {
            sideViewCamera.rect = new Rect(0.5f, 0.5f, 0.5f, 0.5f);
            sideViewCamera.transform.position = (Vector3)RobotToUnityPos(robust_center + new float3(0f, 1.5f, 0f));
            sideViewCamera.transform.LookAt((Vector3)RobotToUnityPos(robust_center));
        }

        // Perspective camera covers total bottom width (1.0f) and half height (0.5f)
        // Starting at X=0, Y=0
        if (perspectiveCamera != null)
        {
            perspectiveCamera.rect = new Rect(0.0f, 0.0f, 1.0f, 0.5f);
            float3 camPos = new float3(2.5f, 1.2f, .7f);
            perspectiveCamera.transform.position = (Vector3)RobotToUnityPos(camPos);
            perspectiveCamera.transform.LookAt((Vector3)RobotToUnityPos(robust_center));
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
    /// Pushes a goal trajectory to the visualizer.
    /// If span < 10, the last valid point is repeated for remaining spheres.
    /// If span >= 10, the first 10 points are used.
    /// </summary>
    public void PushGoalTrajectory(ReadOnlySpan<RBState> trajectory)
    {
        if (trajectory.IsEmpty) return;

        lock (dataLock)
        {
            int len = trajectory.Length;
            int copyCount = (len < GoalTrajSteps) ? len : GoalTrajSteps;

            // 1. Copy available points
            for (int i = 0; i < copyCount; i++)
                _goalTrajectoryCache[i] = trajectory[i].p;

            // 2. Clamp remaining if short trajectory
            if (len < GoalTrajSteps)
            {
                double3 lastP = trajectory[len - 1].p;
                for (int i = len; i < GoalTrajSteps; i++)
                    _goalTrajectoryCache[i] = lastP;
            }
        }
    }

    private void Update()
    {
        if (!isInitialized || !isActive) return;

        // Local cache vars
        double4x4 comMat, eeMat;
        double3 cop0, grf0, cop1, grf1;
        Span<double3> goalSnapshot = stackalloc double3[GoalTrajSteps];

        lock (dataLock)
        {
            // Simple snapshotting - dump latest state
            comMat = _comPose_Robot;
            eeMat = _eePose_Robot;
            cop0 = _cop0_Robot;
            grf0 = _grf0_Robot;
            cop1 = _cop1_Robot;
            grf1 = _grf1_Robot;
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

        UpdateForceCapsule(grf0Capsule, RobotToUnityPos((float3)cop0), RobotToUnityPos((float3)grf0));
        UpdateForceCapsule(grf1Capsule, RobotToUnityPos((float3)cop1), RobotToUnityPos((float3)grf1));

        // Update Goal Trajectory
        for (int i = 0; i < GoalTrajSteps; i++)
            goalTrajectorySpheres[i].position = (Vector3)RobotToUnityPos((float3)goalSnapshot[i]);

    }

    // =========================================================
    // Helpers
    // =========================================================

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
        pulleySpheres = new Transform[robot.FramePulleyPositions.Length];
        
        for (int i = 0; i < robot.FramePulleyPositions.Length; i++)
        {
            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"Pulley_{i}";
            sphere.transform.SetParent(root);
            sphere.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
            
            Destroy(sphere.GetComponent<Collider>());
            sphere.GetComponent<Renderer>().material.color = Color.gray;

            sphere.transform.position = (Vector3)RobotToUnityPos((float3)robot.FramePulleyPositions[i]);
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
        var trajRoot = new GameObject("Goal_Trajectory").transform;
        trajRoot.SetParent(root);

        // Attempt URP Unlit (fastest/cleanest), fallback to Standard
        var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Standard");
        var mat = new Material(shader);

        // Setup Transparency (Works for both usually)
        mat.SetFloat("_Surface", 1); // URP
        mat.SetFloat("_Mode", 3);    // Standard
        mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        mat.SetInt("_ZWrite", 0);
        mat.EnableKeyword("_ALPHABLEND_ON");
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT"); 
        mat.renderQueue = 3000;

        // Set Color (URP uses _BaseColor, Standard uses _Color)
        mat.SetColor("_BaseColor", GoalTrajColor); 
        mat.SetColor("_Color", GoalTrajColor);

        for (int i = 0; i < GoalTrajSteps; i++)
        {
            var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"Goal_Step_{i}";
            sphere.transform.SetParent(trajRoot);
            
            Destroy(sphere.GetComponent<Collider>());
            sphere.GetComponent<Renderer>().sharedMaterial = mat; 
            sphere.transform.localScale = Vector3.one * GoalSphereDiameter;
            sphere.transform.position = Vector3.zero;
            goalTrajectorySpheres[i] = sphere.transform;
        }
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