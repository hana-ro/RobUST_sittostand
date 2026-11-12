using Unity.Mathematics;
using UnityEngine;
using UnityEngine.InputSystem;

// Full-screen sagittal plot (AP vs. vertical) of the pelvis and trunk paths and their live positions,
// for the bodies the trial's cables drive. Press V to toggle.
public class SubjectVisualFeedbackView : MonoBehaviour
{
    public bool show = false;
    [Tooltip("Largest share of the screen width and height the plot may fill, centred.")]
    [Range(0.2f, 1f)] public float plotSize = 0.8f;
    public float dotSize = 36f;
    [Tooltip("Extra movement room at the top and right of the reference paths [cm], plus clearance for the tracking circle.")]
    [Min(0f)] public float pathMarginCm = 10f;
    public Color pathColor = new Color(0.45f, 0.45f, 0.45f);

    private static readonly Color Ink = new Color(0.15f, 0.15f, 0.15f);

    private RobotController controller;
    private RobotVisualizer visualizer;
    private Material lineMaterial;
    private GUIStyle titleStyle, countdownStyle, directionStyle;
    private double3 origin;  // seated pelvis position, the plot's (0, 0)
    private float2 origin_px; // where (0, 0) sits on screen
    private float pxPerCm;

    private void Start()
    {
        controller = FindFirstObjectByType<RobotController>();
        visualizer = FindFirstObjectByType<RobotVisualizer>();
        lineMaterial = new Material(Shader.Find("Hidden/Internal-Colored"));
        lineMaterial.SetInt("_Cull", 0);
    }

    private void OnDestroy() => Destroy(lineMaterial);

    private void Update()
    {
        if (Keyboard.current != null && Keyboard.current.vKey.wasPressedThisFrame) show = !show;
    }

    private void OnGUI()
    {
        GUI.depth = -10; // draw over the other overlays
        if (!show || Event.current.type != EventType.Repaint) return;
        // Hover keeps the normal colour (the default label style greys on mouse-over)
        titleStyle ??= new GUIStyle(GUI.skin.label) { fontSize = 32, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter, normal = { textColor = Ink }, hover = { textColor = Ink } };
        countdownStyle ??= new GUIStyle(titleStyle) { fontSize = 140, normal = { textColor = Color.black }, hover = { textColor = Color.black } };
        directionStyle ??= new GUIStyle(titleStyle) { fontSize = 64 };

        GUI.color = Color.white;
        GUI.DrawTexture(new Rect(0, 0, Screen.width, Screen.height), Texture2D.whiteTexture);

        // Only the bodies this trial's cables drive
        bool showCoM = RobotController.DrivesCoM(controller.ActiveCables);
        bool showEE = RobotController.DrivesEE(controller.ActiveCables);
        double3[] comPath = showCoM ? controller.CoMPath : null;
        double3[] eePath = showEE ? controller.EEPath : null;

        // Both bodies are drawn relative to the seated pelvis, so the trunk sits above it.
        // Before the first trial there is no path, so show the live positions on default 50 cm axes.
        origin = controller.CoMPath != null ? controller.CoMPath[0] : visualizer.CoMPosition;
        float2 min = float2.zero, max = float2.zero;
        if (comPath == null && eePath == null)
        {
            max = new float2(50f, 50f);
            if (showEE) { min = math.min(min, ToCm(visualizer.EEPosition)); max = math.max(max, ToCm(visualizer.EEPosition)); }
        }
        foreach (double3[] path in new[] { comPath, eePath })
            if (path != null)
                foreach (double3 p in path) { min = math.min(min, ToCm(p)); max = math.max(max, ToCm(p)); }
        // Keep the original 5 cm spacing on the left/bottom; add room only top/right.
        min -= 5f;
        max += math.max(5f, pathMarginCm);

        // Reserve circle clearance only on the top/right, keeping equal axis scales.
        // Bounds depend on the reference, so moving the tracker never rescales the plot.
        float2 range = math.max(max - min, new float2(1f));
        float circleClearancePx = math.max(0f, dotSize) * 0.5f + 8f;
        float2 available = math.max(new float2(Screen.width, Screen.height) * plotSize
            - circleClearancePx, new float2(1f));
        pxPerCm = math.min(available.x / range.x, available.y / range.y);
        max += circleClearancePx / pxPerCm;
        range = max - min;
        float2 lo = new float2((Screen.width - range.x * pxPerCm) / 2f, (Screen.height + range.y * pxPerCm) / 2f); // bottom left
        origin_px = lo + new float2(-min.x, min.y) * pxPerCm;
        float2 hi = ToScreen(max); // top right

        // Lines: axes, the paths, then the pelvis-trunk segment when both are shown
        lineMaterial.SetPass(0);
        GL.PushMatrix();
        GL.LoadPixelMatrix(0, Screen.width, Screen.height, 0);
        GL.Begin(GL.QUADS);
        Line(lo, new float2(hi.x, lo.y), 3f, Ink);
        Line(lo, new float2(lo.x, hi.y), 3f, Ink);
        ArrowHead(new float2(hi.x, lo.y), new float2(1f, 0f));
        ArrowHead(new float2(lo.x, hi.y), new float2(0f, -1f));
        DrawPath(comPath);
        DrawPath(eePath);
        if (showCoM && showEE) Line(ToScreen(ToCm(visualizer.CoMPosition)), ToScreen(ToCm(visualizer.EEPosition)), 4f, Ink);
        GL.End();
        GL.PopMatrix();

        GUI.Label(new Rect(lo.x, lo.y + 12, hi.x - lo.x, 44), "Forward", titleStyle);
        // "Vertical" centred along the y axis, rotated to read bottom to top
        float2 yMid = new float2(lo.x - 36f, (lo.y + hi.y) / 2f);
        GUIUtility.RotateAroundPivot(-90f, yMid);
        GUI.Label(new Rect(yMid.x - 200, yMid.y - 22, 400, 44), "Vertical", titleStyle);
        GUI.matrix = Matrix4x4.identity;

        // Live positions, coloured by distance from the path during a trial
        if (showCoM) LiveDot(visualizer.CoMPosition, visualizer.CoMDeviation);
        if (showEE) LiveDot(visualizer.EEPosition, visualizer.EEDeviation);

        GUI.color = Color.white;
        string direction = comPath == null && eePath == null ? "Select A or I, then press Space to capture a trajectory"
            : controller.IsPostTrialTransparent ? "Move freely; return to sitting, then press Space"
            : showCoM && showEE ? "Follow the pelvis and trunk trajectories"
            : showEE ? "Follow the trunk trajectory" : "Follow the pelvis trajectory";
        GUI.Label(new Rect(0, 0, Screen.width, hi.y), direction, directionStyle); // centred between the screen top and the arrow tip

        string countdown = controller.CountdownText;
        if (countdown != null) GUI.Label(new Rect(0, 0, Screen.width, Screen.height), countdown, countdownStyle);
    }

    // Robot frame: forward is -X, up is +Z
    private float2 ToCm(double3 p) => new float2((float)-(p.x - origin.x), (float)(p.z - origin.z)) * 100f;
    private float2 ToScreen(float2 cm) => new float2(origin_px.x + cm.x * pxPerCm, origin_px.y - cm.y * pxPerCm);

    private void DrawPath(double3[] path)
    {
        if (path == null) return;
        for (int i = 1; i < path.Length; i++) Line(ToScreen(ToCm(path[i - 1])), ToScreen(ToCm(path[i])), 5f, pathColor);
    }

    // Green on the path, yellow at the inner wall, red at the outer; green outside a trial.
    private void LiveDot(double3 p, double deviation) =>
        Dot(p, RobotVisualizer.DeviationColor(math.max(deviation, 0.0)), dotSize);

    private void Dot(double3 p, Color color, float size)
    {
        float2 s = ToScreen(ToCm(p));
        GUI.color = color;
        GUI.DrawTexture(new Rect(s.x - size / 2, s.y - size / 2, size, size), RobotController.GetCircleTexture());
    }

    // Filled triangle with its tip at tip, pointing along dir (a unit vector in screen pixels)
    private static void ArrowHead(float2 tip, float2 dir)
    {
        const float length = 22f, halfWidth = 10f;
        float2 back = tip - dir * length, side = new float2(-dir.y, dir.x) * halfWidth;
        GL.Color(Ink);
        GL.Vertex3(tip.x, tip.y, 0);
        GL.Vertex3(tip.x, tip.y, 0);
        GL.Vertex3(back.x + side.x, back.y + side.y, 0);
        GL.Vertex3(back.x - side.x, back.y - side.y, 0);
    }

    private static void Line(float2 a, float2 b, float width, Color color)
    {
        float2 n = math.normalizesafe(new float2(a.y - b.y, b.x - a.x)) * (width / 2f);
        GL.Color(color);
        GL.Vertex3(a.x + n.x, a.y + n.y, 0);
        GL.Vertex3(b.x + n.x, b.y + n.y, 0);
        GL.Vertex3(b.x - n.x, b.y - n.y, 0);
        GL.Vertex3(a.x - n.x, a.y - n.y, 0);
    }
}
