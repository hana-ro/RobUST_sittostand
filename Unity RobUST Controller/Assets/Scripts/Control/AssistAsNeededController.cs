using Unity.Mathematics;

/// <summary>
/// Assist-as-needed control for one body segment: a 2D tunnel around its reference path in the sagittal
/// plane (robot X and Z), with no timing, so the subject sets the pace. Each tick the body is projected
/// onto the path: d is its distance from the nearest point N, and s is how far N is along the path.
/// Normal force, toward N:
///   Fn = 0 if d &lt;= InnerRadius, linear to MaxForce at OuterRadius, MaxForce beyond
/// Tangential force, along the path at N:
///   Ft = K_t * (1 - d / OuterRadius) * onset(s)
/// where onset rises from 0 to 1 between OnsetDistance and OnsetDistance + OnsetRamp of the subject's own
/// progress. Both act in full up to the end of the path; Completed then tells the caller to release.
/// No side-to-side force.
/// </summary>
public class AssistAsNeededController : BaseController<Wrench>
{
    /// <summary>Inside this distance from the path there is no normal force [m]</summary>
    public const double InnerRadius = 0.03;
    /// <summary>Normal force is at its maximum, and tangential force zero, from this distance on [m]</summary>
    public const double OuterRadius = 0.06;
    private const double OnsetDistance = 0.03; // progress the subject makes alone before the push starts [m]
    private const double OnsetRamp = 0.02;     // progress over which the push ramps in [m]
    private const int SearchWindow = 5;        // path segments searched either side of the last match

    private readonly double maxForce;
    private readonly double tangentialForce;

    private double3[] path;
    private double[] arcLength; // sagittal distance along the path to each point [m]
    private int segment;        // segment holding the nearest point, kept between ticks
    private double3 currentPos;

    /// <summary>Distance from the path as a fraction of InnerRadius: 1 at the inner wall, 2 at the outer.</summary>
    public double Deviation { get; private set; }
    /// <summary>Progress along the path, 0 to 1.</summary>
    public double Progress { get; private set; }
    /// <summary>Nearest point on the path, in the Robot Frame.</summary>
    public double3 NearestPoint { get; private set; }
    /// <summary>True from the first tick the nearest point reaches the end of the path, until the next SetPath.</summary>
    public bool Completed { get; private set; }
    /// <summary>The path point this many points ahead of the nearest point, stopping at the end.</summary>
    public double3 PathAhead(int points) => path[math.min(segment + points, path.Length - 1)];

    /// <param name="maxForce">Normal force reached at OuterRadius and beyond [N]</param>
    /// <param name="tangentialForce">Largest push along the path, K_t [N]</param>
    public AssistAsNeededController(double maxForce, double tangentialForce)
    {
        this.maxForce = maxForce;
        this.tangentialForce = tangentialForce;
    }

    /// <summary>Sets this trial's path, in the Robot Frame, starting at the seated position.</summary>
    public void SetPath(double3[] path_RF)
    {
        path = path_RF;
        arcLength = new double[path.Length];
        for (int i = 1; i < path.Length; i++)
            arcLength[i] = arcLength[i - 1] + math.length(Sagittal(path[i] - path[i - 1]));
        segment = 0;
        Completed = false;
    }

    public void UpdateState(in double4x4 pose_RF)
    {
        currentPos = pose_RF.c3.xyz;
    }

    public override Wrench computeNextControl()
    {
        if (path == null) return new Wrench(double3.zero, double3.zero);

        // Nearest point on the path, searched near the last match so it cannot jump across the curve
        double3 q = Sagittal(currentPos);
        double bestDistSq = double.MaxValue, bestT = 0.0;
        int best = segment;
        int last = path.Length - 2;
        for (int i = math.max(0, segment - SearchWindow); i <= math.min(last, segment + SearchWindow); i++)
        {
            double3 a = Sagittal(path[i]), ab = Sagittal(path[i + 1]) - a;
            double lenSq = math.lengthsq(ab);
            double t = lenSq > 0.0 ? math.saturate(math.dot(q - a, ab) / lenSq) : 0.0;
            double distSq = math.distancesq(q, a + t * ab);
            if (distSq < bestDistSq) { bestDistSq = distSq; best = i; bestT = t; }
        }
        segment = best;
        NearestPoint = math.lerp(path[best], path[best + 1], bestT);

        double total = arcLength[path.Length - 1];
        double s = math.lerp(arcLength[best], arcLength[best + 1], bestT);
        double3 toPath = Sagittal(NearestPoint) - q;
        double d = math.length(toPath);
        Deviation = d / InnerRadius;
        Progress = s / total;
        if (Progress >= 1.0) Completed = true;

        double3 normal = d > InnerRadius
            ? toPath / d * math.saturate((d - InnerRadius) / (OuterRadius - InnerRadius)) * maxForce
            : double3.zero;
        double3 tangent = math.normalizesafe(Sagittal(path[best + 1] - path[best]));
        double push = tangentialForce
            * math.saturate(1.0 - d / OuterRadius)
            * math.saturate((s - OnsetDistance) / OnsetRamp);
        return new Wrench(normal + tangent * push, double3.zero);
    }

    // Drops the side-to-side (robot Y) component
    private static double3 Sagittal(double3 v) => new double3(v.x, 0.0, v.z);
}
