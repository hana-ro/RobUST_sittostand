using Unity.Mathematics;

/// <summary>
/// The reference schedule shared by every tunnel in a trial, so pelvis and trunk are timed
/// against one movement. The first tunnel whose segment starts moving starts it.
/// </summary>
public sealed class ReferenceClock
{
    public bool IsRunning { get; private set; }

    /// <summary>Where the subject should be along the movement, 0 to 1.</summary>
    public double Tau { get; private set; }

    /// <summary>Target duration of the whole movement [s].</summary>
    public double Duration { get; private set; } = 1.0;

    public void Reset(double duration)
    {
        Duration = math.max(0.1, duration);
        IsRunning = false;
        Tau = 0.0;
    }

    /// <summary>Starts at the given point of the movement. Ignored if already running.</summary>
    public void Start(double tau)
    {
        if (IsRunning) return;
        IsRunning = true;
        Tau = tau;
    }

    /// <summary>Call once per control tick.</summary>
    public void Advance(double dt)
    {
        if (IsRunning) Tau = math.min(1.0, Tau + (dt / Duration));
    }
}

/// <summary>
/// Assist-as-needed control for one body segment: a force tunnel around a normalized reference
/// trajectory, with a timing band along it.
/// Across the path: zero force inside the tunnel, and outside an ImpedanceController pulls the
/// subject back to the wall, resisting further outward motion.
/// Along the path: once the segment starts moving, a reference clock runs over the target duration.
/// Within the timing band there is no force; outside it the subject is pushed along the path, or
/// held back, to the edge of the band. Force grows from zero at every boundary.
/// </summary>
public class AssistAsNeededController : BaseController<Wrench>
{
    // Path samples searched ahead each tick. Keeps progress local, so it cannot jump along the path.
    private const int SearchWindow = 10;
    // Penetration over which damping fades in, so force stays continuous at the wall [m]
    private const double DampingRampDistance = 0.02;
    // Distance the reference path moves before the clock starts. Lets the subject choose when to
    // rise, and avoids timing the trunk-flexion phase, where the pelvis barely moves [m].
    private const double OnsetDistance = 0.02;

    private readonly ImpedanceController wallImpedance;
    private readonly double3[] normalizedPath;
    private readonly double3[] normalizedSD;
    private readonly double3[] pathPoints;
    private readonly double3[] halfWidths;
    private readonly double widthInSD;
    private readonly double minHalfWidth;
    private readonly double timingBand;
    private readonly ReferenceClock clock;

    // Internal State
    private double4x4 currentPose;
    private double3 currentLinVel;
    private double phase;
    private double onsetPhase;
    private bool isStateValid = false;

    /// <summary>Distance from the path as a fraction of the tunnel width: 0 on the path, 1 at the wall.</summary>
    public double Deviation { get; private set; }

    /// <summary>Timing error as a fraction of the timing band: 0 on schedule, 1 at the edge of the band.</summary>
    public double TimingDeviation { get; private set; }

    /// <summary>Subject progress along the path, 0 to 1.</summary>
    public double Progress => phase / (pathPoints.Length - 1);

    /// <summary>Where the subject should be along the path, 0 to 1. Follows the subject until the clock starts.</summary>
    public double ReferenceProgress => clock.IsRunning ? clock.Tau : Progress;

    /// <param name="userMass">User body mass [kg]</param>
    /// <param name="normalizedPath">Robot-frame path, as fractions of the trial scale</param>
    /// <param name="normalizedSD">Robot-frame SD per axis, as fractions of the trial scale</param>
    /// <param name="widthInSD">Tunnel half-width in SDs of the reference</param>
    /// <param name="minHalfWidth">Smallest allowed half-width on any axis [m]</param>
    /// <param name="timingBand">Allowed time ahead of or behind the reference clock [s]</param>
    /// <param name="maxForce">Force limit [N]</param>
    /// <param name="damping">Damping against outward motion [N·s/m]</param>
    /// <param name="clock">Reference schedule, shared with the other tunnels of the trial</param>
    public AssistAsNeededController(float userMass, double3[] normalizedPath, double3[] normalizedSD,
                                 double widthInSD, double minHalfWidth, double timingBand,
                                 double maxForce, double damping, ReferenceClock clock)
    {
        this.normalizedPath = normalizedPath;
        this.normalizedSD = normalizedSD;
        this.widthInSD = widthInSD;
        this.minHalfWidth = minHalfWidth;
        this.timingBand = timingBand;
        this.clock = clock;
        pathPoints = new double3[normalizedPath.Length];
        halfWidths = new double3[normalizedPath.Length];

        wallImpedance = new ImpedanceController(userMass)
        {
            D_pos = new double3(damping, damping, damping),
            K_ori = double3.zero,
            D_ori = double3.zero,
            MaxForce = maxForce
        };
    }

    /// <summary>
    /// Places the tunnel for a new trial. Call before the first UpdateState of the trial,
    /// after resetting the shared clock.
    /// </summary>
    /// <param name="origin">Seated position in Robot Frame [m]</param>
    /// <param name="scale">Per-axis scale of the normalized path [m]</param>
    public void BeginTrial(in double3 origin, in double3 scale)
    {
        for (int i = 0; i < pathPoints.Length; i++)
        {
            pathPoints[i] = origin + (normalizedPath[i] * scale);
            halfWidths[i] = math.max(widthInSD * normalizedSD[i] * scale, minHalfWidth);
        }

        onsetPhase = pathPoints.Length - 1;
        for (int i = 1; i < pathPoints.Length; i++)
        {
            if (math.distance(pathPoints[i], pathPoints[0]) >= OnsetDistance)
            {
                onsetPhase = i;
                break;
            }
        }

        phase = 0.0;
        Deviation = 0.0;
        TimingDeviation = 0.0;
        isStateValid = false;
    }

    /// <summary>
    /// Updates the controller with the latest sensor data.
    /// Call this immediately before computeNextControl().
    /// </summary>
    /// <param name="pose_RF">Current pelvis pose in Robot Frame</param>
    /// <param name="linVel_RF">Current pelvis linear velocity in Robot Frame [m/s]</param>
    public void UpdateState(double4x4 pose_RF, double3 linVel_RF)
    {
        currentPose = pose_RF;
        currentLinVel = linVel_RF;
        isStateValid = true;
    }

    public override Wrench computeNextControl()
    {
        if (!isStateValid) return new Wrench(double3.zero, double3.zero);

        double3 p_curr = currentPose.c3.xyz;
        UpdatePhase(p_curr);

        // Starts the shared clock as this segment passes its onset, at the reference time for that
        // point, so a subject who starts on time is on schedule
        if (phase >= onsetPhase) clock.Start(onsetPhase / (pathPoints.Length - 1));

        // Across the path: target the wall along the line back to the path
        double3 center = PathPointAt(phase);
        double3 halfWidth = HalfWidthAt(phase);
        double3 offset = p_curr - center;
        Deviation = math.length(offset / halfWidth);

        double3 target = p_curr;
        double3 targetVel = currentLinVel;
        bool isOutsideTunnel = Deviation > 1.0;
        if (isOutsideTunnel)
        {
            target = center + (offset / Deviation);
            // Match velocity except for outward motion, which is damped
            double3 outward = math.normalize(offset);
            double penetration = math.length(p_curr - target);
            double outwardSpeed = math.max(0.0, math.dot(currentLinVel, outward));
            double dampingRamp = math.saturate(penetration / DampingRampDistance);
            targetVel -= outward * outwardSpeed * dampingRamp;
        }

        // Along the path: shift the target to the nearest edge of the timing band
        bool isOutsideBand = false;
        if (clock.IsRunning)
        {
            double tau = Progress;
            double bandTau = timingBand / clock.Duration;
            double boundTau = math.clamp(tau, clock.Tau - bandTau, clock.Tau + bandTau);
            TimingDeviation = math.abs(tau - clock.Tau) / math.max(1e-6, bandTau);
            if (boundTau != tau)
            {
                isOutsideBand = true;
                int last = pathPoints.Length - 1;
                target += PathPointAt(boundTau * last) - PathPointAt(phase);
            }
        }

        if (!isOutsideTunnel && !isOutsideBand) return new Wrench(double3.zero, double3.zero);

        RBState targetState = new RBState(target, double3.zero, targetVel, double3.zero);
        wallImpedance.UpdateState(currentPose, currentLinVel, double3.zero, targetState);
        return wallImpedance.computeNextControl();
    }

    private double3 PathPointAt(double samplePhase)
    {
        int i0 = (int)samplePhase;
        int i1 = math.min(i0 + 1, pathPoints.Length - 1);
        return math.lerp(pathPoints[i0], pathPoints[i1], samplePhase - i0);
    }

    private double3 HalfWidthAt(double samplePhase)
    {
        int i0 = (int)samplePhase;
        int i1 = math.min(i0 + 1, pathPoints.Length - 1);
        return math.lerp(halfWidths[i0], halfWidths[i1], samplePhase - i0);
    }

    // Moves progress to the closest point on the path ahead. It never moves back, so a subject
    // who stops or leaves the path is held to where they had reached.
    private void UpdatePhase(in double3 p)
    {
        int start = (int)phase;
        int end = math.min(start + SearchWindow, pathPoints.Length - 1);
        double bestDistSq = double.MaxValue;
        double bestPhase = phase;

        for (int i = start; i < end; i++)
        {
            double3 a = pathPoints[i];
            double3 ab = pathPoints[i + 1] - a;
            double lenSq = math.dot(ab, ab);
            double t = lenSq > 0.0 ? math.saturate(math.dot(p - a, ab) / lenSq) : 0.0;
            double distSq = math.distancesq(p, a + (ab * t));
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                bestPhase = i + t;
            }
        }

        phase = math.max(phase, bestPhase);
    }
}
