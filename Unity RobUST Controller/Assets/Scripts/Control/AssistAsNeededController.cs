using Unity.Mathematics;

/// <summary>
/// Assist-as-needed control for one body segment: a pacer moves along the reference trajectory at
/// the target pace, and the segment is pulled toward it by a force that depends only on the 3D
/// distance d between them:
///   F = 0                                          if d &lt;= InnerRadius
///   F = (d - InnerRadius) / (OuterRadius - InnerRadius) * MaxForce   if InnerRadius &lt; d &lt; OuterRadius
///   F = MaxForce                                   if d &gt;= OuterRadius
/// Because the pacer moves on schedule, this bounds both leaving the path and falling behind or
/// running ahead of the target pace.
/// </summary>
public class AssistAsNeededController : BaseController<Wrench>
{
    /// <summary>Radius of the target sphere around the pacer, inside which there is no force [m]</summary>
    public const double InnerRadius = 0.03;
    /// <summary>Radius beyond which the force stays at its maximum [m]</summary>
    public const double OuterRadius = 0.06;

    private readonly double maxForce;

    // Internal State
    private double3 currentPos;
    private double3 pacerPos;
    private bool isStateValid = false;

    /// <summary>Distance from the pacer as a fraction of InnerRadius: 0 at the pacer, 1 at the inner sphere, 2 at the outer.</summary>
    public double Deviation { get; private set; }

    /// <param name="maxForce">Force reached at OuterRadius and beyond [N]</param>
    public AssistAsNeededController(double maxForce)
    {
        this.maxForce = maxForce;
    }

    /// <summary>
    /// Updates the controller with the latest sensor data and pacer position.
    /// Call this immediately before computeNextControl().
    /// </summary>
    /// <param name="pose_RF">Current pose of this segment in Robot Frame</param>
    /// <param name="pacerPos_RF">Where this segment should be now, in Robot Frame [m]</param>
    public void UpdateState(in double4x4 pose_RF, in double3 pacerPos_RF)
    {
        currentPos = pose_RF.c3.xyz;
        pacerPos = pacerPos_RF;
        isStateValid = true;
    }

    public override Wrench computeNextControl()
    {
        if (!isStateValid) return new Wrench(double3.zero, double3.zero);

        double3 toPacer = pacerPos - currentPos;
        double d = math.length(toPacer);
        Deviation = d / InnerRadius;
        if (d <= InnerRadius) return new Wrench(double3.zero, double3.zero);

        double force = math.saturate((d - InnerRadius) / (OuterRadius - InnerRadius)) * maxForce;
        return new Wrench(toPacer / d * force, double3.zero);
    }
}
