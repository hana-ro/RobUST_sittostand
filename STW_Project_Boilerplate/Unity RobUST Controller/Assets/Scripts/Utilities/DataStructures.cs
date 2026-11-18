using UnityEngine;
using Unity.Mathematics;
using Unity.Profiling;

public static class RobotProfiler
{
    public static readonly ProfilerCategory Workloads = new("Robot Thread Workloads");
    public static readonly ProfilerCategory Intervals = new("Robot Thread Intervals");
}
/// <summary>
/// A simple data structure to hold the position and rotation of a tracker.
/// NOTE: Data is stored in a RIGHT-HANDED coordinate system (OpenVR standard).
/// </summary>
[System.Serializable]
public struct TrackerData
{
    /// <summary>
    /// The full 4x4 homogeneous transformation matrix representing the tracker's pose.
    /// NOTE: This matrix is in the RIGHT-HANDED coordinate system (OpenVR standard).
    /// </summary>
    public Matrix4x4 PoseMatrix;

    public TrackerData(Matrix4x4 poseMatrix)
    {
        PoseMatrix = poseMatrix;
    }
}

/// <summary>
/// A simple data structure to hold force and moment data from a force plate.
/// </summary>
[System.Serializable]
public struct ForcePlateData
{
    public double3 Force;
    public double3 CoP;

    public ForcePlateData(double3 force, double3 cop)
    {
        Force = force;
        CoP = cop;
    }
}

/// <summary>
/// Rigid body state for MPC prediction.
/// </summary>
[System.Serializable]
public struct RBState
{
    public double3 p;   // position [m]
    public double3 th;  // ZYX Euler angles [rad]
    public double3 v;   // linear velocity [m/s]
    public double3 w;   // angular velocity [rad/s]

    public RBState(double3 position, double3 eulerAngles, double3 linearVelocity, double3 angularVelocity)
    {
        p = position;
        th = eulerAngles;
        v = linearVelocity;
        w = angularVelocity;
    }
}


[System.Serializable]
public struct Wrench
{
    public double3 Force;
    public double3 Torque;

    public Wrench(double3 force, double3 torque)
    {
        Force = force;
        Torque = torque;
    }
}

/// <summary>
/// Abstract base class for any high-level controller that outputs cable tensions.
/// Both MPCController and StabilityController derive from this.
/// </summary>
public abstract class BaseController<T>
{
    public abstract T computeNextControl();
}
