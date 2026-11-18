using Unity.Mathematics;
using UnityEngine;

/// <summary>
/// Filters raw robot-frame state data (EMA filter).
/// Frame agnostic - assumes input is already transformed into the desired reference frame.
/// </summary>
public class SensorFilter
{
    // Public filtered state properties
    public double3 CoMLinearVelocity { get; private set; }
    public double3 CoMAngularVelocity { get; private set; }
    public double3 EELinearVelocity { get; private set; }
    public double3 EEAngularVelocity { get; private set; }
    public double3 FilteredGRF { get; private set; }
    public double3 FilteredCoP { get; private set; }

    // Constants for filtering
    private readonly double dt, alpha;
    private double3 comPositionPrev, eePositionPrev;
    private double3x3 R_comPrev, R_eePrev;
    private bool isFirstUpdate = true;

    public SensorFilter(double frequency, double cutoffHz)
    {
        dt = 1.0 / frequency;
        double tau = 1.0 / (2.0 * math.PI * cutoffHz);
        alpha = dt / (tau + dt);
    }

    /// <summary>
    /// Updates the filter with the latest state.
    /// Inputs should be in the same frame (e.g. Robot Frame).
    /// </summary>
    public void Update(double4x4 comPose, double4x4 eePose, double3 rawGRF, double3 rawCoP)
    {
        double3 comPos = comPose.c3.xyz;
        double3x3 R_com = new double3x3(comPose.c0.xyz, comPose.c1.xyz, comPose.c2.xyz);
        double3 eePos = eePose.c3.xyz;
        double3x3 R_ee = new double3x3(eePose.c0.xyz, eePose.c1.xyz, eePose.c2.xyz);

        if (isFirstUpdate)
        {
            comPositionPrev = comPos; R_comPrev = R_com;
            eePositionPrev = eePos; R_eePrev = R_ee;
            FilteredGRF = rawGRF; FilteredCoP = rawCoP;
            
            CoMLinearVelocity = double3.zero; CoMAngularVelocity = double3.zero;
            EELinearVelocity = double3.zero; EEAngularVelocity = double3.zero;
            
            isFirstUpdate = false;
            return;
        }

        // Calculate Velocities (Finite Differencing)
        double3 rawCoMLinVel = (comPos - comPositionPrev) / dt;
        double3x3 R_com_diff = math.mul(R_com, math.transpose(R_comPrev));
        double3 rawCoMAngVel = new double3(R_com_diff.c1.z - R_com_diff.c2.y, R_com_diff.c2.x - R_com_diff.c0.z, R_com_diff.c0.y - R_com_diff.c1.x) / (2.0 * dt);

        double3 rawEELinVel = (eePos - eePositionPrev) / dt;
        double3x3 R_ee_diff = math.mul(R_ee, math.transpose(R_eePrev));
        double3 rawEEAngVel = new double3(R_ee_diff.c1.z - R_ee_diff.c2.y, R_ee_diff.c2.x - R_ee_diff.c0.z, R_ee_diff.c0.y - R_ee_diff.c1.x) / (2.0 * dt);

        // Low Pass Filter (Exponential Moving Average)
        CoMLinearVelocity = math.lerp(CoMLinearVelocity, rawCoMLinVel, alpha);
        CoMAngularVelocity = math.lerp(CoMAngularVelocity, rawCoMAngVel, alpha);
        EELinearVelocity = math.lerp(EELinearVelocity, rawEELinVel, alpha);
        EEAngularVelocity = math.lerp(EEAngularVelocity, rawEEAngVel, alpha);
        FilteredGRF = math.lerp(FilteredGRF, rawGRF, alpha);
        FilteredCoP = math.lerp(FilteredCoP, rawCoP, alpha);

        // Store History
        comPositionPrev = comPos; R_comPrev = R_com;
        eePositionPrev = eePos; R_eePrev = R_ee;
    }
    
    // Legacy support overload if needed
    public void Update(double4x4 comPose, double4x4 eePose, ForcePlateData fpData) 
    {
        Update(comPose, eePose, fpData.Force, fpData.CoP);
    }
}
