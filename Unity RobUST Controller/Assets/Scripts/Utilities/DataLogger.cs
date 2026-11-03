using UnityEngine;
using Unity.Mathematics;
using System.IO;
using System.Text;
using System;
using System.Globalization;

/// <summary>
/// High-performance logger that uses a pre-allocated array of structs
/// to store data during the hot control loop, preventing GC spikes.
/// Writes to disk only when requested (e.g., on shutdown).
/// </summary>
public class DataLogger
{
    // A Blittable-ish struct to keep memory contiguous.
    // CoM = pelvis (CoM tracker, pelvis cables), EE = trunk (end-effector chest tracker, trunk cables).
    public struct DataFrame
    {
        public double Timestamp;
        public double4x4 ComPoseRF;
        public double4x4 EEPoseRF;
        public ForcePlateData FP1;
        public ForcePlateData FP2;
        public Wrench CoMGoalWrench;
        public Wrench CoMSolverWrench;
        public Wrench CoMMeasuredWrench;
        public RBState CoMGoalState;
        public Wrench EEGoalWrench;
        public Wrench EESolverWrench;
        public Wrench EEMeasuredWrench;
        public RBState EEGoalState;
        public double CoMDeviation;
        public double EEDeviation;
        public double ReferenceProgress;
    }

    private DataFrame[] _buffer;
    private int _cursor = 0;
    private int _maxFrames;
    private string _baseFolder;

    private readonly double _anchorEpochSeconds; // Seconds elapsed from Unix epoch (January 1970)
    private readonly long   _anchorTick;
    private readonly double _tickToSeconds;

    public int FrameCount => _cursor;

    /// <param name="maxMinutes">Maximum duration to buffer in RAM.</param>
    /// <param name="frequency">Control frequency in Hz.</param>
    public DataLogger(int maxMinutes, int frequency)
    {
        // Allocation: 60 sec * Freq * Minutes
        _maxFrames = 60 * frequency * maxMinutes;

        // Eager allocation: Request all memory now.
        // ~400 bytes per frame @ 100Hz = ~2.4MB per minute.
        _buffer = new DataFrame[_maxFrames];
        _cursor = 0;

        _baseFolder = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "ExperimentLogs");
        if (!Directory.Exists(_baseFolder)) Directory.CreateDirectory(_baseFolder);

        // Anchor the wall-clock epoch and the hardware counter at the same instant.
        _anchorEpochSeconds = (DateTime.UtcNow - DateTime.UnixEpoch).TotalSeconds;
        _anchorTick = System.Diagnostics.Stopwatch.GetTimestamp();
        _tickToSeconds = 1.0 / System.Diagnostics.Stopwatch.Frequency;
    }

    /// <summary>
    /// Records a frame into the buffer. This is thread-safe for a single writer.
    /// </summary>
    /// <param name="comGoalWrench">Wrench the controller requested for the CoM (pelvis)</param>
    /// <param name="comSolverWrench">Wrench the commanded CoM cable tensions apply (S * T)</param>
    /// <param name="comMeasuredWrench">Wrench the measured CoM cable tensions apply (S * T)</param>
    /// <param name="comGoalState">CoM target</param>
    /// <param name="eeGoalWrench">Wrench the controller requested for the EE (trunk)</param>
    /// <param name="eeSolverWrench">Wrench the commanded EE cable tensions apply (S * T)</param>
    /// <param name="eeMeasuredWrench">Wrench the measured EE cable tensions apply (S * T)</param>
    /// <param name="eeGoalState">EE target</param>
    /// <param name="comDeviation">CoM distance from its pacer / 30 mm (1 = inner sphere, 2 = outer sphere), also measured when its cables are not driven; -1 outside an assist-as-needed trial or when untracked</param>
    /// <param name="eeDeviation">EE tunnel deviation, same rules as comDeviation</param>
    /// <param name="referenceProgress">Reference progress along the movement, 0 to 1</param>
    public void Log(long timestamp_tick, in double4x4 comPose, in double4x4 eePose,
                    in ForcePlateData fp1, in ForcePlateData fp2,
                    in Wrench comGoalWrench, in Wrench comSolverWrench, in Wrench comMeasuredWrench, in RBState comGoalState,
                    in Wrench eeGoalWrench, in Wrench eeSolverWrench, in Wrench eeMeasuredWrench, in RBState eeGoalState,
                    double comDeviation, double eeDeviation, double referenceProgress)
    {
        if (_cursor >= _maxFrames) return;

        // Direct array access by reference to avoid structure copy
        ref DataFrame frame = ref _buffer[_cursor];

        // Absolute UTC timestamp in seconds since Unix epoch
        frame.Timestamp = _anchorEpochSeconds + ((timestamp_tick - _anchorTick) * _tickToSeconds);
        frame.ComPoseRF = comPose;
        frame.EEPoseRF = eePose;
        frame.FP1 = fp1;
        frame.FP2 = fp2;
        frame.CoMGoalWrench = comGoalWrench;
        frame.CoMSolverWrench = comSolverWrench;
        frame.CoMMeasuredWrench = comMeasuredWrench;
        frame.CoMGoalState = comGoalState;
        frame.EEGoalWrench = eeGoalWrench;
        frame.EESolverWrench = eeSolverWrench;
        frame.EEMeasuredWrench = eeMeasuredWrench;
        frame.EEGoalState = eeGoalState;
        frame.CoMDeviation = comDeviation;
        frame.EEDeviation = eeDeviation;
        frame.ReferenceProgress = referenceProgress;

        _cursor++;
    }

    /// <summary>
    /// Flushes the internal buffer to a CSV file.
    /// Warning: This allocates memory for string building and performs File I/O.
    /// Do not call during the control loop.
    /// </summary>
    public void WriteToDisk(string sessionName)
    {
        if (_cursor == 0) return;

        string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        string fileName = $"{sessionName}_{timestamp}.csv";
        string fullPath = Path.Combine(_baseFolder, fileName);

        Debug.Log($"[DataLogger] Writing {_cursor} frames to {fileName}...");

        try
        {
            using (StreamWriter writer = new StreamWriter(fullPath))
            {
                // CSV Header
                StringBuilder sb = new StringBuilder();
                sb.Append("Timestamp,");

                // Helper to expand headers
                AppendMatrixHeader(sb, "CoM");
                AppendMatrixHeader(sb, "EE");
                sb.Append("FP1_Fx,FP1_Fy,FP1_Fz,FP1_CoPx,FP1_CoPy,FP1_CoPz,");
                sb.Append("FP2_Fx,FP2_Fy,FP2_Fz,FP2_CoPx,FP2_CoPy,FP2_CoPz,");
                sb.Append("CoM_Goal_Fx,CoM_Goal_Fy,CoM_Goal_Fz,CoM_Goal_Tx,CoM_Goal_Ty,CoM_Goal_Tz,");
                sb.Append("CoM_Solver_Fx,CoM_Solver_Fy,CoM_Solver_Fz,CoM_Solver_Tx,CoM_Solver_Ty,CoM_Solver_Tz,");
                sb.Append("CoM_Measured_Fx,CoM_Measured_Fy,CoM_Measured_Fz,CoM_Measured_Tx,CoM_Measured_Ty,CoM_Measured_Tz,");
                // RBState fields: p (position), th (euler), v (linear vel), w (angular vel)
                sb.Append("CoM_Goal_Px,CoM_Goal_Py,CoM_Goal_Pz,CoM_Goal_Ex,CoM_Goal_Ey,CoM_Goal_Ez,CoM_Goal_Vx,CoM_Goal_Vy,CoM_Goal_Vz,CoM_Goal_Wx,CoM_Goal_Wy,CoM_Goal_Wz,");
                sb.Append("EE_Goal_Fx,EE_Goal_Fy,EE_Goal_Fz,EE_Goal_Tx,EE_Goal_Ty,EE_Goal_Tz,");
                sb.Append("EE_Solver_Fx,EE_Solver_Fy,EE_Solver_Fz,EE_Solver_Tx,EE_Solver_Ty,EE_Solver_Tz,");
                sb.Append("EE_Measured_Fx,EE_Measured_Fy,EE_Measured_Fz,EE_Measured_Tx,EE_Measured_Ty,EE_Measured_Tz,");
                sb.Append("EE_Goal_Px,EE_Goal_Py,EE_Goal_Pz,EE_Goal_Ex,EE_Goal_Ey,EE_Goal_Ez,EE_Goal_Vx,EE_Goal_Vy,EE_Goal_Vz,EE_Goal_Wx,EE_Goal_Wy,EE_Goal_Wz,");
                sb.Append("CoM_Deviation,EE_Deviation,Reference_Progress");

                writer.WriteLine(sb.ToString());

                // Write Data
                for (int i = 0; i < _cursor; i++)
                {
                    ref DataFrame f = ref _buffer[i];
                    sb.Clear();

                    // G17 + InvariantCulture guarantees exact IEEE 754 round-trip (no mantissa truncation).
                    sb.Append(f.Timestamp.ToString("G17", CultureInfo.InvariantCulture)).Append(',');

                    AppendMatrix(sb, f.ComPoseRF);
                    AppendMatrix(sb, f.EEPoseRF);

                    AppendVec3(sb, f.FP1.Force); AppendVec3(sb, f.FP1.CoP);
                    AppendVec3(sb, f.FP2.Force); AppendVec3(sb, f.FP2.CoP);

                    AppendVec3(sb, f.CoMGoalWrench.Force); AppendVec3(sb, f.CoMGoalWrench.Torque);
                    AppendVec3(sb, f.CoMSolverWrench.Force); AppendVec3(sb, f.CoMSolverWrench.Torque);
                    AppendVec3(sb, f.CoMMeasuredWrench.Force); AppendVec3(sb, f.CoMMeasuredWrench.Torque);

                    // RBState unpacking
                    AppendVec3(sb, f.CoMGoalState.p);
                    AppendVec3(sb, f.CoMGoalState.th);
                    AppendVec3(sb, f.CoMGoalState.v);
                    AppendVec3(sb, f.CoMGoalState.w);

                    AppendVec3(sb, f.EEGoalWrench.Force); AppendVec3(sb, f.EEGoalWrench.Torque);
                    AppendVec3(sb, f.EESolverWrench.Force); AppendVec3(sb, f.EESolverWrench.Torque);
                    AppendVec3(sb, f.EEMeasuredWrench.Force); AppendVec3(sb, f.EEMeasuredWrench.Torque);
                    AppendVec3(sb, f.EEGoalState.p);
                    AppendVec3(sb, f.EEGoalState.th);
                    AppendVec3(sb, f.EEGoalState.v);
                    AppendVec3(sb, f.EEGoalState.w);

                    sb.Append(f.CoMDeviation).Append(',').Append(f.EEDeviation).Append(',').Append(f.ReferenceProgress);

                    writer.WriteLine(sb.ToString());
                }
            }
            Debug.Log($"[DataLogger] Saved successfully.");
        }
        catch (Exception e)
        {
            Debug.LogError($"[DataLogger] Failed to write log: {e.Message}");
        }
    }

    // Helpers to keep the loop clean
    private void AppendMatrixHeader(StringBuilder sb, string prefix)
    {
        for(int r=0; r<4; r++)
            for(int c=0; c<4; c++)
                sb.Append($"{prefix}_m{r}{c},");
    }

    private void AppendMatrix(StringBuilder sb, in double4x4 m)
    {
        sb.Append(m.c0.x).Append(',').Append(m.c1.x).Append(',').Append(m.c2.x).Append(',').Append(m.c3.x).Append(',');
        sb.Append(m.c0.y).Append(',').Append(m.c1.y).Append(',').Append(m.c2.y).Append(',').Append(m.c3.y).Append(',');
        sb.Append(m.c0.z).Append(',').Append(m.c1.z).Append(',').Append(m.c2.z).Append(',').Append(m.c3.z).Append(',');
        sb.Append(m.c0.w).Append(',').Append(m.c1.w).Append(',').Append(m.c2.w).Append(',').Append(m.c3.w).Append(',');
    }

    private void AppendVec3(StringBuilder sb, in double3 v)
    {
        sb.Append(v.x).Append(',').Append(v.y).Append(',').Append(v.z).Append(',');
    }
}
