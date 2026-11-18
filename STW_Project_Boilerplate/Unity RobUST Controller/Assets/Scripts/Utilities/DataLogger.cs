using UnityEngine;
using Unity.Mathematics;
using System.IO;
using System.Text;
using System;
using System.Globalization;
using System.Collections.Generic;

/// <summary>
/// High-performance logger that uses a pre-allocated array of structs
/// to store data during the hot control loop, preventing GC spikes.
/// Writes to disk only when requested (e.g., on shutdown).
/// </summary>
public class DataLogger
{
    // A Blittable-ish struct to keep memory contiguous.
    // CoM = pelvis (CoM tracker, pelvis cables), EE = trunk (chest tracker, trunk cables).
    public struct DataFrame
    {
        public double Timestamp;
        public RobotController.CONTROL_MODE ControlMode;
        public double4x4 CoMPoseRF;
        public double3 CoMVirtualFrontPositionRF;
        public double3 EEVirtualFrontPositionRF;
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
        public double CoMProgress;
        public double EEProgress;
        public int TrialIndex; // 1-based trial in _trials; 0 before the first trial
    }

    // Captured once per trial on Space. Paths are the full reference paths in the Robot Frame.
    private struct TrialInfo
    {
        public int StartFrame;
        public string CableConfig;
        public double LegLength, ChairHeight, SeatedCoMHeight, SeatedEEHeight, Rise;
        public double3 SeatedCoM, SeatedEE;
        public double3[] CoMPath, EEPath; // null when that trajectory did not load
    }
    private readonly List<TrialInfo> _trials = new List<TrialInfo>();

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
    /// <param name="comDeviation">CoM sagittal distance from its path / 30 mm (1 = inner wall, 2 = outer wall), also measured when its cables are not driven; -1 outside an assist-as-needed trial or when untracked</param>
    /// <param name="eeDeviation">EE tunnel deviation, same rules as comDeviation</param>
    /// <param name="comProgress">CoM progress, 0 to 1: along its path in assist-as-needed, through the timed trajectory in impedance</param>
    /// <param name="eeProgress">EE progress, same rules as comProgress</param>
    public void Log(long timestamp_tick, in double4x4 comPose, in double4x4 eePose,
                    in ForcePlateData fp1, in ForcePlateData fp2,
                    in Wrench comGoalWrench, in Wrench comSolverWrench, in Wrench comMeasuredWrench, in RBState comGoalState,
                    in Wrench eeGoalWrench, in Wrench eeSolverWrench, in Wrench eeMeasuredWrench, in RBState eeGoalState,
                    double comDeviation, double eeDeviation, double comProgress, double eeProgress,
                    in double3 comVirtualFrontPosition, in double3 eeVirtualFrontPosition,
                    RobotController.CONTROL_MODE controlMode)
    {
        if (_cursor >= _maxFrames) return;

        // Direct array access by reference to avoid structure copy
        ref DataFrame frame = ref _buffer[_cursor];

        // Absolute UTC timestamp in seconds since Unix epoch
        frame.Timestamp = _anchorEpochSeconds + ((timestamp_tick - _anchorTick) * _tickToSeconds);
        frame.ControlMode = controlMode;
        frame.CoMPoseRF = comPose;
        frame.CoMVirtualFrontPositionRF = comVirtualFrontPosition;
        frame.EEVirtualFrontPositionRF = eeVirtualFrontPosition;
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
        frame.CoMProgress = comProgress;
        frame.EEProgress = eeProgress;
        frame.TrialIndex = _trials.Count;

        _cursor++;
    }

    /// <summary>
    /// Records a new trial's reference, captured on Space. Frames logged from now on belong to it.
    /// Call outside the hot loop: it copies the paths. Must not run concurrently with Log.
    /// </summary>
    /// <param name="seatedCoMPos">Seated CoM tracking position, the CoM path origin [m, Robot Frame]</param>
    /// <param name="seatedEEPos">Seated EE tracking position, the EE path origin [m, Robot Frame]</param>
    /// <param name="seatedCoMHeight">Seated CoM tracking point above the floor [m]</param>
    /// <param name="seatedEEHeight">Seated EE tracking point above the floor [m]</param>
    /// <param name="rise">Vertical path scale: leg length minus seated CoM height [m]</param>
    public void BeginTrial(string cableConfig, double legLength, double chairHeight,
                           double seatedCoMHeight, double seatedEEHeight, double rise,
                           in double3 seatedCoMPos, in double3 seatedEEPos,
                           double3[] comPath, double3[] eePath)
    {
        _trials.Add(new TrialInfo
        {
            StartFrame = _cursor,
            CableConfig = cableConfig,
            LegLength = legLength,
            ChairHeight = chairHeight,
            SeatedCoMHeight = seatedCoMHeight,
            SeatedEEHeight = seatedEEHeight,
            Rise = rise,
            SeatedCoM = seatedCoMPos,
            SeatedEE = seatedEEPos,
            CoMPath = (double3[])comPath?.Clone(),
            EEPath = (double3[])eePath?.Clone(),
        });
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
                sb.Append("CoM_Deviation,EE_Deviation,CoM_Progress,EE_Progress,CoM_VirtualFront_Px,CoM_VirtualFront_Py,CoM_VirtualFront_Pz,ControlMode");
                sb.Append(",EE_VirtualFront_Px,EE_VirtualFront_Py,EE_VirtualFront_Pz");
                // Per-trial reference: constant over a trial's rows. Path point k sits on the trial's
                // k-th row (Path_Index = k); later rows leave the path columns blank.
                sb.Append(",TrialIndex,Trial_CableConfig,Trial_LegLength,Trial_ChairHeight");
                sb.Append(",Trial_SeatedCoMHeight,Trial_SeatedEEHeight,Trial_Rise");
                sb.Append(",Trial_SeatedCoM_Px,Trial_SeatedCoM_Py,Trial_SeatedCoM_Pz");
                sb.Append(",Trial_SeatedEE_Px,Trial_SeatedEE_Py,Trial_SeatedEE_Pz");
                sb.Append(",Path_Index,CoM_Path_Px,CoM_Path_Py,CoM_Path_Pz,EE_Path_Px,EE_Path_Py,EE_Path_Pz");

                writer.WriteLine(sb.ToString());

                // Write Data
                for (int i = 0; i < _cursor; i++)
                {
                    ref DataFrame f = ref _buffer[i];
                    sb.Clear();

                    // G17 + InvariantCulture guarantees exact IEEE 754 round-trip (no mantissa truncation).
                    sb.Append(f.Timestamp.ToString("G17", CultureInfo.InvariantCulture)).Append(',');

                    AppendMatrix(sb, f.CoMPoseRF);
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

                    AppendNum(sb, f.CoMDeviation).Append(',');
                    AppendNum(sb, f.EEDeviation).Append(',');
                    AppendNum(sb, f.CoMProgress).Append(',');
                    AppendNum(sb, f.EEProgress);
                    sb.Append(',').Append(f.CoMVirtualFrontPositionRF.x.ToString("G17", CultureInfo.InvariantCulture));
                    sb.Append(',').Append(f.CoMVirtualFrontPositionRF.y.ToString("G17", CultureInfo.InvariantCulture));
                    sb.Append(',').Append(f.CoMVirtualFrontPositionRF.z.ToString("G17", CultureInfo.InvariantCulture));
                    sb.Append(',').Append(f.ControlMode.ToString());
                    sb.Append(',').Append(f.EEVirtualFrontPositionRF.x.ToString("G17", CultureInfo.InvariantCulture));
                    sb.Append(',').Append(f.EEVirtualFrontPositionRF.y.ToString("G17", CultureInfo.InvariantCulture));
                    sb.Append(',').Append(f.EEVirtualFrontPositionRF.z.ToString("G17", CultureInfo.InvariantCulture));
                    AppendTrial(sb, f.TrialIndex, i);

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

    private const int TrialColumnCount = 13; // TrialIndex through Trial_SeatedEE_Pz
    private const int PathColumnCount = 7;   // Path_Index and the six path coordinates

    // Trial columns, each preceded by a comma, ending the row
    private void AppendTrial(StringBuilder sb, int trialIndex, int frame)
    {
        sb.Append(',').Append(trialIndex);
        if (trialIndex <= 0 || trialIndex > _trials.Count)
        {
            sb.Append(',', TrialColumnCount - 1 + PathColumnCount);
            return;
        }
        TrialInfo t = _trials[trialIndex - 1];
        sb.Append(',').Append(t.CableConfig);
        sb.Append(','); AppendNum(sb, t.LegLength);
        sb.Append(','); AppendNum(sb, t.ChairHeight);
        sb.Append(','); AppendNum(sb, t.SeatedCoMHeight);
        sb.Append(','); AppendNum(sb, t.SeatedEEHeight);
        sb.Append(','); AppendNum(sb, t.Rise);
        AppendPoint(sb, t.SeatedCoM, true);
        AppendPoint(sb, t.SeatedEE, true);

        int k = frame - t.StartFrame;
        bool hasCoM = t.CoMPath != null && k < t.CoMPath.Length;
        bool hasEE = t.EEPath != null && k < t.EEPath.Length;
        sb.Append(',');
        if (hasCoM || hasEE) sb.Append(k);
        AppendPoint(sb, hasCoM ? t.CoMPath[k] : default, hasCoM);
        AppendPoint(sb, hasEE ? t.EEPath[k] : default, hasEE);
    }

    // ",x,y,z", or ",,," when absent
    private void AppendPoint(StringBuilder sb, in double3 p, bool present)
    {
        if (!present) { sb.Append(',', 3); return; }
        sb.Append(','); AppendNum(sb, p.x);
        sb.Append(','); AppendNum(sb, p.y);
        sb.Append(','); AppendNum(sb, p.z);
    }

    // Helpers to keep the loop clean
    private void AppendMatrixHeader(StringBuilder sb, string prefix)
    {
        for(int r=0; r<4; r++)
            for(int c=0; c<4; c++)
                sb.Append($"{prefix}_m{r}{c},");
    }

    // Always '.' as the decimal separator, whatever the PC's regional settings,
    // so a value like 0.5 never becomes "0,5" and splits into two CSV columns.
    private static StringBuilder AppendNum(StringBuilder sb, double value)
    {
        return sb.Append(value.ToString(CultureInfo.InvariantCulture));
    }

    private void AppendMatrix(StringBuilder sb, in double4x4 m)
    {
        // Row-major: each row is c0..c3 at that component.
        AppendRow(sb, m.c0.x, m.c1.x, m.c2.x, m.c3.x);
        AppendRow(sb, m.c0.y, m.c1.y, m.c2.y, m.c3.y);
        AppendRow(sb, m.c0.z, m.c1.z, m.c2.z, m.c3.z);
        AppendRow(sb, m.c0.w, m.c1.w, m.c2.w, m.c3.w);
    }

    private void AppendRow(StringBuilder sb, double a, double b, double c, double d)
    {
        AppendNum(sb, a).Append(','); AppendNum(sb, b).Append(',');
        AppendNum(sb, c).Append(','); AppendNum(sb, d).Append(',');
    }

    private void AppendVec3(StringBuilder sb, in double3 v)
    {
        AppendNum(sb, v.x).Append(','); AppendNum(sb, v.y).Append(','); AppendNum(sb, v.z).Append(',');
    }
}
