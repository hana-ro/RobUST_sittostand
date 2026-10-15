using UnityEngine;
using Unity.Profiling;

using System.Threading;
using System;
using Valve.VR;

/// <summary>
/// Manages Vive trackers using direct OpenVR API with dedicated threading.
/// Provides high-frequency (90Hz) tracker data independent of Unity's framerate.
/// Bypasses Unity transforms for maximum performance and background thread safety.
/// </summary>
public class TrackerManager : MonoBehaviour
{
    static readonly ProfilerCounterValue<long> s_WorkloadNs = new(RobotProfiler.Workloads, "Tracker Manager Workload", ProfilerMarkerDataUnit.TimeNanoseconds);
    static readonly ProfilerCounterValue<long> s_IntervalNs = new(RobotProfiler.Intervals, "Tracker Manager Interval", ProfilerMarkerDataUnit.TimeNanoseconds);

    [Header("Tracker Serial Numbers")]
    [Tooltip("Serial number for the Center of Mass (CoM) tracker.")]
    public string comTrackerSerial = "LHR-FFFFFFFF"; 

    [Tooltip("Serial number for the End-Effector tracker.")]
    public string endEffectorSerial = "LHR-FFFFFFFF";

    [Tooltip("Serial number for the Frame tracker, which defines the world origin.")]
    public string frameTrackerSerial = "LHR-FFFFFFFF";

    // OpenVR system and poses
    private CVRSystem vrSystem;
    private TrackedDevicePose_t[] trackedDevicePoses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
    
    // Indices for the trackers
    private uint comTrackerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    private uint endEffectorIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
    private uint frameTrackerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;

    // Thread-safe data storage
    private readonly object dataLock = new object();
    private TrackerData comTrackerData = new TrackerData();
    private TrackerData endEffectorData = new TrackerData();
    private TrackerData frameTrackerData = new TrackerData();

    // High-frequency update thread
    private Thread trackingThread;
    private volatile bool isRunning = false;

    /// <summary>
    /// Initializes the TrackerManager, setting up OpenVR and starting the tracking thread.
    /// </summary>
    /// <returns> True if initialization was successful, false otherwise.</returns>
    public bool Initialize()
    {
        try
        {
            // Initialize OpenVR in a way that doesn't require a headset
            EVRInitError eVRInitError = EVRInitError.None;
            vrSystem = OpenVR.Init(ref eVRInitError, EVRApplicationType.VRApplication_Background);
            
            if (eVRInitError != EVRInitError.None)
            {
                Debug.LogError($"OpenVR initialization failed: {eVRInitError}. Ensure SteamVR is running.");
                return false;
            }

            // Find all trackers in a single scan
            if (!FindAllTrackers()) return false;

            // Start the high-frequency tracking thread
            isRunning = true;
            trackingThread = new Thread(TrackingLoop)
            {
                IsBackground = true,
                Name = "Tracker Manager Thread",
                Priority = System.Threading.ThreadPriority.AboveNormal
            };
            trackingThread.Start();

            Debug.Log("TrackerManager initialized - Direct OpenVR tracking at 90Hz");
            return true;
        }
        catch (Exception e)
        {
            Debug.LogError($"TrackerManager initialization failed: {e.Message}");
            return false;
        }
    }

    /// <summary>
    /// Efficiently finds all required trackers in a single device scan.
    /// </summary>
    private bool FindAllTrackers()
    {
        comTrackerIndex = endEffectorIndex = frameTrackerIndex = OpenVR.k_unTrackedDeviceIndexInvalid;
        var buffer = new System.Text.StringBuilder((int)OpenVR.k_unMaxPropertyStringSize);
        
        for (uint deviceId = 0; deviceId < OpenVR.k_unMaxTrackedDeviceCount; deviceId++)
        {
            if (vrSystem.GetTrackedDeviceClass(deviceId) != ETrackedDeviceClass.GenericTracker)
                continue;

            var error = ETrackedPropertyError.TrackedProp_Success;
            vrSystem.GetStringTrackedDeviceProperty(deviceId, ETrackedDeviceProperty.Prop_SerialNumber_String, buffer, OpenVR.k_unMaxPropertyStringSize, ref error);

            if (error != ETrackedPropertyError.TrackedProp_Success)
                continue;

            string serial = buffer.ToString();
            Debug.Log($"Discovered tracker - Device: {deviceId}, Serial: {serial}");

            if (string.Equals(serial, comTrackerSerial, StringComparison.OrdinalIgnoreCase))
            {
                comTrackerIndex = deviceId;
                Debug.Log($"--> Matched CoM tracker: {serial}");
            }
            else if (string.Equals(serial, endEffectorSerial, StringComparison.OrdinalIgnoreCase))
            {
                endEffectorIndex = deviceId;
                Debug.Log($"--> Matched End-Effector tracker: {serial}");
            }
            else if (string.Equals(serial, frameTrackerSerial, StringComparison.OrdinalIgnoreCase))
            {
                frameTrackerIndex = deviceId;
                Debug.Log($"--> Matched Frame tracker: {serial}");
            }
        }

        // Validate all trackers were found
        bool valid = true;
        if (comTrackerIndex == OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            Debug.LogError($"CoM tracker '{comTrackerSerial}' not found.", this);
            valid = false;
        }
        if (endEffectorIndex == OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            Debug.LogError($"End-effector tracker '{endEffectorSerial}' not found.", this);
            valid = false;
        }
        if (frameTrackerIndex == OpenVR.k_unTrackedDeviceIndexInvalid)
        {
            Debug.LogError($"Frame tracker '{frameTrackerSerial}' not found.", this);
            valid = false;
        }

        return valid;
    }

    /// <summary>
    /// High-frequency tracking loop running in dedicated background thread. Currently execution time roughly 0.07 ms, total time roughly 10.7ms
    /// </summary>
    private void TrackingLoop()
    {
        double frequency = System.Diagnostics.Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / frequency;
        long intervalTicks = (long)(frequency / 90.0);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + intervalTicks;
        long lastLoopTick = System.Diagnostics.Stopwatch.GetTimestamp();

        while (isRunning)
        {
            long loopStartTick = System.Diagnostics.Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            vrSystem.GetDeviceToAbsoluteTrackingPose(ETrackingUniverseOrigin.TrackingUniverseRawAndUncalibrated, 0, trackedDevicePoses);

            lock (dataLock)
            {
                if (comTrackerIndex != OpenVR.k_unTrackedDeviceIndexInvalid && trackedDevicePoses[comTrackerIndex].bPoseIsValid)
                    UpdateMatrixFromOpenVR(trackedDevicePoses[comTrackerIndex].mDeviceToAbsoluteTracking, ref comTrackerData.PoseMatrix);
                if (endEffectorIndex != OpenVR.k_unTrackedDeviceIndexInvalid && trackedDevicePoses[endEffectorIndex].bPoseIsValid)
                    UpdateMatrixFromOpenVR(trackedDevicePoses[endEffectorIndex].mDeviceToAbsoluteTracking, ref endEffectorData.PoseMatrix);
                if (frameTrackerIndex != OpenVR.k_unTrackedDeviceIndexInvalid && trackedDevicePoses[frameTrackerIndex].bPoseIsValid)
                    UpdateMatrixFromOpenVR(trackedDevicePoses[frameTrackerIndex].mDeviceToAbsoluteTracking, ref frameTrackerData.PoseMatrix);
            }

            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
            
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime)
            {
                // (BURN CPU cycles to hold timing precision)
                // prevents OS scheduler from yielding the thread 
            }
            nextTargetTime += intervalTicks;
            
            // If we are late (processing took > 11.1ms), reset the pacer
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime)
                nextTargetTime = now + intervalTicks;
        }
    }

    public void GetCoMTrackerData(out TrackerData data)
    {
        lock (dataLock)
        {
            data = comTrackerData;
        }
    }

    public void GetEndEffectorTrackerData(out TrackerData data)
    {
        lock (dataLock)
        {
            data = endEffectorData;
        }
    }

    public void GetFrameTrackerData(out TrackerData data)
    {
        lock (dataLock)
        {
            data = frameTrackerData;
        }
    }

    /// <summary>
    /// Converts raw OpenVR HmdMatrix34_t directly into an existing Matrix4x4.
    /// Applies coordinate system conversion from OpenVR (Y-up) to HTC Guidelines (Z-up).
    /// Runtime-optimized for high-frequency calls.
    /// </summary>
    /// <param name="openVRMatrix">Raw OpenVR matrix from tracker</param>
    /// <param name="targetMatrix">Pre-allocated Matrix4x4 to update in-place</param>
    private static void UpdateMatrixFromOpenVR(HmdMatrix34_t openVRMatrix, ref Matrix4x4 targetMatrix)
    {
        // Row 0
        targetMatrix.m00 = -openVRMatrix.m0;  targetMatrix.m01 = -openVRMatrix.m2;  targetMatrix.m02 = -openVRMatrix.m1;  targetMatrix.m03 = openVRMatrix.m3;
        // Row 1
        targetMatrix.m10 = -openVRMatrix.m4;  targetMatrix.m11 = -openVRMatrix.m6;  targetMatrix.m12 = -openVRMatrix.m5;  targetMatrix.m13 = openVRMatrix.m7;
        // Row 2
        targetMatrix.m20 = -openVRMatrix.m8;  targetMatrix.m21 = -openVRMatrix.m10;  targetMatrix.m22 = -openVRMatrix.m9; targetMatrix.m23 = openVRMatrix.m11;
        targetMatrix.m33 = 1f; // enforce homogeneous row
    }

    private void OnDestroy()
    {
        isRunning = false;
        if (trackingThread != null && trackingThread.IsAlive)
        {
            trackingThread.Join();
        }
        OpenVR.Shutdown();
    }
}

