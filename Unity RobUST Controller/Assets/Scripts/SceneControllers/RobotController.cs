using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Mathematics;
using Unity.Profiling;

using System;
using System.Threading;

/// <summary>
/// Main controller for the cable-driven robot.
/// This class centralizes the robot's state management, coordinating trackers,
/// force plates, physics calculations, and communication with LabVIEW.
/// </summary>
public class RobotController : MonoBehaviour
{
    static readonly ProfilerCounterValue<long> s_WorkloadNs = new(RobotProfiler.Workloads, "Controller Workload", ProfilerMarkerDataUnit.TimeNanoseconds);
    static readonly ProfilerCounterValue<long> s_IntervalNs = new(RobotProfiler.Intervals, "Controller Execution Interval", ProfilerMarkerDataUnit.TimeNanoseconds);

    [Header("Module References")]
    [Tooltip("The TrackerManager instance that provides tracker data.")]
    [SerializeField] private TrackerManager trackerManager;
    [Tooltip("The ForcePlateManager instance for reading force plate data.")]
    [SerializeField] private ForcePlateManager forcePlateManager;
    [Tooltip("The LabviewTcpCommunicator instance for sending motor commands.")]
    [SerializeField] private LabviewTcpCommunicator tcpCommunicator;

    [Header("Visualization")]
    [Tooltip("Handles tracker/camera visuals. Keeps RobotController logic-only.")]
    [SerializeField] private RobotVisualizer visualizer;

    [Header("Control Settings")]
    [Tooltip("Enable or disable sending commands to LabVIEW.")]
    [SerializeField] private bool isLabviewControlEnabled = true;
    private bool isForcePlateEnabled = true;

    public enum CONTROL_MODE { OFF, TRANSPARENT, IMPEDANCE, ASSIST_AS_NEEDED }
    [SerializeField] private volatile CONTROL_MODE currentControlMode = CONTROL_MODE.OFF;

    [Header("Robot Geometry Configuration")]
    [Tooltip("Number of cables in the system. We currently support 8 or 4 cables")]
    [SerializeField] private int numCables = 8;
    [Tooltip("Measured thickness of the pelvis in the anterior-posterior direction [m].")]
    [SerializeField] private float pelvisAPDistance = 0.2f;
    [Tooltip("Measured width of the pelvis in the medial-lateral direction [m].")]
    [SerializeField] private float pelvisMLDistance = 0.3f;
    [Tooltip("Measured thickness of the trunk in the anterior-posterior direction [m].")]
    [SerializeField] private float trunkAPDistance = 0.20f;
    [Tooltip("Measured width of the trunk/chest in the medial-lateral direction [m].")]
    [SerializeField] private float trunkMLDistance = 0.28f;
    [Tooltip("User body mass [kg].")]
    [SerializeField] private float userMass = 70.0f;

    [Header("Sit-to-Walk Trajectories")]
    [Tooltip("Filename of the Pelvis GPR trajectory CSV located in Assets/Scripts.")]
    [SerializeField] private string pelvisTrajectoryCsvName = "SitToWalk_GPR_Pelvis_NoVR_normalized.csv";
    [Tooltip("Filename of the Trunk GPR trajectory CSV located in Assets/Scripts.")]
    [SerializeField] private string trunkTrajectoryCsvName = "SitToWalk_GPR_Trunk_NoVR_normalized.csv";
    [Tooltip("Target duration of the movement [s]. IMPEDANCE plays the target over this time. ASSIST_AS_NEEDED trains to this pace once the pelvis starts moving. NoVR subjects averaged 0.79 s.")]
    [SerializeField] private double stsDuration = 2.0;
    [Tooltip("Subject leg length: pelvis tracker to floor while standing [m]. Scales AP/ML and, with the seated pelvis height, sets the vertical rise.")]
    [SerializeField] private double userLegLength = 1.07;

    [Header("Assist-as-Needed Force Tunnels (Pelvis and Trunk)")]
    [Tooltip("Tunnel half-width in SDs of the GPR reference.")]
    [SerializeField] private double tunnelWidthSD = 2.0;
    [Tooltip("Smallest allowed tunnel half-width on any axis [m].")]
    [SerializeField] private double tunnelMinHalfWidth = 0.02;
    [Tooltip("Allowed time ahead of or behind the target pace before force is applied along the path [s]. 2 SD of NoVR movement duration.")]
    [SerializeField] private double tunnelTimingBand = 0.26;
    [Tooltip("Pelvis tunnel force limit [N].")]
    [SerializeField] private double tunnelMaxForce = 100.0;
    [Tooltip("Trunk tunnel force limit [N]. Lower than the pelvis until the four trunk cables are tested.")]
    [SerializeField] private double trunkTunnelMaxForce = 50.0;
    [Tooltip("Damping against moving further out of the tunnel [N·s/m].")]
    [SerializeField] private double tunnelDamping = 100.0;

    public enum ASSISTANCE_TARGET { PELVIS_ONLY, TRUNK_ONLY, COMBINED }
    [Header("Assistance Strategy")]
    [Tooltip("Which bodies IMPEDANCE and ASSIST_AS_NEEDED assist. Each body is pushed by its own cables. A change takes effect when the next trial starts.")]
    [SerializeField] private volatile ASSISTANCE_TARGET assistanceTarget = ASSISTANCE_TARGET.PELVIS_ONLY;
    private volatile ASSISTANCE_TARGET trialAssistanceTarget; // assistanceTarget as of the current trial's start

    // Trajectory buffers (Zero-allocation during runtime)
    // Robot-frame fractions: x/y of leg length, z of rise height. Velocities are per unit tau.
    private double3[] pelvisTrajDisplacements;
    private double3[] pelvisTrajVelocities;
    private double3[] pelvisTrajSD;
    private double3[] trunkTrajDisplacements;
    private double3[] trunkTrajVelocities;
    private double3[] trunkTrajSD;

    private const double MinRiseHeight = 0.1; // below this the subject is not seated [m]
    private const double MaxRestartDrift = 0.10; // allowed distance from the last seated position on restart [m]
    private const double MockSwayAmplitude = 0.10; // [m]
    private const double MockSlowdown = 1.25; // virtual subject runs late, to show the timing band
    private const double ControlFrequency = 100.0; // [Hz]
    private const int TrajectoryHorizonSteps = 10;
    private const double TunnelPreviewStep = 0.05; // spacing of the path preview, in tau

    private double3 seatedPelvisPos_RF = double3.zero;
    private double3 seatedTrunkPos_RF = double3.zero;
    private volatile bool isSeatedPoseValid = false;
    private double riseHeight = MinRiseHeight; // standing minus seated pelvis height [m], set when a trajectory starts
    private double trajectoryElapsedTime = 0.0;
    private double tunnelProgress = 0.0;
    private volatile bool trajectoryTriggered = false;
    private volatile bool isTrajectoryPaused = false;
    private volatile bool isReplayPending = false; // reset to TRANSPARENT, and Space returns to replayControlMode
    private CONTROL_MODE replayControlMode = CONTROL_MODE.IMPEDANCE;
    private bool isTrackerMockMode = false;
    private double4x4 mockEEPose_RF;
    private double4x4 mockComPose_RF;

    [Header("Data Logging")]
    [Tooltip("Check this to record data to RAM. It will be saved to disk on Stop.")]
    [SerializeField] private volatile bool isLogging = false;
    [Tooltip("Name of the experiment session for the log file.")]
    [SerializeField] private string sessionName = "Experiment";

    private RobUSTDescription robotDescription;
    private DataLogger dataLogger;

    // Controllers
    private ImpedanceController pelvisImpedanceController;
    private ImpedanceController trunkImpedanceController;
    private AssistAsNeededController pelvisAssistController;
    private AssistAsNeededController trunkAssistController;
    private readonly ReferenceClock tunnelClock = new ReferenceClock();
    private CableTensionPlanner pelvisTensionPlanner;
    private CableTensionPlanner trunkTensionPlanner; // null until the trunk cables are configured

    // Static frame reference captured only at startup to prevent drift
    private TrackerData robot_frame_tracker;
    private Thread controllerThread;
    private volatile bool isRunning = false;
    private volatile bool isTrajectoryActive = false;

    private void Start()
    {
        using (System.Diagnostics.Process p = System.Diagnostics.Process.GetCurrentProcess())
        {
            p.PriorityClass = System.Diagnostics.ProcessPriorityClass.High;
        }

        if (!ValidateModules())
        {
            enabled = false; 
            return;
        }

        if (pelvisAPDistance <= 0 || pelvisMLDistance <= 0 || trunkAPDistance <= 0 || trunkMLDistance <= 0)
        {
            Debug.LogError("Pelvis and trunk dimensions must be positive values.", this);
            enabled = false;
            return;
        }

        // Create RobUST description (single allocation at startup)
        robotDescription = RobUSTDescription.Create(numCables, pelvisAPDistance, pelvisMLDistance, trunkAPDistance, trunkMLDistance, userMass);
        Debug.Log($"RobUST description created: {numCables} cables, Pelvis AP={pelvisAPDistance}m ML={pelvisMLDistance}m, Trunk AP={trunkAPDistance}m ML={trunkMLDistance}m");

        // Initialize Modules
        tcpCommunicator.Initialize();
        if (!visualizer.Initialize(robotDescription))
        {
            Debug.LogError("Failed to initialize RobotVisualizer.", this);
            enabled = false;
            return;
        }
        if (!trackerManager.Initialize())
        {
            Debug.LogWarning("TrackerManager failed to initialize (SteamVR / Trackers offline). Operating in virtual simulation mode.", this);
            isTrackerMockMode = true;
        }
        if (!forcePlateManager.Initialize(robotDescription))
        {
            Debug.LogWarning("Failed to initialize ForcePlateManager.", this);
            isForcePlateEnabled = false;
        }
        
        if (isLabviewControlEnabled)
        {
            tcpCommunicator.ConnectToServer();
            tcpCommunicator.SetClosedLoopControl();
        }
        dataLogger = new DataLogger(60, 100);

        if (!isTrackerMockMode)
        {
            System.Threading.Thread.Sleep(500); // allow tracker thread to go live
            trackerManager.GetFrameTrackerData(out robot_frame_tracker); // import Capture static frame at startup
        }
        else
        {
            robot_frame_tracker = new TrackerData { PoseMatrix = Matrix4x4.identity };
        }

        // Finally Initialize Controller modules and begin control thread
        pelvisImpedanceController = new ImpedanceController(userMass);
        trunkImpedanceController = new ImpedanceController(userMass);
        pelvisTensionPlanner = CableTensionPlanner.ForPelvis(robotDescription);
        if (robotDescription.HasTrunkCables)
            trunkTensionPlanner = CableTensionPlanner.ForTrunk(robotDescription);
        else
            Debug.LogWarning("[RobotController] Trunk cable pulley positions and motor numbers are not entered in RobUSTDescription. Trunk assistance is computed and logged but not sent to any motor.");
        mockEEPose_RF = MakePose(new double3(0.2, 0.825, 0.01), quaternion.RotateZ(-math.PI / 2));
        mockComPose_RF = MakePose(new double3(0.2, 0.825, -0.14), quaternion.identity);
        LoadTrajectories();
        if (pelvisTrajDisplacements != null && pelvisTrajSD != null)
        {
            pelvisAssistController = new AssistAsNeededController(userMass, pelvisTrajDisplacements, pelvisTrajSD,
                tunnelWidthSD, tunnelMinHalfWidth, tunnelTimingBand, tunnelMaxForce, tunnelDamping, tunnelClock);
            if (trunkTrajSD != null)
            {
                trunkAssistController = new AssistAsNeededController(userMass, trunkTrajDisplacements, trunkTrajSD,
                    tunnelWidthSD, tunnelMinHalfWidth, tunnelTimingBand, trunkTunnelMaxForce, tunnelDamping, tunnelClock);
            }
        }
        else if (currentControlMode == CONTROL_MODE.ASSIST_AS_NEEDED)
        {
            currentControlMode = CONTROL_MODE.OFF; // the Inspector can start here before the tunnel exists
        }

        trialAssistanceTarget = assistanceTarget;

        controllerThread = new Thread(controlLoop)
        {
            Name = "Robot Controller Main",
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.AboveNormal
        };

        isRunning = true;
        controllerThread.Start();
    }

    private void Update()
    {
        if (Keyboard.current == null) return;

        if (Keyboard.current.iKey.wasPressedThisFrame)
        {
            SetControlMode(CONTROL_MODE.IMPEDANCE);
        }

        if (Keyboard.current.aKey.wasPressedThisFrame)
        {
            if (pelvisAssistController != null) SetControlMode(CONTROL_MODE.ASSIST_AS_NEEDED);
            else Debug.LogWarning("[RobotController] Assist-as-needed unavailable: the pelvis trajectory or its SD columns did not load.");
        }

        if (Keyboard.current.spaceKey.wasPressedThisFrame)
        {
            TriggerOrToggleTrajectory();
        }

        if (Keyboard.current.rKey.wasPressedThisFrame)
        {
            ResetTrajectory();
        }

        if (Keyboard.current.oKey.wasPressedThisFrame)
        {
            SetControlMode(CONTROL_MODE.OFF);
        }

        if (Keyboard.current.tKey.wasPressedThisFrame)
        {
            SetControlMode(CONTROL_MODE.TRANSPARENT);
        }
    }

    // Changing mode ends any trial. The seated pose is dropped so that no old target is restored.
    private void SetControlMode(CONTROL_MODE mode)
    {
        if (mode == currentControlMode) return;

        isTrajectoryActive = false;
        isTrajectoryPaused = false;
        trajectoryTriggered = false;
        isReplayPending = false;
        isSeatedPoseValid = false;
        trajectoryElapsedTime = 0.0;
        tunnelProgress = 0.0;
        currentControlMode = mode;
    }

    private void StartTrajectory()
    {
        trajectoryElapsedTime = 0.0;
        tunnelProgress = 0.0;
        isTrajectoryPaused = false;
        trajectoryTriggered = true;
        isTrajectoryActive = true;
    }

    public void TriggerOrToggleTrajectory()
    {
        if (currentControlMode == CONTROL_MODE.TRANSPARENT && isReplayPending)
        {
            isReplayPending = false;
            StartTrajectory();
            currentControlMode = replayControlMode;
            Debug.Log($"[RobotController] Sit-to-Walk Trajectory Replayed in {replayControlMode} mode.");
        }
        else if (currentControlMode != CONTROL_MODE.IMPEDANCE && currentControlMode != CONTROL_MODE.ASSIST_AS_NEEDED)
        {
            Debug.LogWarning($"[RobotController] Spacebar pressed, but robot is currently in {currentControlMode} mode. Press 'I' for IMPEDANCE or 'A' for ASSIST_AS_NEEDED first.");
        }
        else if (!isTrajectoryActive)
        {
            StartTrajectory();
            Debug.Log($"[RobotController] Sit-to-Walk Trajectory Started in {currentControlMode} mode.");
        }
        else if (currentControlMode == CONTROL_MODE.IMPEDANCE && trajectoryElapsedTime < stsDuration)
        {
            isTrajectoryPaused = !isTrajectoryPaused;
            Debug.Log($"[RobotController] Sit-to-Walk Trajectory {(isTrajectoryPaused ? "Paused" : "Resumed")}.");
        }
        else
        {
            Debug.LogWarning("[RobotController] Trajectory is already playing or finished. Press 'R' to reset first.");
        }
    }

    /// <summary>
    /// Ends the trial by going to TRANSPARENT, so the subject can sit down with no force on them.
    /// The target bodies return to the seated position, and Space replays in the previous mode.
    /// </summary>
    public void ResetTrajectory()
    {
        if (!isTrajectoryActive || (currentControlMode != CONTROL_MODE.IMPEDANCE && currentControlMode != CONTROL_MODE.ASSIST_AS_NEEDED))
        {
            return;
        }

        ReleaseToTransparent();
        Debug.Log("[RobotController] Reset to TRANSPARENT. Sit down, then press Space to play again.");
    }

    // Goes to TRANSPARENT with the replay still available. The trial state is cleared on the control thread.
    private void ReleaseToTransparent()
    {
        replayControlMode = currentControlMode;
        isReplayPending = true;
        currentControlMode = CONTROL_MODE.TRANSPARENT;
    }

    // Captures the seated pose for a new trial, or cancels the trial if the subject is not seated.
    private bool TryBeginTrial(in double3 pelvisPos, in double3 trunkPos)
    {
        trajectoryTriggered = false;

        double seatedPelvisHeight = pelvisPos.z - robotDescription.FloorZ;
        double rise = userLegLength - seatedPelvisHeight;
        bool hasDrifted = isSeatedPoseValid && math.distance(pelvisPos, seatedPelvisPos_RF) > MaxRestartDrift;
        if (rise < MinRiseHeight || hasDrifted)
        {
            Debug.LogWarning($"[RobotController] Trial not started: subject is not seated (pelvis height {seatedPelvisHeight:F3}m, leg length {userLegLength:F3}m).");
            isTrajectoryActive = false;
            // A hold at the old seated pose would pull on a subject who is still up
            if (isSeatedPoseValid) ReleaseToTransparent();
            return false;
        }

        seatedPelvisPos_RF = pelvisPos;
        seatedTrunkPos_RF = trunkPos;
        isSeatedPoseValid = true;
        riseHeight = rise;

        // Fixed for the whole trial, so an Inspector change cannot switch body mid-movement
        trialAssistanceTarget = assistanceTarget;
        if (trialAssistanceTarget != ASSISTANCE_TARGET.PELVIS_ONLY)
        {
            if (trunkTensionPlanner == null)
                Debug.LogWarning("[RobotController] Trunk cables are not configured, so trunk force is computed and logged but not sent to any motor.");
            if (trunkAssistController == null && currentControlMode == CONTROL_MODE.ASSIST_AS_NEEDED)
                Debug.LogWarning("[RobotController] Trunk tunnel unavailable: the trunk trajectory or its SD columns did not load.");
        }
        //Debug.Log($"[RobotController] Seated pelvis height {seatedPelvisHeight:F3}m, leg length {userLegLength:F3}m, rise height {riseHeight:F3}m.");
        return true;
    }

    // Holds both targets at the seated pose, or at the live pose before a trial has captured one.
    private void FillHoldHorizon(Span<RBState> pelvisHorizon, Span<RBState> trunkHorizon, in double3 livePelvisPos, in double3 liveTrunkPos)
    {
        double3 holdPelvisPos = isSeatedPoseValid ? seatedPelvisPos_RF : livePelvisPos;
        double3 holdTrunkPos = isSeatedPoseValid ? seatedTrunkPos_RF : liveTrunkPos;
        pelvisHorizon.Fill(new RBState(holdPelvisPos, new double3(0, 0, -math.PI / 2), double3.zero, double3.zero));
        trunkHorizon.Fill(new RBState(holdTrunkPos, new double3(0, 0, -math.PI / 2), double3.zero, double3.zero));
    }

    // Virtual subject for checking the tunnels without trackers: follows both paths late while swaying
    // sideways through the walls, so the targets sweep through their full colour range.
    private void MakeMockTunnelPoses(out double4x4 pelvisPose, out double4x4 trunkPose)
    {
        double mockTime = trajectoryElapsedTime / MockSlowdown;
        double tau = math.saturate(mockTime / math.max(0.1, stsDuration));
        double sway = MockSwayAmplitude * math.sin(4.0 * math.PI * tau);
        double3 pelvis = EvaluateTrajectory(pelvisTrajDisplacements, pelvisTrajVelocities, mockTime, seatedPelvisPos_RF).p;
        double3 trunk = EvaluateTrajectory(trunkTrajDisplacements, trunkTrajVelocities, mockTime, seatedTrunkPos_RF).p;
        pelvis.y += sway;
        trunk.y += sway;
        pelvisPose = MakePose(pelvis, quaternion.identity);
        trunkPose = MakePose(trunk, quaternion.RotateZ(-math.PI / 2));
    }

    // Sends a wrench through the pelvis cables.
    private void CommandPelvisCables(in double4x4 comPose_RF, Wrench wrench, ref double[] tensions, Span<double> motorCommand)
    {
        if (robotDescription.NumCables == 4 && wrench.Force.z < 0.0) wrench.Force.z = 0.0;
        tensions = pelvisTensionPlanner.CalculateTensions(comPose_RF, wrench);
        MapTensionsToMotors(tensions, robotDescription.SolverToMotorMap, motorCommand);
    }

    // Sends a wrench through the trunk cables. A zero wrench settles them at minimum tension.
    // Does nothing until the trunk cables are configured, so no trunk motor is ever guessed.
    private void CommandTrunkCables(in double4x4 eePose_RF, Wrench wrench, ref double[] tensions, Span<double> motorCommand)
    {
        if (trunkTensionPlanner == null) return;
        tensions = trunkTensionPlanner.CalculateTensions(eePose_RF, wrench);
        MapTensionsToMotors(tensions, robotDescription.TrunkSolverToMotorMap, motorCommand);
    }

    /// <summary>
    ///  This control loop is the main "driver" of one or more `Controller` instances. 
    ///  It needs to grab the latest tracker data, update visuals, compute cable tensions, and send commands to LabVIEW.
    ///  a `Controller` functions more like a `Control Policy/Solver` here, with RobotController managing the data flow.
    /// </summary>
    private void controlLoop()
    {
        double ctrl_freq = ControlFrequency;
        double dt = 1.0 / ctrl_freq;

        Span<double> motor_tension_command = stackalloc double[14];
        double[] zeroPelvisTensions = new double[robotDescription.NumCables];
        double[] zeroTrunkTensions = new double[robotDescription.NumTrunkCables];
        double[] fixedPelvisTensions = new double[robotDescription.NumCables];
        Array.Fill(fixedPelvisTensions, 10.0);
        double[] pelvis_tensions = zeroPelvisTensions; // points at the planner's latest result
        double[] trunk_tensions = zeroTrunkTensions;
        Wrench zeroWrench = new Wrench(double3.zero, double3.zero);


        double4x4 framePose = ToDouble4x4(robot_frame_tracker.PoseMatrix);
        double4x4 frameInv = math.fastinverse(framePose);
        SensorFilter filter_10Hz = new SensorFilter(ctrl_freq, 10.0);
        Span<RBState> Xref_pelvis_horizon = stackalloc RBState[TrajectoryHorizonSteps];
        Span<RBState> Xref_trunk_horizon = stackalloc RBState[TrajectoryHorizonSteps];

        double system_frequency = System.Diagnostics.Stopwatch.Frequency;
        double ticksToNs = 1_000_000_000.0 / system_frequency;
        long intervalTicks = (long)(system_frequency / ctrl_freq);
        long nextTargetTime = System.Diagnostics.Stopwatch.GetTimestamp() + intervalTicks;
        long lastLoopTick = System.Diagnostics.Stopwatch.GetTimestamp();

        while (isRunning)
        {
            long loopStartTick = System.Diagnostics.Stopwatch.GetTimestamp();
            s_IntervalNs.Value = (long)((loopStartTick - lastLoopTick) * ticksToNs);
            lastLoopTick = loopStartTick;

            double4x4 eePose_RF, comPose_RF;
            if (!isTrackerMockMode)
            {
                trackerManager.GetEndEffectorTrackerData(out TrackerData rawEndEffectorData);
                trackerManager.GetCoMTrackerData(out TrackerData rawComData);

                eePose_RF = math.mul(frameInv, ToDouble4x4(rawEndEffectorData.PoseMatrix));
                comPose_RF = math.mul(frameInv, ToDouble4x4(rawComData.PoseMatrix));
            }
            else
            {
                eePose_RF = mockEEPose_RF;
                comPose_RF = mockComPose_RF;
                if (currentControlMode == CONTROL_MODE.ASSIST_AS_NEEDED && isTrajectoryActive && !trajectoryTriggered)
                    MakeMockTunnelPoses(out comPose_RF, out eePose_RF);
            }

            // A lost tracker reads as the origin. Force must never act on a lost or estimated pose.
            bool isPelvisTracked = math.lengthsq(comPose_RF.c3.xyz) > 0.001;
            bool isTrunkTracked = math.lengthsq(eePose_RF.c3.xyz) > 0.001;

            // Fallback: If chest tracker is unassigned, offline, or lost, estimate trunk pose above pelvis
            if (math.lengthsq(eePose_RF.c3.xyz) <= 0.001)
            {
                if (math.lengthsq(comPose_RF.c3.xyz) > 0.001)
                {
                    // Synthesize chest pose 25 cm above pelvis along robot Z (Up)
                    eePose_RF = MakePose(comPose_RF.c3.xyz + new double3(0.0, 0.0, 0.25), quaternion.RotateZ(-math.PI / 2));
                }
                else
                {
                    eePose_RF = mockEEPose_RF;
                }
            }




            // Debug.Log($"comPose_RF: {comPose_RF.c3}");

            // FP already in robot frame
            forcePlateManager.GetForcePlateData(0, out ForcePlateData fp0);
            forcePlateManager.GetForcePlateData(1, out ForcePlateData fp1);
            double3 netForce = fp0.Force + fp1.Force;
            double3 netCoP = double3.zero;
            if (math.abs(netForce.z) > 1e-3) // Handle division by zero
                netCoP = (fp0.CoP * fp0.Force.z + fp1.CoP * fp1.Force.z) / netForce.z;
            ForcePlateData netFPData = new ForcePlateData(netForce, netCoP);
            
            filter_10Hz.Update(comPose_RF, eePose_RF, netFPData);

            double pelvisDeviation = -1.0;
            double trunkDeviation = -1.0;
            double referenceProgress = 0.0;
            CONTROL_MODE activeControlMode = currentControlMode;
            ASSISTANCE_TARGET activeTarget = trialAssistanceTarget;
            bool assistPelvis = activeTarget != ASSISTANCE_TARGET.TRUNK_ONLY;
            bool assistTrunk = activeTarget != ASSISTANCE_TARGET.PELVIS_ONLY;

            switch (activeControlMode)
            {
                case CONTROL_MODE.OFF:
                    motor_tension_command.Clear();
                    pelvis_tensions = zeroPelvisTensions;
                    trunk_tensions = zeroTrunkTensions;
                    visualizer.PushGoalTrajectories(ReadOnlySpan<RBState>.Empty, ReadOnlySpan<RBState>.Empty);
                    break;
                case CONTROL_MODE.TRANSPARENT:
                    motor_tension_command.Clear();
                    if (robotDescription.NumCables == 4)
                    {
                        // Four top cables cannot hold a zero wrench, so they hold a light fixed tension
                        pelvis_tensions = fixedPelvisTensions;
                        MapTensionsToMotors(pelvis_tensions, robotDescription.SolverToMotorMap, motor_tension_command);
                    }
                    else
                    {
                        CommandPelvisCables(comPose_RF, zeroWrench, ref pelvis_tensions, motor_tension_command);
                    }
                    CommandTrunkCables(eePose_RF, zeroWrench, ref trunk_tensions, motor_tension_command);

                    if (isReplayPending)
                    {
                        isTrajectoryActive = false;
                        isTrajectoryPaused = false;
                        tunnelProgress = 0.0;
                        FillHoldHorizon(Xref_pelvis_horizon, Xref_trunk_horizon, comPose_RF.c3.xyz, eePose_RF.c3.xyz);
                        visualizer.PushGoalTrajectories(Xref_pelvis_horizon, Xref_trunk_horizon);
                    }
                    else
                    {
                        visualizer.PushGoalTrajectories(ReadOnlySpan<RBState>.Empty, ReadOnlySpan<RBState>.Empty);
                    }
                    break;
                case CONTROL_MODE.IMPEDANCE:
                    if (trajectoryTriggered)
                    {
                        // A refused start keeps this tick's tensions and takes no new target
                        if (!TryBeginTrial(comPose_RF.c3.xyz, eePose_RF.c3.xyz)) break;
                        trajectoryElapsedTime = 0.0;
                    }

                    if (isTrajectoryActive)
                    {
                        if (!isTrajectoryPaused) trajectoryElapsedTime += dt;
                        referenceProgress = math.saturate(trajectoryElapsedTime / math.max(0.1, stsDuration));

                        for (int h = 0; h < TrajectoryHorizonSteps; h++)
                        {
                            double t_h = trajectoryElapsedTime + (h * dt);
                            Xref_pelvis_horizon[h] = EvaluateTrajectory(pelvisTrajDisplacements, pelvisTrajVelocities, t_h, seatedPelvisPos_RF);
                            Xref_trunk_horizon[h] = EvaluateTrajectory(trunkTrajDisplacements, trunkTrajVelocities, t_h, seatedTrunkPos_RF);
                        }
                    }
                    else
                    {
                        FillHoldHorizon(Xref_pelvis_horizon, Xref_trunk_horizon, comPose_RF.c3.xyz, eePose_RF.c3.xyz);
                    }

                    RBState targetPelvisState = Xref_pelvis_horizon[0];
                    RBState targetTrunkState = Xref_trunk_horizon[0];

                    visualizer.PushGoalTrajectories(Xref_pelvis_horizon, Xref_trunk_horizon);

                    pelvisImpedanceController.UpdateState(comPose_RF, filter_10Hz.CoMLinearVelocity, filter_10Hz.CoMAngularVelocity, targetPelvisState);
                    trunkImpedanceController.UpdateState(eePose_RF, filter_10Hz.EELinearVelocity, filter_10Hz.EEAngularVelocity, targetTrunkState);

                    // Each body is pushed by its own cables. An unassisted body gets a zero wrench.
                    Wrench pelvisGoalWrench = assistPelvis && isPelvisTracked ? pelvisImpedanceController.computeNextControl() : zeroWrench;
                    Wrench trunkGoalWrench = assistTrunk && isTrunkTracked ? trunkImpedanceController.computeNextControl() : zeroWrench;

                    motor_tension_command.Clear();
                    CommandPelvisCables(comPose_RF, pelvisGoalWrench, ref pelvis_tensions, motor_tension_command);
                    CommandTrunkCables(eePose_RF, trunkGoalWrench, ref trunk_tensions, motor_tension_command);
                    break;
                case CONTROL_MODE.ASSIST_AS_NEEDED:
                    if (trajectoryTriggered)
                    {
                        // A refused start keeps this tick's tensions and takes no new target
                        if (!TryBeginTrial(comPose_RF.c3.xyz, eePose_RF.c3.xyz)) break;
                        double3 trialScale = new double3(userLegLength, userLegLength, riseHeight);
                        tunnelClock.Reset(stsDuration);
                        pelvisAssistController.BeginTrial(seatedPelvisPos_RF, trialScale);
                        trunkAssistController?.BeginTrial(seatedTrunkPos_RF, trialScale);
                        trajectoryElapsedTime = 0.0;
                    }

                    Wrench pelvisTunnelWrench = zeroWrench;
                    Wrench trunkTunnelWrench = zeroWrench;
                    if (isTrajectoryActive)
                    {
                        trajectoryElapsedTime += dt;
                        tunnelClock.Advance(dt);

                        // Deviation is the larger of leaving the tunnel and falling outside the timing band
                        double progress = 0.0;
                        if (assistPelvis)
                        {
                            if (isPelvisTracked)
                            {
                                pelvisAssistController.UpdateState(comPose_RF, filter_10Hz.CoMLinearVelocity);
                                pelvisTunnelWrench = pelvisAssistController.computeNextControl();
                                pelvisDeviation = math.max(pelvisAssistController.Deviation, pelvisAssistController.TimingDeviation);
                            }
                            progress = pelvisAssistController.Progress;
                        }
                        if (assistTrunk && trunkAssistController != null)
                        {
                            if (isTrunkTracked)
                            {
                                trunkAssistController.UpdateState(eePose_RF, filter_10Hz.EELinearVelocity);
                                trunkTunnelWrench = trunkAssistController.computeNextControl();
                                trunkDeviation = math.max(trunkAssistController.Deviation, trunkAssistController.TimingDeviation);
                            }
                            progress = math.max(progress, trunkAssistController.Progress);
                        }
                        tunnelProgress = progress;

                        // The first target is where the subject should be now, followed by a preview of what is ahead
                        double tunnelDuration = math.max(0.1, stsDuration);
                        referenceProgress = tunnelClock.IsRunning ? tunnelClock.Tau : tunnelProgress;
                        for (int h = 0; h < TrajectoryHorizonSteps; h++)
                        {
                            double t_h = (referenceProgress + (h * TunnelPreviewStep)) * tunnelDuration;
                            Xref_pelvis_horizon[h] = EvaluateTrajectory(pelvisTrajDisplacements, pelvisTrajVelocities, t_h, seatedPelvisPos_RF);
                            Xref_trunk_horizon[h] = EvaluateTrajectory(trunkTrajDisplacements, trunkTrajVelocities, t_h, seatedTrunkPos_RF);
                        }
                    }
                    else
                    {
                        FillHoldHorizon(Xref_pelvis_horizon, Xref_trunk_horizon, comPose_RF.c3.xyz, eePose_RF.c3.xyz);
                    }

                    visualizer.PushGoalTrajectories(Xref_pelvis_horizon, Xref_trunk_horizon);

                    motor_tension_command.Clear();
                    CommandPelvisCables(comPose_RF, pelvisTunnelWrench, ref pelvis_tensions, motor_tension_command);
                    CommandTrunkCables(eePose_RF, trunkTunnelWrench, ref trunk_tensions, motor_tension_command);
                    break;
            }
            tcpCommunicator.UpdateTensionSetpoint(motor_tension_command);
            visualizer.PushState(comPose_RF, eePose_RF, fp0, fp1);
            visualizer.PushTunnelDeviation(pelvisDeviation, trunkDeviation);
            if (isLogging)
            {
                // Logged wrenches are what the cables actually apply, from the commanded tensions
                Wrench loggedPelvisWrench = pelvisTensionPlanner.CalculateResultantWrench(comPose_RF, pelvis_tensions);
                Wrench loggedTrunkWrench = trunkTensionPlanner != null
                    ? trunkTensionPlanner.CalculateResultantWrench(eePose_RF, trunk_tensions)
                    : zeroWrench;
                dataLogger.Log(loopStartTick, comPose_RF, eePose_RF, fp0, fp1, loggedPelvisWrench, Xref_pelvis_horizon[0],
                    loggedTrunkWrench, Xref_trunk_horizon[0], pelvisDeviation, trunkDeviation, referenceProgress);
            }

            s_WorkloadNs.Value = (long)((System.Diagnostics.Stopwatch.GetTimestamp() - loopStartTick) * ticksToNs);
            while (System.Diagnostics.Stopwatch.GetTimestamp() < nextTargetTime) { } // BURN wait

            nextTargetTime += intervalTicks;
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            if (now > nextTargetTime) nextTargetTime = now + intervalTicks; // drift correction
        }
    }


    // Validates that all required module references are assigned in the Inspector.
    // returns True if all modules are assigned, false otherwise.
    private bool ValidateModules()
    {
        bool allValid = true;
        if (trackerManager == null) { Debug.LogError("Module not assigned in Inspector: trackerManager", this); allValid = false; }
        if (forcePlateManager == null) { Debug.LogError("Module not assigned in Inspector: forcePlateManager", this); allValid = false; }
        if (tcpCommunicator == null) { Debug.LogError("Module not assigned in Inspector: tcpCommunicator", this); allValid = false; }
        if (visualizer == null) { Debug.LogError("Visualizer not assigned in Inspector: visualizer", this); allValid = false; }

        return allValid;
    }

    private void OnDestroy()
    {
        // Clean shutdown of threaded components.
        // TrackerManager handles its own shutdown via its OnDestroy method.
        isRunning = false;
        tcpCommunicator?.Disconnect();
        if (dataLogger != null && dataLogger.FrameCount > 0)
            dataLogger.WriteToDisk(sessionName);

        if (s_CircleTexture != null)
        {
            Destroy(s_CircleTexture);
            s_CircleTexture = null;
        }
    }


    /// <summary>
    /// Writes one cable set's solver tensions into the 14-motor driver array. Clear the array first.
    /// </summary>
    private static void MapTensionsToMotors(double[] solverResult, int[] solverToMotorMap, Span<double> output)
    {
        for (int i = 0; i < solverToMotorMap.Length; i++)
        {
            output[solverToMotorMap[i]] = solverResult[i];
        }
    }

    // Local helpers for loading both Pelvis and Trunk trajectories
    private void LoadTrajectories()
    {
        if (string.IsNullOrWhiteSpace(pelvisTrajectoryCsvName)) pelvisTrajectoryCsvName = "SitToWalk_GPR_Pelvis_NoVR_normalized.csv";
        if (string.IsNullOrWhiteSpace(trunkTrajectoryCsvName)) trunkTrajectoryCsvName = "SitToWalk_GPR_Trunk_NoVR_normalized.csv";

        bool pSuccess = LoadSingleTrajectory(pelvisTrajectoryCsvName, out pelvisTrajDisplacements, out pelvisTrajVelocities, out pelvisTrajSD, "Pelvis");
        bool tSuccess = LoadSingleTrajectory(trunkTrajectoryCsvName, out trunkTrajDisplacements, out trunkTrajVelocities, out trunkTrajSD, "Trunk");

        if (!tSuccess && pSuccess)
        {
            // Good enough for the IMPEDANCE target, but not for a trunk tunnel, which stays off
            Debug.LogWarning("[RobotController] Trunk trajectory failed to load. Falling back to Pelvis trajectory for trunk, with no trunk tunnel.");
            trunkTrajDisplacements = pelvisTrajDisplacements;
            trunkTrajVelocities = pelvisTrajVelocities;
            trunkTrajSD = null;
        }
    }

    private bool LoadSingleTrajectory(string csvName, out double3[] displacements, out double3[] velocities, out double3[] sd, string tag)
    {
        displacements = null;
        velocities = null;
        sd = null;
        if (string.IsNullOrEmpty(csvName))
        {
            Debug.LogWarning($"[{tag} Trajectory] Trajectory filename is empty.");
            return false;
        }

        try
        {
            string path = System.IO.Path.Combine(Application.dataPath, "Scripts", csvName);
            if (!System.IO.File.Exists(path))
            {
                string alt = csvName.Contains("_normalized")
                    ? csvName.Replace("_normalized", "")
                    : csvName.Replace(".csv", "_normalized.csv");
                path = System.IO.Path.Combine(Application.dataPath, "Scripts", alt);
                if (!System.IO.File.Exists(path))
                {
                    // Fallbacks to generic files
                    string fallback = System.IO.Path.Combine(Application.dataPath, "Scripts", "SitToWalk_GPR_NoVR_normalized.csv");
                    if (!System.IO.File.Exists(fallback))
                        fallback = System.IO.Path.Combine(Application.dataPath, "Scripts", "SitToWalk_GPR_NoVR.csv");

                    if (System.IO.File.Exists(fallback))
                    {
                        path = fallback;
                    }
                    else
                    {
                        Debug.LogError($"[{tag} Trajectory] File not found: {csvName} (also searched {path})");
                        return false;
                    }
                }
            }

            string[] rawLines = System.IO.File.ReadAllLines(path);
            if (rawLines.Length <= 1)
            {
                Debug.LogError($"[{tag} Trajectory] File {path} has no data rows.");
                return false;
            }

            // Filter out any blank or whitespace lines to avoid IndexOutOfRangeException
            var validLines = new System.Collections.Generic.List<string>(rawLines.Length);
            for (int l = 0; l < rawLines.Length; l++)
            {
                string trimmed = rawLines[l].Trim();
                if (!string.IsNullOrEmpty(trimmed))
                    validLines.Add(trimmed);
            }

            if (validLines.Count <= 1)
            {
                Debug.LogError($"[{tag} Trajectory] File {path} contains no valid data lines.");
                return false;
            }

            string[] headers = validLines[0].Split(',');
            for (int h = 0; h < headers.Length; h++) headers[h] = headers[h].Trim();

            int ap = Array.IndexOf(headers, "delta_AP_forward_pct");
            int ml = Array.IndexOf(headers, "delta_ML_left_pct");
            int vert = Array.IndexOf(headers, "delta_Vertical_up_pct");
            int sdAp = Array.IndexOf(headers, "sd_AP_pct");
            int sdMl = Array.IndexOf(headers, "sd_ML_pct");
            int sdVert = Array.IndexOf(headers, "sd_Vertical_pct");
            if (ap < 0 || ml < 0 || vert < 0 || sdAp < 0 || sdMl < 0 || sdVert < 0)
            {
                Debug.LogError($"[{tag} Trajectory] File {path} is missing delta_*_pct or sd_*_pct columns. Regenerate it with trunk_trajectory_analysis.py.");
                return false;
            }

            int count = validLines.Count - 1;
            displacements = new double3[count];
            velocities = new double3[count];
            sd = new double3[count];
            int lastColumn = Math.Max(Math.Max(ap, Math.Max(ml, vert)), Math.Max(sdAp, Math.Max(sdMl, sdVert)));

            for (int i = 0; i < count; i++)
            {
                string[] cols = validLines[i + 1].Split(',');
                if (cols.Length <= lastColumn)
                {
                    Debug.LogWarning($"[{tag} Trajectory] Row {i + 1} has insufficient columns ({cols.Length}).");
                    continue;
                }

                // Robot frame: Forward is -X, Lateral is -Y, Up is +Z
                displacements[i] = new double3(
                    -double.Parse(cols[ap].Trim(), System.Globalization.CultureInfo.InvariantCulture) / 100.0,
                    -double.Parse(cols[ml].Trim(), System.Globalization.CultureInfo.InvariantCulture) / 100.0,
                    double.Parse(cols[vert].Trim(), System.Globalization.CultureInfo.InvariantCulture) / 100.0
                );
                sd[i] = new double3(
                    double.Parse(cols[sdAp].Trim(), System.Globalization.CultureInfo.InvariantCulture) / 100.0,
                    double.Parse(cols[sdMl].Trim(), System.Globalization.CultureInfo.InvariantCulture) / 100.0,
                    double.Parse(cols[sdVert].Trim(), System.Globalization.CultureInfo.InvariantCulture) / 100.0
                );
            }

            // Central differences, one-sided at the ends
            double dTau = 1.0 / (count - 1);
            for (int i = 0; i < count; i++)
            {
                int prev = Math.Max(i - 1, 0);
                int next = Math.Min(i + 1, count - 1);
                velocities[i] = (displacements[next] - displacements[prev]) / ((next - prev) * dTau);
            }

            Debug.Log($"<color=green>[RobotController]</color> Successfully loaded {count} {tag} trajectory points from {path}. First d: {displacements[0]}, Last d: {displacements[count - 1]}");
            return true;
        }
        catch (Exception ex)
        {
            Debug.LogError($"[{tag} Trajectory] Exception while parsing '{csvName}': {ex.Message}\n{ex.StackTrace}");
            return false;
        }
    }

    private RBState EvaluateTrajectory(double3[] displacements, double3[] velocities, double elapsedTime, in double3 origin)
    {
        if (displacements == null || displacements.Length == 0)
        {
            return new RBState(origin, new double3(0, 0, -math.PI / 2), double3.zero, double3.zero);
        }

        double tau = math.clamp(elapsedTime / math.max(0.1, stsDuration), 0.0, 1.0);
        int maxIdx = displacements.Length - 1;
        double floatIdx = tau * maxIdx;
        int i0 = (int)floatIdx;
        int i1 = math.min(i0 + 1, maxIdx);
        double frac = floatIdx - i0;

        double3 d = math.lerp(displacements[i0], displacements[i1], frac);
        double3 v = velocities != null && velocities.Length > maxIdx
            ? math.lerp(velocities[i0], velocities[i1], frac)
            : double3.zero;

        double3 scale = new double3(userLegLength, userLegLength, riseHeight);
        double3 p = origin + (d * scale);
        double3 vel = (v * scale) / math.max(0.1, stsDuration);
        return new RBState(p, new double3(0, 0, -math.PI / 2), vel, double3.zero);
    }
    private static double4x4 MakePose(in double3 pos, in quaternion rot)
    {
        float3 p = (float3)pos;
        return ToDouble4x4(Matrix4x4.TRS(new Vector3(p.x, p.y, p.z), (Quaternion)rot, Vector3.one));
    }

    private static double4x4 ToDouble4x4(in Matrix4x4 m)
    {
        return new double4x4(
            m.m00, m.m01, m.m02, m.m03,
            m.m10, m.m11, m.m12, m.m13,
            m.m20, m.m21, m.m22, m.m23,
            m.m30, m.m31, m.m32, m.m33
        );
    }

    private static Texture2D s_CircleTexture;
    private static Texture2D GetCircleTexture()
    {
        if (s_CircleTexture != null) return s_CircleTexture;
        int res = 32;
        s_CircleTexture = new Texture2D(res, res, TextureFormat.RGBA32, false)
        {
            filterMode = FilterMode.Bilinear,
            wrapMode = TextureWrapMode.Clamp
        };
        float center = res * 0.5f;
        float radius = (res * 0.5f) - 1.5f;
        for (int y = 0; y < res; y++)
        {
            for (int x = 0; x < res; x++)
            {
                float dist = Mathf.Sqrt((x + 0.5f - center) * (x + 0.5f - center) + (y + 0.5f - center) * (y + 0.5f - center));
                float alpha = Mathf.Clamp01(radius + 1.0f - dist);
                s_CircleTexture.SetPixel(x, y, new Color(1f, 1f, 1f, alpha));
            }
        }
        s_CircleTexture.Apply();
        return s_CircleTexture;
    }

    private void OnGUI()
    {
        bool isTunnelMode = currentControlMode == CONTROL_MODE.ASSIST_AS_NEEDED;
        if (currentControlMode != CONTROL_MODE.IMPEDANCE && !isTunnelMode) return;

        float trackWidth = 240f;
        float marginX = 24f;
        float lineThickness = 3f;
        float circleDiameter = 14f;

        float progress = isTunnelMode
            ? (float)tunnelProgress
            : (float)math.clamp(trajectoryElapsedTime / math.max(0.1, stsDuration), 0.0, 1.0);
        float lineY = Screen.height - 24f;

        // Label above track
        GUIStyle labelStyle = new GUIStyle(GUI.skin.label)
        {
            alignment = TextAnchor.MiddleLeft,
            fontStyle = FontStyle.Bold,
            fontSize = 12,
            normal = { textColor = Color.white }
        };
        GUI.Label(new Rect(marginX, lineY - 24f, trackWidth + 40f, 18f), $"Trajectory: {progress:P0}", labelStyle);

        Color prevColor = GUI.color;

        // 1. Background line track (dark gray)
        GUI.color = new Color(0.3f, 0.3f, 0.3f, 0.85f);
        GUI.DrawTexture(new Rect(marginX, lineY, trackWidth, lineThickness), Texture2D.whiteTexture);

        // 2. Traveled portion of track (bright green)
        if (progress > 0.001f)
        {
            GUI.color = new Color(0.1f, 0.85f, 0.2f, 0.95f);
            GUI.DrawTexture(new Rect(marginX, lineY, trackWidth * progress, lineThickness), Texture2D.whiteTexture);
        }

        // 3. Progress circle moving along the line
        Texture2D circle = GetCircleTexture();
        float circleX = marginX + (trackWidth * progress);
        float circleCenterY = lineY + (lineThickness * 0.5f);

        // Outer dark ring / shadow for contrast
        GUI.color = new Color(0.05f, 0.05f, 0.05f, 0.85f);
        GUI.DrawTexture(new Rect(circleX - (circleDiameter * 0.5f) - 1.5f, circleCenterY - (circleDiameter * 0.5f) - 1.5f, circleDiameter + 3f, circleDiameter + 3f), circle);

        // Inner circle knob (bright green)
        GUI.color = new Color(0.1f, 0.95f, 0.25f, 1.0f);
        GUI.DrawTexture(new Rect(circleX - (circleDiameter * 0.5f), circleCenterY - (circleDiameter * 0.5f), circleDiameter, circleDiameter), circle);

        GUI.color = prevColor;
    }
}
