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

    // Values are the cable counts, so a scene saved with numCables as a number keeps its setting
    public enum CABLE_CONFIG
    {
        [InspectorName("4 cables: EE only")] EE_ONLY = 4,
        [InspectorName("8 cables: CoM only")] COM_ONLY = 8,
        [InspectorName("12 cables: CoM and EE")] COM_AND_EE = 12
    }
    [Header("Robot Geometry Configuration")]
    [Tooltip("Driven cables, which also sets the assisted bodies. Can be changed during Play: a running trial keeps its cables until the next trial. 4 and 12 need the EE motor numbers in RobUSTDescription.")]
    [SerializeField] private volatile CABLE_CONFIG numCables = CABLE_CONFIG.COM_ONLY;
    private volatile CABLE_CONFIG trialCableConfig; // numCables as of the current trial's start
    private CABLE_CONFIG acceptedCableConfig; // last accepted numCables, restored when a change is refused
    private CABLE_CONFIG shownCableConfig; // cables whose pulleys the visualizer shows
    [Tooltip("Measured anterior-posterior thickness at the CoM cable attachments [m].")]
    [SerializeField] private float comAPDistance = 0.2f;
    [Tooltip("Measured medial-lateral width at the CoM cable attachments [m].")]
    [SerializeField] private float comMLDistance = 0.3f;
    [Tooltip("Measured anterior-posterior thickness at the EE cable attachments [m].")]
    [SerializeField] private float eeAPDistance = 0.20f;
    [Tooltip("Measured medial-lateral width at the EE cable attachments [m].")]
    [SerializeField] private float eeMLDistance = 0.28f;
    [Tooltip("User body mass [kg].")]
    [SerializeField] private float userMass = 70.0f;

    [Header("Sit-to-Walk Trajectories")]
    [Tooltip("Filename of the CoM GPR trajectory CSV located in Assets/Scripts.")]
    [SerializeField] private string comTrajectoryCsvName = "SitToWalk_GPR_Pelvis_NoVR_normalized.csv";
    [Tooltip("Filename of the EE GPR trajectory CSV located in Assets/Scripts.")]
    [SerializeField] private string eeTrajectoryCsvName = "SitToWalk_GPR_Trunk_NoVR_normalized.csv";
    [Tooltip("Target duration of the movement [s]. IMPEDANCE and ASSIST_AS_NEEDED play from the position captured when Space starts the trial. NoVR subjects averaged 0.79 s.")]
    [SerializeField] private double stsDuration = 2.0;
    [Tooltip("Subject leg length: CoM tracker to floor while standing [m]. Scales AP/ML and, with the seated CoM height, sets the vertical rise.")]
    [SerializeField] private double userLegLength = 1.07;

    // Measured floor-to-seat height [m]. The pelvis offset is captured live at each trial start.
    private const double chairHeight = 0.4318;

    // Trajectory buffers (Zero-allocation during runtime)
    // Robot-frame fractions: x/y of leg length, z of rise height. Velocities are per unit tau.
    private double3[] comTrajDisplacements;
    private double3[] comTrajVelocities;
    private double3[] eeTrajDisplacements;
    private double3[] eeTrajVelocities;

    // Assist-as-needed force limits, as fractions of body weight
    private const double CoMMaxForceFraction = 0.15;
    private const double EEMaxForceFraction = 0.10;
    private const double Gravity = 9.81; // [m/s^2]

    private const double MaxRestartDrift = 0.10; // allowed distance from the last seated position on restart [m]
    private const double MockSwayAmplitude = 0.10; // [m]
    private const double MockSlowdown = 1.25; // virtual subject runs late, so it falls behind the pacer
    private const double ControlFrequency = 100.0; // [Hz]
    private const int TrajectoryHorizonSteps = 10;
    private const double TunnelPreviewStep = 0.05; // spacing of the path preview, in tau

    private double3 seatedCoMPos_RF = double3.zero;
    private double3 seatedEEPos_RF = double3.zero;
    private volatile bool isSeatedPoseValid = false;
    private double riseHeight = 0.0; // vertical scale [m], set when a trajectory starts
    private double trajectoryElapsedTime = 0.0;
    private volatile bool trajectoryTriggered = false;
    private volatile bool isTrajectoryPaused = false;
    private volatile bool isReplayPending = false; // reset to TRANSPARENT, and Space returns to replayControlMode
    private CONTROL_MODE replayControlMode = CONTROL_MODE.IMPEDANCE;
    private float countdownEndTime = -1f;
    private bool isCountingDown;
    private CONTROL_MODE countdownMode;
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
    private ImpedanceController comImpedanceController;
    private ImpedanceController eeImpedanceController;
    private AssistAsNeededController comAssistController;
    private AssistAsNeededController eeAssistController; // null when the EE trajectory did not load
    private CableTensionPlanner comTensionPlanner;
    private CableTensionPlanner eeTensionPlanner; // null until the EE motor numbers are entered

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

        if (comAPDistance <= 0 || comMLDistance <= 0 || eeAPDistance <= 0 || eeMLDistance <= 0)
        {
            Debug.LogError("CoM and EE dimensions must be positive values.", this);
            enabled = false;
            return;
        }

        // Create RobUST description (single allocation at startup)
        robotDescription = RobUSTDescription.Create(comAPDistance, comMLDistance, eeAPDistance, eeMLDistance, userMass);
        string cableError = CheckCableConfig(numCables);
        if (cableError != null)
        {
            Debug.LogError($"[RobotController] Robot not started, so there is no control loop, trajectory or motor command. {cableError}", this);
            robotDescription = null;
            enabled = false;
            return;
        }
        acceptedCableConfig = numCables;
        trialCableConfig = numCables;
        Debug.Log($"RobUST description created: {numCables}, CoM AP={comAPDistance}m ML={comMLDistance}m, EE AP={eeAPDistance}m ML={eeMLDistance}m");

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
        comImpedanceController = new ImpedanceController(userMass);
        eeImpedanceController = new ImpedanceController(userMass);
        // Both planners exist whenever their motors are known, so the cables can be switched during Play
        comTensionPlanner = CableTensionPlanner.ForCoM(robotDescription);
        if (robotDescription.EEMotorError == null)
            eeTensionPlanner = CableTensionPlanner.ForEE(robotDescription);
        mockEEPose_RF = MakePose(new double3(0.2, 0.825, 0.01), quaternion.RotateZ(-math.PI / 2));
        mockComPose_RF = MakePose(new double3(0.2, 0.825, -0.14), quaternion.identity);
        bool isEETrajectoryLoaded = LoadTrajectories();
        if (comTrajDisplacements != null)
        {
            double bodyWeight = userMass * Gravity;
            comAssistController = new AssistAsNeededController(CoMMaxForceFraction * bodyWeight);
            if (isEETrajectoryLoaded)
                eeAssistController = new AssistAsNeededController(EEMaxForceFraction * bodyWeight);
        }
        else if (currentControlMode == CONTROL_MODE.ASSIST_AS_NEEDED)
        {
            currentControlMode = CONTROL_MODE.OFF; // the Inspector can start here without a CoM trajectory
        }

        controllerThread = new Thread(controlLoop)
        {
            Name = "Robot Controller Main",
            IsBackground = true,
            Priority = System.Threading.ThreadPriority.AboveNormal
        };

        isRunning = true;
        controllerThread.Start();
    }

    // An Inspector change to numCables during Play. Refused changes are undone here; accepted ones are
    // read by the control loop, and shown by Update, since OnValidate cannot change GameObjects.
    private void OnValidate()
    {
        if (!Application.isPlaying || robotDescription == null || numCables == acceptedCableConfig) return;

        string cableError = CheckCableConfig(numCables);
        if (cableError != null)
        {
            Debug.LogError($"[RobotController] Cables not changed, still {acceptedCableConfig}. {cableError}", this);
            numCables = acceptedCableConfig;
            return;
        }
        acceptedCableConfig = numCables;
        Debug.Log(isTrajectoryActive
            ? $"[RobotController] Cables changed to {numCables}. The running trial keeps {trialCableConfig} until the next trial."
            : $"[RobotController] Cables changed to {numCables}.", this);
    }

    // Why a cable configuration cannot be driven, or null when it can
    private string CheckCableConfig(CABLE_CONFIG config)
    {
        if (!Enum.IsDefined(typeof(CABLE_CONFIG), config))
            return $"Unsupported cable count: {(int)config}. Valid options: 4 (EE only), 8 (CoM only), 12 (CoM and EE).";
        if (DrivesEE(config) && robotDescription.EEMotorError != null)
            return robotDescription.EEMotorError;
        return null;
    }

    private static bool DrivesCoM(CABLE_CONFIG config) => config != CABLE_CONFIG.EE_ONLY;
    private static bool DrivesEE(CABLE_CONFIG config) => config != CABLE_CONFIG.COM_ONLY;

    private void Update()
    {
        CABLE_CONFIG cablesNow = isTrajectoryActive ? trialCableConfig : numCables;
        if (cablesNow != shownCableConfig)
        {
            visualizer.SetPulleysVisible(DrivesCoM(cablesNow), DrivesEE(cablesNow));
            shownCableConfig = cablesNow;
        }

        if (Keyboard.current == null) return;

        if (Keyboard.current.iKey.wasPressedThisFrame)
        {
            SetControlMode(CONTROL_MODE.IMPEDANCE);
        }

        if (Keyboard.current.aKey.wasPressedThisFrame)
        {
            if (comAssistController != null) SetControlMode(CONTROL_MODE.ASSIST_AS_NEEDED);
            else Debug.LogWarning("[RobotController] Assist-as-needed unavailable: the CoM trajectory did not load.");
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

    private void LateUpdate()
    {
        if (!isCountingDown) return;
        if (currentControlMode != countdownMode)
        {
            isCountingDown = false;
            return;
        }
        if (Time.unscaledTime < countdownEndTime) return;

        isCountingDown = false;
        bool replay = currentControlMode == CONTROL_MODE.TRANSPARENT && isReplayPending;
        isReplayPending = false;
        StartTrajectory(); // The control loop captures the live poses when this starts.
        if (replay) currentControlMode = replayControlMode;
    }

    // Changing mode ends any trial. The seated pose is dropped so that no old target is restored.
    private void SetControlMode(CONTROL_MODE mode)
    {
        if (mode == currentControlMode) return;

        isCountingDown = false;
        isTrajectoryActive = false;
        isTrajectoryPaused = false;
        trajectoryTriggered = false;
        isReplayPending = false;
        isSeatedPoseValid = false;
        trajectoryElapsedTime = 0.0;
        currentControlMode = mode;
    }

    private void StartTrajectory()
    {
        trajectoryElapsedTime = 0.0;
        isTrajectoryPaused = false;
        trajectoryTriggered = true;
        // Fixed for the whole trial, so a change cannot add or drop a body mid-movement. Set before
        // the trial goes active, so the control loop never reads the previous trial's cables.
        trialCableConfig = numCables;
        isTrajectoryActive = true;
        if (DrivesEE(trialCableConfig) && eeAssistController == null && currentControlMode == CONTROL_MODE.ASSIST_AS_NEEDED)
            Debug.LogWarning("[RobotController] EE assistance unavailable: the EE trajectory did not load. The EE cables stay at minimum tension.");
    }

    public void TriggerOrToggleTrajectory()
    {
        if (isCountingDown) return;
        if ((currentControlMode == CONTROL_MODE.TRANSPARENT && isReplayPending)
            || (!isTrajectoryActive && (currentControlMode == CONTROL_MODE.IMPEDANCE
                || currentControlMode == CONTROL_MODE.ASSIST_AS_NEEDED)))
        {
            countdownMode = currentControlMode;
            countdownEndTime = Time.unscaledTime + 3f;
            isCountingDown = true;
        }
        else if (currentControlMode != CONTROL_MODE.IMPEDANCE && currentControlMode != CONTROL_MODE.ASSIST_AS_NEEDED)
        {
            Debug.LogWarning($"[RobotController] Spacebar pressed, but robot is currently in {currentControlMode} mode. Press 'I' for IMPEDANCE or 'A' for ASSIST_AS_NEEDED first.");
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
        isCountingDown = false;
        countdownEndTime = -1f;
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

    // Captures the starting pose once and validates the trajectory scale.
    private bool TryBeginTrial(in double3 comPos, in double3 eePos, bool checkRestartDrift = true)
    {
        trajectoryTriggered = false;

        double liveSeatedCoMHeight = comPos.z - robotDescription.FloorZ;
        bool hasDrifted = checkRestartDrift && isSeatedPoseValid && math.distance(comPos, seatedCoMPos_RF) > MaxRestartDrift;
        if (hasDrifted)
        {
            Debug.LogWarning("[RobotController] Trial not started: position is too far from the previous impedance start.");
            isTrajectoryActive = false;
            // A hold at the old seated pose would pull on a subject who is still up
            if (isSeatedPoseValid) ReleaseToTransparent();
            return false;
        }

        // Measure once from this trial's live pose; never update the offset during playback.
        double pelvisTrackerHeightAboveSeat = liveSeatedCoMHeight - chairHeight;
        double seatedCoMHeight = chairHeight + pelvisTrackerHeightAboveSeat;
        double rise = userLegLength - seatedCoMHeight;
        if (!math.isfinite(chairHeight) || chairHeight <= 0.0
            || !math.isfinite(pelvisTrackerHeightAboveSeat) || pelvisTrackerHeightAboveSeat < 0.0
            || !math.isfinite(rise) || rise <= 0.0)
        {
            Debug.LogWarning($"[RobotController] Trial not started: invalid trajectory scale. Chair height {chairHeight:F4}m, captured pelvis height {seatedCoMHeight:F3}m, leg-length scale {userLegLength:F3}m, rise {rise:F3}m. The pelvis must be at or above the seat and the calculated rise must be positive.");
            isTrajectoryActive = false;
            if (isSeatedPoseValid) ReleaseToTransparent();
            return false;
        }

        seatedCoMPos_RF = comPos;
        seatedEEPos_RF = eePos;
        isSeatedPoseValid = true;
        riseHeight = rise;
        //Debug.Log($"[RobotController] Seated CoM height {seatedCoMHeight:F3}m, leg length {userLegLength:F3}m, rise height {riseHeight:F3}m.");
        return true;
    }

    // Holds both targets at the seated pose, or at the live pose before a trial has captured one.
    private void FillHoldHorizon(Span<RBState> comHorizon, Span<RBState> eeHorizon, in double3 liveCoMPos, in double3 liveEEPos)
    {
        double3 holdCoMPos = isSeatedPoseValid ? seatedCoMPos_RF : liveCoMPos;
        double3 holdEEPos = isSeatedPoseValid ? seatedEEPos_RF : liveEEPos;
        comHorizon.Fill(new RBState(holdCoMPos, new double3(0, 0, -math.PI / 2), double3.zero, double3.zero));
        eeHorizon.Fill(new RBState(holdEEPos, new double3(0, 0, -math.PI / 2), double3.zero, double3.zero));
    }

    // Virtual subject for checking assist-as-needed without trackers: follows both paths late while swaying
    // sideways, so the targets sweep through their full colour range.
    private void MakeMockTunnelPoses(out double4x4 comPose, out double4x4 eePose)
    {
        double mockTime = trajectoryElapsedTime / MockSlowdown;
        double tau = math.saturate(mockTime / math.max(0.1, stsDuration));
        double sway = MockSwayAmplitude * math.sin(4.0 * math.PI * tau);
        double3 CoM = EvaluateTrajectory(comTrajDisplacements, comTrajVelocities, mockTime, seatedCoMPos_RF).p;
        double3 EE = EvaluateTrajectory(eeTrajDisplacements, eeTrajVelocities, mockTime, seatedEEPos_RF).p;
        CoM.y += sway;
        EE.y += sway;
        comPose = MakePose(CoM, quaternion.identity);
        eePose = MakePose(EE, quaternion.RotateZ(-math.PI / 2));
    }

    // Sends a wrench through the CoM cables. A zero wrench settles them at minimum tension.
    // Does nothing when the CoM cables are not driven (4).
    private void CommandCoMCables(bool isDriven, in double4x4 comPose_RF, Wrench wrench, ref double[] tensions, Span<double> motorCommand)
    {
        if (!isDriven) return;
        tensions = comTensionPlanner.CalculateTensions(comPose_RF, wrench);
        MapTensionsToMotors(tensions, robotDescription.SolverToMotorMap, motorCommand);
    }

    // Sends a wrench through the EE cables. A zero wrench settles them at minimum tension.
    // Does nothing when the EE cables are not driven (8).
    private void CommandEECables(bool isDriven, in double4x4 eePose_RF, Wrench wrench, ref double[] tensions, Span<double> motorCommand)
    {
        if (!isDriven) return;
        tensions = eeTensionPlanner.CalculateTensions(eePose_RF, wrench);
        MapTensionsToMotors(tensions, robotDescription.EESolverToMotorMap, motorCommand);
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
        double[] zeroCoMTensions = new double[robotDescription.NumCoMCables];
        double[] zeroEETensions = new double[robotDescription.NumEECables];
        double[] fixedEETensions = { 10.0, 10.0, 10.0, 10.0 };
        double[] com_tensions = zeroCoMTensions; // points at the planner's latest result
        double[] ee_tensions = zeroEETensions;
        Wrench zeroWrench = new Wrench(double3.zero, double3.zero);
        Span<double> measured_tensions = stackalloc double[14]; // load cell tensions from LabVIEW, by motor
        double[] actuated_com_tensions = new double[robotDescription.NumCoMCables];
        double[] actuated_ee_tensions = new double[robotDescription.NumEECables];

        double4x4 framePose = ToDouble4x4(robot_frame_tracker.PoseMatrix);
        double4x4 frameInv = math.fastinverse(framePose);
        SensorFilter filter_10Hz = new SensorFilter(ctrl_freq, 10.0);
        Span<RBState> Xref_com_horizon = stackalloc RBState[TrajectoryHorizonSteps];
        Span<RBState> Xref_ee_horizon = stackalloc RBState[TrajectoryHorizonSteps];

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
            bool isCoMTracked = math.lengthsq(comPose_RF.c3.xyz) > 0.001;
            bool isEETracked = math.lengthsq(eePose_RF.c3.xyz) > 0.001;

            // Fallback: If EE tracker is unassigned, offline, or lost, estimate EE pose above CoM
            if (math.lengthsq(eePose_RF.c3.xyz) <= 0.001)
            {
                if (math.lengthsq(comPose_RF.c3.xyz) > 0.001)
                {
                    // Synthesize EE pose 25 cm above CoM along robot Z (Up)
                    eePose_RF = MakePose(comPose_RF.c3.xyz + new double3(0.0, 0.0, 0.25), quaternion.RotateZ(-math.PI / 2));
                }
                else
                {
                    eePose_RF = mockEEPose_RF;
                }
            }




            //Debug.Log($"eePose_RF: {eePose_RF.c3}");

            // FP already in robot frame
            forcePlateManager.GetForcePlateData(0, out ForcePlateData fp0);
            forcePlateManager.GetForcePlateData(1, out ForcePlateData fp1);
            double3 netForce = fp0.Force + fp1.Force;
            double3 netCoP = double3.zero;
            if (math.abs(netForce.z) > 1e-3) // Handle division by zero
                netCoP = (fp0.CoP * fp0.Force.z + fp1.CoP * fp1.Force.z) / netForce.z;
            ForcePlateData netFPData = new ForcePlateData(netForce, netCoP);
            
            filter_10Hz.Update(comPose_RF, eePose_RF, netFPData);

            double comDeviation = -1.0;
            double eeDeviation = -1.0;
            double referenceProgress = 0.0;
            Wrench comGoalWrench = zeroWrench; // what the controller requested, for logging
            Wrench eeGoalWrench = zeroWrench;
            CONTROL_MODE activeControlMode = currentControlMode;
            // A running trial keeps the cables it started with. Undriven cables log zero tension.
            CABLE_CONFIG activeCables = isTrajectoryActive ? trialCableConfig : numCables;
            bool assistCoM = DrivesCoM(activeCables);
            bool assistEE = DrivesEE(activeCables);
            if (!assistCoM) com_tensions = zeroCoMTensions;
            if (!assistEE) ee_tensions = zeroEETensions;

            switch (activeControlMode)
            {
                case CONTROL_MODE.OFF:
                    motor_tension_command.Clear();
                    com_tensions = zeroCoMTensions;
                    ee_tensions = zeroEETensions;
                    visualizer.PushGoalTrajectories(ReadOnlySpan<RBState>.Empty, ReadOnlySpan<RBState>.Empty);
                    break;
                case CONTROL_MODE.TRANSPARENT:
                    motor_tension_command.Clear();
                    // 8 CoM cables hold a zero wrench (8, 12)
                    CommandCoMCables(assistCoM, comPose_RF, zeroWrench, ref com_tensions, motor_tension_command);
                    if (assistEE)
                    {
                        // Four EE cables cannot hold a zero wrench, so they hold a light fixed tension (4, 12)
                        ee_tensions = fixedEETensions;
                        MapTensionsToMotors(ee_tensions, robotDescription.EESolverToMotorMap, motor_tension_command);
                    }

                    if (isReplayPending)
                    {
                        isTrajectoryActive = false;
                        isTrajectoryPaused = false;
                        FillHoldHorizon(Xref_com_horizon, Xref_ee_horizon, comPose_RF.c3.xyz, eePose_RF.c3.xyz);
                        visualizer.PushGoalTrajectories(Xref_com_horizon, Xref_ee_horizon);
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
                            Xref_com_horizon[h] = EvaluateTrajectory(comTrajDisplacements, comTrajVelocities, t_h, seatedCoMPos_RF);
                            Xref_ee_horizon[h] = EvaluateTrajectory(eeTrajDisplacements, eeTrajVelocities, t_h, seatedEEPos_RF);
                        }
                    }
                    else
                    {
                        FillHoldHorizon(Xref_com_horizon, Xref_ee_horizon, comPose_RF.c3.xyz, eePose_RF.c3.xyz);
                    }

                    RBState targetCoMState = Xref_com_horizon[0];
                    RBState targetEEState = Xref_ee_horizon[0];

                    visualizer.PushGoalTrajectories(Xref_com_horizon, Xref_ee_horizon);

                    comImpedanceController.UpdateState(comPose_RF, filter_10Hz.CoMLinearVelocity, filter_10Hz.CoMAngularVelocity, targetCoMState);
                    eeImpedanceController.UpdateState(eePose_RF, filter_10Hz.EELinearVelocity, filter_10Hz.EEAngularVelocity, targetEEState);

                    // Each body is pushed by its own cables. A body without driven cables gets a zero wrench.
                    comGoalWrench = assistCoM && isCoMTracked ? comImpedanceController.computeNextControl() : zeroWrench;
                    eeGoalWrench = assistEE && isEETracked ? eeImpedanceController.computeNextControl() : zeroWrench;

                    motor_tension_command.Clear();
                    CommandCoMCables(assistCoM, comPose_RF, comGoalWrench, ref com_tensions, motor_tension_command);
                    CommandEECables(assistEE, eePose_RF, eeGoalWrench, ref ee_tensions, motor_tension_command);
                    break;
                case CONTROL_MODE.ASSIST_AS_NEEDED:
                    if (trajectoryTriggered)
                    {
                        // Capture both live positions once for each start, including replay. Body
                        // movement since the previous trial must not reject or re-anchor this path.
                        if (!TryBeginTrial(comPose_RF.c3.xyz, eePose_RF.c3.xyz, checkRestartDrift: false)) break;
                        trajectoryElapsedTime = 0.0;
                    }

                    if (isTrajectoryActive)
                    {
                        trajectoryElapsedTime += dt;
                        referenceProgress = math.saturate(trajectoryElapsedTime / math.max(0.1, stsDuration));

                        // The first target is the pacer, where the subject should be now, followed by a preview of what is ahead
                        double tunnelDuration = math.max(0.1, stsDuration);
                        for (int h = 0; h < TrajectoryHorizonSteps; h++)
                        {
                            double t_h = (referenceProgress + (h * TunnelPreviewStep)) * tunnelDuration;
                            Xref_com_horizon[h] = EvaluateTrajectory(comTrajDisplacements, comTrajVelocities, t_h, seatedCoMPos_RF);
                            Xref_ee_horizon[h] = EvaluateTrajectory(eeTrajDisplacements, eeTrajVelocities, t_h, seatedEEPos_RF);
                        }

                        // Each body is pulled toward its pacer. A tracked body without driven cables is
                        // still measured, but gets no wrench.
                        if (isCoMTracked)
                        {
                            comAssistController.UpdateState(comPose_RF, Xref_com_horizon[0].p);
                            Wrench comPacerWrench = comAssistController.computeNextControl();
                            if (assistCoM) comGoalWrench = comPacerWrench;
                            comDeviation = comAssistController.Deviation;
                        }
                        if (eeAssistController != null && isEETracked)
                        {
                            eeAssistController.UpdateState(eePose_RF, Xref_ee_horizon[0].p);
                            Wrench eePacerWrench = eeAssistController.computeNextControl();
                            if (assistEE) eeGoalWrench = eePacerWrench;
                            eeDeviation = eeAssistController.Deviation;
                        }
                    }
                    else
                    {
                        FillHoldHorizon(Xref_com_horizon, Xref_ee_horizon, comPose_RF.c3.xyz, eePose_RF.c3.xyz);
                    }

                    visualizer.PushGoalTrajectories(Xref_com_horizon, Xref_ee_horizon);

                    motor_tension_command.Clear();
                    CommandCoMCables(assistCoM, comPose_RF, comGoalWrench, ref com_tensions, motor_tension_command);
                    CommandEECables(assistEE, eePose_RF, eeGoalWrench, ref ee_tensions, motor_tension_command);
                    break;
            }
            tcpCommunicator.UpdateTensionSetpoint(motor_tension_command);
            visualizer.PushState(comPose_RF, eePose_RF, fp0, fp1);
            visualizer.PushTunnelDeviation(comDeviation, eeDeviation);
            if (isLogging)
            {
                // Goal is the requested wrench. Solver and Measured are what the commanded and the
                // measured tensions apply (S * T).
                tcpCommunicator.GetMeasuredTensions(measured_tensions);
                MapMotorsToTensions(measured_tensions, robotDescription.SolverToMotorMap, actuated_com_tensions);
                Wrench solverWrench = comTensionPlanner.CalculateResultantWrench(comPose_RF, com_tensions);
                Wrench measuredWrench = comTensionPlanner.CalculateResultantWrench(comPose_RF, actuated_com_tensions);
                Wrench eeSolverWrench = zeroWrench;
                Wrench eeMeasuredWrench = zeroWrench;
                if (eeTensionPlanner != null)
                {
                    MapMotorsToTensions(measured_tensions, robotDescription.EESolverToMotorMap, actuated_ee_tensions);
                    eeSolverWrench = eeTensionPlanner.CalculateResultantWrench(eePose_RF, ee_tensions);
                    eeMeasuredWrench = eeTensionPlanner.CalculateResultantWrench(eePose_RF, actuated_ee_tensions);
                }
                dataLogger.Log(loopStartTick, comPose_RF, eePose_RF, fp0, fp1,
                    comGoalWrench, solverWrench, measuredWrench, Xref_com_horizon[0],
                    eeGoalWrench, eeSolverWrench, eeMeasuredWrench, Xref_ee_horizon[0],
                    comDeviation, eeDeviation, referenceProgress);
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

    /// <summary>
    /// Reads one cable set's tensions out of the 14-motor array, in solver order. Inverse of MapTensionsToMotors.
    /// </summary>
    private static void MapMotorsToTensions(ReadOnlySpan<double> motorTensions, int[] solverToMotorMap, double[] actuated_tensions)
    {
        for (int actuatedCableIndex = 0; actuatedCableIndex < solverToMotorMap.Length; actuatedCableIndex++)
        {
            actuated_tensions[actuatedCableIndex] = motorTensions[solverToMotorMap[actuatedCableIndex]];
        }
    }

    // Local helpers for loading both CoM and EE trajectories. Returns whether the EE trajectory loaded.
    private bool LoadTrajectories()
    {
        if (string.IsNullOrWhiteSpace(comTrajectoryCsvName)) comTrajectoryCsvName = "SitToWalk_GPR_Pelvis_NoVR_normalized.csv";
        if (string.IsNullOrWhiteSpace(eeTrajectoryCsvName)) eeTrajectoryCsvName = "SitToWalk_GPR_Trunk_NoVR_normalized.csv";

        bool pSuccess = LoadSingleTrajectory(comTrajectoryCsvName, out comTrajDisplacements, out comTrajVelocities, "CoM");
        bool tSuccess = LoadSingleTrajectory(eeTrajectoryCsvName, out eeTrajDisplacements, out eeTrajVelocities, "EE");

        if (!tSuccess && pSuccess)
        {
            // Good enough for the IMPEDANCE target, but not for EE assist-as-needed, which stays off
            Debug.LogWarning("[RobotController] EE trajectory failed to load. Falling back to CoM trajectory for EE, with no EE assist-as-needed.");
            eeTrajDisplacements = comTrajDisplacements;
            eeTrajVelocities = comTrajVelocities;
        }
        return tSuccess;
    }

    private bool LoadSingleTrajectory(string csvName, out double3[] displacements, out double3[] velocities, string tag)
    {
        displacements = null;
        velocities = null;
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
            if (ap < 0 || ml < 0 || vert < 0)
            {
                Debug.LogError($"[{tag} Trajectory] File {path} is missing delta_*_pct columns. Regenerate it with trunk_trajectory_analysis.py.");
                return false;
            }

            int count = validLines.Count - 1;
            displacements = new double3[count];
            velocities = new double3[count];
            int lastColumn = Math.Max(ap, Math.Max(ml, vert));

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
        if (isCountingDown || (isTrajectoryActive && Time.unscaledTime < countdownEndTime + 0.5f))
        {
            GUIStyle countdownStyle = new GUIStyle(GUI.skin.label)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 64,
                fontStyle = FontStyle.Bold,
                normal = { textColor = Color.white }
            };
            string text = isCountingDown ? Mathf.Max(1, Mathf.CeilToInt(countdownEndTime - Time.unscaledTime)).ToString() : "Go!";
            GUI.Label(new Rect(0, Screen.height / 2f - 50f, Screen.width, 100f), text, countdownStyle);
        }
        if (currentControlMode != CONTROL_MODE.IMPEDANCE && currentControlMode != CONTROL_MODE.ASSIST_AS_NEEDED) return;

        float trackWidth = 240f;
        float marginX = 24f;
        float lineThickness = 3f;
        float circleDiameter = 14f;

        float progress = (float)math.clamp(trajectoryElapsedTime / math.max(0.1, stsDuration), 0.0, 1.0);
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
