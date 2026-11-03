using UnityEngine;
using Unity.Mathematics;
using System;

/// <summary>
/// Complete robot description for the RobUST cable-driven parallel robot.
/// Contains all constant parameters needed for kinematics, dynamics, and control.
/// Allocated once at startup - zero runtime allocations.
/// </summary>
public sealed class RobUSTDescription
{
    /// <summary>Number of CoM cables</summary>
    public readonly int NumCoMCables;

    /// <summary>Pulley positions in robot frame [m] (double3 for SIMD)</summary>
    public readonly double3[] FramePulleyPositions;
    
    /// <summary>CoM cable attachment points, in the CoM tracker frame [m]</summary>
    public readonly double3[] LocalAttachmentPoints;
    
    /// <summary>CoM center in the CoM tracker frame [m]</summary>
    public readonly double3 CoMCenter_CoM_Frame;

    /// <summary>Number of EE cables, which act on the EE harness separately from the CoM cables</summary>
    public readonly int NumEECables;

    /// <summary>EE cable pulley positions in robot frame [m]</summary>
    public readonly double3[] EEFramePulleyPositions;

    /// <summary>EE cable attachment points on the EE harness, in EE tracker frame [m]</summary>
    public readonly double3[] LocalEEAttachmentPoints;

    /// <summary>EE center in EE tracker frame [m]</summary>
    public readonly double3 EECenter_EE_Frame;

    /// <summary>Mapping from EE solver index to motor driver index</summary>
    public readonly int[] EESolverToMotorMap;

    /// <summary>
    /// Why the EE cables cannot be driven, or null when their motor numbers are entered and valid.
    /// Which cables are driven is chosen in RobotController.
    /// </summary>
    public readonly string EEMotorError;
    
    // ============ User Parameters ============
    /// <summary>User body mass [kg]</summary>
    public readonly double UserMass;
    
    /// <summary>CoM anterior-posterior distance [m]</summary>
    public readonly double CoMAPDistance;
    
    /// <summary>CoM medial-lateral distance [m]</summary>
    public readonly double CoMMLDistance;

    /// <summary>EE anterior-posterior distance [m]</summary>
    public readonly double EEAPDistance;

    /// <summary>EE medial-lateral distance [m]</summary>
    public readonly double EEMLDistance;

    // ============ Force Plate Geometry ============
    public readonly double3 FP_BackLeft;
    public readonly double3 FP_BackRight;
    public readonly double3 FP_FrontLeft;
    public readonly double3 FP_FrontRight;

    /// <summary>Floor height in robot frame, taken from the force plate surface [m]</summary>
    public readonly double FloorZ;

    // Full 12-cable hardware definition (Static Database from vive tracker measurement)
    // 0-7 are the CoM cables, 8-11 the EE cables
    private static readonly double3[] AllPulleyPositions = new double3[]
    {
/*         new double3(-0.8000, 1.650, 0.9875),   // 0: Front-Right Top (Motor 10)
        new double3(-0.7900, 0.0022, 0.9540),   // 1: Front-Left Top (Motor 5)
        new double3(0.9580, 0.0335, 0.9828),    // 2: Back-Left Top (Motor 4)
        new double3(0.9650, 1.6420, 1.0000),    // 3: Back-Right Top (Motor 11)
        new double3(-0.7850, 1.6750, -0.2920),  // 4: Front-Right Bottom (Motor 8)
        new double3(-0.7800, 0.0330, -0.3340),  // 5: Front-Left Bottom (Motor 7)
        new double3(0.9850, 0.0520, -0.5165),   // 6: Back-Left Bottom (Motor 2)
        new double3(0.9800, 1.6890, -0.4994)    // 7: Back-Right Bottom (Motor 13) */

        //new positive x: towards gait mat; positive y: right towards bathroom; positive z: up
        new double3(-0.576835048372789, 2.57648944314966, 0.252733189583757),   // 0: Front-Right Top (Motor 10)
        new double3(-0.588436088357761, 0.0273606629681548, 0.182261108739911),   // 1: Front-Left Top (Motor 5)
        new double3(1.18955071881626, 0.0229493275829191, 0.264120148165004),    // 2: Back-Left Top (Motor 4)
        new double3(1.19164684688485, 2.58195169592441, 0.33072052196806),   // 3: Back-Right Top (Motor 11)
        new double3(-0.527316212320867, 2.60276181483673, -1.10992095940637),  // 4: Front-Right Bottom (Motor 8)
        new double3(-0.535379256296376, 0.0557650522283454, -1.17363481318352),  // 5: Front-Left Bottom (Motor 7)
        new double3(1.25455282289238, 0.0542982128558426, -1.23426325420294),   // 6: Back-Left Bottom (Motor 2)
        new double3(1.26826084676152, 2.61087432020563, -1.18361414421777),   // 7: Back-Right Bottom (Motor 13)

        // TODO(EE cables): replace with the measured EE pulley positions. These placeholders
        // copy the four top CoM pulleys so the solver has a plausible geometry for offline testing.
        new double3(-0.576835048372789, 2.57648944314966, 0.252733189583757),   // 8: Front-Right Middle (placeholder)
        new double3(-0.588436088357761, 0.0273606629681548, 0.182261108739911),  // 9: Front-Left Middle (placeholder)
        new double3(1.18955071881626, 0.0229493275829191, 0.264120148165004),    // 10: Back-Left Middle (placeholder)
        new double3(1.19164684688485, 2.58195169592441, 0.33072052196806)        // 11: Back-Right Middle (placeholder)
    };

    // Mapping from solver index to motor driver index for the full set
    // Corresponds to the order in AllPulleyPositions
    // motor mapping is: 10, 5, 4, 11, 8, 7, 2, 13
    // If we change to the middle four motors, the mapping is: _, _, _, _, 8, 7, 2, 13
    // TODO(EE cables): replace the four -1 with the EE motor driver indices (0-13).
    // Until then, 4 and 12 cables are refused.
    public static readonly int[] FullMotorMapping = new int[] { 9, 4, 3, 10, 7, 6, 1, 12, -1, -1, -1, -1 };

    // Which entries of AllPulleyPositions and FullMotorMapping belong to each body
    private static readonly int[] CoMCableIndices = new int[] { 0, 1, 2, 3, 4, 5, 6, 7 };
    private static readonly int[] EECableIndices = new int[] { 8, 9, 10, 11 };

    public readonly int[] SolverToMotorMap;

    private RobUSTDescription(double comAP, double comML,
                              double eeAP, double eeML, double userMass)
    {
        CoMAPDistance = comAP;
        CoMMLDistance = comML;
        EEAPDistance = eeAP;
        EEMLDistance = eeML;
        UserMass = userMass;

        // Force Plate Configuration ----
        double3 back_left_correction   = new double3( 0.03, -0.03, -0.01);
        double3 back_right_correction  = new double3( 0.03,  0.03, -0.01);
        double3 front_left_correction  = new double3(-0.03, -0.03, -0.01);
        double3 front_right_correction = new double3(-0.03,  0.03, -0.01);
        // vive tracker jig readings:
        FP_BackLeft   = new double3( 0.5385, 0.5793, -0.9520) + back_left_correction;
        FP_BackRight  = new double3( 0.5234, 1.1350, -0.9463) + back_right_correction;
        FP_FrontLeft  = new double3(-0.3202, 0.5510, -0.9525) + front_left_correction;
        FP_FrontRight = new double3(-0.3341, 1.1077, -0.9458) + front_right_correction;
        FloorZ = (FP_BackLeft.z + FP_BackRight.z + FP_FrontLeft.z + FP_FrontRight.z) / 4.0;
        
        NumCoMCables = CoMCableIndices.Length;
        FramePulleyPositions = new double3[NumCoMCables];
        LocalAttachmentPoints = new double3[NumCoMCables];
        SolverToMotorMap = new int[NumCoMCables];

        double halfCoMML = comML / 2.0;
        double halfEEML = eeML / 2.0;

        // Populate arrays based on active indices
        for (int i = 0; i < NumCoMCables; i++)
        {
            int srcIdx = CoMCableIndices[i];

            FramePulleyPositions[i] = AllPulleyPositions[srcIdx];
            SolverToMotorMap[i] = FullMotorMapping[srcIdx];

            // CoM Attachment Points
            if (srcIdx == 0 || srcIdx == 4) LocalAttachmentPoints[i] = new double3(-halfCoMML, -comAP, 0);
            else if (srcIdx == 1 || srcIdx == 5) LocalAttachmentPoints[i] = new double3(halfCoMML, -comAP, 0);
            else if (srcIdx == 2 || srcIdx == 6) LocalAttachmentPoints[i] = new double3(halfCoMML, 0, 0);
            else if (srcIdx == 3 || srcIdx == 7) LocalAttachmentPoints[i] = new double3(-halfCoMML, 0, 0);
        }

        CoMCenter_CoM_Frame = new double3(0, -comAP / 2.0, 0);

        // EE Cables ----
        NumEECables = EECableIndices.Length;
        EEFramePulleyPositions = new double3[NumEECables];
        EESolverToMotorMap = new int[NumEECables];
        for (int i = 0; i < NumEECables; i++)
        {
            EEFramePulleyPositions[i] = AllPulleyPositions[EECableIndices[i]];
            EESolverToMotorMap[i] = FullMotorMapping[EECableIndices[i]];
        }
        // Same corner order as the EE entries of AllPulleyPositions
        LocalEEAttachmentPoints = new double3[]
        {
            new double3(-halfEEML, -eeAP, 0), // 8: Front-Right
            new double3(halfEEML, -eeAP, 0),  // 9: Front-Left
            new double3(halfEEML, 0, 0),         // 10: Back-Left
            new double3(-halfEEML, 0, 0)         // 11: Back-Right
        };
        EECenter_EE_Frame = new double3(0, -eeAP / 2.0, 0);
        EEMotorError = CheckEEMotors(SolverToMotorMap, EESolverToMotorMap);
    }

    // EE motors must all be entered, in range, and not already CoM motors. Returns null when they are.
    private static string CheckEEMotors(int[] comMotors, int[] eeMotors)
    {
        for (int i = 0; i < eeMotors.Length; i++)
        {
            if (eeMotors[i] < 0)
                return "EE motor numbers are not entered in RobUSTDescription.FullMotorMapping. Enter them, or use 8 cables (CoM only).";
            if (eeMotors[i] > 13 || Array.IndexOf(comMotors, eeMotors[i]) >= 0 || Array.IndexOf(eeMotors, eeMotors[i]) != i)
                return $"EE motor {eeMotors[i]} in RobUSTDescription.FullMotorMapping is out of range (0-13) or already used by another cable.";
        }
        return null;
    }

    /// <summary>
    /// Factory method to create RobUST description from the CoM and EE configurations.
    /// All allocations happen here at init - nothing at runtime.
    /// </summary>
    /// <param name="comAPDistance">CoM anterior-posterior distance [m]</param>
    /// <param name="comMLDistance">CoM medial-lateral distance [m]</param>
    /// <param name="eeAPDistance">EE anterior-posterior distance [m]</param>
    /// <param name="eeMLDistance">EE medial-lateral distance [m]</param>
    /// <param name="userMass">User body mass [kg]</param>
    public static RobUSTDescription Create(double comAPDistance, double comMLDistance,
                                           double eeAPDistance, double eeMLDistance, double userMass)
    {
        return new RobUSTDescription(comAPDistance, comMLDistance, eeAPDistance, eeMLDistance, userMass);
    }

    public static RobUSTDescription Create(double comAPDistance, double comMLDistance, double userMass)
    {
        return new RobUSTDescription(comAPDistance, comMLDistance, 0.20, 0.28, userMass);
    }
}
