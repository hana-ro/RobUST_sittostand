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
    public readonly int NumCables;
    
    /// <summary>Pulley positions in robot frame [m] (double3 for SIMD)</summary>
    public readonly double3[] FramePulleyPositions;
    
    /// <summary>Cable attachment points on belt, in end-effector frame [m]</summary>
    public readonly double3[] LocalAttachmentPoints;
    
    /// <summary>Belt center in end-effector frame [m]</summary>
    public readonly double3 BeltCenter_EE_Frame;

    /// <summary>Number of trunk cables, which act on the trunk harness separately from the pelvis cables</summary>
    public readonly int NumTrunkCables;

    /// <summary>Trunk cable pulley positions in robot frame [m]</summary>
    public readonly double3[] TrunkFramePulleyPositions;

    /// <summary>Trunk cable attachment points on the trunk harness, in chest tracker frame [m]</summary>
    public readonly double3[] LocalTrunkAttachmentPoints;

    /// <summary>Trunk center in chest tracker frame [m]</summary>
    public readonly double3 TrunkCenter_EE_Frame;

    /// <summary>Mapping from trunk solver index to motor driver index</summary>
    public readonly int[] TrunkSolverToMotorMap;

    /// <summary>
    /// True once the trunk pulley positions and motor numbers have been entered and checked.
    /// Until then, trunk tensions are never sent to a motor.
    /// </summary>
    public readonly bool HasTrunkCables;
    
    // ============ User Parameters ============
    /// <summary>User body mass [kg]</summary>
    public readonly double UserMass;
    
    /// <summary>Pelvis anterior-posterior distance [m]</summary>
    public readonly double PelvisAPDistance;
    
    /// <summary>Pelvis medial-lateral distance [m]</summary>
    public readonly double PelvisMLDistance;

    /// <summary>Trunk anterior-posterior distance [m]</summary>
    public readonly double TrunkAPDistance;

    /// <summary>Trunk medial-lateral distance [m]</summary>
    public readonly double TrunkMLDistance;

    // ============ Force Plate Geometry ============
    public readonly double3 FP_BackLeft;
    public readonly double3 FP_BackRight;
    public readonly double3 FP_FrontLeft;
    public readonly double3 FP_FrontRight;

    /// <summary>Floor height in robot frame, taken from the force plate surface [m]</summary>
    public readonly double FloorZ;

    // Full 12-cable hardware definition (Static Database from vive tracker measurement)
    // 0-7 act on the pelvis belt, 8-11 on the trunk harness
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

        // TODO(trunk cables): replace with the measured trunk pulley positions. These placeholders
        // copy the four top pelvis pulleys so the solver has a plausible geometry for offline testing.
        new double3(-0.576835048372789, 2.57648944314966, 0.252733189583757),   // 8: Front-Right Middle (placeholder)
        new double3(-0.588436088357761, 0.0273606629681548, 0.182261108739911),  // 9: Front-Left Middle (placeholder)
        new double3(1.18955071881626, 0.0229493275829191, 0.264120148165004),    // 10: Back-Left Middle (placeholder)
        new double3(1.19164684688485, 2.58195169592441, 0.33072052196806)        // 11: Back-Right Middle (placeholder)
    };

    // Mapping from solver index to motor driver index for the full set
    // Corresponds to the order in AllPulleyPositions
    // motor mapping is: 10, 5, 4, 11, 8, 7, 2, 13
    // If we change to the middle four motors, the mapping is: _, _, _, _, 8, 7, 2, 13
    // TODO(trunk cables): replace the four -1 with the trunk motor driver indices (0-13).
    // Trunk motors are never commanded while any of them is -1.
    public static readonly int[] FullMotorMapping = new int[] { 9, 4, 3, 10, 7, 6, 1, 12, -1, -1, -1, -1 };

    // Which entries of AllPulleyPositions and FullMotorMapping belong to each body
    private static readonly int[] PelvisCableIndices = new int[] { 0, 1, 2, 3, 4, 5, 6, 7 };
    private static readonly int[] TrunkCableIndices = new int[] { 8, 9, 10, 11 };

    public readonly int[] SolverToMotorMap;

    private RobUSTDescription(int numCables, double pelvisAP, double pelvisML, 
                              double trunkAP, double trunkML, double userMass)
    {
        NumCables = numCables;
        PelvisAPDistance = pelvisAP;
        PelvisMLDistance = pelvisML;
        TrunkAPDistance = trunkAP;
        TrunkMLDistance = trunkML;
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
        
        FramePulleyPositions = new double3[numCables];
        LocalAttachmentPoints = new double3[numCables];
        SolverToMotorMap = new int[numCables];

        // Determine active subset based on requested numCables
        int[] activeIndices = numCables switch
        {
            8 => PelvisCableIndices,
            4 => new int[] { 0, 1, 2, 3 }, // Top cables only
            _ => throw new System.ArgumentException($"Unsupported cable count: {numCables}. valid options: 4, 8")
        };

        double halfPelvisML = pelvisML / 2.0;
        double halfTrunkML = trunkML / 2.0;

        // Populate arrays based on active indices
        for (int i = 0; i < numCables; i++)
        {
            int srcIdx = activeIndices[i];

            FramePulleyPositions[i] = AllPulleyPositions[srcIdx];
            SolverToMotorMap[i] = FullMotorMapping[srcIdx];

            // Pelvis Attachment Points
            if (srcIdx == 0 || srcIdx == 4) LocalAttachmentPoints[i] = new double3(-halfPelvisML, -pelvisAP, 0);
            else if (srcIdx == 1 || srcIdx == 5) LocalAttachmentPoints[i] = new double3(halfPelvisML, -pelvisAP, 0);
            else if (srcIdx == 2 || srcIdx == 6) LocalAttachmentPoints[i] = new double3(halfPelvisML, 0, 0);
            else if (srcIdx == 3 || srcIdx == 7) LocalAttachmentPoints[i] = new double3(-halfPelvisML, 0, 0);
        }

        BeltCenter_EE_Frame = new double3(0, -pelvisAP / 2.0, 0);

        // Trunk Cables ----
        NumTrunkCables = TrunkCableIndices.Length;
        TrunkFramePulleyPositions = new double3[NumTrunkCables];
        TrunkSolverToMotorMap = new int[NumTrunkCables];
        for (int i = 0; i < NumTrunkCables; i++)
        {
            TrunkFramePulleyPositions[i] = AllPulleyPositions[TrunkCableIndices[i]];
            TrunkSolverToMotorMap[i] = FullMotorMapping[TrunkCableIndices[i]];
        }
        // Same corner order as the trunk entries of AllPulleyPositions
        LocalTrunkAttachmentPoints = new double3[]
        {
            new double3(-halfTrunkML, -trunkAP, 0), // 8: Front-Right
            new double3(halfTrunkML, -trunkAP, 0),  // 9: Front-Left
            new double3(halfTrunkML, 0, 0),         // 10: Back-Left
            new double3(-halfTrunkML, 0, 0)         // 11: Back-Right
        };
        TrunkCenter_EE_Frame = new double3(0, -trunkAP / 2.0, 0);
        HasTrunkCables = ValidateTrunkMotors(SolverToMotorMap, TrunkSolverToMotorMap);
    }

    // Trunk motors are used only when every one is set, in range, and not already a pelvis motor.
    private static bool ValidateTrunkMotors(int[] pelvisMotors, int[] trunkMotors)
    {
        for (int i = 0; i < trunkMotors.Length; i++)
        {
            if (trunkMotors[i] < 0) return false; // not entered yet
            if (trunkMotors[i] > 13 || Array.IndexOf(pelvisMotors, trunkMotors[i]) >= 0 || Array.IndexOf(trunkMotors, trunkMotors[i]) != i)
            {
                Debug.LogError($"[RobUSTDescription] Trunk motor {trunkMotors[i]} is out of range or already used. Trunk cables are disabled.");
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Factory method to create RobUST description from belt/pelvis and trunk configurations.
    /// All allocations happen here at init - nothing at runtime.
    /// </summary>
    /// <param name="numCables">Number of cables in the system</param>
    /// <param name="pelvisAPDistance">Pelvis anterior-posterior distance [m]</param>
    /// <param name="pelvisMLDistance">Pelvis medial-lateral distance [m]</param>
    /// <param name="trunkAPDistance">Trunk anterior-posterior distance [m]</param>
    /// <param name="trunkMLDistance">Trunk medial-lateral distance [m]</param>
    /// <param name="userMass">User body mass [kg]</param>
    public static RobUSTDescription Create(int numCables, double pelvisAPDistance, double pelvisMLDistance,
                                           double trunkAPDistance, double trunkMLDistance, double userMass)
    {
        return new RobUSTDescription(numCables, pelvisAPDistance, pelvisMLDistance, trunkAPDistance, trunkMLDistance, userMass);
    }

    public static RobUSTDescription Create(int numCables, double pelvisAPDistance, double pelvisMLDistance,
                                           double userMass)
    {
        return new RobUSTDescription(numCables, pelvisAPDistance, pelvisMLDistance, 0.20, 0.28, userMass);
    }
}
