using UnityEngine;
using Unity.Mathematics;
using System;
using static Unity.Mathematics.math;

/// <summary>
/// Performs the physics calculations to determine the required cable tensions
/// based on the robot's state and external forces.
/// One planner serves one set of cables on one body (the CoM cables or the EE cables).
///
/// OPTIMIZATION NOTES:
/// - Uses Unity.Mathematics (SIMD-optimized) for all vector/matrix operations
/// - Pre-allocates all arrays during construction - no runtime allocations
/// - ALGLIB still allocates internally (~9.2 KB per solve) - see CableTensionPlannerNative for zero-alloc version
/// - Reuses QP state with warm-start for faster convergence
/// </summary>
public class CableTensionPlanner
{
    // Cable set geometry (shared with the RobUST description, never modified here)
    private readonly int numCables;
    private readonly double3[] pulleyPositions;
    private readonly double3[] localAttachmentPoints;
    private readonly double3 bodyCenter;
    private readonly bool isExactForce;

    // Pre-allocated QP matrices (allocated once, reused every calculation)
    private double[] tensions;
    private double[,] quadratic;
    private double[] linear;
    private double[,] sMatrix; // 6xN structure matrix for force/torque constraints
    private double[] tensionLower; // Box constraint lower bounds
    private double[] tensionUpper; // Box constraint upper bounds
    private double[] lowerWrenchBounds;
    private double[] upperWrenchBounds;

    // Solver parameters
    private double forceEpsilon = 0.01;
    private double torqueEpsilon = 1000;
    private double minTension = 10.0;
    private double maxTension = 200;
    private const double TensionRegularization = 1e-5; // closest-force mode: keeps tensions low among equal fits
    private const int MaxConsecutiveFailures = 5; // 50 ms at 100 Hz
    private int consecutiveFailures = 0;

    // Add new fields for QP state and warm start solution
    private alglib.minqpstate qpState;
    private double[] previousSolution;

    /// <summary>
    /// Planner for the CoM cables. Tensions must produce the requested force exactly.
    /// </summary>
    public static CableTensionPlanner ForCoM(RobUSTDescription robot)
    {
        if (robot == null) throw new ArgumentNullException(nameof(robot));
        return new CableTensionPlanner(robot.FramePulleyPositions, robot.LocalAttachmentPoints, robot.CoMCenter_CoM_Frame, true);
    }

    /// <summary>
    /// Planner for the EE cables. Too few cables to reach every force, so tensions produce the
    /// closest achievable force instead, and a zero request settles at minimum tension.
    /// </summary>
    public static CableTensionPlanner ForEE(RobUSTDescription robot)
    {
        if (robot == null) throw new ArgumentNullException(nameof(robot));
        return new CableTensionPlanner(robot.EEFramePulleyPositions, robot.LocalEEAttachmentPoints, robot.EECenter_EE_Frame, false);
    }

    /// <summary>
    /// Constructs and initializes the tension planner for one cable set.
    /// All allocations happen here - zero runtime allocations.
    /// </summary>
    /// <param name="pulleyPositions">Pulley positions in robot frame [m]</param>
    /// <param name="localAttachmentPoints">Attachment points in the body tracker frame [m]</param>
    /// <param name="bodyCenter">Body center in the body tracker frame [m], for torque arms</param>
    /// <param name="exactForce">True to require the requested force, false to get as close as the cables allow</param>
    public CableTensionPlanner(double3[] pulleyPositions, double3[] localAttachmentPoints, double3 bodyCenter, bool exactForce)
    {
        this.pulleyPositions = pulleyPositions ?? throw new ArgumentNullException(nameof(pulleyPositions));
        this.localAttachmentPoints = localAttachmentPoints ?? throw new ArgumentNullException(nameof(localAttachmentPoints));
        this.bodyCenter = bodyCenter;
        isExactForce = exactForce;
        numCables = pulleyPositions.Length;

        // Pre-allocate QP-specific data structures
        tensions = new double[numCables];
        quadratic = new double[numCables, numCables];
        linear = new double[numCables];
        sMatrix = new double[6, numCables]; // 6xnumCables structure matrix
        tensionLower = new double[numCables]; // box constraints arrays
        tensionUpper = new double[numCables];
        lowerWrenchBounds = new double[6]; // linear constraints arrays
        upperWrenchBounds = new double[6];

        // Initialize quadratic matrix: P = 2*Identity (constant for the exact-force mode)
        for (int i = 0; i < numCables; i++)
            quadratic[i, i] = 2.0;

        // Initialize tension bounds (constant, never changes)
        for (int i = 0; i < numCables; i++)
        {
            tensionLower[i] = minTension;  // Minimum tension
            tensionUpper[i] = maxTension; // Maximum tension
        }

        // Initialize previousSolution with a reasonable default
        previousSolution = new double[numCables];
        for (int i = 0; i < numCables; i++)
            previousSolution[i] = 10.0;

        // initialize qp solver
        alglib.minqpcreate(numCables, out qpState); // Initialize QP state once and reuse it for subsequent solves
        alglib.minqpsetalgodenseipm(qpState, 1e-6);  // Stopping tolerance: 1e-6
        alglib.minqpsetquadraticterm(qpState, quadratic, true);
        alglib.minqpsetlinearterm(qpState, linear);
        alglib.minqpsetbc(qpState, tensionLower, tensionUpper);
    }

    /// <summary>
    /// Calculates the desired cable tensions based on real-time tracker and force data.
    /// All calculations use Unity.Mathematics SIMD types for performance.
    /// </summary>
    /// <param name="bodyInRobotFrame">The 4x4 pose matrix of the body tracker already transformed into Robot Frame.</param>
    /// <param name="desiredWrench">The desired wrench (force and torque) to be applied by the cables.</param>
    public double[] CalculateTensions(double4x4 bodyInRobotFrame, Wrench desiredWrench)
    {
        BuildStructureMatrix(bodyInRobotFrame);

        // Extract wrench components (already double3, no conversion needed)
        double3 desiredForce = desiredWrench.Force;
        double3 desiredTorque = desiredWrench.Torque;

        if (isExactForce)
        {
            // Update constraint bounds (2-sided with epsilon tolerance)
            lowerWrenchBounds[0] = desiredForce.x - forceEpsilon;
            upperWrenchBounds[0] = desiredForce.x + forceEpsilon;
            lowerWrenchBounds[1] = desiredForce.y - forceEpsilon;
            upperWrenchBounds[1] = desiredForce.y + forceEpsilon;
            lowerWrenchBounds[2] = desiredForce.z - forceEpsilon;
            upperWrenchBounds[2] = desiredForce.z + forceEpsilon;
            lowerWrenchBounds[3] = desiredTorque.x - torqueEpsilon;
            upperWrenchBounds[3] = desiredTorque.x + torqueEpsilon;
            lowerWrenchBounds[4] = desiredTorque.y - torqueEpsilon;
            upperWrenchBounds[4] = desiredTorque.y + torqueEpsilon;
            lowerWrenchBounds[5] = desiredTorque.z - torqueEpsilon;
            upperWrenchBounds[5] = desiredTorque.z + torqueEpsilon;

            alglib.minqpsetlc2dense(qpState, sMatrix, lowerWrenchBounds, upperWrenchBounds, 6);
        }
        else
        {
            // Closest force: minimize |S_f T - F|^2 + reg |T|^2, torque left free like the exact mode
            for (int i = 0; i < numCables; i++)
            {
                linear[i] = -2.0 * ((sMatrix[0, i] * desiredForce.x) + (sMatrix[1, i] * desiredForce.y) + (sMatrix[2, i] * desiredForce.z));
                for (int j = 0; j < numCables; j++)
                {
                    double sTs = (sMatrix[0, i] * sMatrix[0, j]) + (sMatrix[1, i] * sMatrix[1, j]) + (sMatrix[2, i] * sMatrix[2, j]);
                    quadratic[i, j] = 2.0 * (sTs + (i == j ? TensionRegularization : 0.0));
                }
            }
            alglib.minqpsetquadraticterm(qpState, quadratic, true);
            alglib.minqpsetlinearterm(qpState, linear);
        }

        // Solve QP Problem using Dense IPM (ALGLIB - still allocates internally)
        alglib.minqpsetstartingpoint(qpState, previousSolution); // warm start

        alglib.minqpoptimize(qpState);
        alglib.minqpresults(qpState, out tensions, out var report);

        // Check solver result
        if (report.terminationtype < 0)
        {
            // Brief failures hold the last tensions. Sustained ones drop to minimum tension
            // rather than holding a wrench computed for a pose the subject has left.
            consecutiveFailures++;
            if (consecutiveFailures == MaxConsecutiveFailures)
            {
                Debug.LogWarning($"[CableTensionPlanner] QP solver failed {consecutiveFailures} times in a row (code {report.terminationtype}). Dropping to minimum tension.");
                for (int i = 0; i < numCables; i++)
                    previousSolution[i] = minTension;
            }
            return previousSolution;
        }
        consecutiveFailures = 0;

        // Update warm-start buffer using Buffer.BlockCopy (faster than Array.Copy for primitives)
        Buffer.BlockCopy(tensions, 0, previousSolution, 0, numCables * sizeof(double));
        return tensions;
    }

    /// <summary>
    /// Calculates the resultant wrench (force and torque) that would be applied to the body
    /// given a specific set of cable tensions. Effectivley performs W_resultant = S * T.
    /// </summary>
    /// <param name="bodyInRobotFrame">The 4x4 pose matrix of the body tracker in Robot Frame.</param>
    /// <param name="solver_tensions">The array of cable tension values.</param>
    /// <returns>The calculated resultant Wrench.</returns>
    public Wrench CalculateResultantWrench(double4x4 bodyInRobotFrame, double[] solver_tensions)
    {
        double3 resultantForce = new double3(0);
        double3 resultantTorque = new double3(0);

        // Iterate over each cable to sum up forces and torques
        for (int i = 0; i < numCables; i++)
        {
            // 1. Transform attachment point to robot frame
            double3 attachRobotFrame = TransformPoint(bodyInRobotFrame, localAttachmentPoints[i]);

            // 2. Calculate cable unit direction vector u_i (pulley -> attachment)
            double3 cableVec = pulleyPositions[i] - attachRobotFrame;
            double3 u_i = normalize(cableVec);

            // 3. Force Contribution: F_i = T_i * u_i
            double3 force_i = u_i * solver_tensions[i];
            resultantForce += force_i;

            // 4. Calculate torque arm: r = attachment - body center (in global frame)
            double3 r_robotFrame = TransformVector(bodyInRobotFrame, localAttachmentPoints[i] - bodyCenter); // vector rotation only

            // 5. Torque Contribution: Tau_i = r x F_i
            double3 torque_i = cross(r_robotFrame, force_i);
            resultantTorque += torque_i;
        }

        return new Wrench(resultantForce, resultantTorque);
    }

    // Fills the 6xN structure matrix for the current body pose
    private void BuildStructureMatrix(in double4x4 bodyInRobotFrame)
    {
        for (int i = 0; i < numCables; i++)
        {
            // Transform attachment point to robot frame
            double3 attachRobotFrame = TransformPoint(bodyInRobotFrame, localAttachmentPoints[i]);

            // Calculate cable direction vector (pulley - attachment)
            double3 cableVec = pulleyPositions[i] - attachRobotFrame;
            double3 u_i = normalize(cableVec);

            // Calculate torque arm: from body center to attachment point
            double3 r_robotFrame = TransformVector(bodyInRobotFrame, localAttachmentPoints[i] - bodyCenter);

            // Torque component using SIMD cross product
            double3 torque_i = cross(r_robotFrame, u_i);

            // Fill structure matrix directly
            sMatrix[0, i] = u_i.x;
            sMatrix[1, i] = u_i.y;
            sMatrix[2, i] = u_i.z;
            sMatrix[3, i] = torque_i.x;
            sMatrix[4, i] = torque_i.y;
            sMatrix[5, i] = torque_i.z;
        }
    }


    // ============ Unity.Mathematics Helpers (SIMD) ============

    /// <summary>
    /// Transform point by 4x4 matrix (includes translation).
    /// Uses SIMD mul operation.
    /// </summary>
    private static double3 TransformPoint(double4x4 m, double3 p)
    {
        double4 p4 = new double4(p, 1.0);
        double4 result = mul(m, p4);
        return result.xyz;
    }

    /// <summary>
    /// Transform vector by 4x4 matrix (rotation only, no translation).
    /// Uses SIMD mul operation.
    /// </summary>
    private static double3 TransformVector(double4x4 m, double3 v)
    {
        double4 v4 = new double4(v, 0.0);   // w = 0
        double4 result = mul(m, v4);
        return result.xyz;
    }

}
