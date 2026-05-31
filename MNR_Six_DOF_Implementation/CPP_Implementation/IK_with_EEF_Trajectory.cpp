/*
 * MNR_Trajectory_IK.cpp
 *
 *  Created on: May 20, 2026
 *      Author: Dr. Hamza Khan
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

#include <Eigen/Dense>

#include "MNR_FK.h"
#include "MNR_J.h"

using namespace Eigen;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// =========================================================================
// MNR-Based Inverse Kinematics with End-Effector Trajectory Tracking
//
// This script implements a trajectory-based inverse kinematics (IK)
// algorithm using the proposed Modified Newton–Raphson (MNR) formulation.
//
// The algorithm tracks a Cartesian straight-line trajectory of the
// end-effector in both position and orientation space.
//
// Key Characteristics:
//   - Full pose error is defined using homogeneous transformation residuals
//     (rotation matrix frame alignment + position error).
//   - A modified Jacobian J_n (12×6) is used, derived symbolically offline
//     and implemented numerically in MNR_Jn.m.
//   - The solution at each trajectory point is used as the initial guess
//     for the next point to ensure smooth joint evolution.
//
// This implementation prioritizes clarity and reproducibility over
// computational efficiency, making it suitable for research and validation.
// =========================================================================


/* ------------------------------------------------------------------------
 * Helper Function: linspace
 * ------------------------------------------------------------------------- */
VectorXd linspace(double start, double end, int num)
{
    VectorXd values(num);

    if (num == 1)
    {
        values(0) = start;
        return values;
    }

    double step = (end - start) / static_cast<double>(num - 1);

    for (int i = 0; i < num; i++)
    {
        values(i) = start + step * static_cast<double>(i);
    }

    return values;
}


/* ------------------------------------------------------------------------
 * Main Function
 * ------------------------------------------------------------------------- */
int main()
{
    MNR_FK fk_solver;
    MNR_J  j_solver;

    // ------------------------------------------------------------------------
    // 1. Initialization
    // -------------------------------------------------------------------------

    // Initial joint configuration (radians)
    VectorXd q_ini(6);
    q_ini << 0, 89, 91, 0, 0, 180;
    q_ini = q_ini * M_PI / 180.0;

    VectorXd q = q_ini;

    // Trajectory and solver parameters
    int    N       = 1000;      // Number of Cartesian trajectory points
    double tol     = 1e-6;      // Convergence tolerance on task-space error
    int    maxIter = 200;       // Maximum NR iterations per trajectory point
    double C       = 1.0;       // Newton–Raphson step size (gain)

    // Storage (optional, useful for analysis and plotting)
    MatrixXd Q_traj  = MatrixXd::Zero(6, N);   // Joint trajectory
    VectorXd pos_err = VectorXd::Zero(N);      // Position error (mm)
    VectorXd ori_err = VectorXd::Zero(N);      // Orientation error (frame alignment norm)

    // ------------------------------------------------------------------------
    // 2. Cartesian Trajectory Definition
    // -------------------------------------------------------------------------

    // Initial and desired end-effector positions (meters)
    Vector3d P_ini;
    P_ini << 0, 0, 2.4465;

    Vector3d P_des;
    P_des << 0.6, -0.1, 0.8;

    // Initial and desired end-effector orientations (ZYX Euler angles)
    Vector3d Ori_ini;
    Ori_ini << 0, 0, 0;

    Vector3d Ori_des;
    Ori_des << 0, 90, 0;
    Ori_des = Ori_des * M_PI / 180.0;

    // Generate straight-line Cartesian position trajectory
    VectorXd Px = linspace(P_ini(0), P_des(0), N);
    VectorXd Py = linspace(P_ini(1), P_des(1), N);
    VectorXd Pz = linspace(P_ini(2), P_des(2), N);

    // Generate smooth orientation interpolation (Euler angles)
    VectorXd Rx = linspace(Ori_ini(0), Ori_des(0), N);
    VectorXd Ry = linspace(Ori_ini(1), Ori_des(1), N);
    VectorXd Rz = linspace(Ori_ini(2), Ori_des(2), N);

    // ------------------------------------------------------------------------
    // 3. Trajectory-Based Modified Newton–Raphson IK
    // -------------------------------------------------------------------------

    auto t_start = chrono::high_resolution_clock::now();

    VectorXd f = VectorXd::Zero(12);

    for (int k = 0; k < N; k++)
    {
        // -------------------------------------------------------------
        // Desired End-Effector Pose at Trajectory Point k
        // -------------------------------------------------------------
        double X  = Px(k);
        double Y  = Py(k);
        double Z  = Pz(k);
        double Tx = Rx(k);
        double Ty = Ry(k);
        double Tz = Rz(k);

        // Rotation matrices about X, Y, and Z axes
        Matrix3d RotX;
        RotX << 1,       0,        0,
                0, cos(Tx), -sin(Tx),
                0, sin(Tx),  cos(Tx);

        Matrix3d RotY;
        RotY <<  cos(Ty), 0, sin(Ty),
                      0, 1,       0,
                -sin(Ty), 0, cos(Ty);

        Matrix3d RotZ;
        RotZ << cos(Tz), -sin(Tz), 0,
                sin(Tz),  cos(Tz), 0,
                      0,        0, 1;

        // Desired end-effector rotation matrix
        Matrix3d R_des = RotX * RotY * RotZ;

        // Desired homogeneous transformation
        Matrix4d T_des = Matrix4d::Identity();
        T_des.block<3, 3>(0, 0) = R_des;
        T_des.block<3, 1>(0, 3) << X, Y, Z;

        // Use previous trajectory solution as initial guess
        VectorXd qk = q;

        // -------------------------------------------------------------
        // Newton–Raphson Iteration at Trajectory Point k
        // -------------------------------------------------------------
        for (int iter = 0; iter < maxIter; iter++)
        {
            // Forward kinematics at current joint configuration
            MatrixXd T_cur = fk_solver.FK(qk);

            // ---------------------------------------------------------
            // Task-Space Error Vector (12×1)
            // ---------------------------------------------------------
            // Rotation matrix element errors
            double f01  = T_cur(0, 0) - T_des(0, 0);
            double f02  = T_cur(0, 1) - T_des(0, 1);
            double f03  = T_cur(0, 2) - T_des(0, 2);

            double f05  = T_cur(1, 0) - T_des(1, 0);
            double f06  = T_cur(1, 1) - T_des(1, 1);
            double f07  = T_cur(1, 2) - T_des(1, 2);

            double f09  = T_cur(2, 0) - T_des(2, 0);
            double f010 = T_cur(2, 1) - T_des(2, 1);
            double f011 = T_cur(2, 2) - T_des(2, 2);

            // Position errors
            double f04  = T_cur(0, 3) - T_des(0, 3);
            double f08  = T_cur(1, 3) - T_des(1, 3);
            double f012 = T_cur(2, 3) - T_des(2, 3);

            // Combined task-space error vector (12×1)
            f << f01, f02, f03,
                 f05, f06, f07,
                 f09, f010, f011,
                 f04, f08, f012;

            // Convergence check
            if (f.norm() < tol)
            {
                break;
            }

            // ---------------------------------------------------------
            // Modified Jacobian Evaluation
            // ---------------------------------------------------------
            // J_n is evaluated numerically using expressions obtained
            // from symbolic differentiation (see Jacobian generation script).
            MatrixXd Jn = j_solver.MNR_Jacobian(qk);

            // ---------------------------------------------------------
            // Damped Least-Squares Newton–Raphson Update
            // ---------------------------------------------------------
            double Lamda = 0.001;  // Damping coefficient

            MatrixXd A = Jn.transpose() * Jn + MatrixXd::Identity(6, 6) * Lamda * Lamda;
            MatrixXd B = Jn.transpose();

            MatrixXd Jinv = A.inverse() * B;

            // Joint increment
            VectorXd dq = Jinv * f;

            // Joint update
            qk = qk - C * dq;
        }

        // Store solution for this trajectory point
        q = qk;
        Q_traj.col(k) = q;

        // -------------------------------------------------------------
        // Error Metrics (for analysis and reporting)
        // -------------------------------------------------------------
        pos_err(k) = (1000.0 * f.segment<3>(9)).norm();     // Position error in mm
        ori_err(k) = f.segment<9>(0).norm() * 180.0;        // Orientation frame error
    }

    // Total computation time
    auto t_end = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = t_end - t_start;

    double t_total = elapsed.count();

    // Average IK solution time (microseconds)
    double t_avg_us = (t_total / static_cast<double>(N)) * 1e6;

    // ------------------------------------------------------------------------
    // Results
    // ------------------------------------------------------------------------

    cout << fixed << setprecision(6);

    cout << "\nMNR-Based IK Trajectory Completed\n";
    cout << "Total trajectory points        : " << N << "\n";
    cout << "Total computation time (s)     : " << t_total << "\n";

    cout << setprecision(3);
    cout << "Average IK solution time (us)  : " << t_avg_us << "\n\n";

    cout << setprecision(6);
    cout << "Final joint configuration (rad):\n";
    cout << q.transpose() << "\n";

    cout << "Final position error (mm)      : " << pos_err(N - 1) << "\n";
    cout << "Final orientation error (deg)  : " << ori_err(N - 1) << "\n";


    // -------------------------------------------------------------------------
    // Notes on Trajectory Selection and Convergence
    // -------------------------------------------------------------------------
    // The convergence speed and numerical robustness of the proposed MNR-based
    // inverse kinematics algorithm strongly depend on the selection of the
    // end-effector trajectory.
    //
    // In practice, Cartesian trajectories should be planned while considering
    // the robot's physical structure, joint arrangement, and reachable workspace.
    // Trajectories that are kinematically feasible and consistent with the
    // manipulator's natural motion typically lead to faster convergence and
    // reduced iteration counts.
    //
    // Poorly conditioned trajectories (e.g., large orientation changes over
    // short distances, proximity to singular configurations, or motion near
    // joint limits) may increase the number of Newton–Raphson iterations or
    // degrade convergence performance.
    //
    // This example is intended as a reference implementation to demonstrate
    // the use of the modified Jacobian and MNR-based IK formulation. The
    // trajectory generation, solver parameters, and task formulation can and
    // should be adapted based on the specific robot geometry, task requirements,
    // and hardware constraints.
    //
    // -------------------------------------------------------------------------

    return 0;
}