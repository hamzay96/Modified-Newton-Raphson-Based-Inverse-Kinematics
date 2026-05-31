/*
 * Example: MNR-Based Inverse Kinematics (Basic Implementation)
 *
 * This script demonstrates a basic implementation of the proposed
 * Modified Newton–Raphson (MNR) inverse kinematics algorithm using the
 * modified Jacobian J_n.
 *
 * Key features:
 *   - Orientation error is defined using rotation matrix elements
 *     (frame alignment), not Euler angles.
 *   - The Jacobian is evaluated numerically using the function MNR_Jn,
 *     whose expressions were derived symbolically offline.
 *   - The implementation focuses on clarity and reproducibility rather
 *     than computational optimization.
 */

#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "MNR_FK.h"
#include "MNR_J.h"

using namespace Eigen;
using namespace std;


// -------------------------------------------------------------------------
// Utility Function: Degree to Radian
// -------------------------------------------------------------------------
double deg2rad(double deg)
{
    return deg * 3.141592653589793 / 180.0;
}


// -------------------------------------------------------------------------
// Utility Function: Euler ZYX to Rotation Matrix
// -------------------------------------------------------------------------
// Equivalent to MATLAB:
// R_des = eul2rotm(deg2rad([0 90 0]), 'ZYX');
//
// ZYX means:
// R = Rz(yaw) * Ry(pitch) * Rx(roll)
// -------------------------------------------------------------------------
Matrix3d eul2rotm_ZYX(double z, double y, double x)
{
    Matrix3d Rz;
    Rz << cos(z), -sin(z), 0,
          sin(z),  cos(z), 0,
               0,       0, 1;

    Matrix3d Ry;
    Ry <<  cos(y), 0, sin(y),
                0, 1,      0,
          -sin(y), 0, cos(y);

    Matrix3d Rx;
    Rx << 1,      0,       0,
          0, cos(x), -sin(x),
          0, sin(x),  cos(x);

    return Rz * Ry * Rx;
}


int main()
{
    // =========================================================================
    // Example: MNR-Based Inverse Kinematics (Basic Implementation)
    //
    // This script demonstrates a basic implementation of the proposed
    // Modified Newton–Raphson (MNR) inverse kinematics algorithm using the
    // modified Jacobian J_n.
    //
    // Key features:
    //   - Orientation error is defined using rotation matrix elements
    //     (frame alignment), not Euler angles.
    //   - The Jacobian is evaluated numerically using the function MNR_Jn.m,
    //     whose expressions were derived symbolically offline.
    //   - The implementation focuses on clarity and reproducibility rather
    //     than computational optimization.
    // =========================================================================


    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    // Initial joint configuration (rad)
    VectorXd q = VectorXd::Zero(6);

    // Convergence parameters
    double tol      = 1e-6;    // convergence tolerance
    int maxIter     = 1000;    // maximum number of iterations
    double C        = 0.6;     // Damping Coefficient (NR gain)

    // Storage for joint history (optional)
    vector<VectorXd> Theta;
    Theta.push_back(q);

    // Storage for error history
    vector<double> err;


    // -------------------------------------------------------------------------
    // Desired end-effector pose
    // -------------------------------------------------------------------------

    // Desired position (meters)
    Vector3d p_des;
    p_des << 0.5, 0.3, 0.6;

    // Desired orientation (example, defined via Euler angles)
    // NOTE: Euler angles are used ONLY to generate R_d.
    // The IK error is computed using rotation matrix elements.
    Matrix3d R_des = eul2rotm_ZYX(
        deg2rad(0.0),
        deg2rad(90.0),
        deg2rad(0.0)
    );

    // Desired homogeneous transformation
    Matrix4d T_des = Matrix4d::Identity();
    T_des.block<3,1>(0,3) = p_des;
    T_des.block<3,3>(0,0) = R_des;


    // -------------------------------------------------------------------------
    // Create FK and Jacobian objects
    // -------------------------------------------------------------------------

    MNR_FK fk_solver;
    MNR_J  jac_solver;


    // -------------------------------------------------------------------------
    // Modified Newton–Raphson Iteration
    // -------------------------------------------------------------------------

    int iter = 0;

    for (iter = 1; iter <= maxIter; iter++)
    {
        // Forward kinematics at current joint configuration
        MatrixXd T_cur = fk_solver.FK(q);

        // -----------------------------------------------------
        // Task-space error vector (frame alignment + position)
        // -----------------------------------------------------
        // Rotation matrix element errors
        double f01  = T_cur(0,0) - T_des(0,0);
        double f02  = T_cur(0,1) - T_des(0,1);
        double f03  = T_cur(0,2) - T_des(0,2);

        double f05  = T_cur(1,0) - T_des(1,0);
        double f06  = T_cur(1,1) - T_des(1,1);
        double f07  = T_cur(1,2) - T_des(1,2);

        double f09  = T_cur(2,0) - T_des(2,0);
        double f010 = T_cur(2,1) - T_des(2,1);
        double f011 = T_cur(2,2) - T_des(2,2);

        // Position errors
        double f04  = T_cur(0,3) - T_des(0,3);
        double f08  = T_cur(1,3) - T_des(1,3);
        double f012 = T_cur(2,3) - T_des(2,3);

        // Combined task-space error vector (12×1)
        VectorXd f(12);
        f << f01, f02, f03,
             f05, f06, f07,
             f09, f010, f011,
             f04, f08, f012;

        // Error norm for convergence monitoring
        double current_err = f.norm();
        err.push_back(current_err);

        // Convergence check
        if (current_err < tol)
        {
            break;
        }

        // -----------------------------------------------------
        // Modified Jacobian evaluation
        // -----------------------------------------------------
        MatrixXd Jn = jac_solver.MNR_Jacobian(q);

        // -----------------------------------------------------
        // Newton–Raphson joint update
        // -----------------------------------------------------
        // Pseudoinverse is used here for robustness
        double Lamda = 0.1;

        MatrixXd I6 = MatrixXd::Identity(6, 6);

        MatrixXd invJ =
            (Jn.transpose() * Jn + I6 * Lamda * Lamda).inverse()
            * Jn.transpose();

        VectorXd dq = invJ * f;

        // Joint update with step size
        q = q - C * dq;

        // Store joint history
        Theta.push_back(q);
    }


    // -------------------------------------------------------------------------
    // Results
    // -------------------------------------------------------------------------

    cout << endl;
    cout << "MNR-Based IK completed in " << iter << " iterations" << endl;

    cout << "Final joint configuration (rad):" << endl;
    cout << q.transpose() << endl;

    // Final forward kinematics
    MatrixXd T_final = fk_solver.FK(q);

    // Position error (converted to mm)
    Vector3d pos_err;
    pos_err << T_final(0,3) - T_des(0,3),
               T_final(1,3) - T_des(1,3),
               T_final(2,3) - T_des(2,3);

    pos_err = 1000.0 * pos_err;

    // Orientation error (frame alignment error)
    VectorXd ori_err(9);
    ori_err << T_final(0,0) - T_des(0,0),
               T_final(0,1) - T_des(0,1),
               T_final(0,2) - T_des(0,2),
               T_final(1,0) - T_des(1,0),
               T_final(1,1) - T_des(1,1),
               T_final(1,2) - T_des(1,2),
               T_final(2,0) - T_des(2,0),
               T_final(2,1) - T_des(2,1),
               T_final(2,2) - T_des(2,2);

    cout << "Final position error norm (mm): "
         << pos_err.norm() << endl;

    cout << "Final orientation error (deg): "
         << ori_err.norm() * 180.0 << endl << endl;

    cout << "Desired pose T_d:" << endl;
    cout << T_des << endl << endl;

    cout << "Final end-effector pose T(q):" << endl;
    cout << T_final << endl;

    return 0;
}