clc; clear; close all;
% =========================================================================
% MNR-Based Inverse Kinematics with End-Effector Trajectory Tracking
%
% This script implements a trajectory-based inverse kinematics (IK)
% algorithm using the proposed Modified Newton–Raphson (MNR) formulation.
%
% The algorithm tracks a Cartesian straight-line trajectory of the
% end-effector in both position and orientation space.
%
% Key Characteristics:
%   - Full pose error is defined using homogeneous transformation residuals
%     (rotation matrix frame alignment + position error).
%   - A modified Jacobian J_n (12×6) is used, derived symbolically offline
%     and implemented numerically in MNR_Jn.m.
%   - The solution at each trajectory point is used as the initial guess
%     for the next point to ensure smooth joint evolution.
%
% This implementation prioritizes clarity and reproducibility over
% computational efficiency, making it suitable for research and validation.
% =========================================================================


%% ------------------------------------------------------------------------
% 1. Initialization
% -------------------------------------------------------------------------

% Initial joint configuration (radians)
q_ini = [0; 90; 90; 0; 0; 180] * pi/180;
q     = q_ini;

% Trajectory and solver parameters
N       = 1000;      % Number of Cartesian trajectory points
tol     = 1e-6;      % Convergence tolerance on task-space error
maxIter = 200;       % Maximum NR iterations per trajectory point
C       = 1.0;       % Newton–Raphson step size (gain)

% Storage (optional, useful for analysis and plotting)
Q_traj  = zeros(6, N);   % Joint trajectory
pos_err = zeros(1, N);   % Position error (mm)
ori_err = zeros(1, N);   % Orientation error (frame alignment norm)

%% ------------------------------------------------------------------------
% 2. Cartesian Trajectory Definition
% -------------------------------------------------------------------------

% Initial and desired end-effector positions (meters)
P_ini = [0; 0; 2.4465];
P_des = [0.6; 0; 0.8];

% Initial and desired end-effector orientations (ZYX Euler angles)
Ori_ini = [0 0 0];
Ori_des = [0 90 0] * pi/180;

% Generate straight-line Cartesian position trajectory
Px = linspace(P_ini(1), P_des(1), N);
Py = linspace(P_ini(2), P_des(2), N);
Pz = linspace(P_ini(3), P_des(3), N);

% Generate smooth orientation interpolation (Euler angles)
Rx = linspace(Ori_ini(1), Ori_des(1), N);
Ry = linspace(Ori_ini(2), Ori_des(2), N);
Rz = linspace(Ori_ini(3), Ori_des(3), N);

%% ------------------------------------------------------------------------
% 3. Trajectory-Based Modified Newton–Raphson IK
% -------------------------------------------------------------------------
tic
for k = 1:N

    % -------------------------------------------------------------
    % Desired End-Effector Pose at Trajectory Point k
    % -------------------------------------------------------------
    X = Px(k);  Y = Py(k);  Z = Pz(k);
    Tx = Rx(k); Ty = Ry(k); Tz = Rz(k);

    % Rotation matrices about X, Y, and Z axes
    RotX = [ 1     0           0;
             0 cos(Tx) -sin(Tx);
             0 sin(Tx)  cos(Tx) ];

    RotY = [ cos(Ty) 0 sin(Ty);
               0     1    0;
            -sin(Ty) 0 cos(Ty) ];

    RotZ = [ cos(Tz) -sin(Tz) 0;
             sin(Tz)  cos(Tz) 0;
               0         0    1 ];

    % Desired end-effector rotation matrix
    R_des = RotX * RotY * RotZ;

    % Desired homogeneous transformation
    T_des = eye(4);
    T_des(1:3,1:3) = R_des;
    T_des(1:3,4)   = [X; Y; Z];

    % Use previous trajectory solution as initial guess
    qk = q;

    % -------------------------------------------------------------
    % Newton–Raphson Iteration at Trajectory Point k
    % -------------------------------------------------------------
    for iter = 1:maxIter

        % Forward kinematics at current joint configuration
        T_cur = MNR_FK_(qk);

        % ---------------------------------------------------------
        % Task-Space Error Vector (12×1)
        % ---------------------------------------------------------
        % Rotation matrix element errors
        f01  = T_cur(1,1) - T_des(1,1);
        f02  = T_cur(1,2) - T_des(1,2);
        f03  = T_cur(1,3) - T_des(1,3);
    
        f05  = T_cur(2,1) - T_des(2,1);
        f06  = T_cur(2,2) - T_des(2,2);
        f07  = T_cur(2,3) - T_des(2,3);
    
        f09  = T_cur(3,1) - T_des(3,1);
        f010 = T_cur(3,2) - T_des(3,2);
        f011 = T_cur(3,3) - T_des(3,3);
    
        % Position errors
        f04  = T_cur(1,4) - T_des(1,4);
        f08  = T_cur(2,4) - T_des(2,4);
        f012 = T_cur(3,4) - T_des(3,4);

        % Combined task-space error vector (12×1)
        f = [ f01; f02; f03;
              f05; f06; f07;
              f09; f010; f011;
              f04; f08; f012 ];

        % Convergence check
        if norm(f) < tol
            break;
        end

        % ---------------------------------------------------------
        % Modified Jacobian Evaluation
        % ---------------------------------------------------------
        % J_n is evaluated numerically using expressions obtained
        % from symbolic differentiation (see Jacobian generation script).
        Jn = MNR_Jn_(qk);

        % ---------------------------------------------------------
        % Damped Least-Squares Newton–Raphson Update
        % ---------------------------------------------------------
        Lamda  = 0.01;  % Damping coefficient
        Jinv   = inv((Jn' * Jn + eye(6) * Lamda^2)) * Jn';

        % Joint increment
        dq = Jinv * f;
 
        % Joint update
        qk = qk - C * dq;
    end

    % Store solution for this trajectory point
    q = qk;
    Q_traj(:,k) = q;

    % -------------------------------------------------------------
    % Error Metrics (for analysis and reporting)
    % -------------------------------------------------------------
    pos_err(k) = norm(1000 * f(10:12));     % Position error in mm
    ori_err(k) = norm(f(1:9)) * 180;        % Orientation frame error

end

% Total computation time
t_total = toc;

% Average IK solution time (microseconds)
t_avg_us = (t_total / N) * 1e6;

%% ------------------------------------------------------------------------
% Results
% ------------------------------------------------------------------------

fprintf('\nMNR-Based IK Trajectory Completed\n');
fprintf('Total trajectory points        : %d\n', N);
fprintf('Total computation time (s)     : %.6f\n', t_total);
fprintf('Average IK solution time (µs)  : %.3f\n\n', t_avg_us);

fprintf('Final joint configuration (rad):\n');
disp(q');

fprintf('Final position error (mm)      : %.6f\n', pos_err(end));
fprintf('Final orientation error (deg)  : %.6f\n', ori_err(end));


% -------------------------------------------------------------------------
% Notes on Trajectory Selection and Convergence
% -------------------------------------------------------------------------
% The convergence speed and numerical robustness of the proposed MNR-based
% inverse kinematics algorithm strongly depend on the selection of the
% end-effector trajectory.
%
% In practice, Cartesian trajectories should be planned while considering
% the robot's physical structure, joint arrangement, and reachable workspace.
% Trajectories that are kinematically feasible and consistent with the
% manipulator's natural motion typically lead to faster convergence and
% reduced iteration counts.
%
% Poorly conditioned trajectories (e.g., large orientation changes over
% short distances, proximity to singular configurations, or motion near
% joint limits) may increase the number of Newton–Raphson iterations or
% degrade convergence performance.
%
% This example is intended as a reference implementation to demonstrate
% the use of the modified Jacobian and MNR-based IK formulation. The
% trajectory generation, solver parameters, and task formulation can and
% should be adapted based on the specific robot geometry, task requirements,
% and hardware constraints.
%
% -------------------------------------------------------------------------
