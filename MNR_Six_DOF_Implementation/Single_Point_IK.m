clc;
clear;

% =========================================================================
% Example: MNR-Based Inverse Kinematics (Basic Implementation)
%
% This script demonstrates a basic implementation of the proposed
% Modified Newton–Raphson (MNR) inverse kinematics algorithm using the
% modified Jacobian J_n.
%
% Key features:
%   - Orientation error is defined using rotation matrix elements
%     (frame alignment), not Euler angles.
%   - The Jacobian is evaluated numerically using the function MNR_Jn.m,
%     whose expressions were derived symbolically offline.
%   - The implementation focuses on clarity and reproducibility rather
%     than computational optimization.
% =========================================================================


%% ------------------------------------------------------------------------
% Initialization
% -------------------------------------------------------------------------

% Initial joint configuration (rad)
q = zeros(6,1);

% Convergence parameters
tol      = 1e-6;     % convergence tolerance
maxIter  = 1000;     % maximum number of iterations
C        = 0.6;      % Damping Coefficient (NR gain)

% Storage for joint history (optional)
Theta = q;


%% ------------------------------------------------------------------------
% Desired end-effector pose
% -------------------------------------------------------------------------

% Desired position (meters)
p_des = [0.5; 0.3; 0.6];

% Desired orientation (example, defined via Euler angles)
% NOTE: Euler angles are used ONLY to generate R_d.
% The IK error is computed using rotation matrix elements.
R_des = eul2rotm(deg2rad([0 90 0]), 'ZYX');

% Desired homogeneous transformation
T_des = eye(4);
T_des(1:3,4)   = p_des;
T_des(1:3,1:3) = R_des;


%% ------------------------------------------------------------------------
% Modified Newton–Raphson Iteration
% -------------------------------------------------------------------------

for iter = 1:maxIter

    % Forward kinematics at current joint configuration
    T_cur = MNR_FK_(q);

    % -----------------------------------------------------
    % Task-space error vector (frame alignment + position)
    % -----------------------------------------------------
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

    % Error norm for convergence monitoring
    err(iter) = norm(f);

    % Convergence check
    if err(iter) < tol
        break;
    end

    % -----------------------------------------------------
    % Modified Jacobian evaluation
    % -----------------------------------------------------
    Jn = MNR_Jn_(q);

    % -----------------------------------------------------
    % Newton–Raphson joint update
    % -----------------------------------------------------
    % Pseudoinverse is used here for robustness
    Lamda  = 0.1;
    invJ   = inv((Jn' * Jn + eye(6) * Lamda^2)) * Jn';

    dq = invJ * f;

    % Joint update with step size
    q = q - C * dq;

    % Store joint history
    Theta(:,iter+1) = q;

end

%% ------------------------------------------------------------------------
% Results
% -------------------------------------------------------------------------

fprintf('\nMNR-Based IK completed in %d iterations\n', iter);

disp('Final joint configuration (rad):');
disp(q');

% Final forward kinematics
T_final = MNR_FK_(q);

% Position error (converted to mm)
pos_err = 1000 * [ T_final(1,4) - T_des(1,4);
                   T_final(2,4) - T_des(2,4);
                   T_final(3,4) - T_des(3,4) ];

% Orientation error (frame alignment error)
ori_err = [ T_final(1,1) - T_des(1,1);
            T_final(1,2) - T_des(1,2);
            T_final(1,3) - T_des(1,3);
            T_final(2,1) - T_des(2,1);
            T_final(2,2) - T_des(2,2);
            T_final(2,3) - T_des(2,3);
            T_final(3,1) - T_des(3,1);
            T_final(3,2) - T_des(3,2);
            T_final(3,3) - T_des(3,3) ];

fprintf('Final position error norm (mm): %.6f\n', norm(pos_err));
fprintf('Final orientation error (deg): %.6f\n\n', norm(ori_err) * 180);

disp('Desired pose T_d:');
disp(T_des);

disp('Final end-effector pose T(q):');
disp(T_final);
