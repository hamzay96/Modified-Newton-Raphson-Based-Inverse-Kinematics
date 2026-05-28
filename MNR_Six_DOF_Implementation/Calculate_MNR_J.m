clc; clear;

% =========================================================================
% Example: Symbolic Computation of the Modified Jacobian for a 6-DOF Robot
%
% This script derives the modified Jacobian matrix used in the proposed
% MNR-based inverse kinematics algorithm. The Jacobian is obtained by
% differentiating the end-effector rotation matrix elements and position
% vector with respect to the joint variables.
%
% Key characteristics:
%   - Orientation is represented using rotation matrix elements (frame
%     alignment) instead of Euler angles.
%   - The resulting Jacobian maps joint velocities to changes in
%     end-effector frame alignment and position in the global frame.
%   - The final Jacobian is saved in symbolic text form and can be directly
%     copied into a numerical function (e.g., MNR_J.m).
% =========================================================================


%% ------------------------------------------------------------------------
% Robot geometric parameters (link lengths in meters)
% -------------------------------------------------------------------------

L1 = 0.692;
L2 = 0.65;
L3 = 0.65;
L4 = 0.4545;

Lengths = [L1 L2 L3 L4]; % Add lengths variable if more such as L5 L6...

syms L1 L2 L3 L4 real

%% ------------------------------------------------------------------------
% Denavit–Hartenberg (DH) parameters
% -------------------------------------------------------------------------
% d     : link offsets
% a     : link lengths
% alpha : link twists
%
% NOTE:
% alpha values are provided directly in radians as symbolic constants
% (e.g., 90degrees = 1.5708 radians ≈ pi/2) to improve symbolic simplification stability.

d1=L1         ; a1=0        ;  alpha1 = pi/2;
d2=0          ; a2=L2       ;  alpha2 = 0;
d3=0          ; a3=0        ;  alpha3 = pi/2;
d4=L3         ; a4=0        ;  alpha4 =-pi/2;
d5=0          ; a5=0        ;  alpha5 = pi/2;
d6=L4         ; a6=0        ;  alpha6 = 0;

d       = [d1 d2 d3 d4 d5 d6];
a       = [a1 a2 a3 a4 a5 a6];
alpha   = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6];

%% ------------------------------------------------------------------------
% Symbolic joint variables
% -------------------------------------------------------------------------
% Joint variables are assumed real-valued, which assists symbolic
% simplification of trigonometric expressions.

syms t1 t2 t3 t4 t5 t6 real
q = [t1 t2 t3 t4 t5 t6];
n = length(q);

%% ------------------------------------------------------------------------
% Symbolic forward kinematics
% -------------------------------------------------------------------------
% The homogeneous transformation matrix from the base frame to the
% end-effector frame is computed by successive multiplication of DH
% transformation matrices.

T = eye(4);

for i = 1:n
    A = dh_transform(q(i), d(i), a(i), alpha(i));
    T = T * A;
end

% End-effector transformation matrix
% A small numerical precision is applied to reduce symbolic noise.
T_ee = vpa(T, 5);

%% ------------------------------------------------------------------------
% Task vector construction
% -------------------------------------------------------------------------
% The task vector consists of:
%   - The 9 elements of the rotation matrix (frame alignment)
%   - The 3 elements of the position vector
%
% Task vector ordering:
% [ r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz ]'

% The following expression represents the Eq. (13) of the paper
f = [ T_ee(1,1);
      T_ee(1,2);
      T_ee(1,3);

      T_ee(2,1);
      T_ee(2,2);
      T_ee(2,3);

      T_ee(3,1);
      T_ee(3,2);
      T_ee(3,3);

      T_ee(1,4);
      T_ee(2,4);
      T_ee(3,4) ];

%% ------------------------------------------------------------------------
% Modified Jacobian computation
% -------------------------------------------------------------------------
% The modified Jacobian Jn is obtained by differentiating each element of
% the task vector with respect to each joint variable.
%
% Jn is a 12×n matrix:
%   - Rows correspond to task variables
%   - Columns correspond to joint variables

Jn = cell(12,n); % This represents the Eq. (14) of the paper.

for i = 1:n              % Joint index
    for j = 1:12         % Task index
        
        Jn{j,i} = vpa(diff(f(j), q(i)), 5);

    end
end

%% ------------------------------------------------------------------------
% Replace trigonometric terms:
% cos(ti) -> ci
% sin(ti) -> si
% -------------------------------------------------------------------------

% Define shorthand symbolic variables
syms c1 c2 c3 c4 c5 c6 real
syms s1 s2 s3 s4 s5 s6 real

% Original trig expressions
old = [ cos(t1), cos(t2), cos(t3), cos(t4), cos(t5), cos(t6), ...
        sin(t1), sin(t2), sin(t3), sin(t4), sin(t5), sin(t6) ];

% Replacement symbols
new = [ c1, c2, c3, c4, c5, c6, ...
        s1, s2, s3, s4, s5, s6 ];

% Convert cell Jacobian -> symbolic matrix
Jn_sym = sym(cell2mat(Jn));

% Apply substitutions
Jn_sym = subs(Jn_sym, old, new);

T_ee   = subs(T_ee, old, new);

%% Remove tiny floating-point coefficients

tol = 1e-5;

Jn_sym = FP_Removing(tol, Jn_sym);

T_ee   = FP_Removing(tol, T_ee);

disp('Jacobian is calculated')
%% ------------------------------------------------------------------------
% Saving the Jacobian to a text file
% -------------------------------------------------------------------------
% Each Jacobian element is written as a symbolic expression in text form.
%
% The generated file (NewJ.txt) can be directly copied into a MATLAB
% function (e.g., MNR_J.m), where joint angles are provided as inputs and
% the Jacobian Jn is numerically evaluated.

fid = fopen('MNR_Jn.txt','w');

for i = 1:size(Jn_sym,1)
    for j = 1:size(Jn_sym,2)
        % Access symbolic expression from cell
        % exprSym = Jn{i,j};
        exprSym = Jn_sym(i,j);
        
        % Generate a variable name (e.g., J0102)
        varName = sprintf('J%02d%02d', i, j);
        
        % Convert symbolic expression to string
        exprStr = char(exprSym);
        
        % Remove unnecessary line breaks and extra spaces
        exprStr = regexprep(exprStr, '\s+', ' ');
        
        % Write to file
        fprintf(fid, '%s = %s;\n', varName, exprStr);
    end
end

fclose(fid);

disp('Jacobian Expression is Saved in .txt file')

%% ------------------------------------------------------------------------
% Making Ready to Use MATLAB and CPP functions
% -------------------------------------------------------------------------
disp('Generatng MATLAB Ready Functions')

run('Jn_Fcn_Generator.m')
run('FK_Fcn_Generator.m')

disp('MATLAB Function is Ready :)')

disp('Generatng cpp Ready Functions')

run('Jn_Fcn_Generator_cpp.m')
run('FK_Fcn_Generator_cpp.m')

disp('CPP Function is Ready :)')

%% ------------------------------------------------------------------------
% Denavit–Hartenberg Transformation Function
% -------------------------------------------------------------------------
function A = dh_transform(theta, d, a, alpha)
    % Computes the standard DH homogeneous transformation matrix
    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),             cos(alpha),            d;
          0,           0,                      0,                     1 ];
end

%% ------------------------------------------------------------------------
% Tiny floating-point coefficients removing Function
% -------------------------------------------------------------------------
function Output_Matrix = FP_Removing(tol, Input_Matrix)

    for r = 1:size(Input_Matrix,1)
        for c = 1:size(Input_Matrix,2)
    
            expr = expand(vpa(Input_Matrix(r,c),16));
    
            % Convert tiny floating values to exact zero
            [C,T] = coeffs(expr);
    
            C(abs(double(C)) < tol) = 0;
    
            expr_clean = sum(C .* T);
    
            Output_Matrix(r,c) = simplify(expr_clean);
    
        end
    end

end