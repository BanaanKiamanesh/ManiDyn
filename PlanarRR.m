% Validation for ManipulatorKinematics on a 2-link planar R-R arm
clear
close all
clc

%% Parameter Declaration
L1 = 1.0;
L2 = 0.7;

alpha = [0, 0];
a     = [L1, L2];
d     = [0, 0];
theta = [0, 0];
type  = 'rr';

DH  = DHStruct('alpha', alpha, ...
               'a', a, ...
               'd', d, ...
               'theta', theta, ...
               'type', type);

%% Symbolic Forward Kinematics
RR = ManipulatorKinematics(DH);
[Eul, Pos] = RR.CalculateFK;

fprintf('--- Symbolic FK ---\n');
disp(Pos);
disp(Eul);

%% Quick analytical check
syms q1 q2 real

PExpected = [L1*cos(q1)+L2*cos(q1+q2);  L1*sin(q1)+L2*sin(q1+q2); 0];

fprintf('Matches expected?  %s\n\n', ...
        matlab.lang.OnOffSwitchState(all(simplify(Pos-PExpected)==0)));

%% Jacobians
Jg = RR.Jacobian;
Ja = RR.Jacobian('Type', 'analytical');

fprintf('--- Geometric Jacobian Jg ---\n');  disp(Jg);
fprintf('--- Analytical Jacobian Ja ---\n'); disp(Ja);

%% Evaluate at a sample configuration
q_num = [pi/4; pi/6];
Pg = double(subs(Pos, {'q1';'q2'}, num2cell(q_num)));

fprintf('P(%.2f, %.2f) = [% .3f  % .3f  % .3f]\n', q_num, Pg);

%% Inverse-Kinematics Validation (Rows option)
fprintf('\n================  Inverse Kinematics Validation  ================\n');
RowsSel = [1 2 4];                     % Px, Py, Yaw

fkFun = RR.CalculateFK('Rows', RowsSel, 'Return', 'handle');
jacFun  = RR.Jacobian('Type', 'analytical', ...
                       'Rows', RowsSel, ...
                       'Return', 'handle');

q_des = [pi/3; -pi/4];                 % Target Joint Set
x_des = fkFun(q_des);                  % Desired Pose
q0    = [0; 0];                        % Initial Guess

fprintf('--- Newton IK ---\n');
[qN, errN] = IK_Newton(fkFun, jacFun, q0, x_des, 50, 1e-8, 1e-4);
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f]\n', ...
        numel(errN), errN(end), qN);

fprintf('\n--- Gradient-Descent IK ---\n');
[qG, errG] = IK_Gradient(fkFun, jacFun, q0, x_des, 300, 1e-8, 0.4); % α = 0.05
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f]\n', ...
        numel(errG), errG(end), qG);

fprintf('\nGround-Truth q_des = [% .4f  % .4f]\n', q_des);
fprintf('q Error (Newton)   = [% .3e  % .3e]\n', qN-q_des);
fprintf('q Error (Gradient) = [% .3e  % .3e]\n', qG-q_des);
