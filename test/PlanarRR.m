% Validation for ManipulatorKinematics on a 2-link planar R-R arm
clear
close all
clc

addpath '..\utils'
addpath '..\src'

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
    'type', type, ...
    'notation', 'original');

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
Pg = vpa(subs(Pos, {'q1';'q2'}, num2cell(q_num)), 4);

fprintf('P(%.2f, %.2f, %.2f) = %s\n', q_num, char(Pg));

%% Inverse-Kinematics Validation (Rows option)
fprintf('\n================  Inverse Kinematics Validation  ================\n');
RowsSel = [1 2 4];                     % Px, Py, Yaw

FKFun = RR.CalculateFK('Rows', RowsSel, 'Return', 'handle');
JFun  = RR.Jacobian('Type', 'analytical', ...
    'Rows', RowsSel, ...
    'Return', 'handle');

qDes = [pi/3; -pi/4];                 % Target Joint Set
XDes = FKFun(qDes);                  % Desired Pose
q0    = [0; 0];                        % Initial Guess

fprintf('--- Newton IK ---\n');
[qN, ErrN] = IK_Newton(FKFun, JFun, q0, XDes, 50, 1e-8, 1e-4);
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f]\n', ...
    numel(ErrN), ErrN(end), qN);

fprintf('\n--- Gradient-Descent IK ---\n');
[qG, errG] = IK_Gradient(FKFun, JFun, q0, XDes, 300, 1e-8, 0.4); % α = 0.05
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f]\n', ...
    numel(errG), errG(end), qG);

fprintf('\nGround-Truth q_des = [% .4f  % .4f]\n', qDes);
fprintf('q Error (Newton)   = [% .3e  % .3e]\n', qN-qDes);
fprintf('q Error (Gradient) = [% .3e  % .3e]\n', qG-qDes);

%% Dynamics validation (symbolic and numeric)
M = [1, 1];
L = [L1, L2];
R = [0.05, 0.05];

I1 = 1/12 * M(1) * diag([0 0 L1^2]);
I2 = 1/12 * M(2) * diag([0 0 L2^2]);
I = {sym(I1), sym(I2)};

DynPar = DynStruct('Mass', M, ...
    'Length', L, 'Radius', R, ...
    'Inertia', I, 'DH', DH);

% Test for normal Gravity Direction
fprintf('\n================  Dynamics Validation(g = [0, 0, -9.81])  ==========================\n');
MD = ManipulatorDynamics(DynPar);

B = MD.MassMatrix;  disp('Symbolic Mass matrix B(q):');         disp(B);
C = MD.Coriolis;    disp('Symbolic Coriolis matrix C(q, qd):'); disp(C);
g = MD.Gravity;     disp('Symbolic gravity vector g(q):');      disp(g);

% Test for Costume Gravity Direction
fprintf('\n================  Dynamics Validation(g = [0, -9.81, 0])  ==========================\n');
MD = ManipulatorDynamics(DynPar, 'Gravity', [0 -9.81 0]);

B = MD.MassMatrix;  disp('Symbolic Mass matrix B(q):');         disp(B);
C = MD.Coriolis;    disp('Symbolic Coriolis matrix C(q, qd):'); disp(C);
g = MD.Gravity;     disp('Symbolic gravity vector g(q):');      disp(g);
