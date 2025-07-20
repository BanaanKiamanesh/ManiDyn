% Validation for ManipulatorKinematics on a 3-DOF planar R-P-R arm
clear
close all
clc

%% Parameter Declaration
L1 = 1.0;        % Link-1 length [m]
L2 = 0.7;        % Link-2 offset when q2 = 0 [m]
L3 = 0.5;        % Link-3 length [m]

Alpha = [0,  pi/2,  pi/2, 0];           % DH α
A     = [0,    0 ,    0 , L3];          % DH a
D     = [0, L1+L2,    0 ,  0];          % DH d
Theta = [pi/2,   pi, pi/2, 0];          % DH θ (constant offsets)
Type  = 'rprf';                         % Joint types (R-P-R-Fixed)

DH = DHStruct('alpha', Alpha, 'a', A, 'd', D, ...
    'theta', Theta, 'type', Type, ...
    'notation', 'modified');

%% Symbolic Forward Kinematics
RPR = ManipulatorKinematics(DH);
[Eul, Pos] = RPR.CalculateFK;

fprintf('--- Symbolic FK ---\n');
disp(Pos);
disp(Eul);

%% Quick Analytical Check
syms q1 q2 q3 real

PExpected = [L3*cos(q1 + q3) + L1*cos(q1) + L2*cos(q1) + q2*cos(q1);
    L3*sin(q1 + q3) + L1*sin(q1) + L2*sin(q1) + q2*sin(q1);
    0];

fprintf('Matches expected?  %s\n\n', ...
    matlab.lang.OnOffSwitchState(all(simplify(Pos - PExpected) == 0)));

%% Jacobians
Jg = RPR.Jacobian;
Ja = RPR.Jacobian('Type', 'analytical');

fprintf('--- Geometric Jacobian Jg ---\n');  disp(Jg);
fprintf('--- Analytical Jacobian Ja ---\n'); disp(Ja);

%% Evaluate at a Sample Configuration
QNum = [pi/4; 0.1; pi/6];
Pg = vpa(subs(Pos, {'q1'; 'q2'; 'q3'}, num2cell(QNum)), 4);

fprintf('P(%.2f, %.2f, %.2f) = %s\n', QNum, char(Pg));

%% Inverse-Kinematics Validation (Rows option)
fprintf('\n ================   Inverse Kinematics Validation   ================ \n');
RowsSel = [1 2 4];                     % Px, Py, Yaw

FKFun = RPR.CalculateFK('Rows', RowsSel, 'Return', 'handle');
JFun  = RPR.Jacobian('Type', 'analytical', ...
    'Rows', RowsSel, ...
    'Return', 'handle');

QDes = [pi/3; 0.15; -pi/4];           % Target Joint Set
XDes = FKFun(QDes);                   % Desired Pose
Q0   = [0; 0; 0];                     % Initial Guess

fprintf('--- Newton IK ---\n');
[QN, ErrN] = IK_Newton(FKFun, JFun, Q0, XDes, 50, 1e-8, 1e-4);
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f  % .4f]\n', ...
    numel(ErrN), ErrN(end), QN);

fprintf('\n--- Gradient-Descent IK ---\n');
[QG, ErrG] = IK_Gradient(FKFun, JFun, Q0, XDes, 700, 1e-8, 0.08);
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f  % .4f]\n', ...
    numel(ErrG), ErrG(end), QG);

fprintf('\nGround-Truth q_des = [% .4f  % .4f  % .4f]\n', QDes);
fprintf('q Error (Newton)   = [% .3e  % .3e  % .3e]\n', QN - QDes);
fprintf('q Error (Gradient) = [% .3e  % .3e  % .3e]\n', QG - QDes);

%% Dynamics Validation (Symbolic and Numeric)
M = [1.0, 0.8, 0.5];              % Link masses [kg]
L = [L1,  L2,  L3];               % Characteristic lengths [m]
R = [0.05, 0.05, 0.05];           % Link radii [m]

I1 = 1/12 * M(1) * diag([0 0 L1^2]);
I2 = 1/12 * M(2) * diag([0 0 L2^2]);
I3 = 1/12 * M(3) * diag([0 0 L3^2]);
I  = {sym(I1), sym(I2), sym(I3)};

COM = [L1/2, 0.0, 0.0;            % Centers of mass
    L2/2, 0.0, 0.0;
    L3/2, 0.0, 0.0];

% Use only the first three links for dynamics (exclude fixed end-effector frame)
DHDyn = DH;
fieldsDH = {'alpha','a','d','theta','type'};
for idx = 1:numel(fieldsDH)
    DHDyn.(fieldsDH{idx}) = DHDyn.(fieldsDH{idx})(1:3);
end

DynPar = DynStruct('Mass', M, 'Length', L, 'Radius', R, ...
    'Inertia', I, 'DH', DHDyn, 'COM', COM);

% Test for Normal Gravity Direction
fprintf('\n ================   Dynamics Validation(g = [0, 0, -9.81])   ========================== \n');
MD = ManipulatorDynamics(DynPar);

B = MD.MassMatrix;  disp('Symbolic Mass matrix B(q):');         disp(B);
C = MD.Coriolis;    disp('Symbolic Coriolis matrix C(q, qd):'); disp(C);
g = MD.Gravity;     disp('Symbolic gravity vector g(q):');      disp(g);

% Test for Custom Gravity Direction
fprintf('\n ================   Dynamics Validation(g = [0, -9.81, 0])   ========================== \n');
MD = ManipulatorDynamics(DynPar, 'Gravity', [0 -9.81 0]);

B = MD.MassMatrix;  disp('Symbolic Mass matrix B(q):');         disp(B);
C = MD.Coriolis;    disp('Symbolic Coriolis matrix C(q, qd):'); disp(C);
g = MD.Gravity;     disp('Symbolic gravity vector g(q):');      disp(g);

%% Comprehensive Kinematics Tests
fprintf('\n =====  Kinematics Return Types & Code Generation  ===== \n');

FK_fun = RPR.CalculateFK('Return', 'handle');
fprintf('FK handle at q = [%.2f %.2f %.2f]:\n', QNum); disp(FK_fun(QNum));

RPR.CalculateFK('Generate', 'mfile', 'File', 'PlanarRPR_fk');
FK_mfile = PlanarRPR_fk(QNum);
fprintf('FK mfile output: '); disp(FK_mfile);

RPR.CalculateFK('Generate', 'ccode', 'File', 'PlanarRPR_fk');

JgSym = RPR.Jacobian;
JaSym = RPR.Jacobian('Type', 'analytical');
JgFun = RPR.Jacobian('Return', 'handle');
JaFun = RPR.Jacobian('Type', 'analytical', 'Return', 'handle');

RPR.Jacobian('Generate', 'mfile', 'File', 'PlanarRPR_jac_geo');
RPR.Jacobian('Generate', 'ccode', 'File', 'PlanarRPR_jac_geo');

%% Comprehensive Dynamics Tests
fprintf('\n =====  Dynamics Return Types & Code Generation  ===== \n');
MD = ManipulatorDynamics(DynPar, 'Gravity', [0, -9.81, 0]);

% Symbolic terms
BSym = MD.MassMatrix;  CSym = MD.Coriolis;  gSym = MD.Gravity;

% Handles
BFun = MD.MassMatrix('Return', 'handle');
CFun = MD.Coriolis  ('Return', 'handle');
gFun = MD.Gravity   ('Return', 'handle');

fprintf('B handle at q:');       disp(BFun(QNum));
fprintf('C handle at (q, qd):'); disp(CFun(QNum, zeros(3, 1)));
fprintf('g handle at q:');       disp(gFun(QNum));

% Generate MATLAB files
MD.MassMatrix('Generate', 'mfile', 'File', 'PlanarRPR_dyn');
MD.Coriolis  ('Generate', 'mfile', 'File', 'PlanarRPR_dyn');
MD.Gravity   ('Generate', 'mfile', 'File', 'PlanarRPR_dyn');

% Call generated MATLAB functions
BMfile = PlanarRPR_dyn_B(QNum);
CMfile = PlanarRPR_dyn_C(QNum, zeros(3, 1));
Gmfile = PlanarRPR_dyn_g(QNum);

% Generate C-code files
MD.MassMatrix('Generate', 'ccode', 'File', 'PlanarRPR_dyn');
MD.Coriolis  ('Generate', 'ccode', 'File', 'PlanarRPR_dyn');
MD.Gravity   ('Generate', 'ccode', 'File', 'PlanarRPR_dyn');

% Generate MEX functions
try
    MD.MassMatrix('Generate', 'mex', 'File', 'PlanarRPR_dyn');
    MD.Coriolis  ('Generate', 'mex', 'File', 'PlanarRPR_dyn');
    MD.Gravity   ('Generate', 'mex', 'File', 'PlanarRPR_dyn');

    % Call generated MEX functions
    BMex = PlanarRPR_dyn_B(QNum);
    CMex = PlanarRPR_dyn_C(QNum, zeros(3, 1));
    GMex = PlanarRPR_dyn_g(QNum);

    fprintf('B mex at q:'); disp(BMex);
    fprintf('C mex at (q, qd):'); disp(CMex);
    fprintf('g mex at q:'); disp(GMex);
catch ME
    warning(ME.identifier, '%s', ME.message);
end

%%  ODE Function & Simulation
fprintf('\n =====  ODE RHS (symbolic, handle, mfile)  ===== \n');
ODESym = MD.ODEFunction('Return', 'symbolic');
ODEFun = MD.ODEFunction;
MD.ODEFunction('Generate', 'mfile', 'File', 'PlanarRPR_ode');
ODEMfile = @PlanarRPR_ode_x_dot;

% Zero-Input Simulation for 10 s
x0 = [QNum; 0; 0; 0];   % [q; qd]
Tau = @(t) zeros(3, 1);
[tSim, xSim] = ode45(@(t, x) ODEFun(t, x, Tau(t)), [0 100], x0);

figure('Name', 'PlanarRPR Zero-Input Response');
subplot(2, 1, 1);
plot(tSim, xSim(:, 1:3));
grid on;
legend('q1', 'q2', 'q3');
ylabel('Position');

subplot(2, 1, 2);
plot(tSim, xSim(:, 4:6));
grid on;
legend('q1dot', 'q2dot', 'q3dot');
xlabel('Time [s]');
ylabel('Velocity');

%% Performance Benchmark
fprintf('\n =====  Performance Benchmark (1000 evaluations)  ===== \n');
NIter = 1000;
QSamples  = (rand(3, NIter) - 0.5) * 2*pi;
QSamples(2, :) = (rand(1, NIter) - 0.5) * 1.0;   % Prismatic joint realistic range
QdSamples = randn(3, NIter) * 0.5;

% Mass Matrix
fprintf('\nMass matrix B(q):\n');

% Handle benchmark
tic;
for k = 1:NIter
    BFun(QSamples(:, k));
end
th = toc;

% mfile benchmark
tic;
for k = 1:NIter
    PlanarRPR_dyn_B(QSamples(:, k));
end
tm = toc;

% mex benchmark (if exists)
if exist(['PlanarRPR_dyn_B.' mexext], 'file')
    tic;
    for k = 1:NIter
        PlanarRPR_dyn_B(QSamples(:, k));
    end
    tx = toc;
else
    tx = NaN;
end
fprintf('Handle: %.4f s, mfile: %.4f s, mex: %.4f s\n', th, tm, tx);

% Coriolis matrix
fprintf('\nCoriolis C(q, qd):\n');

% Handle
tic;
for k = 1:NIter
    CFun(QSamples(:, k), QdSamples(:, k));
end
tch = toc;

% mfile
tic;
for k = 1:NIter
    PlanarRPR_dyn_C(QSamples(:, k), QdSamples(:, k));
end
tcm = toc;

% mex
if exist(['PlanarRPR_dyn_C.' mexext], 'file')
    tic;
    for k = 1:NIter
        PlanarRPR_dyn_C(QSamples(:, k), QdSamples(:, k));
    end
    tcx = toc;
else
    tcx = NaN;
end
fprintf('Handle: %.4f s, mfile: %.4f s, mex: %.4f s\n', tch, tcm, tcx);

% Gravity vector
fprintf('\nGravity g(q):\n');

% Handle
tic;
for k = 1:NIter
    gFun(QSamples(:, k));
end
tgh = toc;

% mfile
tic;
for k = 1:NIter
    PlanarRPR_dyn_g(QSamples(:, k));
end
tgm = toc;

% mex
if exist(['PlanarRPR_dyn_g.' mexext], 'file')
    tic;
    for k = 1:NIter
        PlanarRPR_dyn_g(QSamples(:, k));
    end
    tgx = toc;
else
    tgx = NaN;
end

fprintf('Handle: %.4f s, mfile: %.4f s, mex: %.4f s\n', tgh, tgm, tgx);

%% ODE RHS Performance
fprintf('\nODE RHS x_dot (1000 evaluations)\n');

% Handle timing
tic;
for k = 1:NIter
    ODEFun(0, [QSamples(:, k); QdSamples(:, k)], Tau(0));
end
toh = toc;

% mfile timing
for k = 1:NIter
    ODEMfile(0, [QSamples(:, k); QdSamples(:, k)], Tau(0));
end
tom = toc;

fprintf('ODE handle: %.4f s, mfile: %.4f s\n', toh, tom);
