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
    matlab.lang.OnOffSwitchState(all(simplify(Pos-PExpected) == 0)));

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
fprintf('\n ================   Inverse Kinematics Validation   ================ \n');
RowsSel = [1 2 4];                     % Px, Py, Yaw

FKFun = RR.CalculateFK('Rows', RowsSel, 'Return', 'handle');
JFun  = RR.Jacobian('Type', 'analytical', ...
    'Rows', RowsSel, ...
    'Return', 'handle');

qDes = [pi/3; -pi/4];
XDes = FKFun(qDes);
q0    = [0; 0];

fprintf('--- Newton IK ---\n');
[qN, ErrN] = IK_Newton(FKFun, JFun, q0, XDes, 50, 1e-8, 1e-4);
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f]\n', ...
    numel(ErrN), ErrN(end), qN);

fprintf('\n--- Gradient-Descent IK ---\n');
[qG, errG] = IK_Gradient(FKFun, JFun, q0, XDes, 300, 1e-8, 0.4); % α = 0.05
fprintf('Iterations: %d, final err = %.3e, q = [% .4f  % .4f]\n', ...
    numel(errG), errG(end), qG);

fprintf('\nGround-Truth q_des = [% .4f  % .4f]\n', qDes);
fprintf('q Error (Newton)     = [% .3e  % .3e]\n', qN-qDes);
fprintf('q Error (Gradient)   = [% .3e  % .3e]\n', qG-qDes);

%% Dynamics Validation (Symbolic and Numeric)
M = [1, 1];
L = [L1, L2];
R = [0.05, 0.05];

I1 = 1/12 * M(1) * diag([0 0 L1^2]);
I2 = 1/12 * M(2) * diag([0 0 L2^2]);
I = {sym(I1), sym(I2)};

COM = [L1/2, 0.0, 0.0
    L2/2, 0.0, 0.0];

DynPar = DynStruct('Mass', M, ...
    'Length', L, 'Radius', R, ...
    'Inertia', I, 'DH', DH, ...
    'COM', COM);

% Test for Normal Gravity Direction
fprintf('\n ================   Dynamics Validation(g = [0, 0, -9.81])   ========================== \n');
MD = ManipulatorDynamics(DynPar);

B = MD.MassMatrix;  disp('Symbolic Mass matrix B(q):');         disp(B);
C = MD.Coriolis;    disp('Symbolic Coriolis matrix C(q, qd):'); disp(C);
g = MD.Gravity;     disp('Symbolic gravity vector g(q):');      disp(g);

% Test for Costume Gravity Direction
fprintf('\n ================   Dynamics Validation(g = [0, -9.81, 0])   ========================== \n');
MD = ManipulatorDynamics(DynPar, 'Gravity', [0 -9.81 0]);

B = MD.MassMatrix;  disp('Symbolic Mass matrix B(q):');         disp(B);
C = MD.Coriolis;    disp('Symbolic Coriolis matrix C(q, qd):'); disp(C);
g = MD.Gravity;     disp('Symbolic gravity vector g(q):');      disp(g);

%% Comprehensive Kinematics Tests
fprintf('\n =====  Kinematics Return Types & Code Generation  ===== \n');

FK_fun = RR.CalculateFK('Return', 'handle');
fprintf('FK handle at q = [%.2f %.2f]:\n', q_num); disp(FK_fun(q_num));

RR.CalculateFK('Generate', 'mfile', 'File', 'PlanarRR_fk');
FK_mfile = PlanarRR_fk(q_num);
fprintf('FK mfile output: '); disp(FK_mfile);

RR.CalculateFK('Generate', 'ccode', 'File', 'PlanarRR_fk');

Jg_sym = RR.Jacobian;
Ja_sym = RR.Jacobian('Type', 'analytical');
Jg_fun = RR.Jacobian('Return', 'handle');
Ja_fun = RR.Jacobian('Type', 'analytical', 'Return', 'handle');

RR.Jacobian('Generate', 'mfile', 'File', 'PlanarRR_jac_geo');
RR.Jacobian('Generate', 'ccode', 'File', 'PlanarRR_jac_geo');

%% Comprehensive Dynamics Tests
fprintf('\n =====  Dynamics Return Types & Code Generation  ===== \n');
MD = ManipulatorDynamics(DynPar, 'Gravity', [0, -9.81, 0]);

% Symbolic terms
B_sym = MD.MassMatrix;  C_sym = MD.Coriolis;  g_sym = MD.Gravity;

% Handles
B_fun = MD.MassMatrix('Return', 'handle');
C_fun = MD.Coriolis  ('Return', 'handle');
g_fun = MD.Gravity   ('Return', 'handle');

fprintf('B handle at q:');       disp(B_fun(q_num));
fprintf('C handle at (q, qd):'); disp(C_fun(q_num, zeros(2, 1)));
fprintf('g handle at q:');       disp(g_fun(q_num));

% Generate MATLAB files
MD.MassMatrix('Generate', 'mfile', 'File', 'PlanarRR_dyn');
MD.Coriolis  ('Generate', 'mfile', 'File', 'PlanarRR_dyn');
MD.Gravity   ('Generate', 'mfile', 'File', 'PlanarRR_dyn');

% Call generated MATLAB functions
B_mfile = PlanarRR_dyn_B(q_num);
C_mfile = PlanarRR_dyn_C(q_num, zeros(2, 1));
g_mfile = PlanarRR_dyn_g(q_num);

% Generate C-code files
MD.MassMatrix('Generate', 'ccode', 'File', 'PlanarRR_dyn');
MD.Coriolis  ('Generate', 'ccode', 'File', 'PlanarRR_dyn');
MD.Gravity   ('Generate', 'ccode', 'File', 'PlanarRR_dyn');

% Generate MEX functions
try
    MD.MassMatrix('Generate', 'mex', 'File', 'PlanarRR_dyn');
    MD.Coriolis  ('Generate', 'mex', 'File', 'PlanarRR_dyn');
    MD.Gravity   ('Generate', 'mex', 'File', 'PlanarRR_dyn');

    % Call generated MEX functions
    B_mex = PlanarRR_dyn_B(q_num);
    C_mex = PlanarRR_dyn_C(q_num, zeros(2, 1));
    g_mex = PlanarRR_dyn_g(q_num);

    fprintf('B mex at q:'); disp(B_mex);
    fprintf('C mex at (q, qd):'); disp(C_mex);
    fprintf('g mex at q:'); disp(g_mex);

catch ME
    warning(ME.identifier, '%s', ME.message);
end

%%  ODE Function & Simulation
fprintf('\n =====  ODE RHS (symbolic, handle, mfile)  ===== \n');
ODE_sym = MD.ODEFunction('Return', 'symbolic');
ODE_fun = MD.ODEFunction;
MD.ODEFunction('Generate', 'mfile', 'File', 'PlanarRR_ode');
ODE_mfile = @PlanarRR_ode;

% Zero-Input Simulation for 10 s
x0 = [q_num; 0; 0];
Tau = @(t) zeros(2, 1);
[tSim, xSim] = ode45(@(t, x) ODE_fun(t, x, Tau(t)), [0 100], x0);

figure('Name', 'PlanarRR Zero-Input Response');
subplot(2, 1, 1);
plot(tSim, xSim(:, 1:2));
grid on;
legend('q1', 'q2');
ylabel('Position [rad]');

subplot(2, 1, 2);
plot(tSim, xSim(:, 3:4));
grid on;
legend('q1dot', 'q2dot');
xlabel('Time [s]');
ylabel('Velocity [rad/s]');

%% Friction Influence Comparison
% Simulate the same system with viscous and Coulomb joint frictions to
% highlight their impact versus the friction-free baseline.

FvSample = [0.05, 0.05];    % N·m·s/rad
FcSample = [0.1 , 0.08];    % N·m

DynParFric = DynStruct('Mass', M, 'Length', L, 'Radius', R, ...
    'Inertia', I, 'DH', DH, 'COM', COM, ...
    'Fv', FvSample, 'Fc', FcSample);

MD_fric  = ManipulatorDynamics(DynParFric, 'Gravity', [0, -9.81, 0]);
ODE_fric = MD_fric.ODEFunction;

[tF, xF] = ode45(@(t, x) ODE_fric(t, x, Tau(t)), [0 100], x0);

% Overlay comparison
figure('Name', 'Friction vs No-Friction Response');
subplot(2, 1, 1);
plot(tSim, xSim(:, 1), 'b-', tF, xF(:, 1), 'r--', ...
    tSim, xSim(:, 2), 'b-.', tF, xF(:, 2), 'r:');
grid on;
legend('q1 (no fric)', 'q1 (fric)', 'q2 (no fric)', 'q2 (fric)');
ylabel('Position [rad]');

subplot(2, 1, 2);
plot(tSim, xSim(:, 3), 'b-', tF, xF(:, 3), 'r--', ...
    tSim, xSim(:, 4), 'b-.', tF, xF(:, 4), 'r:');
grid on;
legend('q1dot (no fric)', 'q1dot (fric)', 'q2dot (no fric)', 'q2dot (fric)');
xlabel('Time [s]'); ylabel('Velocity [rad/s]');

%% Performance Benchmark
fprintf('\n =====  Performance Benchmark (1000 evaluations)  ===== \n');
NIter = 1000;
qSamples  = (rand(2, NIter) - 0.5) * 2*pi;
qdSamples = randn(2, NIter) * 0.5;

% Mass Matrix
fprintf('\nMass matrix B(q):\n');

% Handle benchmark
tic;
for k = 1:NIter
    B_fun(qSamples(:, k));
end
th = toc;

% mfile benchmark
tic;
for k = 1:NIter
    PlanarRR_dyn_B(qSamples(:, k));
end
tm = toc;

% mex benchmark (if exists)
if exist(['PlanarRR_dyn_B.' mexext], 'file')
    tic;
    for k = 1:NIter
        PlanarRR_dyn_B(qSamples(:, k));
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
    C_fun(qSamples(:, k), qdSamples(:, k));
end
tch = toc;

% mfile
tic;
for k = 1:NIter
    PlanarRR_dyn_C(qSamples(:, k), qdSamples(:, k));
end
tcm = toc;

% mex
if exist(['PlanarRR_dyn_C.' mexext], 'file')
    tic;
    for k = 1:NIter
        PlanarRR_dyn_C(qSamples(:, k), qdSamples(:, k));
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
    g_fun(qSamples(:, k));
end
tgh = toc;

% mfile
tic;
for k = 1:NIter
    PlanarRR_dyn_g(qSamples(:, k));
end
tgm = toc;

% mex
if exist(['PlanarRR_dyn_g.' mexext], 'file')
    tic;
    for k = 1:NIter
        PlanarRR_dyn_g(qSamples(:, k));
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
    ODE_fun(0, [qSamples(:, k); qdSamples(:, k)], Tau(0));
end
toh = toc;

% mfile timing
for k = 1:NIter
    ODE_mfile(0, [qSamples(:, k); qdSamples(:, k)], Tau(0));
end
tom = toc;

fprintf('ODE handle: %.4f s, mfile: %.4f s\n', toh, tom);
