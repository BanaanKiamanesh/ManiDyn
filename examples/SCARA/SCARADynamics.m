clear
close all
clc

%% SCARA Dynamical Properties
M = [   2,    2,    1,  0.5];
L = [0.30, 0.25, 0.20, 0.10];
R = [0.05, 0.04, 0.03, 0.03];

I = cell(4, 1);
for i = 1:4
    I{i} = diag([ ...
        M(i)/12 * L(i)^2, ...
        M(i)/12 * L(i)^2, ...
        0.5 * M(i) * R(i)^2 ...
        ]);
end

COM = [L(1)/2,   0,       0
    L(2)/2,   0,       0
    0,   0, -L(3)/2
    0,   0,       0];

% DH Table
alpha = [   0,   pi, 0, 0];
a     = [L(1), L(2), 0, 0];
d     = [   0,    0, 0, 0];
theta = [   0,    0, 0, 0];
type  = 'rrpr';

% Data Structure Creation
DH     = DHStruct('alpha', alpha, 'a', a, 'd', d, 'theta', theta, 'type', type);
DynPar = DynStruct('Mass', M, 'Length', L, 'Radius', R, 'Inertia', I, ...
    'COM', COM, 'DH', DH);

%% Symbolic Model & Code Generation
kin = ManipulatorKinematics(DH);
dyn = ManipulatorDynamics(DynPar);

% Forward Kinematics
kin.CalculateFK('Rows', 1:6, 'Generate', 'mfile', 'File', 'scara_fk');

% Geometric & Analytical Jacobians
kin.Jacobian('Type', 'geometric' , 'Generate', 'mfile', 'File', 'scara_jac_geo');
kin.Jacobian('Type', 'analytical', 'Generate', 'mfile', 'File', 'scara_jac_ana');

% Dynamics: B, C, g
dyn.MassMatrix ('Generate', 'mex', 'File', 'scara_dyn');
dyn.Coriolis   ('Generate', 'mex', 'File', 'scara_dyn');
dyn.Gravity    ('Generate', 'mex', 'File', 'scara_dyn');
dyn.ODEFunction('Generate', 'mex', 'File', 'scara_dyn');


fprintf('SCARA code generation complete.\n');

%% Quick Numeric Smoke-Test
q    = zeros(4, 1);
qdot = zeros(4, 1);

Pose = scara_fk(q);
Jg   = scara_jac_geo(q);
B    = scara_dyn_B(q);
C    = scara_dyn_C(q, qdot);
g    = scara_dyn_g(q);

disp('FK pose:');            disp(Pose)
disp('Geometric Jacobian:'); disp(Jg)
disp('Mass matrix:');        disp(B)
disp('Coriolis matrix:');    disp(C)
disp('Gravity vector:');     disp(g)

%% Test Simulation
% Build Full-State ODE Function
odeFun = dyn.ODEFunction();   % @(t, x, tau) -> x_dot

% Torque Input Function
tau = @(t) zeros(4, 1);

% Initial State
x0 = zeros(8, 1);     % [q; qd]

% Simulate for 5 seconds
[tSim, xSim] = ode45(@(t,x) odeFun(t, x, tau(t)), [0, 20], x0);

qSim  = xSim(:, 1:4);
qdSim = xSim(:, 5:8);

% Plot results
figure('Name', 'SCARA Zero-Input Response', 'NumberTitle', 'off')
a1 = subplot(2, 1, 1);
plot(tSim, qSim, 'LineWidth', 1.2)
xlabel('Time [s]')
ylabel('Joint Position')
legend({'q_1', 'q_2', 'q_3', 'q_4'}, 'Location', 'ne')
title('Joint Positions')
grid on

a2 = subplot(2, 1, 2);
plot(tSim, qdSim, 'LineWidth', 1.2)
xlabel('Time [s]')
ylabel('Joint Velocity')
legend({'q̇_1', 'q̇_2', 'q̇_3', 'q̇_4'}, 'Location', 'ne')
title('Joint Velocities')
grid on
linkaxes([a1, a2], 'x')
