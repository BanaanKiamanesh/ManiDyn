clear
close all
clc

%% Kinova Gen3 Lite – Approximate Dynamical Properties
M = [1.377     % Link 1  (shoulder)
     1.1636    % Link 2  (upper-arm)
     1.1636    % Link 3  (fore-arm)
     0.930     % Link 4  (lower-wrist)
     0.678     % Link 5  (upper-wrist)
     0.678 ];  % Link 6  (flange / wrist-2)

% Inertia Tensors 
I = { ...
    [ 0.004570,  0.000001,  0.000002;      % Link 1
      0.000001,  0.004831,  0.000448;
      0.000002,  0.000448,  0.001409 ];

    [ 0.011088,  0.000005,  0.000000;      % Link 2
      0.000005,  0.001072, -0.000691;
      0.000000, -0.000691,  0.011255 ];

    [ 0.010932,  0.000000, -0.000007;      % Link 3
      0.000000,  0.011127,  0.000606;
     -0.000007,  0.000606,  0.001043 ];

    [ 0.008147, -0.000001,  0.000000;      % Link 4
     -0.000001,  0.000631, -0.000500;
      0.000000, -0.000500,  0.008316 ];

    [ 0.001596,  0.000000,  0.000000;      % Link 5
      0.000000,  0.001607,  0.000256;
      0.000000,  0.000256,  0.000399 ];

    [ 0.001641,  0.000000,  0.000000;      % Link 6
      0.000000,  0.000410, -0.000278;
      0.000000, -0.000278,  0.001641 ] ...
};

% DH Table
alpha = [ -pi/2, pi/2,   pi/2, pi/2,  -pi/2,   pi/2];
a     = [     0,    0,      0,    0,      0,      0];
d     = [-(0.1564+0.1284), ...
         -(0.0054+0.0064), ...
         -(0.2104+0.2104), ...
         -(0.0064+0.0064), ...
         -(0.2084+0.1059), ...
         0];
theta = [0, pi, pi, pi, pi, pi];
type  = 'rrrrrr';

% Data-Structure Creation
DH     = DHStruct('alpha', alpha, 'a', a, 'd', d, 'theta', theta, 'type', type, 'notation', 'original');
DynPar = DynStruct('DH', DH, 'Mass', M, 'Inertia', I);

%% Symbolic Model & Code Generation
kin = ManipulatorKinematics(DH);
dyn = ManipulatorDynamics(DynPar);

kin.CalculateFK('Rows', 1:6, 'Generate', 'mfile', 'File', 'gen3_fk');
kin.Jacobian('Type', 'geometric' , 'Generate', 'mfile', 'File', 'gen3_jac_geo');
kin.Jacobian('Type', 'analytical', 'Generate', 'mfile', 'File', 'gen3_jac_ana');

dyn.MassMatrix('Generate', 'mfile', 'File', 'gen3_dyn');
dyn.Coriolis  ('Generate', 'mfile', 'File', 'gen3_dyn');
dyn.Gravity   ('Generate', 'mfile', 'File', 'gen3_dyn');

fprintf('\nCode Generation Complete!\n');

%% Quick Numeric Smoke-Test
q    = zeros(6, 1);
qdot = randn(6, 1);

Pose = gen3_fk(q);
Jg   = gen3_jac_geo(q);
B    = gen3_dyn_B(q);
C    = gen3_dyn_C(q, qdot);
g    = gen3_dyn_g(q);

disp('FK at zero config:'); disp(Pose)
disp('Jacobian:');          disp(Jg)
disp('Mass-matrix:');       disp(B)
disp('Coriolis:');          disp(C)
disp('Gravity vector:');    disp(g)
