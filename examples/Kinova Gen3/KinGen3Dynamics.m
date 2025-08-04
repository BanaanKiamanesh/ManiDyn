clear
close all
clc

%% Kinova Gen3 Lite – Approximate Dynamical Properties
M = [2.009      % Link 1  (shoulder)
     1.106      % Link 2  (upper-arm)
     1.106      % Link 3  (fore-arm)
     0.895      % Link 4  (lower-wrist)
     0.654      % Link 5  (upper-wrist)
     0.654      % Link 6  (flange / wrist-2)
     0.31408 ]; % Link 7  (interface)

% Inertia Tensors 
I = { ...
    [ 0.0045081, -0.00000086, -0.00000190;      % Link 1
     -0.00000086,  0.0047695, -0.00045534;
     -0.00000190, -0.00045534,  0.0013903 ];

    [ 0.0089168, -0.00000290, -0.00000017;      % Link 2
     -0.00000290,  0.00073106,  0.00051768;
     -0.00000017,  0.00051768,  0.0090416 ];

    [ 0.0088707,  0.00000023, -0.00000332;      % Link 3
      0.00000023,  0.0090034, -0.00050415;
     -0.00000332, -0.00050415,  0.00071859 ];

    [ 0.0067339,  0.00000070, -0.00000002;      % Link 4
      0.00000070,  0.00045088,  0.00038999;
     -0.00000002,  0.00038999,  0.0068613 ];

    [ 0.0013311,  0.00000000,  0.00000000;      % Link 5
      0.00000000,  0.0013282, -0.00022936;
      0.00000000, -0.00022936,  0.00030696 ];

    [ 0.0013422,  0.00000000,  0.00000000;      % Link 6
      0.00000000,  0.00031025,  0.00023506;
      0.00000000,  0.00023506,  0.0013360 ];

    [ 0.00022640, -0.00000049, -0.00000026;     % Link 7
     -0.00000049,  0.00015697, -0.00004561;
     -0.00000026, -0.00004561,  0.00025981 ] ...
};

COM = [-2.289e-5,  -0.010511,  -0.075159
       -2.782e-5,  -0.097298,  -0.012693
        2.981e-5,  -0.0062391, -0.115520
       -1.102e-5,  -0.075357,  -0.014085
       -3.3e-7,    -0.009617,  -0.062968
       -3.4e-7,    -0.044043,  -0.0097804
       -2.788e-5,  -0.0052162, -0.022692];

% DH Table
alpha = [  pi,  pi/2,   pi/2,  pi/2,  pi/2,  pi/2,   pi];
a     = [   0,     0,      0,     0,     0,     0,    0];
d     = [ -0.2848, ...
          -0.0118, ...
          -0.4208, ...
          -0.0128, ...
          -0.3143, ...
           0, ...
          -0.1674];
theta = [0, pi, pi, pi, pi, pi, pi];
type  = 'rrrrrrr';

% Data-Structure Creation
DH     = DHStruct('alpha', alpha, 'a', a, 'd', d, 'theta', theta, 'type', type);
DynPar = DynStruct('DH', DH, 'Mass', M, 'Inertia', I, 'COM', COM);

%% Symbolic Model & Code Generation
kin = ManipulatorKinematics(DH);
dyn = ManipulatorDynamics(DynPar);

kin.CalculateFK('Rows', 1:7, 'Generate', 'mfile', 'File', 'gen3_fk');
kin.Jacobian   ('Type', 'geometric' , 'Generate', 'mfile', 'File', 'gen3_jac_geo');
kin.Jacobian   ('Type', 'analytical', 'Generate', 'mfile', 'File', 'gen3_jac_ana');

dyn.MassMatrix ('Generate', 'mfile', 'File', 'gen3_dyn');
dyn.Coriolis   ('Generate', 'mfile', 'File', 'gen3_dyn');
dyn.Gravity    ('Generate', 'mfile', 'File', 'gen3_dyn');
dyn.ODEFunction('Generate', 'mfile', 'File', 'gen3_dyn');

fprintf('\nCode Generation Complete!\n');

%% Quick Numeric Smoke-Test
q    = zeros(7, 1);
qdot = randn(7, 1);

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
