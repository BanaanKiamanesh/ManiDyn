clear
close all
clc

%% UR10 Dynamical Properties
M = [   7.1,  12.7,   4.27,        2,      2,  0.365];
L = [0.1273, 0.612, 0.5723, 0.163941, 0.1157, 0.0922];
R = [   0.2,  0.15,   0.15,      0.1,    0.1,    0.1];

I = cell(6, 1);
I{1} = diag([M(1)/12*(L(1)^2+3*R(1)^2),           0.5*M(1)*R(1)^2, M(1)/12*(L(1)^2+3*R(1)^2) ]);
I{2} = diag([          0.5*M(2)*R(2)^2, M(2)/12*(L(2)^2+3*R(2)^2), M(2)/12*(L(2)^2+3*R(2)^2) ]);
I{3} = diag([          0.5*M(3)*R(3)^2, M(3)/12*(L(3)^2+3*R(3)^2), M(3)/12*(L(3)^2+3*R(3)^2) ]);
I{4} = diag([M(4)/12*(L(4)^2+3*R(4)^2),           0.5*M(4)*R(4)^2, M(4)/12*(L(4)^2+3*R(4)^2) ]);
I{5} = diag([M(5)/12*(L(5)^2+3*R(5)^2),           0.5*M(5)*R(5)^2, M(5)/12*(L(5)^2+3*R(5)^2) ]);
I{6} = diag([M(6)/12*(L(6)^2+3*R(6)^2), M(6)/12*(L(6)^2+3*R(6)^2),           0.5*M(6)*R(6)^2 ]);

COM = [ 0.021   0       0.027
        0.380   0       0.158
        0.240   0       0.068
        0       0.007   0.018
       -0       0.007   0.018
        0       0      -0.026 ];

% DH Table
alpha = [   pi/2,       0,        0,     pi/2,  -pi/2,      0];
a     = [      0,  -0.612,  -0.5723,        0,      0,      0];
d     = [ 0.1273,       0,        0, 0.163941, 0.1157, 0.0922];
theta = zeros(6, 1); 
type  = 'rrrrrr';

% Data Structure Creation
DH     = DHStruct('theta', theta, 'a', a, 'd', d, 'alpha', alpha, 'type', type);
DynPar = DynStruct('Mass', M, 'Length', L, 'Radius', R, 'Inertia', I,...
                   'COM', COM, 'DH', DH);

%% Symbolic Model & Code Generation
kin = ManipulatorKinematics(DH);
dyn = ManipulatorDynamics(DynPar);

% Forward Kinematics
kin.CalculateFK('Rows', 1:6, 'Generate', 'mex', 'File', 'ur10_fk');

% Geometric & Analytical Jacobians
kin.Jacobian('Type', 'geometric' , 'Generate', 'mex', 'File', 'ur10_jac_geo');
kin.Jacobian('Type', 'analytical', 'Generate', 'mex', 'File', 'ur10_jac_ana');

% Dynamics: B, C, g 
dyn.MassMatrix('Generate', 'mex', 'File', 'ur10_dyn');
dyn.Coriolis  ('Generate', 'mex', 'File', 'ur10_dyn');
dyn.Gravity   ('Generate', 'mex', 'File', 'ur10_dyn');

fprintf('\nCode Generation Complete!\n');

%% Quick Numeric Smoke-Test
q    = zeros(6, 1);
qdot = randn(6, 1);

Pose = ur10_fk(q);
Jg   = ur10_jac_geo(q);
B    = ur10_dyn_B(q);
C    = ur10_dyn_C(q, qdot);
g    = ur10_dyn_g(q);

disp('FK at Zero Config:');   disp(Pose)
disp('Jacobian:');            disp(Jg)
disp('Mass-Matrix:');         disp(B)
disp('Coriolis:');            disp(C)
disp('Gravity Vector:');      disp(g)