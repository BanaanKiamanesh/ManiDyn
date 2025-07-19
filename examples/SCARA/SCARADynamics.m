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
dyn.MassMatrix('Generate', 'mfile', 'File', 'scara_dyn');
dyn.Coriolis  ('Generate', 'mfile', 'File', 'scara_dyn');
dyn.Gravity   ('Generate', 'mfile', 'File', 'scara_dyn');

fprintf('SCARA code generation complete.\n');

%% Quick Numeric Smoke-Test
q    = [0; 0; 0.05; 0];           % Small Extension for Prismatic Link
qdot = randn(4, 1);

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
