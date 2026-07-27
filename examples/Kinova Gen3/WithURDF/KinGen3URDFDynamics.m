clear
close all
clc

%% Create the Dynamics Model Directly from the URDF
URDFPath = fullfile('urdf\kinova_gen3.urdf');
Dyn = ManipulatorDynamics(URDFPath);

%% Generate MEX Functions

FileBase = 'kinova_gen3_dyn';
Dyn.MassMatrix('Generate', 'mex', 'File', FileBase);
Dyn.Coriolis  ('Generate', 'mex', 'File', FileBase);
Dyn.Gravity   ('Generate', 'mex', 'File', FileBase);
Dyn.Regressor ('Generate', 'mex', 'File', FileBase);
