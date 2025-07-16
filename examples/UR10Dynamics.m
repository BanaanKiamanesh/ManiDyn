clear
close all
clc

%% UR10 Dynamical Properties

% Masses
M = [7.1, 12.7, 4.27, 2, 2, 0.365];

% Lengths and Radiuses
L = [0.1273, 0.612, 0.5723, 0.163941, 0.1157, 0.0922];
R = [0.2, 0.15, 0.15, 0.1, 0.1, 0.1];
         
% Inertia Tensors
I = cell(6,1);

% link 1 (y parallel to the cylinder axis)
I{1} = diag([M(1)/12 * (L(1)^2 + 3*R(1)^2),...
             0.5*M(1) * R(1)^2,...
             M(1)/12 * (L(1)^2 + 3*R(1)^2)]);

% link 2 (x parallel to the cylinder axis)
I{2} = diag([0.5*M(2) * R(2)^2,...
             M(2)/12 * (L(2)^2 + 3*R(2)^2),...
             M(2)/12 * (L(2)^2 + 3*R(2)^2)]);

% link 3 (x parallel to the cylinder axis)
I{3} = diag([0.5*M(3) * R(3)^2,...
             M(3)/12 * (L(3)^2 + 3*R(3)^2),...
             M(3)/12 * (L(3)^2 + 3*R(3)^2)]);

% link 4 (y parallel to the cylinder axis)
I{4} = diag([M(4)/12 * (L(4)^2 + 3*R(4)^2),...
             0.5*M(4) * R(4)^2,...
             M(4)/12 * (L(4)^2 + 3*R(4)^2)]);

% link 5 (y parallel to the cylinder axis)
I{5} = diag([M(5)/12 * (L(5)^2 + 3*R(5)^2),...
             0.5*M(5) * R(5)^2,...
             M(5)/12 * (L(5)^2 + 3*R(5)^2)]);

% link 6 (z parallel to the cylinder axis)
I{6} = diag([M(6)/12 * (L(6)^2 + 3*R(6)^2),...
             M(6)/12 * (L(6)^2 + 3*R(6)^2),...
             0.5*M(6) * R(6)^2]);