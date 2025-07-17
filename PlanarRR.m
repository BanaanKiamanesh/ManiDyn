% Validation for ManipulatorKinematics on a 2-link planar R-R arm
clear
close all
clc

%% Parameter Declaration
% Link lengths
L1 = 1.0;
L2 = 0.7;

% DH
alpha = [0 0];
a     = [L1 L2];
d     = [0 0];
theta = [0 0];
type  = 'rr';

% Build DH struct & kinematics object
DH  = DHStruct('alpha', alpha, 'a', a, 'd', d, 'theta', theta, 'type', type);
kin = ManipulatorKinematics(DH);

%% Symbolic Forward Kinematics
[Eul, Pos] = kin.CalculateFK;

fprintf('--- Symbolic FK ---\n');
disp('Position P(q) ='); disp(Pos);
disp('Euler Angles ='); disp(Eul.');

%% Theoretically expected planar formulas for comparison
syms q1 q2 real
PExpected = [L1*cos(q1) + L2*cos(q1+q2)
             L1*sin(q1) + L2*sin(q1+q2)
             0];
YawExpected = q1 + q2;

fprintf('Matches expected?  %s\n\n', ...
    matlab.lang.OnOffSwitchState(all(simplify(Pos-PExpected) == 0)));

%% Geo Jacobian
Jg = kin.Jacobian;

fprintf('--- Geometric Jacobian Jg(q) ---\n');
disp(Jg);

%% Anal Jacobian
Ja = kin.Jacobian('Type', 'analytical');

fprintf('--- Analytical Jacobian Ja(q) ---\n');
disp(Ja);

%% Evaluate at a Sample Config
q_num = [pi/4;  pi/6];
Pg = double(subs(Pos,  {'q1'; 'q2'}, num2cell(q_num)));
fprintf('P(%4.2f, %4.2f) = [% .3f  % .3f  % .3f]\n', q_num, Pg);

Jg_num = double(subs(Jg, {'q1'; 'q2'}, num2cell(q_num)));
Ja_num = double(subs(Ja, {'q1'; 'q2'}, num2cell(q_num)));

fprintf('\nJg evaluated:\n');
disp(Jg_num);
fprintf('Ja evaluated:\n');
disp(Ja_num);
