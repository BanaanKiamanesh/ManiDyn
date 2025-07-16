clc
clear
close all

%% Declare Robot Params

n  = 3;                         % robot DoF
q  = sym('q',  [n, 1], 'real'); % joints var (position)
dq = sym('dq', [n, 1], 'real'); % joints var (velocity)
l  = sym('l',  [n, 1], 'real'); % links length
lc = sym('lc', [n, 1], 'real'); % links length (COM)
m  = sym('m',  [n, 1], 'real'); % links mass
I  = sym('I',  [n, 1], 'real'); % inertia
syms g real                     % gravity constant

%% DH Table (Modified)
% Note: [alpha_{i-1}, a_{i-1}, d_{i}, theta_{i}]

% alpha_{i-1}
alpha_i_1 = [0, pi/2, pi/2, 0]';

% a_{i-1}
a_i_1 = [0, 0, 0, l(3)]';

% d_{i}
d_i = [0, l(1) + l(2) + q(2), 0, 0]';

% theta{i}
theta_i = [q(1) + pi/2, pi, q(3) + pi/2, 0]';

% construct DH table
DH = zeros(n+1, 4, 'sym'); % init DH table
for i = 1:n+1
    DH(i, :) = [alpha_i_1(i), a_i_1(i), d_i(i), theta_i(i)];
end

%% Forward Kinematics

% T_ee_0, and middle transforms calculation :
HomoTrans = get_fk(DH);

% extract the middle transforms :
T_1_0  = HomoTrans{1, 1}; % homogen trans 1  to 0
T_2_0  = HomoTrans{2, 1}; % homogen trans 2  to 0
T_3_0  = HomoTrans{3, 1}; % homogen trans 3  to 0
T_ee_0 = HomoTrans{4, 1}; % homogen trans ee to 0

% compute the rotations :
R_1_0  = T_1_0(1:3, 1:3);  % rotation of {1}
R_2_0  = T_2_0(1:3, 1:3);  % rotation of {2}
R_3_0  = T_3_0(1:3, 1:3);  % rotation of {3}
R_ee_0 = T_ee_0(1:3, 1:3); % rotation of {ee}

% compute the origins :
P_1_0  = T_1_0(1:3, end);  % origin of {1}
P_2_0  = T_2_0(1:3, end);  % origin of {2}
P_3_0  = T_3_0(1:3, end);  % origin of {3}
P_ee_0 = T_ee_0(1:3, end); % origin of {ee}

% compute the Z directions :
Z_1_0  = R_1_0  * [0, 0, 1]';  % Z_1  direction
Z_2_0  = R_2_0  * [0, 0, 1]';  % Z_2  direction
Z_3_0  = R_3_0  * [0, 0, 1]';  % Z_3  direction
Z_ee_0 = R_ee_0 * [0, 0, 1]';  % Z_ee direction

% compute the COM origins :
P_com1_0 = subs(P_2_0, [l(1), l(2), q(2)], [lc(1), 0, 0]); % origin of COM link 1
P_com2_0 = subs(P_3_0, l(2), lc(2));                           % origin of COM link 2
P_com3_0 = subs(P_ee_0, l(3), lc(3));                          % origin of COM link 3

%% Center of Mass (COM) Jacobians

% linear velocites related jacobians :
Jvc1 = [cross(Z_1_0, (P_com1_0 - P_1_0)),   zeros(3, 1),   zeros(3, 1)];
Jvc2 = [cross(Z_1_0, (P_com2_0 - P_1_0)),         Z_2_0,   zeros(3, 1)];
Jvc3 = [cross(Z_1_0, (P_com3_0 - P_1_0)),         Z_2_0,   cross(Z_3_0, (P_com3_0 - P_3_0))];

% angular velocites related jacobians :
Jwc1 = [Z_1_0,   zeros(3, 1),   zeros(3, 1)];
Jwc2 = [Z_1_0,   zeros(3, 1),   zeros(3, 1)];
Jwc3 = [Z_1_0,   zeros(3, 1),   Z_3_0];

%% Dynamics

% compute the mass matrix :
M = m(1)  * Jvc1' * Jvc1 + ...
    m(2)  * Jvc2' * Jvc2 + ...
    m(3)  * Jvc3' * Jvc3 + ...
    Jwc1' * R_1_0 * I(1) * R_1_0' * Jwc1 + ...
    Jwc2' * R_2_0 * I(2) * R_2_0' * Jwc2 + ...
    Jwc3' * R_3_0 * I(3) * R_3_0' * Jwc3; %#ok

M = simplify(M); % simplifying

% compute corilis and centrifugal accelerations :
C = zeros(n, n, 'sym');
for k = 1:n
    for j = 1:n
        for i = 1:n
            C(k, j) = C(k, j) + 1/2 * (diff(M(k, j), q(i)) + ...
                diff(M(k, i), q(j)) - ...
                diff(M(i, j), q(k))) * dq(i);
        end
    end
end
C = simplify(C); % simplifying

% compute gravity term :
P = - (m(1) * [0, -g, 0] * P_com1_0 + ...
       m(2) * [0, -g, 0] * P_com2_0 + ...
       m(3) * [0, -g, 0] * P_com3_0);
G = zeros(n, 1, 'sym');
for k = 1:n
    G(k, 1) = diff(P, q(k));
end
G = simplify(G); % simplifying

% display M, C, and G
disp('M(q) : ')
disp(M)

disp('C(q, q_dot) : ')
disp(C)

disp('G(q) : ')
disp(G)