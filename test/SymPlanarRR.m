clear
close all
clc

%% Parameter Declaration
syms L1 L2 m1 m2 real   

I1 = diag([0, (1/12)*m1*L1^2, (1/12)*m1*L1^2]);
I2 = diag([0, (1/12)*m2*L2^2, (1/12)*m2*L2^2]);

DH = DHStruct( ...
    'alpha',  [0 0], ...
    'a',      [L1 L2], ...
    'd',      [0 0], ...
    'theta',  [0 0], ...     
    'type',   'rr', ...
    'notation','original' );

DynPar = DynStruct( ...
    'Mass',    [m1 m2], ...
    'Inertia', {I1, I2}, ...
    'COM',     [-L1/2, 0, 0; -L2/2, 0, 0], ...
    'DH',      DH );


Dyn = ManipulatorDynamics(DynPar, 'Gravity', [0, -9.81, 0]);
simplify(Dyn.MassMatrix)
simplify(Dyn.Coriolis)
simplify(Dyn.Gravity)

%% N = Bdot - 2*C Must be Skew-Sym
syms q1 q2 qd1 qd2 real
q  = [q1; q2];
qd = [qd1; qd2];

% Symbolic B(q) and C(q, qd) from the Object
Bsym = Dyn.MassMatrix;
Csym = Dyn.Coriolis;

dB_dq = jacobian(Bsym(:), q);
Bdot  = reshape(dB_dq * qd, size(Bsym));

N = simplify(Bdot - 2*Csym);

disp('Bdot - 2C =');
disp(N);

disp('Skew-symmetry check  (S + S.'') =');
disp(simplify(N + N.'));
