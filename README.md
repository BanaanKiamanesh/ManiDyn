# ManiDyn

A **MATLAB®** toolbox for symbolic modelling of serial-link robot **Manipulators**.  
ManiDyn lets you move seamlessly from high-level geometric descriptions (DH parameters) to
fully-symbolic expressions of

* Forward kinematics  
* Geometric / analytical Jacobians  
* Dynamics – mass matrix **B(q)**, Coriolis/centrifugal matrix **C(q, q̇)** and gravity vector **g(q)**

All results can be returned as symbolic expressions, lightweight function handles, or automatically
exported as `.m` files, C source, or compiled **MEX** binaries for real-time use.

---
## Table of Contents
- [Table of Contents](#table-of-contents)
- [Installation](#installation)
- [Defining a Manipulator](#defining-a-manipulator)
- [Kinematics Workflow](#kinematics-workflow)
- [Dynamics Workflow](#dynamics-workflow)
- [Supported Output Formats](#supported-output-formats)
- [Inverse Kinematics Utilities](#inverse-kinematics-utilities)
- [Inverse Kinematics Algorithms](#inverse-kinematics-algorithms)
- [Examples](#examples)
- [Testing](#testing)
- [Requirements](#requirements)
- [License](#license)

---
## Installation
```matlab
% clone repository then from MATLAB:
>> cd <path_to>/ManiDyn
>> install   % adds folders to MATLAB path
```
Alternatively add the `src` and `utils` folders to the `MATLABPATH` manually.

---
## Defining a Manipulator
1. **Geometry** – Create a DH parameter table with `DHStruct`:
```matlab
DH = DHStruct('alpha',  [0 0], ...
              'a',      [1 1], ...
              'd',      [0 0], ...
              'theta',  [0 0], ...
              'type',   'rr');     % two revolute joints
```
> **Modified DH?**  Add `'notation','modified'` to the call above and all
  kinematics/dynamics functions will automatically follow Craig's *modified DH*
  convention.  The toolbox handles both notations transparently.

2. **Dynamics** – Create a matching dynamic parameter structure with `DynStruct`:
```matlab

DynAll = DynStruct('Mass',    [1 1], ...          % REQUIRED → link masses
                    'Length',  [1 1], ...          % OPTIONAL → link lengths (for visuals)
                    'Radius',  [0.05 0.05], ...    % OPTIONAL → link radii  (for visuals)
                    'Inertia', {zeros(3), zeros(3)}, ... % REQUIRED → 3×3 inertia tensors
                    'COM',     [0.5 0 0; 0.5 0 0], ...   % REQUIRED → centre of mass rows
                    'DH',      DH, ...              % REQUIRED → DH param struct
                    'Fv',      [0.05 0.07], ...     % OPTIONAL → viscous friction (Nm·s/rad)
                    'Fc',      [0.20 0.15]);        % OPTIONAL → Coulomb  friction (Nm)

% Omit any optional field you don’t need – defaults are sensible.
```

---
## Kinematics Workflow
```matlab
kin = ManipulatorKinematics(DH);

% Forward kinematics – pose(x, y, z, φ, θ, ψ)
pose_sym = kin.CalculateFK();

% Function handle – fast numerical evaluation
fk = kin.CalculateFK('Return', 'handle');
pose_num = fk([pi/3; pi/6]);

% Geometric Jacobian
J_sym = kin.Jacobian();
```
Optional name-value pairs allow **code generation**:
```matlab
kin.CalculateFK('Generate','mfile','File','myFK');     % creates myFK.m
kin.Jacobian('Generate','mex','File','myJac');         % creates compiled MEX
```

---
## Dynamics Workflow
```matlab
dyn = ManipulatorDynamics(Dyn);

% Custom gravity direction?  Just pass the vector to the constructor:
dyn_g = ManipulatorDynamics(Dyn, 'Gravity', [0 -9.81 0]); % y-axis gravity

% Friction-aware dynamics
dyn_fric = ManipulatorDynamics(DynFric);

B  = dyn.MassMatrix();                 % symbolic
C  = dyn.Coriolis();
g  = dyn.Gravity();

% Numerical handles
Bf = dyn.MassMatrix('Return','handle');
Cf = dyn.Coriolis  ('Return','handle');
gf = dyn.Gravity   ('Return','handle');

B_num = Bf([pi/3;pi/6]);

%% ODE right-hand side (full state dynamics)
ode = dyn.ODEFunction();           % @(t,x,tau) → [q̇; q̈]

% Zero-input simulation for 5 s
tau = @(t)[0;0];                   % user-defined torque function
x0  = [pi/3; pi/6; 0; 0];          % [q; q̇] initial state
[tSim, xSim] = ode45(@(t,x) ode(t, x, tau(t)), [0 5], x0);

plot(tSim, xSim(:,1:2));
title('Joint positions vs time');
legend('q_1','q_2'); grid on;
```
Export ready-to-run code in one line:
```matlab
dyn.MassMatrix('Generate','ccode','File','mass2R');
```

---
## Supported Output Formats
The core symbolic methods can produce results in several forms via the `Return` and `Generate` name-value pairs:

| Method | Symbolic (default) | Function handle (`'Return','handle'`) | MATLAB `.m` (`'Generate','mfile'`) | C source (`'Generate','ccode'`) | Compiled MEX (`'Generate','mex'`) |
|--------|:------------------:|:-------------------------------------:|:----------------------------------:|:------------------------------:|:--------------------------------:|
| `CalculateFK`  | ✓ | ✓ | ✓ | ✓ | ✓ |
| `Jacobian`     | ✓ | ✓ | ✓ | ✓ | ✓ |
| `MassMatrix`   | ✓ | ✓ | ✓ | ✓ | ✓ |
| `Coriolis`     | ✓ | ✓ | ✓ | ✓ | ✓ |
| `Gravity`      | ✓ | ✓ | ✓ | ✓ | ✓ |

Every generated file/function encodes the exact same symbolic expression, so you can mix-and-match formats depending on performance and deployment needs.

---
## Inverse Kinematics Utilities
Use symbolic FK/Jacobians with the numerical IK solvers provided:
```matlab
fk  = kin.CalculateFK('Return','handle');
jac = kin.Jacobian   ('Return','handle');

q0 = [0;0];                      % seed
x_des = [1.5; 0.1; 0; 0; 0; 0];  % desired pose

[q_sol, err] = IK_Newton(fk, jac, q0, x_des);
```
Choose `IK_Gradient` for a simple gradient descent alternative.

---
## Inverse Kinematics Algorithms
| Algorithm | Access Function | Key Parameters | Pros | Cons |
|-----------|-----------------|----------------|------|------|
| Gradient Descent | `IK_Gradient(fkFun, JFun, q0, xDes, MaxIter, Tol, Alpha)` | `Alpha` – step size (default 0.05) | *Simple*, never requires matrix inversion, stable for most configurations | **Slow** convergence, sensitive to step size, may stall near singularities |
| Newton (Pseudo-Inverse) | `IK_Newton(fkFun, JFun, q0, xDes, MaxIter, Tol, 0)` | (use `Damping = 0`) | *Quadratic* convergence near solution, few iterations | Requires Jacobian pseudo-inverse (instability near singularities) |
| Damped Least-Squares (DLS) | `IK_Newton(fkFun, JFun, q0, xDes, MaxIter, Tol, damping)` with `damping > 0` | `damping` – regularisation term (e.g. 0.01) | Robust near singularities, trades accuracy for stability | Convergence slower than pure Newton, tuning damping is task-dependent |

**Usage Example – Damped LS**
```matlab
fk  = kin.CalculateFK('Return','handle');
J   = kin.Jacobian   ('Return','handle');

q0  = zeros(kin.DOF,1);
xDes= [0.5 0.2 0 0 0 0]';

[q,dErr] = IK_Newton(fk, J, q0, xDes, 200, 1e-6, 0.01); % damping=0.01
```

---
## Examples
Run ready-made demos in the *examples* directory:
```matlab
>> run examples/UR10/UR10Dynamics              % UR10 FK/Jacobian/Dynamics
>> run examples/SCARA/SCARADynamics            % SCARA FK/Jacobian/Dynamics
>> run "examples/Kinova Gen3/KinGen3Dynamics"  % Kinova Gen3 FK/Jacobian/Dynamics
```
Each script constructs `DHStruct` & `DynStruct`, builds kinematics & dynamics, and visualises results.

---
## Testing
Basic regression tests are located in `test/`:
```matlab
>> run test/PlanarRR    % 2-link planar RR arm
>> run test/PlanarRPR   % 3-link planar RPR arm
```

---
## Requirements
* MATLAB R2021a or newer (earlier versions may work)  
* Symbolic Math Toolbox for full functionality  
* MATLAB Coder (optional) for MEX / C code generation

---
## License
This project is released under the terms of the **MIT License** – see [LICENSE](LICENSE) for details.
