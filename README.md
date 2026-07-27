# ManiDyn

A **MATLAB®** toolbox for **symbolic** modelling of serial-link robot **Manipulators**.
ManiDyn lets you move seamlessly from high-level geometric descriptions (DH parameters) to
fully-symbolic expressions of

* Forward kinematics
* Geometric / analytical Jacobians
* Dynamics – mass matrix $$B(q)$$, Coriolis/centrifugal matrix $$C(q, \dot{q})$$, gravity vector $$g(q)$$, and regressor matrix $$Y(\ddot{q}, \dot{q}, q)$$

Every quantity—masses, link lengths, DH parameters, inertias, friction coefficients—can be
**numeric *or* symbolic**.  A completely symbolic workflow is therefore possible end-to-end.

All results can be returned as symbolic expressions, lightweight function handles, or automatically
exported as `.m` files, C source, or compiled **MEX** binaries for real-time use.

*(Don’t miss the **Examples** section below for ready-to-run scripts.)*

---

## Table of Contents

- [ManiDyn](#manidyn)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
  - [Defining a Manipulator](#defining-a-manipulator)
  - [URDF Workflow](#urdf-workflow)
  - [Kinematics Workflow](#kinematics-workflow)
  - [Dynamics Workflow](#dynamics-workflow)
  - [Supported Output Formats](#supported-output-formats)
  - [Inverse Kinematics Utilities](#inverse-kinematics-utilities)
  - [Inverse Kinematics Algorithms](#inverse-kinematics-algorithms)
  - [Examples](#examples)
  - [Testing](#testing)
  - [Requirements](#requirements)
  - [Full Example: Advanced Control on a Complex Robotic Arm](#full-example-advanced-control-on-a-complex-robotic-arm)
  - [License](#license)

---

## Installation

```matlab
% clone repository, then from MATLAB:
>> cd <path_to>/ManiDyn
>> install('save')       % adds folders to MATLAB path
```

Alternatively add `src` and `utils` (including its subfolders) to the MATLAB path manually:

```matlab
addpath('src');
addpath(genpath('utils'));
```

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
> kinematics/dynamics functions will automatically follow Craig’s *modified DH*
> convention.  The toolbox handles both notations transparently.

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

% All fields may be symbolic – e.g. sym('m1'), pi/2, etc.
```

---

## URDF Workflow

ManiDyn can read an expanded URDF without Robotics System Toolbox:

```matlab
robot = URDFRead('robot.urdf');

% Returns 1 only when the complete URDF is one serial manipulator
status = IsSerialManipulator('robot.urdf');

% Generate Mass, Inertia, COM, Length, Radius, friction, and standard DH data
DynAll = DynStruct('robot.urdf');

% Or construct the models directly from the same URDF
kin = ManipulatorKinematics('robot.urdf');
dyn = ManipulatorDynamics('robot.urdf');

% Equivalent name-value form; explicitly supplied fields take precedence
DynAll = DynStruct('URDF', 'robot.urdf', 'Fv', zeros(1, 6));
```

`URDFRead` returns a tree-like structure with the base name, body names,
all link and joint structures, parent-child relationships, transforms,
limits, inertial properties, and primitive visual/collision geometry.

`IsSerialManipulator` applies a strict whole-URDF test. The model must be
one connected rooted tree with at least one revolute, continuous, or
prismatic joint, and every movable joint must lie on one root-to-leaf path.
Fixed tool and sensor branches are allowed; movable branches, floating or
planar joints, disconnected models, and closed or multiply-parented
topologies return `0`.

For `DynStruct`, fixed bodies attached to an actuated body are aggregated
using the parallel-axis theorem. COM vectors and inertia tensors are
expressed in the generated standard-DH link frames. Link dimensions come
from collision primitives (or visual primitives when collisions are absent);
mesh-only links use DH-frame spacing and an inertia-equivalent radius.

The input must be an expanded `.urdf` file; Xacro processing is not performed.
Every extracted moving body must have positive inertial mass data.
Both model constructors use `DynStruct` internally for URDF input; existing
`DHStruct` and `DynStruct` constructor inputs remain supported.

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
dyn = ManipulatorDynamics(DynAll);

% Custom gravity direction?  Just pass the vector to the constructor:
dyn_g = ManipulatorDynamics(DynAll, 'Gravity', [0 -9.81 0]); % y-axis gravity

% Friction-aware dynamics
dyn_fric = ManipulatorDynamics(DynAll);

B  = dyn.MassMatrix();                 % symbolic
C  = dyn.Coriolis();
g  = dyn.Gravity();
Y  = dyn.Regressor();

% Numerical handles
Bf = dyn.MassMatrix('Return','handle');
Cf = dyn.Coriolis  ('Return','handle');
gf = dyn.Gravity   ('Return','handle');
Yf = dyn.Regressor ('Return','handle');

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

| Method        | Symbolic (default) | Function handle (`'Return','handle'`) | MATLAB `.m` (`'Generate','mfile'`) | C source (`'Generate','ccode'`) | Compiled MEX (`'Generate','mex'`) |
| ------------- | :----------------: | :-----------------------------------: | :--------------------------------: | :-----------------------------: | :-------------------------------: |
| `CalculateFK` |          ✓         |                   ✓                   |                  ✓                 |                ✓                |                 ✓                 |
| `Jacobian`    |          ✓         |                   ✓                   |                  ✓                 |                ✓                |                 ✓                 |
| `MassMatrix`  |          ✓         |                   ✓                   |                  ✓                 |                ✓                |                 ✓                 |
| `Coriolis`    |          ✓         |                   ✓                   |                  ✓                 |                ✓                |                 ✓                 |
| `Gravity`     |          ✓         |                   ✓                   |                  ✓                 |                ✓                |                 ✓                 |
| `Regressor`   |          ✓         |                   ✓                   |                  ✓                 |                ✓                |                 ✓                 |
| `ODEFunction` |          ✓         |                   ✓                   |                  ✓                 |                ✗                |                 ✓                 |

Every generated file/function encodes the exact same symbolic expression, so you can mix-and-match formats depending on performance and deployment needs.

> **`ODEFunction` generation rules**
> • `'Generate','mfile'` — all helpers **and** the ODE driver are plain `.m` files.
> • `'Generate','mex'`   — the mass-matrix, Coriolis and gravity helpers are compiled **MEX** binaries while the ODE driver remains a slim `.m` file that calls them.
> • `'Generate','ccode'` is **not supported** for `ODEFunction`.

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

| Algorithm                  | Access Function                                                              | Key Parameters                              | Pros                                                                      | Cons                                                                       |
| -------------------------- | ---------------------------------------------------------------------------- | ------------------------------------------- | ------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| Gradient Descent           | `IK_Gradient(fkFun, JFun, q0, xDes, MaxIter, Tol, Alpha)`                    | `Alpha` – step size (default 0.05)          | *Simple*, never requires matrix inversion, stable for most configurations | **Slow** convergence, sensitive to step size, may stall near singularities |
| Newton (Pseudo-Inverse)    | `IK_Newton(fkFun, JFun, q0, xDes, MaxIter, Tol, 0)`                          | (use `Damping = 0`)                         | *Quadratic* convergence near solution, few iterations                     | Requires Jacobian pseudo-inverse (instability near singularities)          |
| Damped Least-Squares (DLS) | `IK_Newton(fkFun, JFun, q0, xDes, MaxIter, Tol, damping)` with `damping > 0` | `damping` – regularisation term (e.g. 0.01) | Robust near singularities, trades accuracy for stability                  | Convergence slower than pure Newton, tuning damping is task-dependent      |

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
>> run "examples/Kinova Gen3/Direct/KinGen3Dynamics"  % Direct parameter definition

% Extract the DynStruct and standard-DH table from a Kinova Gen3 URDF
>> run "examples/Kinova Gen3/WithURDF/KinGen3URDFParameters"

% Build the dynamics model directly and generate its MEX functions
>> run "examples/Kinova Gen3/WithURDF/KinGen3URDFDynamics"
```

The Kinova Gen3 examples separate direct parameter definition, URDF parameter
extraction, and direct URDF dynamics construction. The URDF dynamics example
generates MEX functions for the mass matrix, Coriolis matrix, gravity vector,
and regressor. Deriving the full seven-joint symbolic dynamics model can take
substantial time.

---

## Testing

Basic regression tests are located in `test/`:

```matlab
>> run test/PlanarRR    % 2-link planar RR arm
>> run test/PlanarRPR   % 3-link planar RPR arm

% URDF reader and serial-manipulator detection (all 21 robot fixtures)
>> runtests('test/SerialManipulatorDetection','IncludeSubfolders',true)

% SCARA, UR10, and Kinova Gen3 extraction against actual reference values
>> runtests('test/DynStructURDF','IncludeSubfolders',true)
```

---

## Requirements

* MATLAB R2021a or newer (earlier versions may work)
* Symbolic Math Toolbox for full functionality
* MATLAB Coder (optional) for MEX / C code generation
* Robotics System Toolbox is not required for URDF reading or extraction

---

## Full Example: Advanced Control on a Complex Robotic Arm

For a complete, real-world example of ManiDyn in action—including advanced control algorithms, simulation, and deployment on a highly complex 7-DOF Kinova Gen3 robotic arm—see:

[Kinova-Gen3-Control (GitHub)](https://github.com/BanaanKiamanesh/Kinova-Gen3-Control/)

This repository demonstrates full manipulator modeling, dynamics, and a variety of modern control strategies (PD, PID, feedback linearization, inverse dynamics, sliding mode, and more) all powered by ManiDyn.

---

## License

This project is released under the terms of the **MIT License** – see [LICENSE](LICENSE) for details.
