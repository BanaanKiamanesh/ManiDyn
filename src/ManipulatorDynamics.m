%MANIPULATORDYNAMICS Derives manipulator dynamics terms from physical parameters.
%   This class computes the terms of the standard manipulator dynamics
%
%   equation: B(q)q_dd + C(q, qd)qd + Fv qd + Fc sgn(qd) + g(q) = tau, where:
%     - B(q) is the n-by-n mass matrix.
%     - C(q, qd) is the n-by-n Coriolis and centrifugal effects matrix.
%     - g(q) is the n-by-1 gravity vector.
%     - q, qd, q_dd are the joint position, velocity, and acceleration vectors.
%     - tau is the vector of joint torques/forces.
%
%   The dynamics are derived symbolically using the Lagrangian formulation.
%   The class can return these terms as symbolic expressions, function
%   handles, or generate M-files, C-code, or MEX files for performance.
%
%   MANIPULATORDYNAMICS Properties (Access = private):
%       Par - Structure containing the manipulator's dynamic parameters.
%       g0  - Gravity acceleration vector.
%       Fv  - Diagonal matrix of viscous friction coefficients.
%       Fc  - Diagonal matrix of Coulomb friction coefficients.
%       B   - Symbolic mass matrix.
%       C   - Symbolic Coriolis/centrifugal matrix.
%       g   - Symbolic gravity vector.
%       q   - Symbolic joint variable vector.
%       qd  - Symbolic joint velocity vector.
%
%   MANIPULATORDYNAMICS Methods:
%       ManipulatorDynamics - Constructor for the class.
%       MassMatrix          - Returns the mass matrix B(q).
%       Coriolis            - Returns the Coriolis matrix C(q, qd).
%       Gravity             - Returns the gravity vector g(q).
%       ODEFunction         - Builds a full-state ODE rhs for simulation.
%
%   Example:
%       % Define DH parameters for a 2-link planar robot using DHStruct
%       dhParams = DHStruct('alpha', [0, 0], 'a', [1, 1], 'd', [0, 0], ...
%                           'theta', [0, 0], 'type', 'rr');
%
%       % Define dynamic parameters using DynStruct
%       dynParams = DynStruct('Mass'   , [1, 1], ...
%                             'Inertia', {zeros(3), zeros(3)}, ...
%                             'COM'    , [0.5 0 0; 0.5 0 0], ...
%                             'DH'     , dhParams);
%
%       % Create dynamics object
%       dyn = ManipulatorDynamics(dynParams);
%
%       % Mass matrix as function handle
%       B_fun = dyn.MassMatrix('Return', 'handle');
%
%       % Evaluate for a given joint configuration
%       q_vals = [pi/4; pi/2];
%       B_val = B_fun(q_vals);
%
%       % Build ODE rhs for simulation (zero input)
%       ode = dyn.ODEFunction();
%       x0  = [q_vals; zeros(2,1)];
%       tau = @(t) zeros(2,1);
%       [t,y] = ode45(@(t,x) ode(t,x,tau(t)), [0 10], x0);
%
%   See also: ManipulatorKinematics, DynStruct, DHStruct.

classdef ManipulatorDynamics < handle
    properties (Access = private)
        Par

        g0 (3, 1) double = [0; 0; -9.80665]

        B  sym = sym.empty
        C  sym = sym.empty
        g  sym = sym.empty

        q  sym
        qd sym
    end

    methods
        % ───────────────────────── Constructor ─────────────────────────
        function obj = ManipulatorDynamics(DynPar, opts)
            %MANIPULATORDYNAMICS Construct an instance of the class.
            %   OBJ = MANIPULATORDYNAMICS(DynPar) creates a dynamics model
            %   object from the dynamic parameters specified in the DynPar struct.
            %
            %   OBJ = MANIPULATORDYNAMICS(DynPar, 'Gravity', G) specifies a
            %   custom gravity vector G as a 1x3 array, e.g., [0, 0, -9.81].
            %
            %   Input Arguments:
            %       DynPar - A structure containing the required dynamic
            %                parameters for the manipulator. It must contain
            %                the following fields:
            %           .Mass   - 1-by-n vector of link masses.
            %           .Inertia- 1-by-n cell array, where each cell contains a
            %                     3-by-3 inertia tensor for a link, defined in
            %                     its own frame.
            %           .COM    - n-by-3 matrix where each row is the center of
            %                     mass [x,y,z] for a link, defined in its own frame.
            %           .DH     - A struct containing Denavit-Hartenberg parameters,
            %                     compatible with the ManipulatorKinematics class.
            %                     It must have fields: alpha, a, d, theta, type.
            %
            %       'Gravity' - (Optional name-value pair) A 1x3 vector specifying the
            %                   gravity acceleration [gx, gy, gz].
            %                   Default: [0, 0, -9.80665].
            %
            %   Output Arguments:
            %       OBJ - The created ManipulatorDynamics object.
            %
            %   Throws:
            %       ManipulatorDynamics:MissingField - If DynPar is missing
            %                                          required fields.
            %       ManipulatorDynamics:BadInertia   - If the Inertia cell
            %                                          array is malformed.
            %       ManipulatorDynamics:BadDH        - If the DH struct is
            %                                          malformed.
            %       ManipulatorDynamics:SizeMismatch - If DH parameter vectors
            %                                          have inconsistent lengths.
            arguments
                DynPar (1, 1) struct
                opts.Gravity (1, 3) double = [0, 0, -9.80665]
            end

            % -------- Validation ----------------------------------------
            req = {'Mass', 'Inertia', 'DH'};
            missing = req(~isfield(DynPar, req));
            if ~isempty(missing)
                error('ManipulatorDynamics:MissingField', ...
                    'DynPar missing required field(s): %s', strjoin(missing, ', '));
            end
            nLinks = numel(DynPar.Mass);
            if ~iscell(DynPar.Inertia) ...
                    || numel(DynPar.Inertia)~=nLinks ...
                    || ~all(cellfun(@(c)isequal(size(c), [3 3]), DynPar.Inertia))
                error('ManipulatorDynamics:BadInertia', 'Inertia list ill-formed.');
            end
            dhReq = {'alpha', 'a', 'd', 'theta', 'type'};
            if ~isstruct(DynPar.DH) || ~all(isfield(DynPar.DH, dhReq))
                error('ManipulatorDynamics:BadDH', 'DH struct malformed.');
            end
            if any([numel(DynPar.DH.alpha), numel(DynPar.DH.a), ...
                    numel(DynPar.DH.d), numel(DynPar.DH.theta), ...
                    numel(DynPar.DH.type)] ~= nLinks)
                error('ManipulatorDynamics:SizeMismatch', 'DH vectors wrong length.');
            end
            opt = {'Length', 'Radius', 'Fv', 'Fc'};
            for k = 1:numel(opt)
                if isfield(DynPar, opt{k}) && ~isempty(DynPar.(opt{k})) ...
                        && numel(DynPar.(opt{k})) ~= nLinks
                    error('ManipulatorDynamics:SizeMismatch', ...
                        '%s must have %d elements.', opt{k}, nLinks);
                end
            end
            obj.Par = orderfields(DynPar);
            obj.g0  = opts.Gravity(:);
        end

        % ───────────────────── Public Getters ──────────────────────────
        function B = MassMatrix(obj, varargin)
            %MASSMATRIX Returns the manipulator mass matrix B(q).
            %   B = MASSMATRIX(OBJ) returns the n-by-n symbolic mass matrix B(q).
            %
            %   B = MASSMATRIX(OBJ, 'Return', 'handle') returns a function handle
            %   B_fun = @(q) ... that evaluates the mass matrix for a given
            %   n-by-1 joint state vector q.
            %
            %   MASSMATRIX(OBJ, 'Generate', 'mfile'/'ccode'/'mex', 'File', FILENAME)
            %   generates files for the symbolic expression.
            %
            %   Input Arguments:
            %       OBJ - A ManipulatorDynamics object.
            %
            %   Name-Value Pair Arguments:
            %       'Return' - Specifies the return type.
            %           "symbolic" (default): Returns a symbolic matrix.
            %           "handle"  : Returns a function handle.
            %
            %       'Generate' - Generates a file from the symbolic expression.
            %           "none"    (default): No file is generated.
            %           "mfile"   : Generates a MATLAB function (.m).
            %           "ccode"   : Generates a C code file (.c).
            %           "mex"     : Generates a compiled MEX function.
            %
            %       'File' - Specifies the base filename for the generated file.
            %                Default: "dynamics".
            %
            %   Output Arguments:
            %       B - The mass matrix, either as a sym or function_handle.
            %
            %   See also: Coriolis, Gravity.
            obj.BuildDynamics;
            B = obj.ReturnFormat(obj.B, 'B', varargin{:});
        end
        function C = Coriolis(obj, varargin)
            %CORIOLIS Returns the manipulator Coriolis and centrifugal matrix C(q, qd).
            %   C = CORIOLIS(OBJ) returns the n-by-n symbolic Coriolis matrix C(q, qd).
            %
            %   C = CORIOLIS(OBJ, 'Return', 'handle') returns a function handle
            %   C_fun = @(q, qd) ... that evaluates the Coriolis matrix for a given
            %   n-by-1 joint state vector q and joint velocity vector qd.
            %
            %   CORIOLIS(OBJ, 'Generate', 'mfile'/'ccode'/'mex', 'File', FILENAME)
            %   generates files for the symbolic expression.
            %
            %   Input Arguments:
            %       OBJ - A ManipulatorDynamics object.
            %
            %   Name-Value Pair Arguments:
            %       'Return' - Specifies the return type.
            %           "symbolic" (default): Returns a symbolic matrix.
            %           "handle"  : Returns a function handle.
            %
            %       'Generate' - Generates a file from the symbolic expression.
            %           "none"    (default): No file is generated.
            %           "mfile"   : Generates a MATLAB function (.m).
            %           "ccode"   : Generates a C code file (.c).
            %           "mex"     : Generates a compiled MEX function.
            %
            %       'File' - Specifies the base filename for the generated file.
            %                Default: "dynamics".
            %
            %   Output Arguments:
            %       C - The Coriolis matrix, either as a sym or function_handle.
            %
            %   See also: MassMatrix, Gravity.
            obj.BuildDynamics;
            C = obj.ReturnFormat(obj.C, 'C', varargin{:});
        end
        function g = Gravity(obj, varargin)
            %GRAVITY Returns the manipulator gravity vector g(q).
            %   g = GRAVITY(OBJ) returns the n-by-1 symbolic gravity vector g(q).
            %
            %   g = GRAVITY(OBJ, 'Return', 'handle') returns a function handle
            %   g_fun = @(q) ... that evaluates the gravity vector for a given
            %   n-by-1 joint state vector q.
            %
            %   GRAVITY(OBJ, 'Generate', 'mfile'/'ccode'/'mex', 'File', FILENAME)
            %   generates files for the symbolic expression.
            %
            %   Input Arguments:
            %       OBJ - A ManipulatorDynamics object.
            %
            %   Name-Value Pair Arguments:
            %       'Return' - Specifies the return type.
            %           "symbolic" (default): Returns a symbolic vector.
            %           "handle"  : Returns a function handle.
            %
            %       'Generate' - Generates a file from the symbolic expression.
            %           "none"    (default): No file is generated.
            %           "mfile"   : Generates a MATLAB function (.m).
            %           "ccode"   : Generates a C code file (.c).
            %           "mex"     : Generates a compiled MEX function.
            %
            %       'File' - Specifies the base filename for the generated file.
            %                Default: "dynamics".
            %
            %   Output Arguments:
            %       g - The gravity vector, either as a sym or function_handle.
            %
            %   See also: MassMatrix, Coriolis.
            obj.BuildDynamics;
            g = obj.ReturnFormat(obj.g, 'g', varargin{:});
        end

        % ───────────────────── ODE Function Builder ──────────────────────
        function odeFun = ODEFunction(obj, varargin)
            %ODEFUNCTION Build a full-state ODE function for numerical solvers.
            %   F = ODEFUNCTION(OBJ) returns a function handle compatible with
            %   MATLAB ODE solvers (e.g., ode45). The handle has the form
            %       F = @(t, x, tau) x_dot
            %   where the state vector x = [q; qd] stacks joint positions q and
            %   joint velocities qd, and tau is the n-by-1 vector of joint input
            %   torques/forces.  Internally the acceleration is solved from
            %
            %       B(q)q̈ = τ − C(q, q̇)q̇ − Fv q̇ − Fc·sgn(q̇) − g(q)
            %
            %   with Fv = diag(Viscous Friction Coefficients) and
            %   Fc = diag(Coulomb Friction Coefficients).  Both friction vectors
            %   are optional and default to zero if not provided in the
            %   `DynStruct`.
            %
            %   The output x_dot = [q̇; q̈].
            %
            %   Name-Value Pair Arguments:
            %       'Return'  – "handle" (default) | "symbolic"
            %           "handle"  – Return a function handle.
            %           "symbolic" – Return the symbolic expression of x_dot.
            %
            %       'Generate' – "none" (default) | "mfile" | "mex"
            %           Optionally generate a MATLAB file or compiled MEX file.
            %
            %       'File' – Base file name for code generation (default "ode").
            %
            %   Example:
            %       dyn = ManipulatorDynamics(dynPar);
            %       ode = dyn.ODEFunction();
            %       tauFun = @(t)[sin(t); cos(t)];
            %       [t, y] = ode45(@(t, y) ode(t, y, tauFun(t)), [0 10], zeros(2*n,1));
            %
            %   See also: ode45, ode15s, MassMatrix, Coriolis, Gravity.

            % Ensure dynamics are available
            obj.BuildDynamics;
            n = numel(obj.Par.Mass);

            % -- Symbolic vector field -------------------------------------------
            x   = sym('x', [2*n 1], 'real');
            q   = x(1:n);          % positions
            qd  = x(n+1:end);      % velocities
            tau = sym('tau', [n 1], 'real');
            t   = sym('t', 'real'); %#ok<NASGU>

            Bq = subs(obj.B,  obj.q,  q);

            % --- Friction Vectors ------------------------------------
            if isfield(obj.Par, 'Fv') && ~isempty(obj.Par.Fv)
                fvVec = sym(obj.Par.Fv(:));
            else
                fvVec = sym(zeros(n,1));
            end
            if isfield(obj.Par, 'Fc') && ~isempty(obj.Par.Fc)
                fcVec = sym(obj.Par.Fc(:));
            else
                fcVec = sym(zeros(n,1));
            end

            gq = subs(obj.g,  obj.q,  q);
            Cq = subs(obj.C, [obj.q; obj.qd], [q; qd]);

            frictionTerm = fvVec.*qd + fcVec.*sign(qd);
            qdd      = Bq \ (tau - Cq*qd - frictionTerm - gq);
            xdotSym  = [qd; qdd];

            HasReturn = any(strcmpi(varargin(1:2:end), 'Return'));
            ExtraArgs = {};
            if ~HasReturn, ExtraArgs = {'Return', 'handle'}; end

            odeFun = obj.ReturnFormat(xdotSym, 'x_dot', varargin{:}, ExtraArgs{:}, ...
                'Vars', {t, x, tau}, 'Output', 'x_dot');
        end
    end

    methods (Access = private)
        % ───────────────────────── Build Dynamics ───────────────────────
        function BuildDynamics(obj)
            if ~isempty(obj.B)
                return
            end        % already built

            n   = numel(obj.Par.Mass);
            q_  = sym('q' , [n 1], 'real');
            qd_ = sym('qd', [n 1], 'real');
            g0_ = sym(obj.g0);

            obj.q  = q_;
            obj.qd = qd_;

            % ---- DH with Joint Variables Added ------------------------
            DHMod = obj.Par.DH;
            DHMod.alpha = sym(DHMod.alpha);  DHMod.a = sym(DHMod.a);
            DHMod.d     = sym(DHMod.d);      DHMod.theta = sym(DHMod.theta);
            for i = 1:n
                if DHMod.type(i) == 'r'
                    DHMod.theta(i) = DHMod.theta(i) + q_(i);
                elseif DHMod.type(i) == 'p'
                    DHMod.d(i)     = DHMod.d(i)     + q_(i);
                end
            end
            [R, P] = ParseDH(DHMod);

            % ---- COM world positions ----------------------------------
            if ~isfield(obj.Par, 'COM') || isempty(obj.Par.COM)
                error('ManipulatorDynamics:MissingCOM', 'DynPar.COM is required.');
            end
            COM = obj.Par.COM;
            PC  = cell(1, n);
            for i = 1:n
                ci   = sym(COM(i, :).');
                PC{i}= P{i} + R{i}*ci;
            end

            % ---- Geometric Jacobian (orientation part) ----------------
            Kin = ManipulatorKinematics(obj.Par.DH);
            Jg  = Kin.Jacobian('Type', 'geometric', 'Return', 'symbolic');
            Jo  = Jg(4:6, :);

            % ---- Mass Matrix B(q) -------------------------------------
            B_ = sym.zeros(n);
            for i = 1:n
                m  = obj.Par.Mass(i);
                Ic = sym(obj.Par.Inertia{i});
                Ri = R{i};
                Jp_ci = jacobian(PC{i}, q_);
                Jo_i  = [Jo(:, 1:i), sym.zeros(3, n-i)];
                B_ = B_ + m*(Jp_ci.'*Jp_ci) + Jo_i.'*Ri*Ic*Ri.'*Jo_i;
            end
            obj.B = B_;

            % ---- Gravity Vector g(q) -----------------------------------
            U = -sum(arrayfun(@(k)obj.Par.Mass(k)*g0_.'*PC{k}, 1:n));
            obj.g = jacobian(U, q_).';

            % ---- Coriolis / Centrifugal Matrix C(q, qdot) --------------
            C_ = sym.zeros(n);
            for i = 1:n
                for j = 1:n
                    cij = sym(0);
                    for k = 1:n
                        cij = cij + 0.5*(diff(B_(i, j), q_(k)) ...
                            + diff(B_(i, k), q_(j)) ...
                            - diff(B_(j, k), q_(i))) * qd_(k);
                    end
                    C_(i, j) = cij;
                end
            end
            obj.C = C_;
        end

        % ───────── Format Output + Optional File / MEX Generation ──────
        function out = ReturnFormat(obj, SymExpr, base, varargin)
            p = inputParser;
            addParameter(p, 'Return'  , "symbolic", @(s)isstring(s)||ischar(s));
            addParameter(p, 'Generate', "none"   , @(s)isstring(s)||ischar(s));
            addParameter(p, 'File'    , "dynamics", @(s)isstring(s)||ischar(s));
            addParameter(p, 'Vars'    , {}, @(c)iscell(c));
            addParameter(p, 'Output'  , "", @(s)isstring(s)||ischar(s));
            parse(p, varargin{:});

            OutType = lower(string(p.Results.Return));
            gType   = lower(string(p.Results.Generate));
            fBase   = char(p.Results.File);

            vars    = p.Results.Vars;
            if isempty(vars)
                % Default variable list for the original three dynamics terms
                vars = {obj.q};
                if base == "C"
                    vars = {obj.q, obj.qd};
                end
            end

            OutName = char(p.Results.Output);
            if isempty(OutName)
                OutName = char(base);
            end

            % ---- Return Type ------------------------------------------
            switch OutType
                case "handle"
                    out = matlabFunction(SymExpr, 'Vars', vars, 'Outputs', {OutName});
                otherwise
                    out = SymExpr;
            end

            % ---- Optional Code Generation -----------------------------
            if gType ~= "none"
                valid = matlab.lang.makeValidName(fBase);
                switch gType
                    case "mfile"
                        matlabFunction(SymExpr, 'File', [valid '_' OutName], ...
                            'Vars', vars, 'Outputs', {OutName});
                        fprintf('MATLAB file "%s_%s.m" generated.\n', valid, OutName);

                    case "ccode"
                        fid = fopen([valid '_' OutName '.c'], 'w');
                        fprintf(fid, '/* %s generated by ManipulatorDynamics */\n', OutName);
                        if numel(SymExpr) == 1
                            cstr = ccode(SymExpr);
                        else
                            cList = arrayfun(@(idx) ccode(SymExpr(idx)), 1:numel(SymExpr), 'UniformOutput', false);
                            cstr  = strjoin(cList, "\n");
                        end
                        fprintf(fid, '%s', cstr);
                        fclose(fid);
                        fprintf('C code "%s_%s.c" generated.\n', valid, OutName);

                    case "mex"
                        % Generic dimension inference is non-trivial → support only
                        % original B, C, g paths for now.
                        if ~ismember(OutName, {'B','C','g'})
                            error('ReturnFormat:MexUnsupported', ...
                                'MEX generation not supported for output "%s".', OutName);
                        end

                        wrap = [valid '_' OutName '_wrap'];
                        matlabFunction(SymExpr, 'File', wrap, 'Vars', vars, 'Outputs', {OutName});
                        n = numel(obj.q);
                        if strcmp(OutName, 'C')
                            codegen(wrap, '-o', [valid '_' OutName], ...
                                '-args', {coder.typeof(0, [n 1], false), ...
                                coder.typeof(0, [n 1], false)});
                        else
                            codegen(wrap, '-o', [valid '_' OutName], ...
                                '-args', {coder.typeof(0, [n 1], false)});
                        end
                        delete([wrap '.m']);
                        fprintf('MEX file "%s_%s.%s" generated.\n', valid, OutName, mexext);
                end
            end
        end
    end
end
