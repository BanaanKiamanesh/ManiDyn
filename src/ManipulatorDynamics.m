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
                    || numel(DynPar.Inertia) ~= nLinks ...
                    || ~all(cellfun(@(c)isequal(size(c), [3, 3]), DynPar.Inertia))
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

            opt = {'Length', 'Radius'};
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
            obj.BuildDynamics;
            B = obj.ReturnFormat(obj.B, 'B', varargin{:});
        end

        function C = Coriolis(obj, varargin)
            obj.BuildDynamics;
            C = obj.ReturnFormat(obj.C, 'C', varargin{:});
        end

        function g = Gravity(obj, varargin)
            obj.BuildDynamics;
            g = obj.ReturnFormat(obj.g, 'g', varargin{:});
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

            % ---- Centre-of-Mass positions -----------------------------
            if ~isfield(obj.Par,'COM') || isempty(obj.Par.COM)
                error('ManipulatorDynamics:MissingCOM', ...
                      'DynPar.COM is required but was not provided.');
            end
            COM = obj.Par.COM;
            PC  = cell(1,n);
            for i = 1:n
                ci   = sym(COM(i,:).');
                PC{i}= P{i} + R{i}*ci;
            end

            % ---- Geometric Jacobian (only orientation part needed) ----
            Kin = ManipulatorKinematics(obj.Par.DH);
            Jg  = Kin.Jacobian('Type', 'geometric', 'Return', 'symbolic');
            Jo  = Jg(4:6, :);

            % ---- Mass Matrix B(q) -------------------------------------
            B_ = sym.zeros(n);
            for i = 1:n
                m  = obj.Par.Mass(i);
                Ic = sym(obj.Par.Inertia{i});
                Ri = R{i};

                % COM linear-velocity Jacobian
                Jp_ci = jacobian(PC{i}, q_);

                % angular-velocity Jacobian (first i columns active)
                Jo_i = [Jo(:, 1:i), sym.zeros(3, n-i)];

                % add link contribution
                B_ = B_ + m*(Jp_ci.'*Jp_ci) + Jo_i.'*Ri*Ic*Ri.'*Jo_i;
            end
            obj.B = B_;

            % ---- Gravity Vector g(q) -----------------------------------
            U = -sum( arrayfun(@(k)obj.Par.Mass(k) * g0_' * PC{k}, 1:n) );
            obj.g = jacobian(U, q_)';

            % ---- Coriolis / Centrifugal Matrix C(q, qdot) --------------
            C_ = sym.zeros(n);
            for i = 1:n
                for j = 1:n
                    cij = sym(0);
                    for k = 1:n
                        cij = cij + 0.5*( diff(B_(i, j), q_(k)) ...
                            + diff(B_(i, k), q_(j)) ...
                            - diff(B_(j, k), q_(i)) ) * qd_(k);
                    end
                    C_(i, j) = cij;
                end
            end
            obj.C = C_;
        end

        % ───────── Format Output + Optional File Generation ─────────────
        function out = ReturnFormat(obj, SymExpr, base, varargin)
            p = inputParser;
            addParameter(p, 'Return', "symbolic", @(s)isstring(s)||ischar(s));
            addParameter(p, 'Generate', "none" , @(s)isstring(s)||ischar(s));
            addParameter(p, 'File', "dynamics", @(s)isstring(s)||ischar(s));
            parse(p, varargin{:});

            outTyp = lower(string(p.Results.Return));
            gType  = lower(string(p.Results.Generate));
            fBase  = char(p.Results.File);

            % ---- Return Type ------------------------------------------
            switch outTyp
                case "handle"
                    vars = {obj.q};
                    if base == "C", vars = {obj.q, obj.qd}; end
                    out = matlabFunction(SymExpr, 'Vars', vars, ...
                        'Outputs', {char(base)});
                otherwise
                    out = SymExpr;
            end

            % ---- Optional Code Generation -----------------------------
            if gType ~= "none"
                valid = matlab.lang.makeValidName(fBase);
                vars = {obj.q};  if base=="C", vars = {obj.q, obj.qd}; end
                switch gType
                    case "mfile"
                        matlabFunction(SymExpr, 'File', [valid '_' base], ...
                            'Vars', vars, 'Outputs', {char(base)});
                        fprintf('MATLAB file "%s_%s.m" generated.\n', valid, base);
                    case "ccode"
                        fid = fopen([valid '_' base '.c'], 'w');
                        fprintf(fid, '/* %s generated by ManipulatorDynamics */\n', base);
                        fprintf(fid, '%s', ccode(SymExpr));
                        fclose(fid);
                        fprintf('C code "%s_%s.c" generated.\n', valid, base);
                end
            end
        end
    end
end
