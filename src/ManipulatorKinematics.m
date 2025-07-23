%MANIPULATORKINEMATICS Computes manipulator kinematics from DH parameters.
%   This class provides tools to compute the forward kinematics and
%   Jacobian matrix of a robotic manipulator defined by its
%   Denavit-Hartenberg (DH) parameters.
%
%   The kinematics can be derived symbolically, returned as function handles,
%   or generated into M-files, C-code, or MEX files for performance.
%
%   MANIPULATORKINEMATICS Properties (Access = private):
%       DH  - A structure containing the DH parameters of the manipulator.
%       DOF - The number of degrees of freedom of the manipulator.
%
%   MANIPULATORKINEMATICS Methods:
%       ManipulatorKinematics - Constructs a ManipulatorKinematics object.
%       CalculateFK           - Computes the forward kinematics.
%       Jacobian              - Computes the manipulator Jacobian matrix.
%
%   Example:
%       % Define DH parameters for a 2-DOF planar manipulator using DHStruct
%       dhParams = DHStruct('alpha', [0, 0], 'a', [1, 1], 'd', [0, 0], ...
%                           'theta', [0, 0], 'type', 'rr');
%
%       % Create kinematics object
%       kin = ManipulatorKinematics(dhParams);
%
%       % Get a function handle for the forward kinematics
%       fk_fun = kin.CalculateFK('Return', 'handle');
%
%       % Calculate the end-effector pose for a given joint configuration
%       q = [pi/2; 0];
%       pose = fk_fun(q);
%
%       % Get the symbolic geometric Jacobian
%       J_geom = kin.Jacobian('Type', 'geometric');
%
%   See also: ManipulatorDynamics, ParseDH, DHStruct.

classdef ManipulatorKinematics
    properties (Access = private)
        DH
        DOF
    end

    methods
        % ───────────────────────── Constructor ─────────────────────────
        function obj = ManipulatorKinematics(DH)
            %MANIPULATORKINEMATICS Construct a ManipulatorKinematics object.
            %   OBJ = MANIPULATORKINEMATICS(DH) creates a kinematics model object
            %   from the Denavit-Hartenberg (DH) parameters.
            %
            %   Input Arguments:
            %       DH - A structure containing the DH parameters. It must have
            %            the following fields, each being a 1-by-n vector:
            %           .alpha - Link twist angles.
            %           .a     - Link lengths.
            %           .d     - Link offsets.
            %           .theta - Joint angles.
            %           .type  - A character array indicating joint types ('r' for
            %                    revolute, 'p' for prismatic, 'f' for fixed).
            %
            %   Output Arguments:
            %       OBJ - The created ManipulatorKinematics object.
            %
            %   Throws:
            %       ManipulatorKinematics:BadDH - If the DH struct is missing
            %                                     required fields.
            %       ManipulatorKinematics:SizeMismatch - If DH parameter vectors
            %                                            have inconsistent lengths.
            arguments, DH (1, 1) struct, end
            dhReq = {'alpha', 'a', 'd', 'theta', 'type'};
            if ~all(isfield(DH, dhReq))
                dhMissing = dhReq(~isfield(DH, dhReq));
                error('ManipulatorKinematics:BadDH', ...
                    'DH struct missing field(s): %s', strjoin(dhMissing, ', '));
            end
            nPrismatic = length(regexp(DH.type, '[p]'));
            nRevolute  = length(regexp(DH.type, '[r]'));
            nFixed     = length(regexp(DH.type, '[f]'));
            nLinks     = nPrismatic + nRevolute + nFixed;
            if any([numel(DH.a), numel(DH.d), numel(DH.theta), numel(DH.type)]  ~=  nLinks)
                error('ManipulatorKinematics:SizeMismatch', ...
                    'Each DH vector must have %d elements.', nLinks);
            end
            obj.DH  = orderfields(DH);
            obj.DOF = nLinks - nFixed;
        end

        % ───────────────────── Forward-Kinematics ──────────────────────
        function [RotKine, TransKine] = CalculateFK(obj, varargin)
            %CALCULATEFK Computes the forward kinematics of the manipulator.
            %   [POSE] = CALCULATEFK(OBJ) returns the symbolic expression for the
            %   end-effector pose (position and orientation).
            %
            %   [ROT, POS] = CALCULATEFK(OBJ) returns the orientation (as Euler
            %   angles) and position as separate symbolic expressions.
            %
            %   FK = CALCULATEFK(OBJ, 'Return', 'handle') returns a function
            %   handle for the forward kinematics.
            %
            %   CALCULATEFK(OBJ, 'Generate', 'mfile'/'ccode'/'mex', 'File', FILENAME)
            %   generates files for the symbolic expression.
            %
            %   Input Arguments:
            %       OBJ - A ManipulatorKinematics object.
            %
            %   Name-Value Pair Arguments:
            %       'Return' - Specifies the return type.
            %           "symbolic" (default): Returns symbolic expression(s).
            %           "handle"  : Returns a function handle.
            %
            %       'Generate' - Generates a file from the symbolic expression.
            %           "none"    (default): No file is generated.
            %           "mfile"   : Generates a MATLAB function (.m).
            %           "ccode"   : Generates a C code file (.c).
            %           "mex"     : Generates a compiled MEX function.
            %
            %       'File' - Specifies the base filename for the generated file.
            %                Default: "forward_kinematics".
            %
            %       'Rows' - A vector specifying which rows of the 6x1 pose vector
            %                [x;y;z;phi;theta;psi] to include in the output.
            %                Default: 1:6 (all rows).
            %
            %   Output Arguments:
            %       RotKine (or POSE) - The primary output, which can be the full
            %                         pose, rotation, a symbolic expression, or a
            %                         function handle, depending on other arguments.
            %       TransKine         - (Optional) The symbolic position vector.
            %
            %   See also: Jacobian.
            Parser = inputParser;
            addParameter(Parser, 'Generate', "none", @(s)isstring(s)||ischar(s));
            addParameter(Parser, 'File'    , "forward_kinematics", @(s)isstring(s)||ischar(s));
            addParameter(Parser, 'Rows'    , [], @(x)isnumeric(x)&&isvector(x));
            addParameter(Parser, 'Return'  , "symbolic", @(s)isstring(s)||ischar(s));
            parse(Parser, varargin{:});

            gType = lower(string(Parser.Results.Generate));
            fBase = char(Parser.Results.File);
            rows  = Parser.Results.Rows; if isempty(rows), rows = 1:6; end
            outTyp =  lower(string(Parser.Results.Return));

            % ---- Symbolic parameter check for codegen/handle ----------
            if (outTyp == "handle" || ismember(gType, ["mfile", "mex"])) && obj.isSymbolicDH()
                error('ManipulatorKinematics:SymbolicCodegen', ...
                    ['Cannot generate MATLAB function, MEX, or function handle when DH parameters are symbolic. ', ...
                    'Please provide numeric DH parameters.']);
            end

            [DHMod, q] = obj.SymbolicDH;
            [R, P]     = ParseDH(DHMod);
            RotSym    = Rot2Eul(R{end});
            PosSym    = P{end};
            PoseFull  = [PosSym(:); RotSym(:)];
            PoseSel   = PoseFull(rows);

            % ---- Choose What to Return ------------------------------------------------------
            if outTyp  ==  "handle"
                RotKine   = matlabFunction(PoseSel, 'Vars', {q}, 'Outputs', {'x'});
                TransKine = [];
            else
                if nargout<2
                    RotKine = PoseSel;  TransKine = [];
                else
                    RotKine = RotSym;   TransKine = PosSym;
                end
            end

            % ---- Optional Code Generation ---------------------------------------------------
            if gType ~= "none"
                c = matlab.lang.makeValidName(fBase);
                switch gType
                    case "mfile"
                        matlabFunction(PoseSel, 'File', c, 'Vars', {q}, 'Outputs', {'x'}, ...
                            'Comment', 'Forward kinematics (selected rows)');
                        fprintf('MATLAB function "%s.m" generated.\n', c);

                    case "ccode"
                        fid = fopen([c '.c'], 'w');
                        fprintf(fid, '/* Forward kinematics generated by CalculateFK */\n');
                        fprintf(fid, '%s', ccode(PoseSel)); fclose(fid);
                        fprintf('C code "%s.c" generated.\n', c);

                    case "mex"
                        wrap = [c '_wrap'];
                        matlabFunction(PoseSel, 'File', wrap, 'Vars', {q}, 'Outputs', {'x'});
                        n = obj.DOF;
                        codegen(wrap, '-o', c, '-args', {coder.typeof(0, [n 1], false)});
                        delete([wrap '.m']);
                        fprintf('MEX file "%s.%s" generated.\n', c, mexext);
                end
            end
        end

        % ───────────────────────── Jacobian ────────────────────────────
        function J = Jacobian(obj, varargin)
            %JACOBIAN Computes the manipulator Jacobian matrix.
            %   J = JACOBIAN(OBJ) returns the symbolic geometric Jacobian matrix.
            %
            %   J = JACOBIAN(OBJ, 'Type', 'analytical') returns the analytical
            %   Jacobian matrix instead.
            %
            %   J = JACOBIAN(OBJ, 'Return', 'handle') returns a function handle
            %   for the Jacobian.
            %
            %   JACOBIAN(OBJ, 'Generate', 'mfile'/'ccode'/'mex', 'File', FILENAME)
            %   generates files for the symbolic expression.
            %
            %   Input Arguments:
            %       OBJ - A ManipulatorKinematics object.
            %
            %   Name-Value Pair Arguments:
            %       'Type' - Specifies the type of Jacobian to compute.
            %           "geometric"  (default): Computes the geometric Jacobian.
            %           "analytical": Computes the analytical Jacobian.
            %
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
            %                Default: "jacobian".
            %
            %       'Rows' - A vector specifying which rows of the Jacobian to return.
            %                Default: all rows.
            %

            %   Output Arguments:
            %       J - The Jacobian matrix, as a symbolic expression or function handle.
            %
            %   See also: CalculateFK.
            Parser = inputParser;
            addParameter(Parser, 'Type'    , "geometric", @(s)isstring(s)||ischar(s));
            addParameter(Parser, 'Generate', "none", @(s)isstring(s)||ischar(s));
            addParameter(Parser, 'File'    , "jacobian", @(s)isstring(s)||ischar(s));
            addParameter(Parser, 'Rows'    , [], @(x)isnumeric(x)&&isvector(x));
            addParameter(Parser, 'Return'  , "symbolic", @(s)isstring(s)||ischar(s));
            parse(Parser, varargin{:});

            jType  = lower(string(Parser.Results.Type));
            gType  = lower(string(Parser.Results.Generate));
            fBase  = char(Parser.Results.File);
            rows   = Parser.Results.Rows;
            outTyp = lower(string(Parser.Results.Return));

            % ---- Symbolic parameter check for codegen/handle ----------
            if (outTyp == "handle" || ismember(gType, ["mfile", "mex"])) && obj.isSymbolicDH()
                error('ManipulatorKinematics:SymbolicCodegen', ...
                    ['Cannot generate MATLAB function, MEX, or function handle when DH parameters are symbolic. ', ...
                    'Please provide numeric DH parameters.']);
            end

            % ---- Symbolic Jacobian ----------------------------------------------------------
            [DHMod, q] = obj.SymbolicDH;

            switch jType
                case "geometric"
                    [R, P] = ParseDH(DHMod);
                    Pe    = P{end};
                    Jp    = sym.zeros(3, obj.DOF);
                    Jo    = sym.zeros(3, obj.DOF);
                    zPrev = [0;0;1]; pPrev = [0;0;0];
                    for i = 1:obj.DOF
                        if strcmp(DHMod.notation, 'original')
                            if i>1, zPrev = R{i-1}(:, 3); pPrev = P{i-1}; end
                        else
                            zPrev = R{i}(:, 3); pPrev = P{i};
                        end
                        if DHMod.type(i)  ==  'r'
                            Jo(:, i) = zPrev; Jp(:, i) = cross(zPrev, Pe-pPrev);
                        else
                            Jo(:, i) = sym([0;0;0]); Jp(:, i) = zPrev;
                        end
                    end
                    Jsym = [Jp; Jo];
                case "analytical"
                    [R, P] = ParseDH(DHMod);
                    xSym = [P{end}; Rot2Eul(R{end})];
                    Jsym = jacobian(xSym, q);
                otherwise
                    error('ManipulatorKinematics:BadType', 'Unknown Jacobian');
            end
            if isempty(rows), rows = 1:size(Jsym, 1); end
            Jsel = Jsym(rows, :);

            % ---- Choose What to Return ------------------------------------------------------
            if outTyp == "handle"
                J = matlabFunction(Jsel, 'Vars', {q}, 'Outputs', {'J'});
            else
                J = Jsel;
            end

            % ---- Optional Code Generation ---------------------------------------------------
            if gType ~= "none"
                c = matlab.lang.makeValidName(fBase);
                switch gType
                    case "mfile"
                        matlabFunction(Jsel, 'File', c, 'Vars', {q}, 'Outputs', {'J'}, ...
                            'Comment', ['Jacobian (' jType ', selected rows)']);
                        fprintf('MATLAB function "%s.m" generated.\n', c);

                    case "ccode"
                        fid = fopen([c '.c'], 'w');
                        fprintf(fid, '/* Jacobian (%s) generated by ManipulatorKinematics */\n', jType);
                        fprintf(fid, '%s', ccode(Jsel)); fclose(fid);
                        fprintf('C code "%s.c" generated.\n', c);

                    case "mex"
                        wrap = [c '_wrap'];
                        matlabFunction(Jsel, 'File', wrap, 'Vars', {q}, 'Outputs', {'J'});
                        n = obj.DOF;
                        codegen(wrap, '-o', c, '-args', {coder.typeof(0, [n 1], false)});
                        delete([wrap '.m']);
                        fprintf('MEX file "%s.%s" generated.\n', c, mexext);
                end
            end
        end
    end

    methods (Access = private)
        function [DHMod, q] = SymbolicDH(obj)
            q = sym('q', [obj.DOF 1], 'real');
            DHMod       = obj.DH;
            DHMod.alpha = sym(DHMod.alpha); DHMod.a = sym(DHMod.a);
            DHMod.d     = sym(DHMod.d);     DHMod.theta = sym(DHMod.theta);
            for i = 1:obj.DOF
                if obj.DH.type(i) == 'r'
                    DHMod.theta(i) = DHMod.theta(i) + q(i);
                elseif obj.DH.type(i) == 'p'
                    DHMod.d(i)     = DHMod.d(i) + q(i);
                end
            end
        end
        function tf = isSymbolicDH(obj)
            %ISSYMBOLICDH Returns true if any DH parameter is symbolic
            tf = false;
            DH = obj.DH;
            dhFields = {'alpha','a','d','theta'};
            for k = 1:numel(dhFields)
                if any(arrayfun(@(x) isa(x,'sym'), DH.(dhFields{k})(:)))
                    tf = true; return;
                end
            end
        end
    end
end
