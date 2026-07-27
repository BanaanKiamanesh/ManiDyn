function DynPar = DynStruct(varargin)
    %DYNSTRUCT Creates a standardized structure for manipulator dynamic parameters.
    %   DynPar = DYNSTRUCT('Mass', M, 'Inertia', I, 'COM', C, 'DH', DH_STRUCT)
    %   creates a structure `DynPar` containing the parameters required for
    %   dynamic analysis.
    %
    %   DynPar = DYNSTRUCT(URDFPATH) reads a serial-manipulator URDF, generates
    %   a standard Denavit-Hartenberg table, transforms the inertial properties
    %   into the generated DH frames, and populates the structure automatically.
    %
    %   DynPar = DYNSTRUCT('URDF', URDFPATH) provides the same URDF workflow
    %   using a name-value pair. Explicit name-value parameters override values
    %   extracted from the URDF.
    %
    %   Input Arguments:
    %       URDFPath  - Path to an expanded serial-manipulator .urdf file.
    %       'Mass'    - A 1-by-n vector of the mass of each link.
    %       'Inertia' - A 1-by-n cell array, where each cell contains the 3x3
    %                   inertia tensor of a link with respect to its own frame.
    %       'COM'     - An n-by-3 matrix where each row specifies the center of
    %                   mass [x, y, z] for the corresponding link in its own frame.
    %       'DH'      - A DH parameter structure, typically created by `DHStruct`.
    %
    %   Name-Value Pair Arguments:
    %       'URDF'    - (Optional) Path to a serial-manipulator .urdf file.
    %       'Length'  - (Optional) A 1-by-n vector of link lengths.
    %       'Radius'  - (Optional) A 1-by-n vector of link radii.
    %       'Fv'      - (Optional) A 1-by-n vector of viscous friction coefficients (N*m*s/rad).
    %       'Fc'      - (Optional) A 1-by-n vector of Coulomb friction coefficients (N*m).
    %
    %   Output Arguments:
    %       DynPar - A structure containing the following fields:
    %                .Mass    - (1xn double)
    %                .Inertia - (1xn cell of 3x3 double)
    %                .COM     - (nx3 double)
    %                .DH      - (struct)
    %                .Length  - (1xn double, optional)
    %                .Radius  - (1xn double, optional)
    %                .Fv      - (1xn double)
    %                .Fc      - (1xn double)
    %
    %   URDF Conversion:
    %       The complete URDF must describe a serial manipulator according to
    %       `IsSerialManipulator`. Fixed bodies rigidly attached to an actuated
    %       link are combined with that link. Collision primitives are used for
    %       Length and Radius when available; otherwise, DH-frame spacing and an
    %       inertia-equivalent radius are used.
    %
    %   Example:
    %       % Create parameters directly
    %       mass = 1;
    %       inertia = eye(3);
    %       com = [0.5, 0, 0];
    %       dh = DHStruct('alpha',0,'a',1,'d',0,'theta',0,'type','r');
    %       dyn_params = DynStruct('Mass', mass, 'Inertia', {inertia}, ...
    %                              'COM', com, 'DH', dh);
    %
    %       % Or create all parameters from a URDF
    %       dyn_params = DynStruct('ur10.urdf');
    %
    %   Throws:
    %       RobotStruct:MissingField - If a required field (Mass, Inertia, COM, DH)
    %                                  is missing.
    %       RobotStruct:SizeMismatch - If input arrays have inconsistent sizes.
    %       DynStruct:NotSerialManipulator - If the URDF is not a serial manipulator.
    %       DynStruct:MissingInertial - If an extracted moving body has no usable
    %                                   inertial data.
    %
    %   See also: ManipulatorDynamics, DHStruct, URDFRead, IsSerialManipulator.


    isNumOrSym = @(x) isnumeric(x) || isa(x, 'sym');
    isTextScalar = @(x) (ischar(x) && isrow(x)) || (isstring(x) && isscalar(x));

    % Support the compact DynStruct(URDFPath) form.
    if ~isempty(varargin) && isTextScalar(varargin{1}) && ...
            (isfile(char(varargin{1})) || endsWith(lower(string(varargin{1})), '.urdf'))
        varargin = [{'URDF', varargin{1}}, varargin(2:end)];
    end

    % Input Parsing
    Parser = inputParser;
    Parser.FunctionName = 'DynStruct';
    addParameter(Parser, 'URDF'   , '', @(x) isempty(x) || isTextScalar(x));
    addParameter(Parser, 'Mass'   , [], @(x) isvector(x) && isNumOrSym(x));
    addParameter(Parser, 'Length' , [], @(x) isvector(x) && isNumOrSym(x));
    addParameter(Parser, 'Radius' , [], @(x) isvector(x) && isNumOrSym(x));
    addParameter(Parser, 'Fv'     , [], @(x) isvector(x) && isNumOrSym(x));
    addParameter(Parser, 'Fc'     , [], @(x) isvector(x) && isNumOrSym(x));
    addParameter(Parser, 'Inertia', [], @(x) iscell(x) && ...
        all(cellfun(@(c) isequal(size(c), [3 3]) && isNumOrSym(c), x)));
    addParameter(Parser, 'COM'    , [], @(x) isempty(x) || size(x, 2) == 3);
    addParameter(Parser, 'DH'     , [], @(s) isempty(s) || (isstruct(s) && ...
        all(isfield(s, {'alpha', 'a', 'd', 'theta'}))));
    parse(Parser, varargin{:});
    R = Parser.Results;

    % Populate missing values from a URDF.
    if ~isempty(R.URDF)
        URDFPath = char(R.URDF);
        if ~IsSerialManipulator(URDFPath)
            error('DynStruct:NotSerialManipulator', ...
                'The URDF must describe one serial chain of movable joints.');
        end

        Extracted = extractURDFParameters(URDFPath);
        ExtractedFields = {'Mass', 'Length', 'Radius', 'Fv', 'Fc', ...
            'Inertia', 'COM', 'DH'};
        for k = 1:numel(ExtractedFields)
            Field = ExtractedFields{k};
            if isempty(R.(Field))
                R.(Field) = Extracted.(Field);
            end
        end
    end

    % Parameter List Length Validation
    required = {'Mass', 'Inertia', 'DH', 'COM'};
    for k = 1:numel(required)
        if isempty(R.(required{k}))
            error('RobotStruct:MissingField', 'Missing required input ''%s''.', required{k});
        end
    end

    nLinks = numel(R.Mass);
    if numel(R.Inertia) ~= nLinks
        error('RobotStruct:SizeMismatch', 'Inertia cell array must have %d elements.', nLinks);
    end
    if any([numel(R.DH.alpha), numel(R.DH.a), numel(R.DH.d), numel(R.DH.theta)] ~= nLinks)
        error('RobotStruct:SizeMismatch', 'DH lists must all have %d elements.', nLinks);
    end

    optVecs = {'Length', 'Radius', 'Fv', 'Fc'};
    for k = 1:numel(optVecs)
        Field = optVecs{k};
        if ~isempty(R.(Field)) && numel(R.(Field)) ~= nLinks
            error('RobotStruct:SizeMismatch', '%s must have %d elements.', Field, nLinks);
        end
    end

    % Default zero friction if not provided
    if isempty(R.Fv)
        R.Fv = zeros(1, nLinks);
    end
    if isempty(R.Fc)
        R.Fc = zeros(1, nLinks);
    end

    % Verify COM Size (n-by-3)
    if size(R.COM, 1) ~= nLinks
        error('RobotStruct:SizeMismatch', 'COM must have %d rows.', nLinks);
    end

    % Build Struct
    DynPar = orderfields(struct( ...
        'Mass'   , R.Mass(:).', ...
        'Length' , R.Length(:).', ...
        'Radius' , R.Radius(:).', ...
        'Fv'     , R.Fv(:).', ...
        'Fc'     , R.Fc(:).', ...
        'Inertia', {R.Inertia}, ...
        'COM'    , R.COM, ...
        'DH'     , R.DH));
end


function Parameters = extractURDFParameters(URDFPath)
    % Read topology and choose the deepest fixed tip on the serial chain.
    Robot = URDFRead(URDFPath);
    [Chain, TipIndex] = serialChain(Robot);
    JointTypes = {Robot.Joints(Chain).Type};
    ActiveMask = ~strcmp(JointTypes, 'fixed');
    ActiveJoints = Chain(ActiveMask);
    NumActive = numel(ActiveJoints);

    [ZeroTransforms, Children] = zeroConfiguration(Robot);
    Frames = generateDHFrames(Robot, ActiveJoints, TipIndex, ZeroTransforms);
    [Alpha, A, D, Theta] = parametersFromFrames(Frames);

    Types = repmat('r', 1, NumActive);
    for k = 1:NumActive
        if strcmp(Robot.Joints(ActiveJoints(k)).Type, 'prismatic')
            Types(k) = 'p';
        end
    end

    Groups = rigidBodyGroups(Robot, ActiveJoints, Children);
    Mass = zeros(1, NumActive);
    Length = zeros(1, NumActive);
    Radius = zeros(1, NumActive);
    COM = zeros(NumActive, 3);
    Inertia = cell(1, NumActive);

    for k = 1:NumActive
        [Mass(k), COM(k, :), Inertia{k}] = aggregateInertial( ...
            Robot, Groups{k}, ZeroTransforms, Frames{k + 1}, k);
        [Length(k), Radius(k)] = bodyDimensions( ...
            Robot, Groups{k}, Mass(k), Inertia{k}, Frames{k}, Frames{k + 1});
    end

    Fv = [Robot.Joints(ActiveJoints).Damping];
    Fc = [Robot.Joints(ActiveJoints).Friction];
    DH = DHStruct('alpha', Alpha, 'a', A, 'd', D, ...
        'theta', Theta, 'type', Types, 'notation', 'original');

    Parameters = struct( ...
        'Mass'   , Mass, ...
        'Length' , Length, ...
        'Radius' , Radius, ...
        'Fv'     , Fv, ...
        'Fc'     , Fc, ...
        'Inertia', {Inertia}, ...
        'COM'    , COM, ...
        'DH'     , DH);
end


function [Chain, TipIndex] = serialChain(Robot)
    LinkNames = {Robot.Links.Name};
    ParentJoint = zeros(1, numel(Robot.Links));
    HasChildren = false(1, numel(Robot.Links));

    for k = 1:numel(Robot.Joints)
        ParentIndex = find(strcmp(LinkNames, Robot.Joints(k).Parent), 1);
        ChildIndex = find(strcmp(LinkNames, Robot.Joints(k).Child), 1);
        ParentJoint(ChildIndex) = k;
        HasChildren(ParentIndex) = true;
    end

    BaseIndex = find(strcmp(LinkNames, Robot.BaseName), 1);
    LeafIndices = find(~HasChildren);
    MovableTypes = {'revolute', 'continuous', 'prismatic'};
    BestScore = [-1, -1];
    Chain = [];
    TipIndex = [];

    for k = 1:numel(LeafIndices)
        CandidateTip = LeafIndices(k);
        CandidateChain = [];
        LinkIndex = CandidateTip;

        while LinkIndex ~= BaseIndex
            JointIndex = ParentJoint(LinkIndex);
            if JointIndex == 0
                CandidateChain = [];
                break;
            end
            CandidateChain(end + 1) = JointIndex;
            LinkIndex = find(strcmp(LinkNames, Robot.Joints(JointIndex).Parent), 1);
        end

        CandidateChain = fliplr(CandidateChain);
        if isempty(CandidateChain)
            continue;
        end
        NumMovable = nnz(ismember({Robot.Joints(CandidateChain).Type}, MovableTypes));
        Score = [NumMovable, numel(CandidateChain)];
        if Score(1) > BestScore(1) || ...
                (Score(1) == BestScore(1) && Score(2) > BestScore(2))
            BestScore = Score;
            Chain = CandidateChain;
            TipIndex = CandidateTip;
        end
    end
end


function [Transforms, Children] = zeroConfiguration(Robot)
    LinkNames = {Robot.Links.Name};
    NumLinks = numel(Robot.Links);
    Children = cell(1, NumLinks);

    for k = 1:numel(Robot.Joints)
        ParentIndex = find(strcmp(LinkNames, Robot.Joints(k).Parent), 1);
        Children{ParentIndex}(end + 1) = k;
    end

    BaseIndex = find(strcmp(LinkNames, Robot.BaseName), 1);
    Transforms = cell(1, NumLinks);
    Transforms{BaseIndex} = eye(4);
    Queue = BaseIndex;

    while ~isempty(Queue)
        ParentIndex = Queue(1);
        Queue(1) = [];
        ChildJoints = Children{ParentIndex};

        for k = 1:numel(ChildJoints)
            JointIndex = ChildJoints(k);
            ChildIndex = find(strcmp(LinkNames, Robot.Joints(JointIndex).Child), 1);
            Transforms{ChildIndex} = Transforms{ParentIndex} * ...
                Robot.Joints(JointIndex).Origin;
            Queue(end + 1) = ChildIndex;
        end
    end
end


function Frames = generateDHFrames(Robot, ActiveJoints, TipIndex, Transforms)
    NumActive = numel(ActiveJoints);
    LinkNames = {Robot.Links.Name};
    Points = zeros(3, NumActive + 1);
    Axes = zeros(3, NumActive + 1);

    for k = 1:NumActive
        Joint = Robot.Joints(ActiveJoints(k));
        ChildIndex = find(strcmp(LinkNames, Joint.Child), 1);
        ChildTransform = Transforms{ChildIndex};
        Points(:, k) = ChildTransform(1:3, 4);
        Axes(:, k) = normalizeVector(ChildTransform(1:3, 1:3) * Joint.Axis(:));
    end

    TipTransform = Transforms{TipIndex};
    Points(:, end) = TipTransform(1:3, 4);
    Axes(:, end) = normalizeVector(TipTransform(1:3, 3));

    Frames = cell(1, NumActive + 1);
    XHint = [];
    for k = 1:NumActive
        [~, Point2, XAxis] = commonNormal( ...
            Points(:, k), Axes(:, k), Points(:, k + 1), Axes(:, k + 1), XHint);

        if k == 1
            Origin = Points(:, 1) - Axes(:, 1) * (Axes(:, 1).' * Points(:, 1));
            Frames{1} = makeTransform(Origin, makeFrame(Axes(:, 1), XAxis));
        end
        if k == NumActive
            Point2 = Points(:, end);
        end

        Frames{k + 1} = makeTransform(Point2, makeFrame(Axes(:, k + 1), XAxis));
        XHint = Frames{k + 1}(1:3, 1);
    end
end


function [Alpha, A, D, Theta] = parametersFromFrames(Frames)
    NumLinks = numel(Frames) - 1;
    Alpha = zeros(1, NumLinks);
    A = zeros(1, NumLinks);
    D = zeros(1, NumLinks);
    Theta = zeros(1, NumLinks);
    Tolerance = 1e-10;

    for k = 1:NumLinks
        Relative = rigidInverse(Frames{k}) * Frames{k + 1};
        ThetaK = atan2(Relative(2, 1), Relative(1, 1));
        AlphaK = atan2(Relative(3, 2), Relative(3, 3));
        AK = Relative(1, 4) * cos(ThetaK) + Relative(2, 4) * sin(ThetaK);
        DK = Relative(3, 4);
        ThetaK = mod(ThetaK + pi, 2 * pi) - pi;

        if abs(ThetaK + pi) < Tolerance
            ThetaK = pi;
        end
        if abs(AlphaK) < Tolerance
            AlphaK = 0;
        end
        if abs(AK) < Tolerance
            AK = 0;
        end
        if abs(DK) < Tolerance
            DK = 0;
        end
        if abs(ThetaK) < Tolerance
            ThetaK = 0;
        end

        Alpha(k) = AlphaK;
        A(k) = AK;
        D(k) = DK;
        Theta(k) = ThetaK;
    end
end


function Groups = rigidBodyGroups(Robot, ActiveJoints, Children)
    LinkNames = {Robot.Links.Name};
    Groups = cell(1, numel(ActiveJoints));

    for k = 1:numel(ActiveJoints)
        StartIndex = find(strcmp(LinkNames, Robot.Joints(ActiveJoints(k)).Child), 1);
        Stack = StartIndex;
        Group = [];

        while ~isempty(Stack)
            LinkIndex = Stack(end);
            Stack(end) = [];
            Group(end + 1) = LinkIndex;
            ChildJoints = Children{LinkIndex};

            for j = 1:numel(ChildJoints)
                JointIndex = ChildJoints(j);
                if strcmp(Robot.Joints(JointIndex).Type, 'fixed')
                    ChildIndex = find(strcmp(LinkNames, Robot.Joints(JointIndex).Child), 1);
                    Stack(end + 1) = ChildIndex;
                end
            end
        end
        Groups{k} = Group;
    end
end


function [Mass, COM, Inertia] = aggregateInertial( ...
        Robot, Group, Transforms, DHFrame, BodyIndex)
    Entries = {};
    Mass = 0;

    for k = 1:numel(Group)
        Link = Robot.Links(Group(k));
        if isempty(Link.Inertial)
            continue;
        end

        InertialTransform = Transforms{Group(k)} * Link.Inertial.Origin;
        Entry = struct( ...
            'Mass'   , Link.Inertial.Mass, ...
            'COM'    , InertialTransform(1:3, 4), ...
            'Inertia', InertialTransform(1:3, 1:3) * Link.Inertial.Inertia * ...
            InertialTransform(1:3, 1:3).');
        Entries{end + 1} = Entry;
        Mass = Mass + Entry.Mass;
    end

    if isempty(Entries) || ~isfinite(Mass) || Mass <= 0
        error('DynStruct:MissingInertial', ...
            'Moving body %d has no positive, usable URDF inertial mass.', BodyIndex);
    end

    COMBase = zeros(3, 1);
    for k = 1:numel(Entries)
        COMBase = COMBase + Entries{k}.Mass * Entries{k}.COM;
    end
    COMBase = COMBase / Mass;

    InertiaBase = zeros(3);
    for k = 1:numel(Entries)
        Offset = Entries{k}.COM - COMBase;
        InertiaBase = InertiaBase + Entries{k}.Inertia + ...
            Entries{k}.Mass * parallelAxis(Offset);
    end

    Rotation = DHFrame(1:3, 1:3);
    COM = (Rotation.' * (COMBase - DHFrame(1:3, 4))).';
    Inertia = Rotation.' * InertiaBase * Rotation;
    Inertia = 0.5 * (Inertia + Inertia.');
end


function [Length, Radius] = bodyDimensions( ...
        Robot, Group, Mass, Inertia, PreviousFrame, CurrentFrame)
    Candidates = zeros(0, 2);

    for k = 1:numel(Group)
        Link = Robot.Links(Group(k));
        Containers = Link.Collisions;
        if isempty(Containers)
            Containers = Link.Visuals;
        end

        for j = 1:numel(Containers)
            Geometry = Containers(j).Geometry;
            switch Geometry.Type
                case 'cylinder'
                    Candidates(end + 1, :) = [Geometry.Length, Geometry.Radius];
                case 'sphere'
                    Candidates(end + 1, :) = [2 * Geometry.Radius, Geometry.Radius];
                case 'box'
                    Size = sort(Geometry.Size);
                    Candidates(end + 1, :) = [Size(3), 0.5 * hypot(Size(1), Size(2))];
            end
        end
    end

    if ~isempty(Candidates)
        Candidates = sortrows(Candidates, [1, 2]);
        Length = Candidates(end, 1);
        Radius = Candidates(end, 2);
        return;
    end

    Length = norm(CurrentFrame(1:3, 4) - PreviousFrame(1:3, 4));
    Eigenvalues = eig(0.5 * (Inertia + Inertia.'));
    Radius = sqrt(max(0, 2 * min(Eigenvalues) / Mass));
end


function [Point1, Point2, XAxis] = commonNormal(PointP, AxisU, PointQ, AxisV, XHint)
    AxisU = normalizeVector(AxisU);
    AxisV = normalizeVector(AxisV);
    CrossAxes = cross(AxisU, AxisV);

    if norm(CrossAxes) > 1e-9
        Coefficients = [AxisU, -AxisV] \ (PointQ - PointP);
        Point1 = PointP + Coefficients(1) * AxisU;
        Point2 = PointQ + Coefficients(2) * AxisV;
        Delta = Point2 - Point1;
        if norm(Delta) > 1e-9
            XAxis = normalizeVector(Delta);
        else
            XAxis = normalizeVector(CrossAxes);
        end
    else
        Point1 = PointP + AxisU * (AxisU.' * (PointQ - PointP));
        Point2 = PointQ;
        Delta = Point2 - Point1;
        if norm(Delta) > 1e-9
            XAxis = normalizeVector(Delta);
        else
            Frame = makeFrame(AxisU, XHint);
            XAxis = Frame(:, 1);
        end
    end

    X1 = XAxis - AxisU * (AxisU.' * XAxis);
    X2 = XAxis - AxisV * (AxisV.' * XAxis);
    if norm(X1) > 1e-9
        X1 = normalizeVector(X1);
    else
        Frame = makeFrame(AxisU, XHint);
        X1 = Frame(:, 1);
    end
    if norm(X2) > 1e-9
        X2 = normalizeVector(X2);
    else
        Frame = makeFrame(AxisV, X1);
        X2 = Frame(:, 1);
    end
    if X1.' * X2 < 0
        X2 = -X2;
    end

    XSum = X1 + X2;
    if norm(XSum) > 1e-9
        XAxis = normalizeVector(XSum);
    else
        XAxis = X1;
    end
end


function Rotation = makeFrame(ZAxis, XHint)
    ZAxis = normalizeVector(ZAxis);
    Candidates = {};
    if ~isempty(XHint)
        Candidates{end + 1} = XHint(:);
    end
    Candidates = [Candidates, {[1; 0; 0], [0; 1; 0], [0; 0; 1]}];

    for k = 1:numel(Candidates)
        XAxis = Candidates{k} - ZAxis * (ZAxis.' * Candidates{k});
        if norm(XAxis) > 1e-10
            XAxis = normalizeVector(XAxis);
            YAxis = normalizeVector(cross(ZAxis, XAxis));
            XAxis = normalizeVector(cross(YAxis, ZAxis));
            Rotation = [XAxis, YAxis, ZAxis];
            return;
        end
    end

    error('DynStruct:FrameConstruction', 'Unable to construct an orthonormal DH frame.');
end


function Transform = makeTransform(XYZ, Rotation)
    Transform = eye(4);
    Transform(1:3, 1:3) = Rotation;
    Transform(1:3, 4) = XYZ(:);
end


function Inverse = rigidInverse(Transform)
    Inverse = eye(4);
    Inverse(1:3, 1:3) = Transform(1:3, 1:3).';
    Inverse(1:3, 4) = -Transform(1:3, 1:3).' * Transform(1:3, 4);
end


function Matrix = parallelAxis(Offset)
    Matrix = (Offset.' * Offset) * eye(3) - Offset * Offset.';
end


function Vector = normalizeVector(Vector)
    Magnitude = norm(Vector);
    if Magnitude < 1e-10
        error('DynStruct:ZeroAxis', 'A URDF joint or tool axis has zero magnitude.');
    end
    Vector = Vector / Magnitude;
end
