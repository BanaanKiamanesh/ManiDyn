function Robot = URDFRead(URDFPath)
    %URDFREAD Reads a Unified Robot Description Format (URDF) file.
    %   ROBOT = URDFREAD(URDFPATH) parses the specified URDF file and returns
    %   a tree-like structure containing the robot links, joints, inertial
    %   properties, primitive geometry, and parent-child relationships.
    %
    %   Input Arguments:
    %       URDFPath - Path to an expanded .urdf XML file. Xacro processing is
    %                  not performed.
    %
    %   Output Arguments:
    %       Robot - A structure containing the following fields:
    %                .Name               - Robot name from the <robot> element.
    %                .Source             - Absolute path to the URDF file.
    %                .BaseName           - Root-link name, or empty when the
    %                                      URDF does not have one unique root.
    %                .BodyNames          - Non-base link names.
    %                .NumBodies          - Number of non-base links.
    %                .NumNonFixedJoints  - Number of movable joints.
    %                .Base               - Root-link structure.
    %                .Bodies             - Cell array of non-base link structures.
    %                .Links              - Structure array of all links.
    %                .Joints             - Structure array of all joints.
    %
    %   Each link contains its parent, children, connecting joint, inertial
    %   data, visuals, and collisions. Each joint contains its type, parent and
    %   child links, origin transform, axis, limits, damping, and friction.
    %
    %   Example:
    %       robot = URDFRead('ur10.urdf');
    %       disp(robot.BaseName);
    %       disp(robot.BodyNames);
    %
    %   Throws:
    %       URDFRead:InvalidPath - If URDFPath is not scalar text.
    %       URDFRead:FileNotFound - If the requested file does not exist.
    %       URDFRead:InvalidURDF - If the file is not a readable URDF document.
    %
    %   See also: IsSerialManipulator, DynStruct.


    if ~(ischar(URDFPath) && isrow(URDFPath)) && ...
            ~(isstring(URDFPath) && isscalar(URDFPath))
        error('URDFRead:InvalidPath', 'URDFPath must be a character vector or string scalar.');
    end

    URDFPath = char(URDFPath);
    if ~isfile(URDFPath)
        error('URDFRead:FileNotFound', 'URDF file not found: %s', URDFPath);
    end

    try
        SourceFile = javaObject('java.io.File', URDFPath);
        SourcePath = char(SourceFile.getCanonicalPath());
        Document = xmlread(SourcePath);
    catch ME
        error('URDFRead:InvalidURDF', 'Unable to read URDF file "%s": %s', ...
            URDFPath, ME.message);
    end

    Root = Document.getDocumentElement();
    if isempty(Root) || ~strcmpi(char(Root.getTagName()), 'robot')
        error('URDFRead:InvalidURDF', ...
            'The root XML element in "%s" must be <robot>.', URDFPath);
    end

    RobotName = getAttribute(Root, 'name', '');
    if isempty(RobotName)
        [~, RobotName] = fileparts(SourcePath);
    end

    LinkTemplate = struct( ...
        'Name'      , '', ...
        'Parent'    , '', ...
        'Children'  , {{}}, ...
        'JointName' , '', ...
        'Joint'     , [], ...
        'Inertial'  , [], ...
        'Visuals'   , [], ...
        'Collisions', []);
    JointTemplate = struct( ...
        'Name'          , '', ...
        'Type'          , '', ...
        'Parent'        , '', ...
        'Child'         , '', ...
        'OriginXYZ'     , zeros(1, 3), ...
        'OriginRPY'     , zeros(1, 3), ...
        'Origin'        , eye(4), ...
        'Axis'          , [1, 0, 0], ...
        'PositionLimits', [NaN, NaN], ...
        'VelocityLimit' , NaN, ...
        'EffortLimit'   , NaN, ...
        'Damping'       , 0, ...
        'Friction'      , 0);

    Links = repmat(LinkTemplate, 1, 0);
    Joints = repmat(JointTemplate, 1, 0);
    Elements = childElements(Root);

    for k = 1:numel(Elements)
        Element = Elements{k};
        Tag = lower(char(Element.getTagName()));

        switch Tag
            case 'link'
                Link = LinkTemplate;
                Link.Name = requireAttribute(Element, 'name', 'link');
                Link.Inertial = readInertial(Element);
                Link.Visuals = readGeometryContainers(Element, 'visual');
                Link.Collisions = readGeometryContainers(Element, 'collision');
                Links(end + 1) = Link;

            case 'joint'
                Joint = JointTemplate;
                Joint.Name = requireAttribute(Element, 'name', 'joint');
                Joint.Type = lower(requireAttribute(Element, 'type', 'joint'));

                ParentNode = firstChild(Element, 'parent');
                ChildNode = firstChild(Element, 'child');
                if isempty(ParentNode) || isempty(ChildNode)
                    error('URDFRead:InvalidURDF', ...
                        'Joint "%s" must contain <parent> and <child> elements.', Joint.Name);
                end
                Joint.Parent = requireAttribute(ParentNode, 'link', 'parent');
                Joint.Child = requireAttribute(ChildNode, 'link', 'child');

                [Joint.OriginXYZ, Joint.OriginRPY, Joint.Origin] = readOrigin(Element);

                AxisNode = firstChild(Element, 'axis');
                if ~isempty(AxisNode)
                    Joint.Axis = readVector(getAttribute(AxisNode, 'xyz', ''), ...
                        3, [1, 0, 0], sprintf('axis of joint "%s"', Joint.Name));
                end

                LimitNode = firstChild(Element, 'limit');
                if ~isempty(LimitNode)
                    Joint.PositionLimits = [ ...
                        readScalarAttribute(LimitNode, 'lower', NaN), ...
                        readScalarAttribute(LimitNode, 'upper', NaN)];
                    Joint.VelocityLimit = readScalarAttribute(LimitNode, 'velocity', NaN);
                    Joint.EffortLimit = readScalarAttribute(LimitNode, 'effort', NaN);
                end
                if strcmp(Joint.Type, 'continuous')
                    Joint.PositionLimits = [-Inf, Inf];
                end

                DynamicsNode = firstChild(Element, 'dynamics');
                if ~isempty(DynamicsNode)
                    Joint.Damping = readScalarAttribute(DynamicsNode, 'damping', 0);
                    Joint.Friction = readScalarAttribute(DynamicsNode, 'friction', 0);
                end
                Joints(end + 1) = Joint;
        end
    end

    if isempty(Links)
        error('URDFRead:InvalidURDF', 'URDF file "%s" contains no links.', URDFPath);
    end

    LinkNames = {Links.Name};
    JointNames = {Joints.Name};
    if numel(unique(LinkNames)) ~= numel(LinkNames)
        error('URDFRead:InvalidURDF', 'URDF link names must be unique.');
    end
    if numel(unique(JointNames)) ~= numel(JointNames)
        error('URDFRead:InvalidURDF', 'URDF joint names must be unique.');
    end

    for k = 1:numel(Joints)
        ParentIndex = find(strcmp(LinkNames, Joints(k).Parent), 1);
        ChildIndex = find(strcmp(LinkNames, Joints(k).Child), 1);
        if isempty(ParentIndex) || isempty(ChildIndex)
            continue;
        end

        Links(ParentIndex).Children{end + 1} = Joints(k).Child;
        if isempty(Links(ChildIndex).Parent)
            Links(ChildIndex).Parent = Joints(k).Parent;
            Links(ChildIndex).JointName = Joints(k).Name;
            Links(ChildIndex).Joint = Joints(k);
        end
    end

    ReferencedChildren = {Joints.Child};
    RootMask = ~ismember(LinkNames, ReferencedChildren);
    if nnz(RootMask) == 1
        BaseIndex = find(RootMask, 1);
        BaseName = Links(BaseIndex).Name;
        Base = Links(BaseIndex);
        BodyMask = true(1, numel(Links));
        BodyMask(BaseIndex) = false;
    else
        BaseName = '';
        Base = [];
        BodyMask = true(1, numel(Links));
    end

    BodyLinks = Links(BodyMask);
    Robot = struct;
    Robot.Name = RobotName;
    Robot.Source = SourcePath;
    Robot.BaseName = BaseName;
    Robot.BodyNames = {BodyLinks.Name};
    Robot.NumBodies = numel(BodyLinks);
    Robot.NumNonFixedJoints = nnz(~strcmp({Joints.Type}, 'fixed'));
    Robot.Base = Base;
    Robot.Bodies = num2cell(BodyLinks);
    Robot.Links = Links;
    Robot.Joints = Joints;
end


function Inertial = readInertial(LinkNode)
    InertialNode = firstChild(LinkNode, 'inertial');
    if isempty(InertialNode)
        Inertial = [];
        return;
    end

    MassNode = firstChild(InertialNode, 'mass');
    InertiaNode = firstChild(InertialNode, 'inertia');
    if isempty(MassNode) || isempty(InertiaNode)
        error('URDFRead:InvalidURDF', ...
            'Every <inertial> element must contain <mass> and <inertia>.');
    end

    [XYZ, RPY, Origin] = readOrigin(InertialNode);
    Ixx = readScalarAttribute(InertiaNode, 'ixx', 0);
    Ixy = readScalarAttribute(InertiaNode, 'ixy', 0);
    Ixz = readScalarAttribute(InertiaNode, 'ixz', 0);
    Iyy = readScalarAttribute(InertiaNode, 'iyy', 0);
    Iyz = readScalarAttribute(InertiaNode, 'iyz', 0);
    Izz = readScalarAttribute(InertiaNode, 'izz', 0);

    Inertial = struct( ...
        'Mass'   , readScalarAttribute(MassNode, 'value', NaN), ...
        'XYZ'    , XYZ, ...
        'RPY'    , RPY, ...
        'Origin' , Origin, ...
        'Inertia', [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz]);
end


function Containers = readGeometryContainers(LinkNode, ContainerTag)
    ContainerTemplate = struct( ...
        'Name'    , '', ...
        'OriginXYZ', zeros(1, 3), ...
        'OriginRPY', zeros(1, 3), ...
        'Origin'  , eye(4), ...
        'Geometry', []);
    Containers = repmat(ContainerTemplate, 1, 0);
    Nodes = childElements(LinkNode, ContainerTag);

    for k = 1:numel(Nodes)
        GeometryNode = firstChild(Nodes{k}, 'geometry');
        if isempty(GeometryNode)
            continue;
        end

        Shapes = childElements(GeometryNode);
        if isempty(Shapes)
            continue;
        end

        Container = ContainerTemplate;
        Container.Name = getAttribute(Nodes{k}, 'name', '');
        [Container.OriginXYZ, Container.OriginRPY, Container.Origin] = ...
            readOrigin(Nodes{k});
        Container.Geometry = readGeometry(Shapes{1});
        Containers(end + 1) = Container;
    end
end


function Geometry = readGeometry(ShapeNode)
    Type = lower(char(ShapeNode.getTagName()));
    Geometry = struct( ...
        'Type'    , Type, ...
        'Size'    , [], ...
        'Radius'  , [], ...
        'Length'  , [], ...
        'Filename', '', ...
        'Scale'   , []);

    switch Type
        case 'box'
            Geometry.Size = readVector(getAttribute(ShapeNode, 'size', ''), ...
                3, [], 'box size');
        case 'cylinder'
            Geometry.Radius = readScalarAttribute(ShapeNode, 'radius', NaN);
            Geometry.Length = readScalarAttribute(ShapeNode, 'length', NaN);
        case 'sphere'
            Geometry.Radius = readScalarAttribute(ShapeNode, 'radius', NaN);
        case 'mesh'
            Geometry.Filename = getAttribute(ShapeNode, 'filename', '');
            Geometry.Scale = readVector(getAttribute(ShapeNode, 'scale', ''), ...
                3, [1, 1, 1], 'mesh scale');
    end
end


function [XYZ, RPY, Transform] = readOrigin(ParentNode)
    OriginNode = firstChild(ParentNode, 'origin');
    if isempty(OriginNode)
        XYZ = zeros(1, 3);
        RPY = zeros(1, 3);
    else
        XYZ = readVector(getAttribute(OriginNode, 'xyz', ''), ...
            3, zeros(1, 3), 'origin xyz');
        RPY = readVector(getAttribute(OriginNode, 'rpy', ''), ...
            3, zeros(1, 3), 'origin rpy');
    end

    Roll = RPY(1);
    Pitch = RPY(2);
    Yaw = RPY(3);
    Cr = cos(Roll);
    Sr = sin(Roll);
    Cp = cos(Pitch);
    Sp = sin(Pitch);
    Cy = cos(Yaw);
    Sy = sin(Yaw);
    Rotation = [ ...
        Cy * Cp, Cy * Sp * Sr - Sy * Cr, Cy * Sp * Cr + Sy * Sr; ...
        Sy * Cp, Sy * Sp * Sr + Cy * Cr, Sy * Sp * Cr - Cy * Sr; ...
        -Sp, Cp * Sr, Cp * Cr];

    Transform = eye(4);
    Transform(1:3, 1:3) = Rotation;
    Transform(1:3, 4) = XYZ(:);
end


function Elements = childElements(ParentNode, Tag)
    if nargin < 2
        Tag = '';
    end

    Elements = {};
    Nodes = ParentNode.getChildNodes();
    for k = 0:Nodes.getLength() - 1
        Node = Nodes.item(k);
        if Node.getNodeType() ~= 1
            continue;
        end
        if isempty(Tag) || strcmpi(char(Node.getTagName()), Tag)
            Elements{end + 1} = Node;
        end
    end
end


function Node = firstChild(ParentNode, Tag)
    Nodes = childElements(ParentNode, Tag);
    if isempty(Nodes)
        Node = [];
    else
        Node = Nodes{1};
    end
end


function Value = requireAttribute(Node, Name, ElementName)
    Value = getAttribute(Node, Name, '');
    if isempty(Value)
        error('URDFRead:InvalidURDF', ...
            'The <%s> element requires a "%s" attribute.', ElementName, Name);
    end
end


function Value = getAttribute(Node, Name, Default)
    if Node.hasAttribute(Name)
        Value = char(Node.getAttribute(Name));
    else
        Value = Default;
    end
end


function Value = readScalarAttribute(Node, Name, Default)
    Text = getAttribute(Node, Name, '');
    if isempty(Text)
        Value = Default;
        return;
    end

    Value = str2double(Text);
    if isnan(Value)
        error('URDFRead:InvalidURDF', ...
            'Attribute "%s" must contain a numeric scalar.', Name);
    end
end


function Value = readVector(Text, Count, Default, Description)
    if isempty(Text)
        Value = Default;
        return;
    end

    Value = sscanf(Text, '%f').';
    if numel(Value) ~= Count
        error('URDFRead:InvalidURDF', ...
            '%s must contain exactly %d numeric values.', Description, Count);
    end
end
