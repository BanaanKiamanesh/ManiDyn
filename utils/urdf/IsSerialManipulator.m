function Status = IsSerialManipulator(URDFPath)
    %ISSERIALMANIPULATOR Determines whether a URDF describes a serial manipulator.
    %   STATUS = ISSERIALMANIPULATOR(URDFPATH) reads the specified URDF and
    %   returns 1 when the complete robot is one open serial chain of movable
    %   joints. Otherwise, it returns 0.
    %
    %   Fixed joints and fixed sensor or tool branches are permitted. A URDF
    %   containing floating or planar joints, disconnected links, cycles,
    %   multiple parents, or movable branches is not classified as serial.
    %
    %   Input Arguments:
    %       URDFPath - Path to an expanded .urdf XML file.
    %
    %   Output Arguments:
    %       Status - 1 if the URDF is a serial manipulator; otherwise, 0.
    %
    %   Example:
    %       status = IsSerialManipulator('ur10.urdf');
    %       if status
    %           disp('The URDF describes a serial manipulator.');
    %       end
    %
    %   Throws:
    %       URDFRead:InvalidPath  - If URDFPath is not scalar text.
    %       URDFRead:FileNotFound - If the requested file does not exist.
    %       URDFRead:InvalidURDF  - If the file is not a readable URDF document.
    %
    %   See also: URDFRead, DynStruct.


    Robot = URDFRead(URDFPath);
    Links = Robot.Links;
    Joints = Robot.Joints;
    LinkNames = {Links.Name};
    JointTypes = lower({Joints.Type});

    Status = 0;
    AllowedTypes = {'revolute', 'continuous', 'prismatic', 'fixed'};
    if isempty(Joints) || ~all(ismember(JointTypes, AllowedTypes))
        return;
    end

    ParentIndices = zeros(1, numel(Joints));
    ChildIndices = zeros(1, numel(Joints));
    ParentCount = zeros(1, numel(Links));
    Children = cell(1, numel(Links));

    for k = 1:numel(Joints)
        ParentIndex = find(strcmp(LinkNames, Joints(k).Parent), 1);
        ChildIndex = find(strcmp(LinkNames, Joints(k).Child), 1);
        if isempty(ParentIndex) || isempty(ChildIndex)
            return;
        end
        ParentIndices(k) = ParentIndex;
        ChildIndices(k) = ChildIndex;

        ParentCount(ChildIndices(k)) = ParentCount(ChildIndices(k)) + 1;
        if ParentCount(ChildIndices(k)) > 1
            return;
        end
        Children{ParentIndices(k)}(end + 1) = k;
    end

    RootIndices = find(ParentCount == 0);
    if numel(RootIndices) ~= 1
        return;
    end

    Movable = ismember(JointTypes, {'revolute', 'continuous', 'prismatic'});
    NumMovable = nnz(Movable);
    if NumMovable == 0
        return;
    end
    MovingAxes = vertcat(Joints(Movable).Axis);
    if any(vecnorm(MovingAxes, 2, 2) < 1e-10)
        return;
    end

    Visited = false(1, numel(Links));
    LinkStack = RootIndices;
    MovingDepthStack = 0;
    LongestMovingChain = 0;

    while ~isempty(LinkStack)
        LinkIndex = LinkStack(end);
        LinkStack(end) = [];
        MovingDepth = MovingDepthStack(end);
        MovingDepthStack(end) = [];

        if Visited(LinkIndex)
            return;
        end
        Visited(LinkIndex) = true;
        LongestMovingChain = max(LongestMovingChain, MovingDepth);

        ChildJoints = Children{LinkIndex};
        for k = numel(ChildJoints):-1:1
            JointIndex = ChildJoints(k);
            LinkStack(end + 1) = ChildIndices(JointIndex);
            MovingDepthStack(end + 1) = MovingDepth + Movable(JointIndex);
        end
    end

    Status = double(all(Visited) && LongestMovingChain == NumMovable);
end
