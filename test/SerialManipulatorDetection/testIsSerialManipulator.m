function Tests = testIsSerialManipulator
    %TESTISSERIALMANIPULATOR Tests URDF reading and serial-chain detection.
    Tests = functiontests(localfunctions);
end


function setupOnce(TestCase)
    TestFolder = fileparts(mfilename('fullpath'));
    ProjectRoot = fileparts(fileparts(TestFolder));
    addpath(genpath(fullfile(ProjectRoot, 'utils')));
    TestCase.TestData.URDFFolder = fullfile(TestFolder, 'urdfs');
end


function testAllCopiedRobotURDFs(TestCase)
    Expectations = classificationExpectations();
    Fixtures = dir(fullfile(TestCase.TestData.URDFFolder, '**', '*.urdf'));
    Discovered = cell(1, numel(Fixtures));

    for k = 1:numel(Fixtures)
        FullPath = fullfile(Fixtures(k).folder, Fixtures(k).name);
        RelativePath = erase(FullPath, [TestCase.TestData.URDFFolder, filesep]);
        Discovered{k} = strrep(RelativePath, filesep, '/');
    end

    ExpectedFiles = Expectations(:, 1).';
    TestCase.verifyEqual(sort(Discovered), sort(ExpectedFiles), ...
        'Every copied URDF fixture must have an explicit expected result.');

    for k = 1:size(Expectations, 1)
        RelativePath = Expectations{k, 1};
        ExpectedStatus = Expectations{k, 2};
        URDFPath = fullfile(TestCase.TestData.URDFFolder, RelativePath);
        TestCase.verifyEqual(IsSerialManipulator(URDFPath), ExpectedStatus, ...
            sprintf('Unexpected classification for %s.', RelativePath));
    end
end


function testURDFReadTreeStructure(TestCase)
    URDFPath = fullfile(TestCase.TestData.URDFFolder, 'serial', 'ur10.urdf');
    Robot = URDFRead(URDFPath);

    TestCase.verifyEqual(Robot.Name, 'ur10_manidyn_roundtrip');
    TestCase.verifyEqual(Robot.BaseName, 'base_link');
    TestCase.verifyEqual(Robot.NumBodies, 13);
    TestCase.verifyEqual(Robot.NumNonFixedJoints, 6);
    TestCase.verifyEqual(numel(Robot.Links), 14);
    TestCase.verifyEqual(numel(Robot.Joints), 13);
    TestCase.verifyEqual(numel(Robot.Bodies), Robot.NumBodies);
    TestCase.verifyTrue(ismember('tool0', Robot.BodyNames));

    LinkIndex = find(strcmp({Robot.Links.Name}, 'link_1'), 1);
    TestCase.verifyEqual(Robot.Links(LinkIndex).Parent, 'base_link');
    TestCase.verifyEqual(Robot.Links(LinkIndex).JointName, 'joint_1');
    TestCase.verifyEqual(Robot.Links(LinkIndex).Joint.Type, 'revolute');
    TestCase.verifyEqual(Robot.Links(LinkIndex).Inertial.Mass, 7.1, 'AbsTol', 1e-12);

    JointIndex = find(strcmp({Robot.Joints.Name}, 'joint_1'), 1);
    TestCase.verifyEqual(Robot.Joints(JointIndex).Type, 'revolute');
    TestCase.verifyEqual(Robot.Joints(JointIndex).Axis, [0, 0, 1]);
    TestCase.verifyEqual(Robot.Joints(JointIndex).Origin, eye(4), 'AbsTol', 1e-12);
    TestCase.verifyEqual(Robot.Joints(JointIndex).PositionLimits, [-pi, pi], ...
        'AbsTol', 1e-12);
    TestCase.verifyEqual(Robot.Joints(JointIndex).EffortLimit, 1000);
    TestCase.verifyEqual(Robot.Joints(JointIndex).VelocityLimit, 10);
end


function testMissingURDF(TestCase)
    MissingPath = fullfile(TestCase.TestData.URDFFolder, 'does_not_exist.urdf');
    TestCase.verifyError(@() URDFRead(MissingPath), 'URDFRead:FileNotFound');
end


function Expectations = classificationExpectations()
    Expectations = { ...
        'serial/kinova_gen3.urdf', 1; ...
        'serial/scara.urdf', 1; ...
        'serial/ur10.urdf', 1; ...
        'serial/franka_panda.urdf', 1; ...
        'serial/kuka_iiwa14.urdf', 1; ...
        'serial/puma560.urdf', 1; ...
        'parallel/delta_robot.urdf', 0; ...
        'parallel/stewart_platform.urdf', 0; ...
        'parallel/five_bar_planar.urdf', 0; ...
        'parallel/planar_3rrr.urdf', 0; ...
        'parallel/parallel_gripper.urdf', 0; ...
        'parallel/pantograph_parallel.urdf', 0; ...
        'humanoid/atlas_humanoid.urdf', 0; ...
        'humanoid/nao_humanoid.urdf', 0; ...
        'quadruped/unitree_go1.urdf', 0; ...
        'quadruped/anymal_c.urdf', 0; ...
        'aerial/crazyflie_quadcopter.urdf', 0; ...
        'mobile/turtlebot3_burger.urdf', 0; ...
        'mobile/husky_mobile_base.urdf', 0; ...
        'mobile_manipulator/fetch_mobile_manipulator.urdf', 0; ...
        'mobile_manipulator/tiago_mobile_manipulator.urdf', 0};
end
