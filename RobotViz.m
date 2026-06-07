
% Import unified-robot-data-format
robot = importrobot('abbIrb120.urdf');
robot.DataFormat = 'row';
showdetails(robot);

% Print exact fixed-transforms between links
for i = 1:numel(robot.Bodies)
    body = robot.Bodies{i};

    fprintf('\nBody: %s\n', body.Name);

    tform = body.Joint.JointToParentTransform;

    disp(body.Joint);
end

% Print URDF geometry transformation matrixes
for i = 1:numel(robot.Bodies)
    body = robot.Bodies{i};

    fprintf('\n%s\n', body.Name);

    disp(body.Joint.JointToParentTransform);
end

% Print joint-axis rotations
for i = 1:numel(robot.Bodies)

    body = robot.Bodies{i};

    fprintf('\n%s\n', body.Name);

    disp(body.Joint.JointAxis);

end

% Check for hidden-rotations around axis
for i = 1:numel(robot.Bodies)

    body = robot.Bodies{i};

    fprintf("\n%s\n", body.Name);

    disp(body.Joint.ChildToJointTransform);
end