function ForwardKinematics(q)

% Import unified-robot-data-format
robot = importrobot('abbIrb120.urdf');
robot.DataFormat = 'row';

% Define joint angles
q = [pi/2, -pi/2, 0, 0, 0, 0];

% Compute joint angles using C mex
Transform_mex = FK(q(1),q(2),q(3),q(4),q(5),q(6));

% Compute joint angles using MATLAB URDF model
Transform_urdf = getTransform(robot,q,'tool0');

% Compute numerical erro between two methods
fprintf('Error = %.15e\n\n', norm(Transform_mex - Transform_urdf));

% Display matrices
disp('Transformation matrix from C mex');
disp(Transform_mex);
disp('Transformation matrix from URDF usng FUN "getTransform"');
disp(Transform_urdf);

end