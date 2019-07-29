function [ configuration ] = setConfiguration_UR5( robot, theta )
% Set the joint angles of UR5 and return its configuration
% robot: RigidBodyTree object
% theta: the vector including angles of joint 1 to 6
% configuration: robot configuration, specified as a structure aray with joint names and positions for all the bodies in the robot model
% You can generate a configuration using homeConfiguration(robot)

for i=1:6
    robot.Bodies{1,i}.Joint.HomePosition = theta(i);
end
configuration = homeConfiguration(robot);

end

