function [ jointPos ] = sim_getUR5JointPositions( vrep, clientID, jointHandles )
%sim_getUR5JointPositions Get UR5's joint positions from the V-REP scene.
% vrep: the vrep object
% clientID: the MATLAB client ID
% jointHandles: the handles of every UR5's joint
% jointPos: the current joint positions

jointPos = zeros(1,6);
for i = 1:6
    [res, jointPos(i)] = vrep.simxGetJointPosition(clientID, jointHandles(i), vrep.simx_opmode_blocking);
    vrchk(vrep,res);
end
