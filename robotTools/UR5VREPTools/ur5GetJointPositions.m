function [ jointPositions ] = ur5GetJointPositions( vrep, clientID, handles )
%ur5GetJointPositions Get UR5's joint positions from the V-REP scene
% vrep: the vrep object
% clientID: the MATLAB client ID
% handles: the handles of V-REP scene
% jointPositions: the current joint positions

jointPositions = zeros(1,6);
for i = 1:6
    [res, jointPositions(i)] = vrep.simxGetJointPosition(clientID, handles.ur5Joints(i), vrep.simx_opmode_blocking);
    vrchk(vrep,res);
end


end

