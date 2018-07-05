function [ jointPositions ] = ur5GetJointPositions( icecube )
%ur5GetJointPositions Get UR5's joint positions from the V-REP scene
% icecube: the icecube object
% jointPositions: the current joint positions

jointPositions = zeros(1,6);
for i = 1:6
    [res, jointPositions(i)] = icecube.vrep.simxGetJointPosition(icecube.clientID, icecube.handles.ur5Joints(i), icecube.vrep.simx_opmode_blocking);
    icecube.vrchk(res);
end

%% ICECUBE 2.2
% % [ jointPositions ] = ur5GetJointPositions( vrep, clientID, handles )
% jointPositions = zeros(1,6);
% for i = 1:6
%     [res, jointPositions(i)] = vrep.simxGetJointPosition(clientID, handles.ur5Joints(i), vrep.simx_opmode_blocking);
%     vrchk(vrep,res);
% end


end

