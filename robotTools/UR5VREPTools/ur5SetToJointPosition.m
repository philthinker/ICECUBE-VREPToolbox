function [  ] = ur5SetToJointPosition( icecube, targetPositions )
%ur5SetToJointPosition Set the UR5 to targetPositions
% There is NO motion plan.
% This is a ONE-SHOT function, DO NOT get something right after running it.
% icecube: the ICECUBE object
% targetPositions: 1 x 6 vector, the target joint positions

icecube.vrep.simxPauseCommunication(icecube.clientID, 1);

for i = 1:6
    icecube.vrep.simxSetJointTargetPosition(icecube.clientID,icecube.handles.ur5Joints(i), targetPositions(i),icecube.vrep.simx_opmode_oneshot);
end

icecube.vrep.simxPauseCommunication(icecube.clientID, 0);

end

% Note that when we use setJointPosition function, it is recommended by the
% official manual to set the Joint Mode as 'Passive mode'. But until now,
% no error occurred if we did not do that.