function [ IKTipPositions ] = ur5RMLJointMotions( vrep, clientID, targetPositionsSet,pickandplaceSign, handles )
%ur5RMLJointMotions Reflexes Motion Library - Joint space - Multi-targets
% vrep: the vrep object
% clientID: the MATLAB client ID
% targetPositionsSet: N x 6 matrix, the target joint positions
% pickandplaceSign: 1 for RG2 action applied, 0 for no RG2 action (default)
% handles: the handles of V-REP scence (not necessary)

% Haopeng Hu
% 2015.05.23

IKTipPositions = 0;
if nargin <5
    getIKTipPosSgin = 0;
    if nargin <4
        pickandplaceSign = 0;
    end
else
    getIKTipPosSgin = 1;
end

ur5RMLMoveToJointPositions(vrep,clientID,targetPositionsSet(1,1:6));
if pickandplaceSign == 1
    rg2Action(vrep,clientID,1);
end
if getIKTipPosSgin == 1
    IKTipPositions = zeros(size(targetPositionsSet,1),3);
    IKTipPositions(1,:) = ur5GetIKTipPosition(vrep,clientID,handles);
end

N = size(targetPositionsSet,1);
for i = 2:N
    ur5RMLMoveToJointPositions(vrep,clientID,targetPositionsSet(i,1:6));
    if getIKTipPosSgin == 1
        IKTipPositions(i,:) = ur5GetIKTipPosition(vrep,clientID,handles);
    end
end

if pickandplaceSign == 1
    rg2Action(vrep,clientID,0);
end

end

