function [ obj ] = getUR5Handles( obj )
%getUR5Handles Get UR5's handls from the vrep scene.
% Haopeng Hu
% 2018.07.05

% @ICECUBE

% handles.ur5Joints: handles of joints of UR5
% handles.ur5Ref: handle of UR5
% handles.ur5ikTip: handle of UR5' ikTip
% handles.ur5connector: handle of UR5's connector ( ForceTorque sensor)

%% Get the UR5's joints' handles
jointNames = {'UR5_joint1','UR5_joint2','UR5_joint3',...
    'UR5_joint4','UR5_joint5','UR5_joint6'};
ur5Joints = zeros(1,6);
for i = 1:6     
    [res, ur5Joints(i)] = obj.vrep.simxGetObjectHandle(obj.clientID, ...
        jointNames{i},obj.vrep.simx_opmode_blocking);   % blocking mode
    obj.vrchk(res);
end
obj.handles.ur5Joints = ur5Joints;
%% Get the UR5's handle
[res, ur5Ref] = obj.vrep.simxGetObjectHandle(obj.clientID,...
    'UR5',obj.vrep.simx_opmode_blocking);
obj.vrchk(res);
obj.handles.ur5Ref = ur5Ref;
%% Get the UR5's ikTip's handle
[res, ur5ikTip] = obj.vrep.simxGetObjectHandle(obj.clientID,'UR5_ikTip',obj.vrep.simx_opmode_blocking);
obj.vrchk(res);
obj.handles.ur5ikTip = ur5ikTip;
%% Get the UR5's connector's handle
[res,ur5connector] = obj.vrep.simxGetObjectHandle(obj.clientID,'UR5_connection',obj.vrep.simx_opmode_blocking);
obj.vrchk(res);
obj.handles.ur5connector = ur5connector;

clear jointNames ur5Joints ur5Ref ur5ikTip ur5connector res

end

