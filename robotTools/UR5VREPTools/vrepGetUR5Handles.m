% vrepGetUR5Handles
% A section of scripts to get UR5's handls from the vrep scene.
% Haopeng Hu
% 2018.06.25

% handles.ur5Joints: handles of joints of UR5
% handles.ur5Ref: handle of UR5
% handles.ur5ikTip: handle of UR5' ikTip
% handles.ur5connector: handle of UR5's connector ( ForceTorque sensor)

% Make sure you have run 'vrepTools\vrepRemApi_init.m' in advance.

%% Get the UR5's joints' handles
jointNames = {'UR5_joint1','UR5_joint2','UR5_joint3',...
    'UR5_joint4','UR5_joint5','UR5_joint6'};
ur5Joints = zeros(1,6);
for i = 1:6     
    [res, ur5Joints(i)] = vrep.simxGetObjectHandle(clientID, ...
        jointNames{i},vrep.simx_opmode_blocking);   % blocking mode
    vrchk(vrep,res);
end
handles.ur5Joints = ur5Joints;
%% Get the UR5's handle
[res, ur5Ref] = vrep.simxGetObjectHandle(clientID,...
    'UR5',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.ur5Ref = ur5Ref;
%% Get the UR5's ikTip's handle
[res, ur5ikTip] = vrep.simxGetObjectHandle(clientID,'UR5_ikTip',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.ur5ikTip = ur5ikTip;
%% Get the UR5's connector's handle
[res,ur5connector] = vrep.simxGetObjectHandle(clientID,'UR5_connection',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.ur5connector = ur5connector;

clear jointNames ur5Joints ur5Ref ur5ikTip ur5connector res