% vrepGetUR5Handles
% A section of scripts to get UR5's handls from the vrep scene.
% Haopeng Hu
% 2018.04.17

% handles.rg2Ref: handle of RG2

% Make sure you have run 'vrepTools\vrepRemApi_init.m' in advance.

%% Get the gripper's handle
[res, rg2Ref] = vrep.simxGetObjectHandle(clientID,...
    'RG2',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.rg2Ref = rg2Ref;

clear rg2Ref res