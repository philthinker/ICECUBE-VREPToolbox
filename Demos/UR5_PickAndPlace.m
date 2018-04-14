% UR5_PickAndPlace
% A demo to show how to control UR5 together with RG2 to pick and place.
% Haopeng Hu
% 2018.04.14

% V-REP scene: UR5plusRG2.ttt

vrepRemApi_init;

%% Get the UR's handles
handles = struct('ID',clientID);

% Get the UR5's joints' handles
jointNames = {'UR5_joint1','UR5_joint2','UR5_joint3',...
    'UR5_joint4','UR5_joint5','UR5_joint6'};
ur5Joints = zeros(1,6);
for i = 1:6     
    [res, ur5Joints(i)] = vrep.simxGetObjectHandle(clientID, ...
        jointNames{i},vrep.simx_opmode_blocking);   % blocking mode
    vrchk(vrep,res);
end
handles.ur5Joints = ur5Joints;

% Get the UR5's handle
[res, ur5Ref] = vrep.simxGetObjectHandle(clientID,...
    'UR5',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.ur5Ref = ur5Ref;
%% Get the gripper's handle
[res, rg2Ref] = vrep.simxGetObjectHandle(clientID,...
    'RG2',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.rg2Ref = rg2Ref;
%% Start your simulation
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
disp('Simulation Started!');
vrchk(vrep,res);
%% 

%% Close the RG2
[res, retInts, retFloats, retStrings, retBuffer] = vrep.simxCallScriptFunction(clientID, 'RG2',...
    vrep.sim_scripttype_childscript,'rg2Close',1,[],[],[],vrep.simx_opmode_blocking);
vrchk(vrep,res);
%% Clean the vrep threads and shut down the program
vrep.simxFinish(clientID);
vrep.delete();

disp('The End');
