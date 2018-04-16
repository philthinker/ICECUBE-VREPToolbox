% UR5_PickAndPlace
% A demo to show how to control UR5 together with RG2 to pick and place.
% Haopeng Hu
% 2018.04.16

% V-REP scene: UR5plusRG2_PickAndPlace.ttt

% addpath('vrepTools');
% addpath('robotTools\UR5VREPTools');

vrepRemApi_init;

%% Get the UR5's handles
handles = struct('ID',clientID);

vrepGetUR5Handles;
%% Get the gripper's handle
[res, rg2Ref] = vrep.simxGetObjectHandle(clientID,...
    'RG2',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.rg2Ref = rg2Ref;
%% Start your simulation
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
disp('Simulation Started!');
vrchk(vrep,res);
%% Move UR5
% Rotate UR5_joint5
for deg = 0:-0.02:-pi/2
   vrep.simxSetJointTargetPosition(clientID, handles.ur5Joints(5),deg,vrep.simx_opmode_oneshot); 
   pause(0.05);
end
%% Close the RG2
[res, retInts, retFloats, retStrings, retBuffer] = vrep.simxCallScriptFunction(clientID, 'RG2',...
    vrep.sim_scripttype_childscript,'rg2Close',0,[],[],[],vrep.simx_opmode_blocking);
vrchk(vrep,res);
disp('RG2 closed');
% disp(retFloats);
%% Move UR5
% Rotate UR5_joint5
for deg = -pi/2:0.02:0
   vrep.simxSetJointTargetPosition(clientID, handles.ur5Joints(5),deg,vrep.simx_opmode_oneshot); 
   pause(0.05);
end
%% Open the RG2
[res, retInts, retFloats, retStrings, retBuffer] = vrep.simxCallScriptFunction(clientID, 'RG2',...
    vrep.sim_scripttype_childscript,'rg2Open',[],[],[],[],vrep.simx_opmode_blocking);
vrchk(vrep,res);
disp('RG2 opened');
% disp(retFloats);
%% Clean the vrep threads and shut down the program
vrep.simxFinish(clientID);
vrep.delete();

disp('The End');
