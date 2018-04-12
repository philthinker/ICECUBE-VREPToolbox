% HelloCIECUBE
% A Demo for icecube-VREPToolbox
% the scene 'UR5plusRG2.ttt' is used

% Haopeng Hu
% 2018.04.11

vrepRemApi_init

% V-REP service is ready, go ahead
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

% Get the UR5's connection's handle
[res, ur5Connection] = vrep.simxGetObjectHandle(clientID,...
    'UR5_connection',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.ur5Connection = ur5Connection;
%% Get the gripper's handle
[res, rg2Ref] = vrep.simxGetObjectHandle(clientID,...
    'RG2',vrep.simx_opmode_blocking);
vrchk(vrep,res);
handles.rg2Ref = rg2Ref;
%% Get the toys' handles
% Edit your own toys list
toyNames = {'HaopengToy01', 'HaopengToy02', 'HaopengToy03'};
toys = zeros(1,length(toyNames));
for i = 1:length(toyNames)
    [res, toys(i)] = vrep.simxGetObjectHandle(clientID,...
        toyNames{i},vrep.simx_opmode_blocking);
    vrchk(vrep,res);
end
handles.toys = toys;
%% Start your simulation
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
disp('Simulation Started!');
% Rotate UR5_joint6
for deg = 0:0.02:pi/2
   vrep.simxSetJointTargetPosition(clientID, handles.ur5Joints(6),deg,vrep.simx_opmode_oneshot); 
   pause(0.05);
end
%% Clean the vrep threads and shut down the program
vrep.simxFinish(clientID);
vrep.delete();

disp('The End');