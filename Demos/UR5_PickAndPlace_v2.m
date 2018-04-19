% UR5_PickAndPlace
% A demo to show how to control UR5 together with RG2 to pick and place.
% Haopeng Hu
% 2018.04.19

% V-REP scene: UR5plusRG2_PickAndPlace.ttt

% Never forget to run 'ICECUBE_init.m' firstly. 
TIMEOUT = 1000;
%% Initialize the simulation
vrepRemApi_init;
vrep.simxAddStatusbarMessage(clientID, 'MATLAB client is connected!', vrep.simx_opmode_oneshot);
%% Get the vrep handles
handles = struct('ID',clientID);

vrepGetUR5Handles;
vrepGetRG2Handles;
toys = -1*ones(1,3);
for i = 1:3
    [res, toys(i)] = vrep.simxGetObjectHandle(clientID, strcat('HaopengToy0',num2str(i)),vrep.simx_opmode_blocking);
    vrchk(vrep,res);
end

disp('All handles have been gotten.');
%% Start your simulation
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
disp('Simulation Started!');
vrchk(vrep,res);
while i < TIMEOUT && vrep.simxGetIntegerSignal(clientID,'ClientRunning',vrep.simx_opmode_blocking) == 1
    i = i + 1;
    pause(0.1);
end
if i>= TIMEOUT
    disp('An error occurred in your UR5 thread');
    vrep.simxFinish(clientID);
    vrep.delete();
    return
else
    disp('UR5 is ready.');
end

%% Open the RG2
% res = sim_rg2Open(vrep, clientID, vrep.simx_opmode_blocking);
% vrchk(vrep,res);
% pause(5)
% res = sim_rg2Close(vrep, clientID, vrep.simx_opmode_blocking);
% vrchk(vrep,res);
% pause(5)
%% Move to target joint positions
tempjpos = deg2rad([-70.1, 18.85, 93.18, 68.02, 109.9, 90]);
sim_rmlMoveToJointPositions(vrep,clientID,tempjpos);
pause(2);
while i < TIMEOUT && vrep.simxGetIntegerSignal(clientID, 'ICECUBE_0', vrep.simx_opmode_blocking) ~= 0
    i = i + 1;
    pause(0.1);
end
%% Move by ik group
% [res, temppos] = vrep.simxGetObjectPosition(clientID, handles.ur5ikTip, -1, vrep.simx_opmode_blocking);
% vrchk(vrep,res);
% [res, tempqua] = vrep.simxGetObjectQuaternion(clientID, handles.ur5ikTip, -1, vrep.simx_opmode_blocking);
% vrchk(vrep,res);
% 
% [res, targetQua] = vrep.simxGetObjectQuaternion(clientID, handles.ur5ikTip, -1, vrep.simx_opmode_blocking);
%     vrchk(vrep, res);
% for i = 1:2
%     [res, toyPos] = vrep.simxGetObjectPosition(clientID,toys(i),-1,vrep.simx_opmode_blocking);
%     vrchk(vrep,res);
%     targetPos = toyPos; targetPos(3) = targetPos(3)+0.3;
%     targetPos
%     res = simx_rmlMoveToPosition(vrep,clientID,vrep.simx_opmode_blocking,targetPos,targetQua); vrchk(vrep, res);
%     targetPos = toyPos;
%     res = simx_rmlMoveToPosition(vrep,clientID,vrep.simx_opmode_blocking,targetPos,targetQua); vrchk(vrep, res);
%     res = simx_rg2Close(vrep,clientID,vrep.simx_opmode_blocking); vrchk(vrep, res);
%     pause(5);
%     targetPos(3) = targetPos(3)+0.1+0.03*(i-1);
%     res = simx_rmlMoveToPosition(vrep,clientID,vrep.simx_opmode_blocking,targetPos,targetQua); vrchk(vrep, res);
%     pause(1)
%     targetPos(2) = targetPos(2)+0.3;
%     res = simx_rmlMoveToPosition(vrep,clientID,vrep.simx_opmode_blocking,targetPos,targetQua); vrchk(vrep, res);
%     targetPos(3) = targetPos(3) -0.1 -0.06*i;
%     res = simx_rmlMoveToPosition(vrep,clientID,vrep.simx_opmode_blocking,targetPos,targetQua); vrchk(vrep, res);
%     res = simx_rg2Open(vrep,clientID,vrep.simx_opmode_blocking); vrchk(vrep, res);
%     pause(5)
%     targetPos(3) = targetPos(3) + 0.1 + 0.06*i;
%     res = simx_rmlMoveToPosition(vrep,clientID,vrep.simx_opmode_blocking,targetPos,targetQua); vrchk(vrep, res);
% end
    
%% Move to initial joint positions
tempjpos = zeros(1,6);
sim_rmlMoveToJointPositions(vrep,clientID,tempjpos);
pause(2);
while i < TIMEOUT && vrep.simxGetIntegerSignal(clientID, 'ICECUBE_0', vrep.simx_opmode_blocking) ~= 0
    i = i + 1;
    pause(0.1);
end
%% Clean the vrep threads and shut down the program
vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 3, vrep.simx_opmode_blocking);
vrep.simxFinish(clientID);
vrep.delete();

disp('The End');
