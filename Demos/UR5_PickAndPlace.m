% UR5_PickAndPlace
% A demo to show how to control UR5 together with RG2 to pick and place.
% Haopeng Hu
% 2018.04.16

% V-REP scene: UR5plusRG2_PickAndPlace.ttt

% addpath('vrepTools');
% addpath('robotTools\UR5VREPTools');

%% Initialize the simulation
vrepRemApi_init;
vrep.simxAddStatusbarMessage(clientID, 'MATLAB client is connected!', vrep.simx_opmode_oneshot);
%% Get the vrep handles
handles = struct('ID',clientID);

vrepGetUR5Handles;

vrepGetRG2Handles;

disp('All handles have been gotten.');
%% Start your simulation
res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
disp('Simulation Started!');
vrchk(vrep,res);
while i < 1000 && vrep.simxGetIntegerSignal(clientID,'ClientRunning',vrep.simx_opmode_blocking) == 1
    i = i + 1;
    pause(0.1);
end
if i>= 1000
    disp('An error occurred in your UR5 thread');
    vrep.simxFinish(clientID);
    vrep.delete();
    return
else
    disp('UR5 is ready.');
end

%% Open the RG2
res = simx_rg2Open(vrep, clientID, vrep.simx_opmode_blocking);
vrchk(vrep,res);
pause(5)
%% Move to target joint positions
tempjpos = deg2rad([-70.1, 18.85, 93.18, 68.02, 109.9, 90]);
res = simx_rmlMoveToJointPositions(vrep,clientID,vrep.simx_opmode_blocking,tempjpos);
vrchk(vrep,res);
%% Close the RG2
res = simx_rg2Close(vrep,clientID,vrep.simx_opmode_blocking);
vrchk(vrep,res);
pause(5);
%% Move to initial joint positions
tempjpos = zeros(1,6);
res = simx_rmlMoveToJointPositions(vrep,clientID,vrep.simx_opmode_blocking,tempjpos);
vrchk(vrep,res);
%% Clean the vrep threads and shut down the program
%res = vrep.simxSetIntegerSignal(clientID,'ClientRunning',0,vrep.simx_opmode_blocking);
%vrchk(vrep,res);
vrep.simxFinish(clientID);
vrep.delete();

disp('The End');
