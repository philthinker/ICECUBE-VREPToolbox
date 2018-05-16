% A demo to drive the UR5 for a pick-and-place task

% Haopeng Hu
% 2018.05.15

% Load 'PickAndPlace_v1_DPA.mat'
% Use scene 'UR5plusRG2_Pro

%% Iniitialize V-REP scence
vrepRemApi_init;
vrep.simxAddStatusbarMessage(clientID, 'MATLAB client is connected!', vrep.simx_opmode_oneshot);

vrepGetUR5Handles;
vrepGetRG2Handles;

TIMEOUT = 1000;
%% Simulation

for j = 5:5
    res = ICECUBE_start(vrep,clientID,0.1,TIMEOUT);
    vrchk(vrep,res);
    % Open the gripper as a default configuration
    rg2Action(vrep,clientID,0);
    gripperFlag = 0;
    TrajSelected = TrajsOptimal{j};
    Nj = size(TrajSelected,1);
    initialConfig = [pi/4, 0, pi/2, 0, -pi/2, 0];
    ur5RMLMoveToJointPositions(vrep,clientID,initialConfig);
    for i = 1:Nj
        % Drive the UR5
        ur5RMLMoveToPosition(vrep,clientID,TrajSelected(i,1:3),TrajSelected(i,4:7))
        if TrajSelected(i,end) ~= gripperFlag
            % Close or open the gripper
            gripperFlag = TrajSelected(i,end);
            res = rg2Action(vrep,clientID,gripperFlag);
            vrchk(vrep,res);
        end
    end
    ICECUBE_stop(vrep, clientID);
    pause(2);
end

vrep.simxFinish(clientID);
vrep.delete();

clear ans i j k
clear res Nj TIMEOUT gripperFlag