% HelloICECUBE_PickAndPlace
% A simple pick-and-place demo
% Haopeng Hu
% 2018.07.05

% Use V-REP scene 'scenes\UR5PickAndPlace.ttt'.

%% Initialize ICECUBE

step = 0.01;
TIMEOUT = 5000;
% Construct ICECUBE object. Note that ICECUBE is not a handle class.
icecube = ICECUBE(step,TIMEOUT);

% Get UR5's handles
icecube = icecube.getUR5Handles();

% Get the objects' handles
icecube = icecube.getObjectHandle('Toy1');
icecube = icecube.getObjectHandle('Toy2');

% Get the objects' position
initPosition = icecube.getObjectPosition('Toy2') + [0,0,0.002];
targetPosition = icecube.getObjectPosition('Toy1') + [0,0,0.05];

%% Move it

% Start simulation
icecube.start();

% Move to initial configuration
initConfig = [0, pi/8, pi/2-pi/8, 0, -pi/2, 0];
ur5MoveToJointPosition(icecube,initConfig);
tempQuat = ur5GetIKTipQuaternion(icecube);
pause(1);

% Move to initPosition
ur5MoveToConfiguration(icecube,initPosition,tempQuat);
pause(1);
icecube.toPage(1);
pause(1);

% Close the gripper
rg2Action(icecube,true);
pause(0.5);

% Move to targetPosition
ur5MoveToConfiguration(icecube,targetPosition,tempQuat);
pause(1);

% Open the gripper
rg2Action(icecube,false);
pause(1);
icecube.toPage(0);
pause(1);

%% Stop and delete

icecube.stop();
icecube.delete();

clear ans