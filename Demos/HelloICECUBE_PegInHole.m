% HelloICECUBE_PegInHole
% A simple peg-in-hole demo
% Haopeng Hu
% 2018.07.05

% Use V-REP scene 'scenes\UR5PegInHole.ttt'.

%% Initialize ICECUBE

step = 0.01;
TIMEOUT = 5000;
% Construct ICECUBE object. Note that ICECUBE is not a handle class.
icecube = ICECUBE(step,TIMEOUT);

% Get UR5's handles
icecube = icecube.getUR5Handles();

% Get object's handle
icecube = icecube.getObjectHandle('Hole');

% Get object's position
targetPosition = icecube.getObjectPosition('Hole');

%% Move it

% Start simulation
icecube.start();

% Move to initial configuration
initConfig = [0, pi/8, pi/2-pi/8, 0, -pi/2, 0];
ur5MoveToJointPosition(icecube,initConfig);
tempQuat = ur5GetIKTipQuaternion(icecube);
pause(1);

% Move to the upside of the hole
ur5MoveToConfiguration(icecube,targetPosition+[0,0,0.15],tempQuat);
tempPosi = ur5GetIKTipPosition(icecube);
pause(1);
icecube.toPage(1);

% Plug
ur5MoveToConfiguration(icecube,tempPosi-[0,0,0.05],tempQuat);
pause(2);
icecube.toPage(0);
pause(1);

%% Stop and delete

icecube.stop();
icecube.delete();

clear ans step TIMEOUT