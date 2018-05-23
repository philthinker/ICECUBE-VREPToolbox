function [  ] = vrepPrint( vrep,clientID,messageString, dispSign )
%vrepPrint Add a status bar message
% vrep: the vrep object
% clientID: the client ID
% messageString: the message to add
% dispSign: 1 for display the message in MATLAB command window, 0 for just
% display in V-REP status bar (default).

if nargin < 4
    dispSign = 0;
end

vrep.simxAddStatusbarMessage(clientID,messageString,vrep.simx_opmode_oneshot);
if dispSign == 1
    disp(messageString);
end

end

