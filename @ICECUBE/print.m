function [ res ] = print( obj,infoString )
%print Print the information to VREP console
% res: 0 for no error

% @ICECUBE

obj.vrep.simxAddStatusbarMessage(obj.clientID,infoString,obj.vrep.simx_opmode_oneshot);
res = 0;

end

