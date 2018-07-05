function [ position ] = getObjectPosition( obj,objectHandle )
%getObjectPosition Get the position of the object in the V-REP scene.
% objectHandle: The handle of the object to be located.
% position: [x,y,z], the abusolte position of the object.

% @ICECUBE

[res, position] = obj.vrep.simxGetObjectPosition(obj.clientID, objectHandle, -1, obj.vrep.simx_opmode_blocking);
obj.vrchk(res);

end

