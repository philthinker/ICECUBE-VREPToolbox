function [ position ] = getObjectPosition( obj,objectName )
%getObjectPosition Get the position of the object in the V-REP scene.
% objectName: The name of the object to be located.
% position: [x,y,z], the abusolte position of the object.
% Make sure the handle of the object is already gotten.

% @ICECUBE

[res, position] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.handles.(objectName), -1, obj.vrep.simx_opmode_blocking);
obj.vrchk(res);

end

