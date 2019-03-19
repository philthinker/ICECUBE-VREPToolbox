function [ position ] = getObjectPosition( obj,objectName )
%getObjectPosition Get the position of the object in the V-REP scene.
% objectName: The name of the object to be located.
% position: [x,y,z], the abusolte position of the object.
% To boost efficiency, get the handle of the object in advance.

% @ICECUBE

if ~isfield(obj.handles,objectName)
    % Got the handle first
    obj = obj.getObjectHandle(objectName);
end
[res, position] = obj.vrep.simxGetObjectPosition(obj.clientID, obj.handles.(objectName), -1, obj.vrep.simx_opmode_blocking);
obj.vrchk(res);

end

