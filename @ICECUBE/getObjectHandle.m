function [ obj ] = getObjectHandle( obj,objectName )
%getObjectHandle Get the handle of 'objectName'
% objectName: String, the name of the object to be handled

% @ICECUBE
% ICECUBE = ICECUBE.getObjectHandle(...)

if ~isfield(obj.handles,objectName)
    % Add the new handle into the handles
    [res, newHandle] = obj.vrep.simxGetObjectHandle(obj.clientID,objectName,obj.vrep.simx_opmode_blocking);
    obj.vrchk(res);
    obj.handles.(objectName) = newHandle;
else
    % Well, the handle is already in the handles
end

end

