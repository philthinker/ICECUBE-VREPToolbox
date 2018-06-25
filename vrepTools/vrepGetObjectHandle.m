function [ handles ] = vrepGetObjectHandle( vrep,clientID,handles,objectName )
%vrepGetObjectHandle Get the handle of 'objectName'
% objectName: String, the name of the object to be handled

if ~isfield(handles,objectName)
    % Add the new handle into the handles
    [res, newHandle] = vrep.simxGetObjectHandle(clientID,objectName,vrep.simx_opmode_blocking);
    vrchk(vrep,res);
    handles.(objectName) = newHandle;
else
    % Well, the handle is already in the handles
end

end

