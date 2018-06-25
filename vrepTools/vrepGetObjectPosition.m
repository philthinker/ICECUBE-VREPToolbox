function [ position ] = vrepGetObjectPosition( vrep, clientID, handle )
%vrepGetObjectPosition Get the position of the object in the V-REP scene.
% position: [x,y,z], the abusolte position of the object.

[res, position] = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_blocking);
vrchk(vrep,res);

end

