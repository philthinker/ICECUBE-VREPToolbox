function [  ] = ICECUBE_delete( vrep, clientID )
%ICECUBE_delete Finish the remote server and delete the vrep object
% vrep: The V-REP object
% clientID: The ID of the MALTAB client
% Never forget to use this function in the end

vrep.simxFinish(clientID);
vrep.delete();

end

