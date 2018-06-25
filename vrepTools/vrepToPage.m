function [ res ] = vrepToPage( vrep,clientID,pageNum )
%vrepToPage To the new page
% vrep: the vrep (remApi) object
% clientID: the client ID
% pageNum: the number of the page to view
% res: the returned code

res = vrep.simxSetIntegerParameter(clientID,vrep.sim_intparam_current_page,pageNum,vrep.simx_opmode_blocking);

end

