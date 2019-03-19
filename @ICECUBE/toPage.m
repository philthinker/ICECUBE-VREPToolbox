function [ res ] = toPage( obj,pageNum )
%toPage To the new VREP page
% pageNum: the number of the page to view
% res: the returned code

% @ICECUBE

res = obj.vrep.simxSetIntegerParameter(obj.clientID, obj.vrep.sim_intparam_current_page, pageNum, obj.vrep.simx_opmode_blocking);
obj.vrchk(res);

end

