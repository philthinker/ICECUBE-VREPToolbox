function [ res ] = delete( obj )
%delete Delete the vrep service
% res: 0 for no error
% Never forget to use this function in the end

% @ICECUBE

obj.vrep.simxFinish(obj.clientID);
obj.vrep.delete();
res = 0;

end

