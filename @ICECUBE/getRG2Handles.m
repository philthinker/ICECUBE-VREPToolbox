function [ obj ] = getRG2Handles( obj )
%getRG2Handles Get RG2's handls from the vrep scene.
% Note that you DO NOT need to run this function if you just wanna control
% the open/close state of RG2

% @ICECUBE

[res, rg2Ref] = obj.vrep.simxGetObjectHandle(obj.clientID,'RG2',obj.vrep.simx_opmode_blocking);
obj.vrchk(res);
obj.handles.rg2Ref = rg2Ref;

end

