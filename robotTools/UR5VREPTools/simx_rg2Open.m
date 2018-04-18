function [ res, rg2Velocity ] = simx_rg2Open( vrep, clientID, simx_opmode )
%simx_rg2Open Open the RG2 gripper
% vrep: the vrep object
% clientID: the MATLAB client ID
% simx_opmode: the operation mode
% res: the return code, you can use 'vrepTools\vrchk' to check it
% reg2Velocity: the openning velocity of RG2

% 'Lua\RG2_Close.lua' is required in your vrep scene

[res, retInts, retFloats, retStrings, retBuffer] = vrep.simxCallScriptFunction(clientID, 'RG2',...
    vrep.sim_scripttype_childscript,'rg2Open',[],[],[],[],simx_opmode);

if simx_opmode == vrep.simx_opmode_oneshot
    res = vrep.simx_error_noerror;
end

rg2Velocity = retFloats;

end

