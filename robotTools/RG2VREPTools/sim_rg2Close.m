function [ res ] = sim_rg2Close(  vrep, clientID, simx_opmode  )
%sim_rg2Close Close your RG2 gripper
% vrep: the vrep object
% clientID: the MATLAB client ID
% simx_opmode: the operation mode
% res: the return code, you can use 'vrepTools\vrchk' to check it

% 'Lua\RG2_Close_Embedded.lua' is required in your vrep scene

res = vrep.simxSetIntegerSignal(clientID, 'RG2Close', 1, simx_opmode);

if simx_opmode == vrep.simx_opmode_oneshot
    res = vrep.simx_error_noerror;
end

end

