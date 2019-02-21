function [ obj ] = getER3AC60Handles( obj )
%getER3AC60Handles Get ER3AC60's handls from the vrep scene.
% Haopeng Hu
% 2019.01.08

% @ICECUBE

% handles.er3ac60Joints: handles of joints of ER3AC60
% handles.er3ac60Ref: handle of ER3AC60
% handles.er3ac60ikTip: handle of ER3AC60' ikTip
% handles.er3ac60connector: handle of ER3AC60's connector ( ForceTorque sensor)
%% Get the ER3AC60's joints' handles
jointNames = {'ER3AC60_joint1','ER3AC60_joint2','ER3AC60_joint3',...
    'ER3AC60_joint4','ER3AC60_joint5','ER3AC60_joint6'};
er3ac60Joints = zeros(1,6);
for i = 1:6     
    [res, er3ac60Joints(i)] = obj.vrep.simxGetObjectHandle(obj.clientID, ...
        jointNames{i},obj.vrep.simx_opmode_blocking);   % blocking mode
    obj.vrchk(res);
end
obj.handles.er3ac60Joints = er3ac60Joints;
%% Get the ER3AC60's handle
[res, er3ac60Ref] = obj.vrep.simxGetObjectHandle(obj.clientID,...
    'ER3AC60',obj.vrep.simx_opmode_blocking);
obj.vrchk(res);
obj.handles.er3ac60Ref = er3ac60Ref;
%% Get the ER3AC60's ikTip's handle
[res, er3ac60ikTip] = obj.vrep.simxGetObjectHandle(obj.clientID,'ER3AC60_ik_tip',obj.vrep.simx_opmode_blocking);
obj.vrchk(res);
obj.handles.er3ac60ikTip = er3ac60ikTip;
%% Get the ER3AC60's connector's handle
[res,er3ac60connector] = obj.vrep.simxGetObjectHandle(obj.clientID,'ER3AC60_connection',obj.vrep.simx_opmode_blocking);
obj.vrchk(res);
obj.handles.er3ac60connector = er3ac60connector;

clear jointNames er3ac60Joints er3ac60Ref er3ac60ikTip er3ac60connector res

end

