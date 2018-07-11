classdef ICECUBE
    %ICECUBE The ICECUBE main class
    % All the functions of ICECUBE are here.
    
    % Haopeng Hu
    % 2018.06.30
    
    % ICECUEB Communication Protocol v2.0
    
    properties ( Access = public )
        vrep;       % The vrep object
        clientID;   % The ID of the MATLAB applicaiton
        handles;    % The set of handles in V-REP scene
        step;       % The sampling interval
        TIMEOUT;    % The timeout
    end
    
    methods
        function obj = ICECUBE(step,TIMEOUT)
            disp('Connecting to V-REP service ...(ICECUBE v2.3.0)');
            obj.step = step;
            obj.TIMEOUT = TIMEOUT;
            obj.vrep = remApi('remoteApi');     % Use remoteApiProto.m
            obj.vrep.simxFinish(-1);            % Stop other connections
            obj.clientID = obj.vrep.simxStart('127.0.0.1',19997,true,true,6000,5);  % Connect to V-REP
            % Attention! port number '19997' is not trivial. It's for the continuous
            % operation mode. See remoteApiConnections.txt for details.
            % If you want temporary operation mode, change the port number '19997' to a larger one.
            if obj.clientID > -1    % Connected!
                disp('V-REP service is connected!');
                obj.handles = struct('ID',obj.clientID);
                obj.vrep.simxAddStatusbarMessage(obj.clientID,'A MATLAB client is found (ICECUBE 2.3)',obj.vrep.simx_opmode_oneshot);
            else
                disp('Failed connecting to remote API server');
                obj.vrep.delete();
                return;
            end
        end
    end
    
    methods ( Access = public )
        % System API
        res = start(obj)                % Start a simulation
        res = stop(obj)                 % Stop the simulation
        res = delete(obj)               % Delete the vrep service
        res = vrchk(obj,res,buffer)     % Check the returned code
        % Auxiliary API
        res = toPage(obj,pageNum)       % To the new page
        res = print(obj,infoString)     % Print the information to VREP console
        % VREP scene API
        obj = getObjectHandle(obj,objectName)           % Get the handle of 'objectName'
        handles = getHandles(obj)                       % Get the handles of ICECUBE
        position = getObjectPosition(obj,objectName)    % Get the position of the object in the V-REP scene by name
        % VREP model API
        obj = getUR5Handles(obj)    % Get the handles of UR5
    end
    
    methods ( Access = public )
        ticks = wait(obj)           % Wait until the task is done
    end
    
end

