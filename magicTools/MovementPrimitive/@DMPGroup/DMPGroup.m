classdef DMPGroup
    %DMPGroup A set of DMPs sharing the same canonical system
    %   Haopeng Hu
    %   2019.03.25
    %   All rights reserved
    
    properties (Access = public)
        dt;         % Time step
    end
    
    properties (Access = private)
        NUM;        % Total number of dmps
        dmps;       % NUM x 1, all the dmps
        alphax;     % alphax of the dmps
        tau;        % Temporal scalar
    end
    
    methods (Access = public)
        function obj = DMPGroup(dmp,NUM,dt)
            % It is designed to group NUM DMPs of the same type.
            obj.NUM = ceil(NUM);
            obj.dmps = repmat(dmp,[obj.NUM,1]);
            obj.dt = dt;
            obj.tau = 1;
            obj.alphax = dmp.alphax;
        end
        % Display
        function [] = DMPDisplay(obj,index)
            if nargin == 1
                % Display all the DMPs
                for i = 1:obj.NUM
                    obj.dmps(i).DMPDisplay();
                end
            else
                % Display the index-th DMP
                obj.dmps(index).DMPDisplay();
            end
        end
        % Plot
        function [] = plot(obj,x,Y,tau)
            obj.dmps(1).plot(x,Y,tau);
        end
        function [] = plotGaussian(obj,x,fx,tau)
            obj.dmps(1).plotGuassian(x,fx,tau);
        end
        function [] = plotCompare(obj,Y,T,tau)
            obj.dmps(1).plotCompare(Y,T,tau);
        end
        % Learn
        function obj = learn(obj,T,tau,index)
            % Locally weighted regression
            % T: M x 3*NUM, target trajectory
            % tau: duration
            % index: the DMP to be learned
            if nargin <4
                % Learn all the DMPs
                for i = 1:obj.NUM
                    % Learn the weights one by one
                    obj.dmps(i) = obj.dmps(i).LWR(T(:,((i-1)*3+1):(i*3)),tau);
                end
            else
                % Learn the index-th DMP
                obj.dmps(index) = obj.dmps(index).LWR(T(:,1:3),tau);
            end
        end
        % Run
        function [Y,x,fx] = run(obj,y0,g,tau,index)
            % Run the DMP
            if nargin < 5
                % Run all the DMPs
                M = floor(tau/obj.dt);
                Y = zeros(M,3*obj.NUM); x = zeors(M,1); fx = zeros(M,obj.NUM);
                for i = 1:obj.NUM
                    % Run the DMPs one by one.
                    [Y(:,((i-1)*3+1):(i*3)),x,fx(:,i)] = obj.dmps(i).run(y0,g,tau,obj.dt);
                end
            else
                % Run the index-th DMP
                [Y,x,fx] = obj.dmps(index).run(y0(1,1),g(1,1),tau,obj.dt);
            end
        end
    end
    
    methods (Access = private)
    end
    
end

