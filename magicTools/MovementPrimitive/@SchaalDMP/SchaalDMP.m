classdef SchaalDMP
    %Stefan Schaal's Modified Dynamic Movement Primitve
    %   Designed for discrete movement only (temporally)
    %   Haopeng Hu
    %   2019.03.22
    %   All rights reserved.
    
    %   tau * dx = -alphas * x, x0 = 1
    %   tau * dy = z
    %   tau * dz = K * (g - y) - D * z - K * (g - y0) * x  + K * f(x)
    
    properties (Access = private)
        y0;     % Start position
        g;      % Goal position
        tau;    % Temporal scalar
        dt;     % Time step
        c;      % Centers of the Gaussian basis
        h;      % Variance of the Gaussian basis
        N;      % Number of Gaussian basis
    end
    
    properties (Access = public)
        K;      % K of dynamic system
        D;      % D of dynamic system
        alphax; % alphax of the canonical system
        w;      % Weights
    end
    
    methods (Access = public)
        % Initialization alphax>0, D>0, K>0, K<D^2/4
        function obj = SchaalDMP(alphax,K,D,N,dt)
            % Only 1D DMP is supported
            % Assigned properties (May not vary any more)
            obj.alphax = alphax(1,1);
            obj.K = K(1,1);
            obj.D = D(1,1);
            obj.N = ceil(N);
            obj.dt = dt(1,1);
            % Default properties
            obj.tau = 1;
            obj.y0 = 0;
            obj.g = 0;
            obj.w = zeros(obj.N,1);
            % Gaussian basis
            x = obj.canonicalSystem();
            [obj.c, obj.h] = obj.GaussiansAssign(x);
        end
        % Display
        function [] = DMPdisplay(obj)
            % Display the main parameters of the DMP
            disp(strcat('K: ',num2str(obj.K)));
            disp(strcat('D: ',num2str(obj.D)));
            disp(strcat('alphax: ',num2str(obj.alphax)));
            disp(strcat('c: ',num2str((obj.c)')));
            disp(strcat('h: ',num2str((obj.h)')));
        end
        % Plot
        function [] = plot(obj,x,Y,tau)
            % Please run the method obj.run first to obtain x and Y
            % tau is exactly the one used in obj.run
            time = linspace(0,tau,size(x,1));
            
            figure(1);clf;
            
            subplot(221);
            plot(time,x);ylabel('x'); xlabel('Time');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            grid on;
            
            subplot(222);
            plot(time,Y(:,1));ylabel('y'); xlabel('Time');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            grid on;
            
            subplot(223);
            plot(time,Y(:,2));ylabel('dy'); xlabel('Time');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            grid on;
            
            subplot(224);
            plot(time,Y(:,3));ylabel('ddy'); xlabel('Time');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            grid on;
        end
        function [] = plotGaussian(obj,x,fx,tau)
            % Show the Gaussians of the DMP
            time = linspace(0,tau,size(fx,1));
            Gs = zeros(size(fx,1),obj.N);    % Gaussian basis
            % Compute the basis
            for i = 1:obj.N
                Gs(:,i) = obj.GaussianBasis(x,obj.c(i),obj.h(i));
            end
            
            figure(2);clf;
            subplot(211);
            plot(time,Gs);title('Gaussian basis'); xlabel('Time');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            
            subplot(212);
            plot(time,fx);ylabel('f(x)'); xlabel('Time');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
        end
        function [] = plotCompare(obj,Y,T,tau)
            % Compare the learned trajectory Y and the target T
            % Please run the method obj.run first to obtain the Y
            % tau is exactly the one used in learning and running method
            ddtar = T(:,3);
            dtar = T(:,2);
            tar = T(:,1);
            time = linspace(0,tau,size(tar,1));
            
            figure(3);clf;
            
            subplot(311);
            plot(time,Y(:,1),time,tar);title('y vs t'); xlabel('Time');legend('y','t');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            grid on;
            
            subplot(312);
            plot(time,Y(:,2),time,dtar);title('dy vs dt'); xlabel('Time');legend('dy','dt');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            grid on;
            
            subplot(313);
            plot(time,Y(:,3),time,ddtar);title('ddy vs ddt'); xlabel('Time');legend('ddy','ddt');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
            grid on;
        end
        % Learn
        function obj = LWR(obj,T,tau)
            % Locally weighted regression
            % T; M x 3, the target trajectory
            % tau: temporal scalar
            
            % J_i = \sum_{t=1}^{TEnd}psi_i(t)(f(t)-w_i x(t))^2
            % J_i = (f_target - w_i S)'Tau_i(f_target - w_i S)
            % f_target = (tau*dz+D*z)/K - (g-y) + (g-y0)*x
            % Assume that the trajectory is complete
            ddy = T(:,3);
            dy = T(:,2);
            y = T(:,1);
            % Compute x for scaled time
            obj.tau = tau;
            obj.dt = tau/size(y,1);
            x = obj.canonicalSystem();
            % Compute f_target, basis S
            % y: M x 1
            obj.g = y(end); obj.y0 = y(1);
            f_target = (tau*tau*ddy + obj.D*tau*dy)/obj.K - (obj.g-y) + (obj.g-obj.y0)*x;
            S = x;
            % Compute Tau_i and w_i
            obj.w = zeros(obj.N,1);
            for i = 1:obj.N
                Tau = diag(obj.GaussianBasis(x, obj.c(i),obj.h(i)));
                obj.w(i) = (S'*Tau*S)\S'*Tau*f_target;
            end
        end
        % Run
        function [Y,x,fx] = run(obj,y0,g,tau,dt)
            % Run the DMP with manual arguments
            if nargin == 5
                %   Run DMP with unmodified dt
                obj.dt = dt(1,1);
            end
            obj.y0 = y0(1,1);
            obj.g = g(1,1);
            obj.tau = tau(1,1);
            x = obj.canonicalSystem(); 
            [Y, fx] = obj.dynamicSystem(x);
        end
    end
    
    methods (Access = protected)
        [c,h] = GaussiansAssign(obj,x);
        x = canonicalSystem(obj);
        fx = forcingFunc(obj,x);
        psi = GaussianBasis(obj,x,c,h);
        [Y,fx] = dynamicSystem(obj,x);
    end
    
end

%% Appendix
% Note that the original DMP has 3 drawbacks:
% 1. If start and goal of a movement are the same, then the forcing term
%    cannot drive the system away from tis initial state.
% 2. The scaling of f with g-y0 is problematic if it is close to 0. Here,a
%    small change in g may lead to huge accelerations.
% 3. Whenever a movement adapts to a new goal g' s.t. g'-y0 changes its
%    sign compared to g-y0, the resulting generalization is mirrored.

% Refer to : P. Pastor, et al. Learning and Generalization of Motor Skills 
% by Learning from Demonstration, ICRA, 2009.
