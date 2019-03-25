classdef IjspeertDMP
    %IjspeertDMP Ijspeert's dynamic movement primitive
    %   Designed for discrete movement only (temporally)
    %   Haopeng Hu
    %   2019.03.25
    %   All rights reserved.
    
    properties (Access = public)
        w;          % weights
        alpha;      % alpha of dynamic system
        beta;       % beta of dynamic system
        alphax;     % alphax of the canonical system
    end
    
    properties (Access = private)
        c;          % centers of the Gaussian basis
        h;          % variance of the Gaussian basis
        N;          % number of Gaussian basis
        g;          % goal
        y0;         % initial state
        tau;        % temporal scalor
        dt;         % time step
    end
    
    methods (Access = public)
        function obj = IjspeertDMP(alphax,alpha,beta,N,dt)
            %   Only 1D DMP is supported
            %   alpha > 4 * beta is a scaler, so is alphax
            obj.alpha = alpha(1,1); obj.beta = beta(1,1); obj.alphax = alphax(1,1);
            obj.N = ceil(N);
            obj.dt = dt;
            %   Default public variables
            obj.w = zeros(N,1);
            obj.g = 0;
            obj.y0 = 0;
            obj.tau = 1;
            obj.dt = 0.001;
            %   Default Gaussian basis
            x = obj.canonicalSystem();
            [obj.c, obj.h] = obj.GaussiansAssign(x);
        end
        %   Display
        function [] = DMPDisplay(obj)
            % Display the private properties.
            disp(strcat('alpha: ',num2str(obj.alpha)));
            disp(strcat('beta: ',num2str(obj.beta)));
            disp(strcat('alphax: ',num2str(obj.alphax)));
            disp(strcat('c: ',num2str((obj.c)')));
            disp(strcat('h: ',num2str((obj.h)')));
        end
        function [] = plot(obj,x,Y,tau)
            % It is highly recommended to run the DMP firstly.
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
            % Show the Gaussian basis and forcing function
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
        function [] = plotCompare(obj,Y,Target,tau)
            % Show the divergence between target trajectory and learned trajectory
            ddtar = Target(:,3);
            dtar = Target(:,2);
            tar = Target(:,1);
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
        %   Learn
        function obj = LWR(obj,Y,tau)
            % Locally weighted regression
            % J_i = \sum_{t=1}^{T}psi_i(t)(f(t)-w_i xi(t))^2
            % J_i = (f_target - w_i S)'Tau_i(f_target - w_i S)
            % Assume that the trajectory is complete
            ddy = Y(:,3);
            dy = Y(:,2);
            y = Y(:,1);
            % Compute x for scaled time
            obj.tau = tau;
            obj.dt = tau/size(y,1);
            x = obj.canonicalSystem();
            % Compute f_target, basis S
            % y: M x 1
            obj.g = y(end); obj.y0 = y(1);
            f_target = tau*tau*ddy - obj.alpha*(obj.beta*(obj.g-y)-tau*dy);
            S = x.*(obj.g-obj.y0)';
            % Compute Tau_i and w_i
            obj.w = zeros(obj.N,1);
            for i = 1:obj.N
                Tau = diag(obj.GaussianBasis(x, obj.c(i),obj.h(i)));
                obj.w(i) = (S'*Tau*S)\S'*Tau*f_target;
            end
        end
        %   Run
        function [Y,x,fx] = run(obj,y0,g,tau,dt)
            % Run the DMP with specified parameters
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

    %   The WP blogs : 
    %   - https://studywolf.wordpress.com/2013/11/16/dynamic-movement-primitives-part-1-the-basics/
    %   - https://studywolf.wordpress.com/2013/12/05/dynamic-movement-primitives-part-2-controlling-a-system-and-comparison-with-direct-trajectory-control/
    %   - https://studywolf.wordpress.com/2016/05/13/dynamic-movement-primitives-part-4-avoiding-obstacles/
    %   are very helpful for you to understand the usage of DMP.