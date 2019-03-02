classdef DMP
    %DMP Ijspeert's dynamic movement primitive
    %Designed for discrete movement only (temporally)
    %   Haopeng Hu
    %   2019.02.23
    
    %   tau * dx = -alphax * x, x0 = 1;
    %   tau * dy = z,
    %   tau * dz = alpha*(beta*(g-y)-z)+f(x).
    
    properties (Access = public)
        % They are public, but DO NOT modify the values by obj.PROPERTY
        alpha;      % alpha of the output system
        beta;       % beta of the output system
        alphax;     % alpha of the canonical system
        N;          % number of the Gaussian kernels
        D;          % dimension (less than 7 as recommended)
        hs;         % variances of the Gaussian kernels
        cs;         % means of the Gaussian kernels
        w;          % weighting params. to be learned/reinforced
        dt;         % the time step
    end
    
    properties (Access = private)
        g;          % goal
        y0;         % initial position
        tau;        % duration (default: 1)
        x;          % phase variable when obj.tau == 1
    end
    
    methods
        % Initialization
        function obj = DMP(alpha, beta, alphax, N)
            %   Dimension of alpha and beta must coincide
            obj.D = min(size(alpha,1),size(beta,1));
            %   Params. initialization
            %   alpha,beta>0, alpha>4*beta
            obj.alpha = alpha(1:obj.D,1);
            obj.beta = beta(1:obj.D,1);
            %   alphax>0
            obj.alphax = alphax(1,1);
            obj.g = ones(obj.D,1);
            obj.y0 = zeros(obj.D,1);
            obj.dt = 0.001;
            
            obj.N = max(ceil(N),1);
            obj.tau = 1;
            obj.w = zeros(obj.N,1);
            
            %   Initialize Gaussians
            obj.x = obj.canonicalSystem();
%             obj.cs = obj.GaussianCentersAssign(obj.x);
%             obj.hs = 100*obj.N./obj.cs;
            [obj.cs,obj.hs] = obj.GaussiansAssign(obj.x);
        end
        % Set origin and goal
        function obj = setGoalOrigin(obj,g,y0)
            if nargin < 3
                y0 = obj.g;
            end
            if size(g,1) ~= obj.D || size(y0,1) ~= obj.D
                error('The goal or origin must be of dimension D');
            else
                obj.g = g;
                obj.y0 = y0;
            end
        end
        % Batch Learning
        function obj = BatchLearnLWR(obj,y, dy, ddy)
            % Locally weighted regression
            % J_i = \sum_{t=1}^{T}psi_i(t)(f(t)-w_i xi(t))^2
            % J_i = (f_target - w_i S)'Tau_i(f_target - w_i S)
            
            % Compute x
            x = obj.canonicalSystem();
            % Compute f_target, basis S
            % y: M x D
            f_target = (obj.tau)*(obj.tau)*ddy - obj.alpha*(obj.beta*(obj.g'-y)-(obj.tau)*dy);
            S = x.(obj.g-obj.y0)';
            % Compute Tau_i and w_i
            obj.w = zeros(obj.N,1);
            for i = 1:obj.N
                Tau = diag(obj.GaussianKernel(x,obj.c(i),obj.h(i)));
                obj.w(i) = (S'*Tau*S)\S'*Tau*f_target;
            end
        end
        % Run DMP dynamics
        function [y,dy,ddy,x,dx,f] = run(obj,tau)
            if tau <= 0
                error('You must specify a positive tau');
            else
                obj.tau = tau;
                [x,dx] = obj.canonicalSystem();
                [y,dy,ddy,f] = obj.dynamicSystem(x);
            end
        end
        % Plot
        function [] = plot(obj,x,y,dy,ddy,tau)
            time = linspace(0,tau,size(x,1));
            for i = 1:obj.D
                figure(i);clf;
                
                subplot(221); 
                plot(time,x);title('x'); xlabel('Time');
                aa=axis; axis([min(time) max(time) aa(3:4)]);
                grid on;
                
                subplot(222); 
                plot(time,y);title('y'); xlabel('Time');
                aa=axis; axis([min(time) max(time) aa(3:4)]);
                grid on;
                
                subplot(223); 
                plot(time,dy);title('dy'); xlabel('Time');
                aa=axis; axis([min(time) max(time) aa(3:4)]);
                grid on;
                
                subplot(224); 
                plot(time,ddy);title('ddy'); xlabel('Time');
                aa=axis; axis([min(time) max(time) aa(3:4)]);
                grid on;
            end
        end
        function [] = plot3d(obj,y)
            % Draw a 3D figure
            if size(y,2)<3
                error('You must specify 3 coordinates');
            else
                figure(1);clf;
                plot3(y(:,1),y(:,2),y(:,3));
                xlabel('x');ylabel('y');zlabel('z');
                grid on;
            end
        end
        function [Gs] = plotGaussian(obj,x,f,tau)
            time = linspace(0,tau,size(f,1));
            Gs = zeros(size(f,1),obj.N);    % Gaussian kernels
            % Compute the kernels
            for i = 1:obj.N
                Gs(:,i) = obj.GaussianKernel(x,obj.cs(i),obj.hs(i));
            end
            
            figure(obj.D+1);clf;
%             subplot(311);
%             plot(x,Gs);title('Gaussian kernels'); xlabel('x');
%             aa=axis; axis([min(x) max(x) aa(3:4)]);
            
            subplot(211);
            plot(time,Gs);title('Gaussian kernels'); xlabel('Time');
            aa=axis; axis([min(x) max(x) aa(3:4)]);
            
            subplot(212);
            plot(time,f);title('f(x)'); xlabel('Time');
            aa=axis; axis([min(time) max(time) aa(3:4)]);
        end
    end
    
    methods (Access = public)   % Not recommended functions
        % Set the weights
        function obj = assignWeights(obj,w)
            if size(w,1) ~= obj.N
                error('You must assign N weights');
            else
                obj.w = w;
            end
        end
        % Initialize Gaussians
        function obj = initGaussian(obj,h,c)
            %   Make sure the number of kernels equals N
            if size(h,1) ~= obj.N || size(c,1)~=obj.N
                error('You must assign N Gaussian kernels');
            else
                obj.hs = h;
                obj.cs = c;
            end
        end
    end
    
    methods (Access = private)
        psi = GaussianKernel(obj,x,c,h);
        fx = nonlinearTerm(obj,x);
        [x, dx] = canonicalSystem(obj);
        [y,dy,ddy,f] = dynamicSystem(obj,x);
        cs = GaussianCentersAssign(obj,x);
        [cs,hs] = GaussiansAssign(obj,x);
    end
    
end

