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
        g;          % goal
        y0;         % initial position
        alphax;     % alpha of the canonical system
        N;          % number of the Gaussian kernels
        D;          % dimension
        tau;        % duration
        hs;         % variances of the Gaussian kernels
        cs;         % means of the Gaussian kernels
        w;          % weighting params. to be learned/reinforced
        dt;         % the time step
    end
    
    methods
        % Initialization
        function obj = DMP(alpha, beta, alphax, N)
            %   Dimension of alpha and beta must coincide
            obj.D = min(size(alpha,1),size(beta,1));
            %   Params. initialization
            obj.alpha = alpha(1:obj.D,1);
            obj.beta = beta(1:obj.D,1);
            obj.alphax = alphax(1,1);
            obj.g = zeros(obj.D,1);
            obj.y0 = zeros(obj.D,1);
            
            obj.N = max(ceil(N),1);
            obj.tau = 1;
            obj.hs = ones(obj.N,1);
            obj.cs = ones(obj.N,1);
            obj.w = zeros(obj.N,1);
            
            obj.dt = 0.001;
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
        function obj = batchLearn(obj,y)
        end
        % Incremental Learning
        function obj = incrementalLearn(obj,y)
        end
        % Run DMP dynamics
        function [y,dy,ddy,x,dx] = run(obj)
            [x,dx] = obj.canonicalSystem();
            [y,dy,ddy] = obj.dynamicSystem(x);
        end
        % Plot
        function [] = plot(obj,x,y,dy,ddy)
            
        end
    end
    
    methods (Access = private)
        psi = GaussianKernel(obj,x);
        fx = nonlinearTerm(obj,x);
        [x, dx] = canonicalSystem(obj);
        [y,dy,ddy] = dynamicSystem(obj);
    end
    
end

