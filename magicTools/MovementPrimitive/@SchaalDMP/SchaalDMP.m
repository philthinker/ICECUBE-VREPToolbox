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
            [obj.c, obj.h] = obj.GaussianAssign(x);
        end
    end
    
    methods (Access = private)
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
