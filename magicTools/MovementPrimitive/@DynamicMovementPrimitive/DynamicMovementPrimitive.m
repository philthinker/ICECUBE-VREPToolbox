classdef DynamicMovementPrimitive
    %DynamicMovementPrimitive Ijspeert's dynamic movement primitive
    %   dot_z = -tau*alpha*z
    %   dot_x_1 = tau*x_2
    %   dot_x_2 = tau*(alpha_x(beta_x(g-x_1)-x_2)+tau*A*f(z)
    
    properties (Access = public)
        tau;        % Time constant
        alpha_z;    % alpha of canonical system
        alpha_x;    % alpha of transformed system
        beta_x;     % beta of transformed system
        goal;       % The goal
        N;          % The number of weighting function
        w;          % Adjustable params.
        A;          % Gains
        psi;        % The Gaussian basis function
    end
    
    methods
        function obj = DynamicMovementPrimitive(alpha_z,alpha_x,beta_x,psi)
            obj.alpha_x = alpha_x;
            obj.alpha_z = alpha_z;
            obj.beta_x = beta_x;
            obj.psi = psi;
            obj.N = psi.number();
            obj.w = ones(obj.N,1);
            obj.goal = 0;
            obj.A = 10;
            obj.tau = 1;
        end
    end
    
    methods (Access = public)
        fz = transformationFunc(obj,z);
        [fzs,psiz] = transformationSignal(obj,z);
        obj = LWLRLearn(obj,data);
    end
    
end
