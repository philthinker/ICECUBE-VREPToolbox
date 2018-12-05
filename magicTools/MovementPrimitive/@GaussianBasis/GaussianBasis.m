classdef GaussianBasis
    %GaussianBasis The Gaussian basis function for DMP
    %   exp(-h(z-c)^2)/SUM
    
    properties (Access = public)
        h;      % variances
        c;      % centers
        N;      % number
    end
    
    methods
        function obj = GaussianBasis(h,c)
            obj.N = size(h,1);
            if obj.N >size(c,1)
                obj.N = size(c,1);
            end
            obj.h = h(1:obj.N,1);
            obj.c = c(1:obj.N,1);
        end
        function weights = weighting(obj,z)
            % Note that here z is a scalar
            exps = exp(-obj.h.*(z-obj.c).^2);
            weights = exps/sum(exps);
        end
        function N = number(obj)
            N = obj.N;
        end
    end
    
end

