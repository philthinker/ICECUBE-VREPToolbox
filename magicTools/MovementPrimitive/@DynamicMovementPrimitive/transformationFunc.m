function [ fz ] = transformationFunc( obj,z )
%transformationFunc The transfermation function
%   @DynamicMovementPrimitive

% Note that here z is a scalar
fz = (obj.psi.weighting(z))'*obj.w*z;

end

