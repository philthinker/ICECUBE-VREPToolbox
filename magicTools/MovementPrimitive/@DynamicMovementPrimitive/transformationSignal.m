function [ fzs ] = transformationSignal( obj,z )
%transformationSignal Show the trasformation signal
%   @DynamicMovementPrimitive

fzs = zeros(size(z));
for i = 1:size(z,1)
    fzs(i,1)= obj.transformationFunc(z(i,1));
end

end

