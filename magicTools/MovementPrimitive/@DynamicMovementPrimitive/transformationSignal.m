function [ fzs, psiz ] = transformationSignal( obj,z )
%transformationSignal Show the trasformation signal
%   @DynamicMovementPrimitive

fzs = zeros(size(z));
psiz = zeros(size(z));
for i = 1:size(z,1)
    fzs(i)= obj.transformationFunc(z(i,1));
    psiz(i) = obj.psi.weighting(z(i,1));
end

end

