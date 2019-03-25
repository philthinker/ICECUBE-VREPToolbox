function [ psi ] = GaussianBasis( obj,x,c,h )
%GaussianBasis The Gaussian basis
%   @SchaalDMP
%   x: M x 1
%   c, h, scalers


psi = exp(-h*(x-c).^2);

end

