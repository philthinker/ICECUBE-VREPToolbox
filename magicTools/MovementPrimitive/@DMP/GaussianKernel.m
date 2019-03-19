function [ psi ] = GaussianKernel( obj,x,c,h )
%GaussianKernel The Gaussian kernel
%   x: M x 1;
%   psi: M x 1;
%   @DMP

% psi = zeros(size(x,1),1);
psi = exp(-h*(x-c).^2);


end

