function [ c,h ] = GaussiansAssign( obj,x )
%GaussiansAssign Assign the Gaussians
%   @DMP2
%   c: N x 1, the centers of Gaussian kernels
%   h: N x 1, the variances of Gaussian kernels

M = size(x,1);
L = floor(M/(obj.N));
INDEX = zeros(M,1);
INDEX(1) = 1;
for i = 2:M-L
    if mod(i,L) == 0
        INDEX(i) = 1;
    end
end

c = x(INDEX==1);

h = (0.4*obj.N./c).^2;

end

