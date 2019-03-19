function [ fx ] = nonlinearTerm( obj,x )
%nonlinearTerm The nonlinear coupling term of dynamic system
%   x: M x 1
%   fx: M x D
%   @DMP

M = size(x,1);
N = obj.N;
h = obj.hs;
c = obj.cs;
g = obj.g;
y0 = obj.y0;
D = obj.D;
w = obj.w;

psis = ones(M,N);
fx = zeros(M,D);
for i = 1:N
    psis(:,i) = obj.GaussianKernel(x,c(i),h(i));
end
for i = 1:M
    fx(i,D) = ((sum(psis(i,:)*w)/sum(psis(i,:)))*x(i)*(g-y0))';
end

end

