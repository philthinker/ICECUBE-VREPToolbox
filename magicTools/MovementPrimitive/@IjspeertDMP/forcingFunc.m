function [ fx ] = forcingFunc( obj,x )
%forceingFunc The forcing 
%   @SchaalDMP
%   fx: M x 1

M = size(x,1);
N = obj.N;
h = obj.h;
c = obj.c;
% y0 = obj.y0;
% g = obj.g;
w = obj.w;


psis = ones(M,N);

for i = 1:N
%     psi = obj.GaussianBasis(x,c(i),h(i));
%     psis(:,i) = psi;
    psis(:,i) = obj.GaussianBasis(x,c(i),h(i));
end

% Psis = (psis*w)./(psis*ones(N,1));
% fx = diag(Psis)*x*(g-y0);

fx = zeros(M,1);
for i = 1:M
    fx(i) = ((sum(psis(i,:)*w)/sum(psis(i,:)))*x(i));
end

end

