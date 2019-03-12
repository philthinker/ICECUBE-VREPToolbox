%function [X,valid] = makeDesignMatrix(Y,order)
% Y = d x T
% X = order*d x T
%
% X = [0 Y(:,1:(end-1); 0 0 Y(:,1:(end-2)); 0 0 0 Y...etc]
%
% valid(t) = 1 for all X(:,t) where zeros were not inserted
%
function [X,valid]= makeDesignMatrix(Y,order)

d = size(Y,1);
T = size(Y,2);

X = zeros(order*d,T);

for lag=1:order
  ii   = d*(lag-1)+1;
  indx = ii:(ii+d-1);
  X(indx, :) = [zeros(d,min(lag,T)) Y(:,1:(T-min(lag,T)))]; 
end

if nargout > 1
  valid = ones(1,T);
  valid(1:order) = 0;
end
