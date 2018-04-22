function [ P ] = probability_assignment0121( X, T )
%PROBABILITY_ASSIGNMENT0121 a smooth function
% X: -\infty to +\infty
% P: 0 to 1
% T: temperature parameter >0

P = zeros(size(X));
for i = 1:size(X,1)
    if X(i)>=0
        P(i)=1-0.5*exp(-X(i)/T);
    else
        P(i) = 0.5*exp(X(i)/T);
    end
end

end

