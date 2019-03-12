function x = randdirichlet_unnorm(a)
% RANDDIRICHLET   Sample from Dirichlet distribution
%
% X = RANDDIRICHLET(A) returns a matrix, the same size as A, where X(:,j)
% is sampled from a Dirichlet(A(:,j)) distribution.

x = randgamma(a);
%Z = sum(x,1);
%x = x./Z(ones(size(a,1),1),:);
