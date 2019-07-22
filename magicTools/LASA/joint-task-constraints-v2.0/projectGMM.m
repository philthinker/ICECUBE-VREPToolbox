function [Mu2, Sigma2] = projectGMM(Mu1, Sigma1, A)
%
% This function computes the linear projection of Gaussian distributions
% through a transformation matrix.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% Inputs -----------------------------------------------------------------
%   o Mu1:    D x K array representing the centers of the Gaussian
%             distributions.
%   o Sigma1: D x D x K array representing the covariance matrices of the 
%             Gaussian distributions.
%   o A:      D x d array representing the linear transformation matrix.
% Outputs ----------------------------------------------------------------
%   o Mu2:    d x K array representing the centers of the resulting 
%             Gaussian distributions.
%   o Sigma2: d x d x K array representing the covariance matrices of the 
%             resulting Gaussian distributions.

nbStates = size(Sigma1,3);
[nbVar,nbVar2] = size(A);

%Project the centers of the Gaussian distributions
Mu2 = A * Mu1;

%Project the covariance matrices of the Gaussian distributions
for i=1:nbStates
  Sigma2(:,:,i) = A * Sigma1(:,:,i) * A';
  %Add a tiny variance to avoid numerical instability
  Sigma2(:,:,i) = Sigma2(:,:,i) + 1E-10.*diag(ones(nbVar,1));
end
