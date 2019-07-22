function Data = gaussSampling(Mu, Sigma, nbSamples)
%
% This function generates stochastic samples from a Gaussian distribution.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
% 
% Inputs -----------------------------------------------------------------
%   o Mu:         D x 1 array representing the center of the Gaussian 
%                 distribution.
%   o Sigma:      D x D array representing the covariance matriData of the 
%                 Gaussian distribution.
%   o nbSamples:  Number of samples to generate.
% Output -----------------------------------------------------------------
%   o Data:       D x nbSamples array representing the data generated
%                 stochastically from the Gaussian distribution.
%

D = size(Sigma, 1);
Mu = reshape(Mu, 1, D);   % Ensure that Mu is a row vector

[evec, eval] = eig(Sigma);
deig = diag(eval);

if (~isreal(deig)) | any(deig<0), 
  warning('Warning: Sigmaiance MatriData has been redefined to be positive definite');
  eval = abs(eval);
end

coeffs = randn(nbSamples,D) * sqrt(eval);
Data = ones(nbSamples,1) * Mu + coeffs * evec';
