function [Mu, Sigma, C] = gaussProduct(Mu1, Sigma1, Mu2, Sigma2)
%
% This function computes the product of two multivariate Gaussian 
% distributions represented by means and covariance matrices.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% Inputs -----------------------------------------------------------------
%   o Mu1:    D x 1 array representing the center of the first Gaussian
%             distribution.
%   o Sigma1: D x D array representing the covariance matrix of the 
%             first Gaussian distribution.
%   o Mu2:    D x 1 array representing the center of the second Gaussian
%             distribution.
%   o Sigma2: D x D array representing the covariance matrix of the 
%             second Gaussian distribution.
% Outputs ----------------------------------------------------------------
%   o Mu:     D x 1 array representing the center of the resulting 
%             Gaussian distribution.
%   o Sigma:  D x D array representing the covariance matrix of the 
%             resulting Gaussian distribution.
%   o C:      Scaling factor of the resulting multiplication of the two
%             Gaussian distributions.

C = gaussPDF(Mu1, Mu2, Sigma1+Sigma2);
Mu = inv(inv(Sigma1)+inv(Sigma2)) * (inv(Sigma1)*Mu1+inv(Sigma2)*Mu2);
Sigma = inv(inv(Sigma1)+inv(Sigma2));