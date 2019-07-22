function [Priors, Mu, Sigma] = EM_init_regularTiming(Data, nbStates)
%
% This function initializes the parameters of a Gaussian Mixture Model 
% (GMM) by segmenting the data into regular clusters in time.
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
% 
% Inputs -----------------------------------------------------------------
%   o Data:     D x N array representing N datapoints of D dimensions.
%   o nbStates: Number K of GMM components.
% Outputs ----------------------------------------------------------------
%   o Priors:   1 x K array representing the prior probabilities of the
%               K GMM components.
%   o Mu:       D x K array representing the centers of the K GMM components.
%   o Sigma:    D x D x K array representing the covariance matrices of the 
%               K GMM components.

TimingSep = linspace(min(Data(1,:)), max(Data(1,:)), nbStates+1);

for i=1:nbStates
  idtmp = find( Data(1,:)>=TimingSep(i) & Data(1,:)<TimingSep(i+1));
  Priors(i) = length(idtmp);
  Mu(:,i) = mean(Data(:,idtmp)');
  Sigma(:,:,i) = cov(Data(:,idtmp)');
end
Priors = Priors ./ sum(Priors);


