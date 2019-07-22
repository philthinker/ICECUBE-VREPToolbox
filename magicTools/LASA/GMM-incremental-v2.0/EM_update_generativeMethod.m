function [Priors, Mu, Sigma, alpha] = EM_update_generativeMethod(Data, Priors0, Mu0, Sigma0, nbData, nbTrainingSamples, alpha)
%
% Parameters update of a Gaussian Mixture Model (GMM) using a generative method.
% This source code is the implementation of the algorithms described in 
% Section 2.6.3, p.51 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
% 
% The  function generates stochastically a new dataset from the current 
% model, add the newly observed trajectory to this dataset and update 
% the GMM parameters using the resulting dataset, through a standard
% Expectation-Maximization (EM) algorithm (generative method).
%
% Inputs -----------------------------------------------------------------
%   o Data:               D x N array representing the N novel datapoints 
%                         of D dimensions.
%   o Priors0:            1 x K array representing the initial prior 
%                         probabilities of the K GMM components.
%   o Mu0:                D x K array representing the initial centers of 
%                         the K GMM components.
%   o Sigma0:             D x D x K array representing the initial 
%                         covariance matrices of the K GMM components.
%   o nbData:             Number of data point in a trajectory.
%   o nbTrainingSamples:  Total number of samples used by the generative
%                         process.
%   o alpha:              Recursive learning rate.
%
% Outputs ----------------------------------------------------------------
%   o Priors:  1 x K array representing the updtaed prior probabilities of 
%              the K GMM components.
%   o Mu:      D x K array representing the updated centers of the 
%              K GMM components.
%   o Sigma:   D x D x K array representing the updated covariance matrices
%              of the K GMM components.
%   o alpha:   Updated recursive learning rate.
%
% This source code is given for free! However, I would be grateful if you refer 
% to the book (or corresponding article) in any academic publication that uses 
% this code or part of it. Here are the corresponding BibTex references: 
%
% @book{Calinon09book,
%   author="S. Calinon",
%   title="Robot Programming by Demonstration: A Probabilistic Approach",
%   publisher="EPFL/CRC Press",
%   year="2009",
%   note="EPFL Press ISBN 978-2-940222-31-5, CRC Press ISBN 978-1-4398-0867-2"
% }
% 
% @inproceedings{Calinon07HRI,
%   author = "S. Calinon and A. Billard",
%   title = "Incremental Learning of Gestures by Imitation in a Humanoid Robot",
%   booktitle = "Proc. of the {ACM/IEEE} Intl Conf. on Human-Robot Interaction ({HRI})",
%   year = "2007",
%   month="March",
%   location="Arlington, VA, USA",
%   pages="255--262"
% }

%% Initialization of the parameters
nbVar = size(Data,1);
nbStates = size(Sigma0,3);

%% Update the recursive learning rate
alpha = alpha/(alpha+1);

%% Compute the number of observed samples and stochastically generated
%% samples
nbDemoData = round(nbTrainingSamples*alpha);
nbStochData = nbTrainingSamples-nbDemoData;

%% Compute a GMR model from the GMM representation
temporalData = Data(1,1:nbData);
[y, Sigma_y] = GMR(Priors0, Mu0, Sigma0, temporalData, [1], [2:nbVar]);

%% Retrieve stochastic samples from the GMR model
for j=1:nbStochData
  for i=1:length(temporalData)
    datatmp = gaussSampling(y(:,i), Sigma_y(:,:,i), 1);
    stochData(:,(j-1)*length(temporalData)+i) = [temporalData(i) datatmp];
  end
end

%% Build the new training set
demoData = repmat(Data,1,nbDemoData);
TrainData = [stochData demoData];

%% Update the GMM parameters
[Priors, Mu, Sigma] = EM_boundingCov(TrainData, Priors0, Mu0, Sigma0);
