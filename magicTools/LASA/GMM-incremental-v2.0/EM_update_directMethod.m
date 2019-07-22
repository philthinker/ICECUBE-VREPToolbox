function [Priors, Mu, Sigma, Pix_tot] = EM_update(Data, Priors0, Mu0, Sigma0, Pix0)
%
% Parameters update of a Gaussian Mixture Model (GMM). 
% This source code is the implementation of the algorithms described in 
% Section 2.6.3, p.51 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% The process uses a recursive 
% Expectation-Maximization (EM) algorithm to update the model, without
% using the historical data used to create the model.
%
% Inputs -----------------------------------------------------------------
%   o Data:    D x N array representing the N novel datapoints of 
%              D dimensions.
%   o Priors0: 1 x K array representing the initial prior probabilities 
%              of the K GMM components.
%   o Mu0:     D x K array representing the initial centers of the K GMM 
%              components.
%   o Sigma0:  D x D x K array representing the initial covariance matrices 
%              of the K GMM components.
%   o Pix0:    1 x M array representing the posterior probabilities for the 
%              previous M datapoints used to create the model (note that
%              only the cumultated sum is used in the algorithm).
% Outputs ----------------------------------------------------------------
%   o Priors:  1 x K array representing the updtaed prior probabilities of 
%              the K GMM components.
%   o Mu:      D x K array representing the updated centers of the 
%              K GMM components.
%   o Sigma:   D x D x K array representing the updated covariance matrices
%              of the K GMM components.
%   o Pix_tot: 1 x (M+N) array representing the posterior probabilities  
%              for the previous M datapoints and for the novel N datapoints 
%              used to create the model.
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

%% Criterion to stop the EM iterative update
loglik_threshold = 1e-10;
nbData0 = size(Pix0,1);
[nbVar,nbData] = size(Data);
nbStates = size(Sigma0,3);
loglik_old = -realmax;
nbStep = 0;
Mu = Mu0;
Sigma = Sigma0;
Priors = Priors0;
%% Compute cumulated posterior probability
E0 = sum(Pix0);

%% EM update algorithm
while 1
  %% E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Compute probability p(x|i)
    Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
  end
  %Compute posterior probability p(i|x)
  for j=1:nbData
    Pix(j,:) = (Priors.*Pxi(j,:))./(sum(Priors.*Pxi(j,:))+realmin);
  end
  %Compute cumulated posterior probability
  E = sum(Pix,1);
  %% M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Update the priors
    Priors(i) = (E0(i)+E(i)) / (nbData0+nbData);
    %Update the centers
    Mu(:,i) = (Mu0(:,i).*E0(i) + Data*Pix(:,i)) / (E0(i)+E(i));
    %Update the covariance matrices
    covtmp = zeros(nbVar,nbVar);
    for j=1:nbData
      covtmp = covtmp + (Data(:,j)-Mu(:,i))*(Data(:,j)-Mu(:,i))'.*Pix(j,i);
    end
    Sigma(:,:,i) = ((Sigma0(:,:,i)+(Mu0(:,i)-Mu(:,i))*(Mu0(:,i)-Mu(:,i))').*E0(i) + covtmp) / (E0(i)+E(i));
  end
  %% Stopping criterion %%%%%%%%%%%%%%%%%%%%
  for i=1:nbStates
    %Compute the new probability p(x|i)
    Pxi(:,i) = gaussPDF(Data, Mu(:,i), Sigma(:,:,i));
  end
  %Compute the log likelihood
  F = Pxi*Priors';
  F(find(F<realmin)) = realmin;
  loglik = mean(log(F));
  %Stop the process depending on the increase of the log likelihood 
  if abs((loglik/loglik_old)-1) < loglik_threshold
    break;
  end
  loglik_old = loglik;
  nbStep = nbStep+1;
end

%% Update posterior probability vector
Pix_tot = [Pix0; Pix];

%% Add a tiny variance to avoid numerical instability
for i=1:nbStates
  Sigma(:,:,i) = Sigma(:,:,i) + 1E-10.*diag(ones(nbVar,1));
end
