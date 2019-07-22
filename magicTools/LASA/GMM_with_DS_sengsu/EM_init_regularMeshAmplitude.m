function [Priors, Mu, Sigma] = EM_init_regularMeshAmplitude(Data, nbData, nbStates)
%
% This function initializes the parameters of a Gaussian Mixture Model 
% (GMM) by using k-means clustering algorithm.
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
% Comments ---------------------------------------------------------------
%   o This function uses the 'kmeans' function from the MATLAB Statistics 
%     toolbox. If you are using a version of the 'netlab' toolbox that also
%     uses a function named 'kmeans', please rename the netlab function to
%     'kmeans_netlab.m' to avoid conflicts. 
%
% Copyright (c) 2006 Sylvain Calinon, LASA Lab, EPFL, CH-1015 Lausanne,
%               Switzerland, http://lasa.epfl.ch

%TimingSep = linspace(min(Data(1,:)), max(Data(1,:))/2-100, ceil(3*nbStates/4));
%TimingSep = [TimingSep linspace(max(Data(1,:))/2, max(Data(1,:)), ceil(nbStates/4))];

%% %%%%%Compute Lengthes of all Trajectories%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbSamples = size(nbData, 2);

LengthData = zeros(1, nbSamples);
ind = 0;
for (i = 1: nbSamples)
    for (j = 1 : nbData(1, i) - 1)
        ind = ind + 1;
        LengthData(1, i) = LengthData(1, i) + norm(Data(:, ind + 1) - Data(:, ind));
    end;
    ind = ind + 1;
end;

ind = 0;
for (i = 1: nbSamples)
    for (j = 1 : nbData(1, i))
        
        ind = ind + 1;
        
        if (i == 1) && (j == 1)
            Index(1, ind) = 0;
        end;
        
        if (i > 1) && (j == 1)
            Index(1, ind) = LengthData(1, 1) - LengthData(1, i);
        end;
        
        if (j > 1)
            Index(1, ind) = Index(1, ind - 1) + norm(Data(:, ind) - Data(:, ind - 1));
        end;
        
    end;
end;
%% %%%%Assume that first trajectory starts from %%%%%%%%%%%%%%%%%
%% zero, if then the length of the other trajectory is bigger, %%
%% then it starts from negative value, otherwise, from positive%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TimingSep = linspace(min(Index), max(Index), nbStates+1);
Data = [Index; Data];
for i=1:size(TimingSep, 2)-1
  idtmp = find(Data(1,:) >= TimingSep(i) & Data(1,:) < TimingSep(i+1));
  LengthScale(1, i) = size(idtmp, 2) ./ abs(TimingSep(i+1) - TimingSep(i));
  Priors(i) = length(idtmp);
  if (size(idtmp, 2) < 2)
      disp 'ATTENTION: degenerated cluster [less than 3 datapoints in a cluster]';
  end;
  Mu(:,i) = mean(Data(2:end,idtmp)', 1);
  Sigma(:,:,i) = cov(Data(2:end,idtmp)')+ 1E-8.*diag(ones(size(Data, 1)-1,1));
end;


Priors = Priors ./ sum(Priors);


