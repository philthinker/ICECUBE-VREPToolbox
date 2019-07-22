function demo1
%
% Encoding and retrieval of motion in a latent space of reduced dimensionality.
% This source code is the implementation of the algorithms described in 
% Section 2.7.1, p.55 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% This program loads a dataset, finds a latent space of lower dimensionality
% encapsulating the important characteristics of thge motion using 
% Principal Component Analysis (PCA), trains a Gaussian Mixture Model (GMM) 
% using the data projected in this latent space, re-projects the Gaussian 
% in the original data space, and plots the result. Training a GMM with 
% EM algorithm usually fails to find a good local optimum when data are
% high-dimensional. By projecting the original dataset in a latent space 
% as a pre-processing step, GMM training can be performed in a robust way,
% and the Gaussian parameters can be projected back to the orginal data
% space.
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
% @article{Calinon07,
%   title="On Learning, Representing and Generalizing a Task in a Humanoid Robot",
%   author="S. Calinon and F. Guenter and A. Billard",
%   journal="IEEE Transactions on Systems, Man and Cybernetics, Part B",
%   year="2007",
%   volume="37",
%   number="2",
%   pages="286--298",
% }

%% Definition of the number of components used in GMM and the number of 
%% principal components.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbStates = 3;
nbPC = 2;

%% Load a dataset consisting of 3 demonstrations of a 4D signal 
%% (3D spatial components + 1D temporal component).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/data.mat');
[nbVar,nbData] = size(Data);

%% Projection of the data in a latent space using PCA.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Re-center the data
Data_mean = repmat(mean(Data(2:end,:),2), 1, nbData);
centeredData = Data(2:end,:) - Data_mean;
%Extract the eigencomponents of the covariance matrix 
[E,v] = eig(cov(centeredData'));
E = fliplr(E);
%Compute the transformation matrix by keeping the first nbPC eigenvectors
A = E(:,1:nbPC);
%Project the data in the latent space
nbVar2 = nbPC+1;
Data2(1,:) = Data(1,:);
Data2(2:nbVar2,:) = A' * centeredData;


%% Training of GMM by EM algorithm, initialized by k-means clustering.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[Priors0, Mu0, Sigma0] = EM_init_kmeans(Data2, nbStates);
[Priors2, Mu2, Sigma2] = EM(Data2, Priors0, Mu0, Sigma0);

%% Re-project the GMM components in the original data space.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Project the centers of the Gaussian distributions, using only the spatial 
%components 
Mu(1,:) = Mu2(1,:);
Mu(2:nbVar,:) = A*Mu2(2:end,:) + Data_mean(:,1:nbStates);
%Project the covariance matrices, using only the spatial components
for i=1:nbStates
  A_tmp = [1 zeros(1,nbVar2-1); zeros(nbVar-1,1) A];
  Sigma(:,:,i) = A_tmp * Sigma2(:,:,i) * A_tmp';
  %Add a tiny variance to avoid numerical instability
  Sigma(:,:,i) = Sigma(:,:,i) + 1E-10.*diag(ones(nbVar,1));
end

%% Plot of the GMM encoding results.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[50,50,1000,400]);
%plot 1D original data space
for n=1:nbVar-1
  subplot(nbVar-1,2,(n-1)*2+1); hold on;
  plotGMM(Mu([1,n+1],:), Sigma([1,n+1],[1,n+1],:), [0 .8 0], 1);
  plot(Data(1,:), Data(n+1,:), 'x', 'markerSize', 4, 'color', [.3 .3 .3]);
  axis([min(Data(1,:)) max(Data(1,:)) min(Data(n+1,:))-0.01 max(Data(n+1,:))+0.01]);
  xlabel('t','fontsize',16); ylabel(['x_' num2str(n)],'fontsize',16);
end
%plot 1D latent space
for n=1:nbVar2-1
  subplot(nbVar-1,2,(n-1)*2+2); hold on;
  plotGMM(Mu2([1,n+1],:), Sigma2([1,n+1],[1,n+1],:), [.8 0 0], 1);
  plot(Data2(1,:), Data2(n+1,:), 'x', 'markerSize', 4, 'color', [.3 .3 .3]);
  axis([min(Data2(1,:)) max(Data2(1,:)) min(Data2(n+1,:))-0.01 max(Data2(n+1,:))+0.01]);
  xlabel('t','fontsize',16); ylabel(['\xi_' num2str(n)],'fontsize',16);
end
print('-depsc2','data/GMM-latentSpace-graph01.eps');

pause;
close all;
