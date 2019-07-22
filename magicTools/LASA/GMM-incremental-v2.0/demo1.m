function demo1
%
% Demonstration of an incremental learning process of Gaussian
% Mixture Model (GMM) using a direct update method.
% This source code is the implementation of the algorithms described in 
% Section 2.6.3, p.51 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% The program loads a dataset consisting of several trajectories 
% which are presented one-by-one to update the GMM parameters by using an
% incremental version of the Expectation-Maximization (EM)
% algorithm (direct update method). The learning mechanism only 
% uses the latest observed trajectory to update the models (no historical
% data is used).
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbStates = 3;
nbData = 100;
nbSamples = 6;
nbVar = 2;

%% Load data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/data.mat'); %load variable 'Data'

%% Initialization of the GMM parameters by k-means. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[M(1).Priors, M(1).Mu, M(1).Sigma] = EM_init_kmeans(Data(:,1:nbData), nbStates);
[M(1).Priors, M(1).Mu, M(1).Sigma, M(1).Pix] = EM_boundingCov(...
  Data(:,1:nbData), M(1).Priors, M(1).Mu, M(1).Sigma);

%% Update of the GMM parameters (direct computation method).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Recursive update
for i=2:nbSamples
  [M(i).Priors, M(i).Mu, M(i).Sigma, M(i).Pix] = EM_update_directMethod(...
    Data(:,(i-1)*nbData+1:i*nbData), M(i-1).Priors, M(i-1).Mu, M(i-1).Sigma, M(i-1).Pix);
end

%% Plot of the incremental update results.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,50,1000,600],'name','GMM-incremental-demo1');
for i=1:nbSamples
  subplot(2,3,i); hold on;
  plotGMM(M(i).Mu, M(i).Sigma, [0 .8 0], 1);
  plot(Data(1,1:i*nbData),Data(2,1:i*nbData), 'x', 'markerSize', 4, 'color', [.6 .9 .6]);
  plot(Data(1,(i-1)*nbData+1:i*nbData),Data(2,(i-1)*nbData+1:i*nbData), '-', 'lineWidth', 2, 'color', [0 0 0]);
  axis([1 nbData min(Data(2,:))-0.02 max(Data(2,:))+0.02]);
  title(['Demonstration ' num2str(i)],'fontsize',16);
  xlabel('t','fontsize',16); 
  ylabel('x_1','fontsize',16);
end

pause;
close all;
