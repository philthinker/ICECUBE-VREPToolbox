function demo1
%
% Reproduction of a generalized trajectory through 
% Gaussian Mixture Regression (GMR) when considering two independent 
% constraints represented separately in two Gaussian Mixture Models 
% (GMMs). 
% This source code is the implementation of the algorithms described in 
% Section 2.5, p.44 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% Through regression, a smooth generalized trajectory satisfying 
% the constraints encapsulated in both GMMs is extracted, with associated
% constraints represented as covariance matrices.
% The program loads two datasets, which are encoded separetely in two 
% GMMs. GMR is then performed separately on the two datasets, and the 
% resulting Gaussian distributions at each time step are multiplied to 
% find an optimal controller satisfying both constraints, producing a 
% smooth generalized trajectory across the two datasets.
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

%% Initialization of the parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbStates = 3;
nbVar = 2;
nbData = 100;

%% Load data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/data1.mat'); %Load variable 'Data1'
load('data/data2.mat'); %Load variable 'Data2'

%% Training of GMM by EM algorithm, initialized by k-means clustering.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Training for GMM1
[Priors1, Mu1, Sigma1] = EM_init_kmeans(Data1, nbStates);
[Priors1, Mu1, Sigma1] = EM_boundingCov(Data1, Priors1, Mu1, Sigma1);
%Training for GMM2
[Priors2, Mu2, Sigma2] = EM_init_kmeans(Data2, nbStates);
[Priors2, Mu2, Sigma2] = EM_boundingCov(Data2, Priors2, Mu2, Sigma2);

%% Use of GMR to retrieve a generalized version of the data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Regression for GMM1
expData1(1,:) = linspace(min(Data1(1,:)), max(Data1(1,:)), 100);
[expData1(2:nbVar,:), expSigma1] = GMR(Priors1, Mu1, Sigma1,  expData1(1,:), [1], [2:nbVar]);
%Regression for GMM2
expData2(1,:) = linspace(min(Data2(1,:)), max(Data2(1,:)), 100);
[expData2(2:nbVar,:), expSigma2] = GMR(Priors2, Mu2, Sigma2,  expData2(1,:), [1], [2:nbVar]);

%% Creation of an optimal controller by multiplying the Gaussian densities
%% extracted by GMR1 and GMR2.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:nbData
  [expData(:,i) expSigma(:,:,i)] = gaussProduct(expData1(:,i), ...
    expSigma1(:,:,i), expData2(:,i), expSigma2(:,:,i));
end

%% Plot model and regression for Data1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,50,500,600],'name','GMM-GMR-demo1');
%Plot Data1
subplot(5,2,1); hold on;
plot(Data1(1,:),Data1(2,:),'.','color',[.5 .5 .5]);
axis([1 100 min([Data1(2,:) Data2(2,:)])-0.02 max([Data1(2,:) Data2(2,:)])+0.02]);
xlabel('t','fontsize',16); ylabel('x^{(1)}','fontsize',16);
%Plot GMM1
subplot(5,2,3); hold on;
plotGMM2(Mu1, Sigma1, [.8 0 0], 1);
axis([1 100 min([Data1(2,:) Data2(2,:)])-0.02 max([Data1(2,:) Data2(2,:)])+0.02]);
xlabel('t','fontsize',16); ylabel('x^{(1)}','fontsize',16);
%Plot GMR1
subplot(5,2,5); hold on;
plotGMM2(expData1, expSigma1, [.8 0 0], 3);
axis([1 100 min([Data1(2,:) Data2(2,:)])-0.02 max([Data1(2,:) Data2(2,:)])+0.02]);
xlabel('t','fontsize',16); ylabel('x^{(1)}','fontsize',16);

%% Plot model and regression for Data2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot Data2
subplot(5,2,2); hold on;
plot(Data2(1,:),Data2(2,:),'.','color',[.5 .5 .5]);
axis([1 100 min([Data1(2,:) Data2(2,:)])-0.02 max([Data1(2,:) Data2(2,:)])+0.02]);
xlabel('t','fontsize',16); ylabel('x^{(2)}','fontsize',16);
%Plot GMM2
subplot(5,2,4); hold on;
plotGMM2(Mu2, Sigma2, [0 .8 0], 1);
axis([1 100 min([Data1(2,:) Data2(2,:)])-0.02 max([Data1(2,:) Data2(2,:)])+0.02]);
xlabel('t','fontsize',16); ylabel('x^{(2)}','fontsize',16);
%Plot GMR2
subplot(5,2,6); hold on;
plotGMM2(expData2, expSigma2, [0 .8 0], 3);
axis([1 100 min([Data1(2,:) Data2(2,:)])-0.02 max([Data1(2,:) Data2(2,:)])+0.02]);
xlabel('t','fontsize',16); ylabel('x^{(2)}','fontsize',16);

%% Plot resulting GMR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(5,2,[7:10]); hold on;
%Plot GMR1
plotGMM2(expData1, expSigma1, [.8 0 0], 4);
%Plot GMR2
plotGMM2(expData2, expSigma2, [0 .8 0], 4);
%Plot GMR
plotGMM2(expData, expSigma, [0 0 .8], 3);
axis([1 100 min([Data1(2,:) Data2(2,:)])-0.02 max([Data1(2,:) Data2(2,:)])+0.02]);
xlabel('t','fontsize',16); ylabel('x','fontsize',16);

pause;
close all;
