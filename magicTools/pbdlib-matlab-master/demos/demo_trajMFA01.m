function demo_trajMFA01
% Trajectory model with either (see lines 79-81): 
%  -a mixture of factor analysers (MFA)
%  -a mixture of probabilistic principal component analyzers (MPPCA)
%  -a high-dimensional data clustering approach proposed by Bouveyron (HD-GMM)
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Calinon16JIST,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%		publisher="Springer Berlin Heidelberg",
%		doi="10.1007/s11370-015-0187-9",
%		year="2016",
%		volume="9",
%		number="1",
%		pages="1--29"
% }
% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 5; %Number of components in the mixture model
model.nbFA = 1; %Dimension of the subspace in each cluster
model.nbVarPos = 4; %Dimension of the position datapoint
model.nbDeriv = 3; %Nb derivatives+1 (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
model.nbVar = model.nbVarPos * model.nbDeriv;
model.dt = 1; %Time step (large values such as 1 will tend to create clusers by following position information)
nbData = 100; %Number of datapoints in a trajectory
nbSamples = 5; %Number of trajectory samples

[PHI,PHI1] = constructPHI(model,nbData,nbSamples); %Construct PHI operator (big sparse matrix)


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/G.mat'); %Load x1,x2 variables
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
end
demos=[];
load('data/2Dletters/C.mat'); %Load x3,x4 variables
Data=[];
for n=1:nbSamples
	s(n).Data = [s(n).Data; spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData))]; %Resampling
	Data = [Data s(n).Data]; 
end

%Re-arrange data in vector form
x = reshape(Data, model.nbVarPos*nbData*nbSamples, 1) * 1E2; %Scale data to avoid numerical computation problem
zeta = PHI*x; %y is for example [x1(1), x2(1), x1d(1), x2d(1), x1(2), x2(2), x1d(2), x2d(2), ...], see Eq. (2.4.5) in doc/TechnicalReport.pdf
Data = reshape(zeta, model.nbVarPos*model.nbDeriv, nbData*nbSamples); %Include derivatives in Data


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kmeans(Data, model);

%[model, GAMMA2] = EM_GMM(Data, model);
[model, GAMMA2] = EM_MFA(Data, model);
%[model, GAMMA2] = EM_MPPCA(Data, model);
%[model, GAMMA2] = EM_HDGMM(Data, model);


%% Reconstruction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute best path for the n-th demonstration
[~,r(1).q] = max(GAMMA2(:,1:nbData),[],1); %works also for nbStates=1

%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q, see Eq. (2.4.8) in doc/TechnicalReport.pdf
MuQ = reshape(model.Mu(:,r(1).q), model.nbVar*nbData, 1); 
%MuQ = zeros(model.nbVar*nbData,1);
SigmaQ = zeros(model.nbVar*nbData);
for t=1:nbData
	id = (t-1)*model.nbVar+1:t*model.nbVar;
	%MuQ(id) = model.Mu(:,r(n).q(t)); 
	SigmaQ(id,id) = model.Sigma(:,:,r(1).q(t));
end
%SigmaQ can alternatively be computed with:
%SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,r(1).q), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));

%Least squares computation
[zeta,~,mse,S] = lscov(PHI1, MuQ, SigmaQ, 'chol'); %Retrieval of data with weighted least squares solution
r(1).Data = reshape(zeta, model.nbVarPos, nbData); %Reshape data for plotting

%Rebuild covariance by reshaping S
for t=1:nbData
	id = (t-1)*model.nbVarPos+1:t*model.nbVarPos;
	r(1).expSigma(:,:,t) = S(id,id) * nbData;
end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,10,1200,600]);

subplot(1,2,1); hold on;
plotGMM(r(1).Data([1,2],:), r(1).expSigma([1,2],[1,2],:), [.5 .8 1], .2);
plotGMM(model.Mu(1:2,:),model.Sigma(1:2,1:2,:), [.8 0 0], .2);
plotGMM(model.Mu(1:2,:),model.P(1:2,1:2,:), [1 .4 .4], .2);
for i=1:model.nbStates
	plotGMM(model.Mu(1:2,i), model.L(1:2,:,i)*model.L(1:2,:,i)'+eye(2)*1E-1, [1 .4 .4]);
end
plot(Data(1,:), Data(2,:),'.','linewidth',2,'color',[0,0,0]);
plot(r(1).Data(1,:), r(1).Data(2,:),'-','linewidth',2,'color',[.8,0,0]);
axis equal;
xlabel('x_1','fontsize',16); ylabel('x_2','fontsize',16);

subplot(1,2,2); hold on;
plotGMM(r(1).Data([3,4],:), r(1).expSigma([3,4],[3,4],:), [.5 .8 1]);
plotGMM(model.Mu(3:4,:),model.Sigma(3:4,3:4,:), [.8 0 0], .2);
plotGMM(model.Mu(3:4,:),model.P(3:4,3:4,:), [1 .4 .4], .2);
for i=1:model.nbStates
	plotGMM(model.Mu(3:4,i), model.L(3:4,:,i)*model.L(3:4,:,i)'+eye(2)*1E-1, [1 .4 .4]);
end
plot(Data(3,:), Data(4,:),'.','linewidth',2,'color',[0,0,0]);
plot(r(1).Data(3,:), r(1).Data(4,:),'-','linewidth',2,'color',[.8,0,0]);
axis equal;
xlabel('x_3','fontsize',16); ylabel('x_4','fontsize',16);

%print('-dpng','graphs/demo_trajMFA01.png');
%pause;
%close all;
