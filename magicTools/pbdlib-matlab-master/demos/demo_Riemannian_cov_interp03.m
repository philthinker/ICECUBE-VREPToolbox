function demo_Riemannian_cov_interp03
% Trajectory morphing through covariance interpolation on Riemannian manifold (with augmented Gaussian trajectory distribution) 
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please cite the related publications.
%
% @article{Jaquier17IROS,
%   author="Jaquier, N. and Calinon, S.",
%   title="Gaussian Mixture Regression on Symmetric Positive Definite Matrices Manifolds: 
%	    Application to Wrist Motion Estimation with s{EMG}",
%   year="2017",
%	  booktitle = "{IEEE/RSJ} Intl. Conf. on Intelligent Robots and Systems ({IROS})",
%	  address = "Vancouver, Canada"
% }
% 
% Copyright (c) 2017 Sylvain Calinon, Idiap Research Institute, http://idiap.ch/
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 2; %Number of states in the GMM
model.nbVar = 2; %Number of variables [t,x1,x2]
nbData = 20; %Length of each trajectory
nbInterp = 50; %Length of each trajectory
nbSamples = 5; %Number of demonstrations


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/S.mat');
for n=1:nbSamples
	DataTmp = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data(:,n,1) = reshape(DataTmp, model.nbVar*nbData, 1); 
end
demos=[];
load('data/2Dletters/N.mat');
for n=1:nbSamples
	DataTmp = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data(:,n,2) = reshape(DataTmp+20, model.nbVar*nbData, 1); 
end


%% Compute normal trajectory distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbVar = model.nbVar*nbData;
for i=1:model.nbStates
	model.Mu(:,i) = mean(Data(:,:,i),2); 
	model.Sigma(:,:,i) = cov(Data(:,:,i)') * 1E0 + eye(size(Data,1)) * 1E-5;
end

%Initialisation of model2 (Gaussians with augmented covariances centered on zero)
model2 = model;
model2.nbVar = model.nbVar+1;
model2.Mu = zeros(model2.nbVar, model2.nbStates);
model2.Sigma = zeros(model2.nbVar, model2.nbVar, model2.nbStates);
for i=1:model.nbStates
	model2.Sigma(:,:,i) = [model.Sigma(:,:,i)+model.Mu(:,i)*model.Mu(:,i)', model.Mu(:,i); model.Mu(:,i)', 1];
end


%% Geodesic interpolation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w = linspace(0,1,nbInterp); %standard interpolation

% wi = [linspace(1,0,nbInterp); linspace(0,1,nbInterp)];
% S = model2.Sigma(:,:,1);

Sigma2 = zeros(model2.nbVar, model2.nbVar, nbInterp*(model.nbStates-1));
for i=2:model.nbStates
	for t=1:nbInterp
% 		%Interpolation between more than 2 covariances can be computed in an iterative form
% 		nbIter = 10; %Number of iterations for the convergence of Riemannian estimate
% 		for n=1:nbIter
% 			W = zeros(model2.nbVar);
% 			for j=1:model.nbStates
% 				W = W + wi(j,t) * logmap(model2.Sigma(:,:,j), S);
% 			end
% 			S = expmap(W,S);
% 		end
% 		Sigma2(:,:,(i-2)*nbInterp+t) = S;

		%Interpolation between two covariances can be computed in closed form
		Sigma2(:,:,(i-2)*nbInterp+t) = expmap(w(t)*logmap(model2.Sigma(:,:,i), model2.Sigma(:,:,i-1)), model2.Sigma(:,:,i-1));
	end	
end

for t=1:size(Sigma2,3)
	beta = Sigma2(end,end,t);
	Mu(:,t) = Sigma2(end,1:end-1,t) ./ beta;
	Sigma(:,:,t) = Sigma2(1:end-1,1:end-1,t) - beta * Mu(:,t)*Mu(:,t)';
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,650,650]); hold on; axis off;
for i=1:model.nbStates
	for n=1:nbSamples
		plot(Data(1:2:end,n,i), Data(2:2:end,n,i), '-','linewidth',1,'color',[.5 .5 .5]);
	end
	plot(model.Mu(1:2:end,i), model.Mu(2:2:end,i), '-','linewidth',1,'color',[0 0 0]);
end
for t=1:nbInterp
	plot(Mu(1:2:end,t), Mu(2:2:end,t), '-','linewidth',1,'color',[.8 0 0]);
end
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%print('-dpng','graphs/demo_Riemannian_cov_interp03.png');

% pause;
% close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X = expmap(U,S)
	X = S^.5 * expm(S^-.5 * U * S^-.5) * S^.5;
end

function U = logmap(X,S)
	U = S^.5 * logm(S^-.5 * X * S^-.5) * S^.5;
end

