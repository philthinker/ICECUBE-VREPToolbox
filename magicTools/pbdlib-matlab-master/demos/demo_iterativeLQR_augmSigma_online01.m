function demo_iterativeLQR_augmSigma_online01
% Iterative LQR with augmented covariance to transform the tracking problem to a 
% regulation problem, recomputed in an online manner.
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
% Copyright (c) 2016 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon (http://calinon.ch/) and Danilo Bruno (danilo.bruno@iit.it)
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
nbSamples = 5; %Number of demonstrations
nbRepros = 1; %Number of reproductions
nbData = 100; %Number of datapoints
nbD = 20; %Time window for LQR computation

model.nbStates = 5; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.dt = 0.01; %Time step duration
model.rfactor = 1E-6;	%Control cost in LQR

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;


%% Dynamical System settings (discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Integration with higher order Taylor series expansion
A1d = zeros(model.nbDeriv);
for i=0:model.nbDeriv-1
	A1d = A1d + diag(ones(model.nbDeriv-i,1),i) * model.dt^i * 1/factorial(i); %Discrete 1D
end
B1d = zeros(model.nbDeriv,1); 
for i=1:model.nbDeriv
	B1d(model.nbDeriv-i+1) = model.dt^i * 1/factorial(i); %Discrete 1D
end
A0 = kron(A1d, eye(model.nbVarPos)); %Discrete nD
B0 = kron(B1d, eye(model.nbVarPos)); %Discrete nD

A = [A0, zeros(model.nbVar,1); zeros(1,model.nbVar), 1]; %Augmented A
B = [B0; zeros(1,model.nbVarPos)]; %Augmented B


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/2Dletters/G.mat');
Data=[];
for n=1:nbSamples
	s(n).Data=[];
	for m=1:model.nbDeriv
		if m==1
			dTmp = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
		else
			dTmp = gradient(dTmp) / model.dt; %Compute derivatives
		end
		s(n).Data = [s(n).Data; dTmp];
	end
	Data = [Data s(n).Data]; 
end


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kbins(Data,model,nbSamples);
[model, H] = EM_GMM(Data, model);

%Transform model to the corresponding version with augmented covariance
model0 = model;
model.nbVar = model0.nbVar+1;
model.Mu = zeros(model.nbVar, model.nbStates);
model.Sigma = zeros(model.nbVar, model.nbVar, model.nbStates);
for i=1:model.nbStates
	model.Sigma(:,:,i) = [model0.Sigma(:,:,i)+model0.Mu(:,i)*model0.Mu(:,i)', model0.Mu(:,i); model0.Mu(:,i)', 1];
end


%% Iterative LQR with augmented state space, recomputed in an online manner
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:nbRepros
	X = [Data(:,1); 1];
	for t=1:nbData
		r(n).Data(:,t) = X; %Log data
		%Set list of states according to first demonstration (alternatively, an HSMM can be used)
		id = [t:min(t+nbD-1,nbData), repmat(nbData,1,t-nbData+nbD-1)];
		[~,q] = max(H(:,id),[],1); %works also for nbStates=1
		%Riccati equation
		P = zeros(model.nbVar,model.nbVar,nbD);
		P(:,:,end) = inv(model.Sigma(:,:,q(end)));
		for s=nbD-1:-1:1
			Q = inv(model.Sigma(:,:,q(s)));
			P(:,:,s) = Q - A' * (P(:,:,s+1) * B / (B' * P(:,:,s+1) * B + R) * B' * P(:,:,s+1) - P(:,:,s+1)) * A;
		end
		K = (B' * P(:,:,1) * B + R) \ B' * P(:,:,1) * A; %FB gain
		u = -K * X; %Acceleration command with FB terms on augmented state (resulting in FB and FF terms)
		X = A * X + B * u; %Update of state vector
	end
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1300 500],'color',[1 1 1]); 
%Plot position
subplot(1,2,1); hold on; 
plotGMM(model0.Mu(1:2,:), model0.Sigma(1:2,1:2,:), [0.5 0.5 0.5], .3);
for n=1:nbSamples
	plot(Data(1,(n-1)*nbData+1:n*nbData), Data(2,(n-1)*nbData+1:n*nbData), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	h(1) = plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]);
end
axis equal; 
xlabel('x_1'); ylabel('x_2');
%Plot velocity
subplot(1,2,2); hold on; 
plotGMM(model0.Mu(3:4,:), model0.Sigma(3:4,3:4,:), [0.5 0.5 0.5], .3);
for n=1:nbSamples
	plot(Data(3,(n-1)*nbData+1:n*nbData), Data(4,(n-1)*nbData+1:n*nbData), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	plot(r(n).Data(3,:), r(n).Data(4,:), '-','linewidth',2,'color',[.8 0 0]); 
	plot(r(n).Data(3,1), r(n).Data(4,1), '.','markersize',18,'color',[.6 0 0]);
end
plot(0,0,'k+');
axis equal;
xlabel('dx_1'); ylabel('dx_2');


%print('-dpng','graphs/demo_iterativeLQR_augmSigma_online01.png');
%pause;
%close all;

