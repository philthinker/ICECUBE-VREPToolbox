function demo_TPMPC01
% Task-parameterized probabilistic model encoding position data, with MPC (batch version of LQR) 
% used to track the associated stepwise reference path.
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
model.nbStates = 3; %Number of Gaussians in the GMM
model.nbFrames = 2; %Number of candidate frames of reference
model.nbVar = 2; %Dimension of data (here: x1,x2)
model.dt = 0.01; %Time step
model.rFactor = 1E-2; %Weighting term for the minimization of control commands in LQR

nbRepros = 8; %Number of reproductions with new situations randomly generated
nbData = 200; %Number of datapoints in a trajectory

%Canonical system parameters
A = kron([0 1; 0 0], eye(model.nbVar)); %See Eq. (5.1.1) in doc/TechnicalReport.pdf
B = kron([0; 1], eye(model.nbVar)); %See Eq. (5.1.1) in doc/TechnicalReport.pdf
C = kron([1,0],eye(model.nbVar));
%Discretize system (Euler method)
Ad = A*model.dt + eye(size(A));
Bd = B*model.dt;
%Control cost matrix
R = eye(model.nbVar) * model.rFactor;
R = kron(eye(nbData),R);


%% Load 3rd order tensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% The MAT file contains a structure 's' with the multiple demonstrations. 's(n).Data' is a matrix data for
% sample n (with 's(n).nbData' datapoints). 's(n).p(m).b' and 's(n).p(m).A' contain the position and
% orientation of the m-th candidate coordinate system for this demonstration. 'Data' contains the observations
% in the different frames. It is a 3rd order tensor of dimension D x P x N, with D=2 the dimension of a
% datapoint, P=2 the number of candidate frames, and N=200x4 the number of datapoints in a trajectory (200)
% multiplied by the number of demonstrations (nbSamples=5).
load('data/Data01.mat');


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM...');
model = init_tensorGMM_kmeans(Data, model); %Initialization
model = EM_tensorGMM(Data, model);


%% Reproduction with LQR for the task parameters used to train the model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with finite-horizon LQR...');
for n=1:nbSamples
	
	%Reconstruct GMM 
	[s(n).Mu, s(n).Sigma] = productTPGMM0(model, s(n).p);
	
	%Compute best path for the n-th demonstration
	[~,s(n).q] = max(model.Pix(:,(n-1)*nbData+1:n*nbData),[],1); %works also for nbStates=1	
	
	%Build stepwise reference trajectory
	MuQ = reshape(s(n).Mu(:,s(n).q), model.nbVar*nbData, 1); 
	SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(s(n).Sigma(:,:,s(n).q), model.nbVar, model.nbVar*nbData)) ...
		.* kron(eye(nbData), ones(model.nbVar));

	%LQR batch (MPC)
	X = [MuQ(1:model.nbVar,1)+randn(model.nbVar,1)*1E-10; zeros(model.nbVar,1)];
	[Phi,F] = constructMPC(Ad, Bd, C, nbData, nbData);
	PhiInvSigmaQ = Phi' / SigmaQ;
	Rq = PhiInvSigmaQ * Phi + R;
	rq = PhiInvSigmaQ * (MuQ - F*X);
	%Simulate plant
 	U = reshape(Rq\rq, model.nbVar, nbData);
	for t=1:nbData
		X = Ad * X + Bd * U(:,t);
		r(n).Data(:,t) = C * X;
	end
% 	Xi = F * X + Phi * Rq\rq;
% 	r(n).Data = reshape(Xi, model.nbVar, nbData);
	
end


%% Reproduction with LQR for new task parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('New reproductions with finite-horizon LQR...');
for n=1:nbRepros
	for m=1:model.nbFrames
		%Random generation of new task parameters
		id=ceil(rand(2,1)*nbSamples);
		w=rand(2); w=w/sum(w);
		anew(n).p(m).b = s(id(1)).p(m).b * w(1) + s(id(2)).p(m).b * w(2);
		anew(n).p(m).A = s(id(1)).p(m).A * w(1) + s(id(2)).p(m).A * w(2);
	end
	%Reconstruct GMM 
	[anew(n).Mu, anew(n).Sigma] = productTPGMM0(model, anew(n).p);
	
	%Compute best path for the 1st demonstration
	[~,anew(n).q] = max(model.Pix(:,1:nbData),[],1); %works also for nbStates=1	

	%Build stepwise reference trajectory
	MuQ = reshape(anew(n).Mu(:,anew(n).q), model.nbVar*nbData, 1); 
	SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(anew(n).Sigma(:,:,anew(n).q), model.nbVar, model.nbVar*nbData)) ...
		.* kron(eye(nbData), ones(model.nbVar));

	%LQR batch (MPC)
	rnew(n).p = anew(n).p;
	rnew(n).Mu = anew(n).Mu;
	rnew(n).Sigma = anew(n).Sigma;
	X = [MuQ(1:model.nbVar,1)+randn(model.nbVar,1)*1E-10; zeros(model.nbVar,1)];
	[Phi,F] = constructMPC(Ad, Bd, C, nbData, nbData);
	PhiInvSigmaQ = Phi' / SigmaQ;
	Rq = PhiInvSigmaQ * Phi + R;
	rq = PhiInvSigmaQ * (MuQ - F*X);
	%Simulate plant
 	U = reshape(Rq\rq, model.nbVar, nbData);
	for t=1:nbData
		X = Ad * X + Bd * U(:,t);
		rnew(n).Data(:,t) = C * X;
	end
% 	Xi = F * X + Phi * Rq\rq;
% 	rnew(n).Data = reshape(Xi, model.nbVar, nbData);
	
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,1300,500]);
xx = round(linspace(1,64,nbSamples));
clrmap = colormap('jet');
clrmap = min(clrmap(xx,:),.95);
limAxes = [-1.2 0.8 -1.1 0.9];
colPegs = [[.9,.5,.9];[.5,.9,.5]];

%DEMOS
subplot(1,3,1); hold on; box on; title('Demonstrations');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
	plot(s(n).Data0(2,1), s(n).Data0(3,1),'.','markersize',12,'color',clrmap(n,:));
	plot(s(n).Data0(2,:), s(n).Data0(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%REPROS
subplot(1,3,2); hold on; box on; title('Reproductions');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbSamples
	%Plot trajectories
	plot(r(n).Data(1,1), r(n).Data(2,1),'.','markersize',12,'color',clrmap(n,:));
	plot(r(n).Data(1,:), r(n).Data(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
for n=1:nbSamples
	%Plot Gaussians
	plotGMM(s(n).Mu(:,:,1), s(n).Sigma(:,:,:,1), [.5 .5 .5], .4);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%NEW REPROS
subplot(1,3,3); hold on; box on; title('New reproductions');
for n=1:nbRepros
	%Plot frames
	for m=1:model.nbFrames
		plot([rnew(n).p(m).b(1) rnew(n).p(m).b(1)+rnew(n).p(m).A(1,2)], [rnew(n).p(m).b(2) rnew(n).p(m).b(2)+rnew(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(rnew(n).p(m).b(1), rnew(n).p(m).b(2), '.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbRepros
	%Plot trajectories
	plot(rnew(n).Data(1,1), rnew(n).Data(2,1),'.','markersize',12,'color',[.2 .2 .2]);
	plot(rnew(n).Data(1,:), rnew(n).Data(2,:),'-','linewidth',1.5,'color',[.2 .2 .2]);
end
for n=1:nbRepros
	%Plot Gaussians
	plotGMM(rnew(n).Mu(:,:,1), rnew(n).Sigma(:,:,:,1), [.5 .5 .5], .4);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%print('-dpng','graphs/demo_TPMPC01.png');
%pause;
%close all;
