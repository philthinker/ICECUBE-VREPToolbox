function demo_TPMPC02
% Task-parameterized probabilistic model encoding position data, with a generalized version of MPC 
% (batch version of LQR) used to track associated stepwise reference paths in multiple frames.
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


%% Reproduction for the task parameters used to train the model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with batch LQR...');
for n=1:nbSamples
	
	%GMM projection
	for i=1:model.nbStates
		for m=1:model.nbFrames
			s(n).p(m).Mu(:,i) = s(n).p(m).A * model.Mu(:,m,i) + s(n).p(m).b;
			s(n).p(m).Sigma(:,:,i) = s(n).p(m).A * model.Sigma(:,:,m,i) * s(n).p(m).A';
		end
	end
	
	%Compute best path for the n-th demonstration
	[~,s(n).q] = max(model.Pix(:,(n-1)*nbData+1:n*nbData),[],1); %works also for nbStates=1	
	
	%Build a reference trajectory for each frame
	%SigmaQ = zeros(model.nbVar*nbData);
	invSigmaQ = zeros(model.nbVar*nbData);
	for m=1:model.nbFrames
		s(n).p(m).MuQ = reshape(s(n).p(m).Mu(:,s(n).q), model.nbVar*nbData, 1);  
		s(n).p(m).SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(s(n).p(m).Sigma(:,:,s(n).q), model.nbVar, model.nbVar*nbData)) ...
			.* kron(eye(nbData), ones(model.nbVar));
		%SigmaQ = SigmaQ + s(n).p(m).SigmaQ;
		invSigmaQ = invSigmaQ + inv(s(n).p(m).SigmaQ);
	end

	%LQR batch (MPC)
	%X = [MuQ(1:model.nbVar,1)+randn(model.nbVar,1)*1E-10; zeros(model.nbVar,1)];
	X = [s(1).p(1).b(1:model.nbVar,1)+randn(model.nbVar,1)*1E-10; zeros(model.nbVar,1)];
	[Phi,F] = constructMPC(Ad, Bd, C, nbData, nbData);
	%PhiInvSigmaQ = Phi' / SigmaQ;
	PhiInvSigmaQ = Phi' * invSigmaQ;
	Rq = PhiInvSigmaQ * Phi + R;
	rq = zeros(model.nbVar*nbData,1);
	for m=1:model.nbFrames
		rq = rq + s(n).p(m).SigmaQ \ (s(n).p(m).MuQ - F*X);
	end
	rq = Phi' * rq; 

% 	MuQbar = zeros(model.nbVar*nbData,1);
% 	for m=1:model.nbFrames
% 		MuQbar = MuQbar + s(n).p(m).SigmaQ \ s(n).p(m).MuQ;
% 	end
% 	rq = Phi' * (MuQbar - invSigmaQ*F*X);
	
	%Simulate plant
 	U = reshape(Rq\rq, model.nbVar, nbData);
	for t=1:nbData
		X = Ad * X + Bd * U(:,t);
		r(n).Data(:,t) = C * X;
	end
% 	Xi = F * X + Phi * Rq\rq;
% 	r(n).Data = reshape(Xi, model.nbVar, nbData);
	
end


%% Reproduction for new task parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('New reproductions with batch LQR...');
for n=1:nbRepros
	for m=1:model.nbFrames
		%Random generation of new task parameters
		id=ceil(rand(2,1)*nbSamples);
		w=rand(2); w=w/sum(w);
		rnew(n).p(m).b = s(id(1)).p(m).b * w(1) + s(id(2)).p(m).b * w(2);
		rnew(n).p(m).A = s(id(1)).p(m).A * w(1) + s(id(2)).p(m).A * w(2);
	end
	
	%GMM projection
	for i=1:model.nbStates
		for m=1:model.nbFrames
			rnew(n).p(m).Mu(:,i) = rnew(n).p(m).A * model.Mu(:,m,i) + rnew(n).p(m).b;
			rnew(n).p(m).Sigma(:,:,i) = rnew(n).p(m).A * model.Sigma(:,:,m,i) * rnew(n).p(m).A';
		end
	end
	
	%Compute best path for the 1st demonstration
	[~,rnew(n).q] = max(model.Pix(:,1:nbData),[],1); %works also for nbStates=1	
	
	%Build stepwise reference trajectory
	%SigmaQ = zeros(model.nbVar*nbData);
	invSigmaQ = zeros(model.nbVar*nbData);
	for m=1:model.nbFrames
		rnew(n).p(m).MuQ = reshape(rnew(n).p(m).Mu(:,rnew(n).q), model.nbVar*nbData, 1);  
		rnew(n).p(m).SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(rnew(n).p(m).Sigma(:,:,rnew(n).q), model.nbVar, model.nbVar*nbData)) ...
			.* kron(eye(nbData), ones(model.nbVar));
		%SigmaQ = SigmaQ + anew(n).p(m).SigmaQ;
		invSigmaQ = invSigmaQ + inv(rnew(n).p(m).SigmaQ);
	end

	%LQR batch (MPC)
	X = [rnew(1).p(1).b(1:model.nbVar,1)+randn(model.nbVar,1)*1E-10; zeros(model.nbVar,1)];
	[Phi,F] = constructMPC(Ad, Bd, C, nbData, nbData);
	%PhiInvSigmaQ = Phi' / SigmaQ;
	PhiInvSigmaQ = Phi' * invSigmaQ;
	Rq = PhiInvSigmaQ * Phi + R;
	rq = zeros(model.nbVar*nbData,1);
	for m=1:model.nbFrames
		rq = rq + rnew(n).p(m).SigmaQ \ (rnew(n).p(m).MuQ - F*X);
	end
	rq = Phi' * rq; 
	
	%Simulate plant
 	U = reshape(Rq\rq, model.nbVar, nbData);
	for t=1:nbData
		X = Ad * X + Bd * U(:,t);
		rnew(n).Data(:,t) = C * X;
	end
	
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
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%print('-dpng','graphs/demo_TPMPC02.png');
%pause;
%close all;
