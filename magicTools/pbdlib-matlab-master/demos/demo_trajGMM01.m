function demo_trajGMM01
% Trajectory synthesis with a GMM with dynamic features (trajectory GMM).
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
model.nbStates = 5; %Number of components in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 3; %Number of static&dynamic features (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.dt = 0.01; %Time step (without rescaling, large values such as 1 has the advantage of creating clusers based on position information)
nbSamples = 4; %Number of demonstrations
nbData = 100; %Number of datapoints in a trajectory

[PHI,PHI1] = constructPHI(model,nbData,nbSamples); %Construct PHI operator (big sparse matrix)


%% Load dataset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Load different types of movements
% load('data/dataTrajGMM01.mat');
% %load('data/dataTrajGMM02.mat'); %periodic motion
% %load('data/dataTrajGMM03.mat'); %motion with options
% %load('data/dataTrajGMM04.mat'); %partial demonstrations
% 
% %Resampling of dataset
% nbData0 = size(Data,2)/nbSamples; %Original number of datapoints in a trajectory
% DataTmp = [];
% for n=1:nbSamples
% 	DataTmp = [DataTmp spline(1:nbData0, Data(2:model.nbVarPos+1,(n-1)*nbData0+1:n*nbData0), linspace(1,nbData0,nbData))]; 
% end
% Data = DataTmp;

%Load handwriting movements
load('data/2Dletters/S.mat');
Data=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data = [Data s(n).Data]; 
end

%Re-arrange data in vector form
x = reshape(Data, model.nbVarPos*nbData*nbSamples, 1) * 1E2; %Scale data to avoid numerical computation problem
zeta = PHI*x; %y is for example [x1(1), x2(1), x1d(1), x2d(1), x1(2), x2(2), x1d(2), x2d(2), ...]
Data = reshape(zeta, model.nbVarPos*model.nbDeriv, nbData*nbSamples); %Include derivatives in Data


%% Parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Initialization with kmeans
% model = init_GMM_kmeans(Data, model);

%Initialization with homogeneous time intervals
model = init_GMM_timeBased([repmat(1:nbData,1,nbSamples); Data], model);
model.Mu = model.Mu(2:end,:);
model.Sigma = model.Sigma(2:end,2:end,:);

%Refinement of parameters
[model, GAMMA2] = EM_GMM(Data, model);
%[model, GAMMA2] = EM_MFA(Data, model);
%[model, GAMMA2] = EM_MPPCA(Data, model);
%[model, GAMMA2] = EM_HDGMM(Data, model);

%Precomputation of inverses (optional)
for i=1:model.nbStates
	model.invSigma(:,:,i) = inv(model.Sigma(:,:,i));
end


%% Reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:1 %nbSamples
	%Compute best path for the n-th demonstration
	[~,r(n).q] = max(GAMMA2(:,(n-1)*nbData+1:n*nbData),[],1); %works also for nbStates=1
	
	%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q
	MuQ = reshape(model.Mu(:,r(n).q), model.nbVar*nbData, 1); 
	%MuQ = zeros(model.nbVar*nbData,1);
	SigmaQ = zeros(model.nbVar*nbData);
	for t=1:nbData
		id = (t-1)*model.nbVar+1:t*model.nbVar;
		%MuQ(id) = model.Mu(:,r(n).q(t)); 
		SigmaQ(id,id) = model.Sigma(:,:,r(n).q(t));
	end
	%SigmaQ can alternatively be computed with:
	%SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,r(n).q), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));


	%Least squares computation method 1 (using lscov Matlab function)
	%%%%%%%%%%%%%%%%%%%
	[zeta,~,mse,S] = lscov(PHI1, MuQ, SigmaQ, 'chol'); %Retrieval of data with weighted least squares solution
	r(n).Data = reshape(zeta, model.nbVarPos, nbData); %Reshape data for plotting

	
% 	%Least squares computation method 2 (most readable but not optimized)
% 	%%%%%%%%%%%%%%%%%%%
% 	PHIinvSigmaQ = PHI1' / SigmaQ;
% 	Rq = PHIinvSigmaQ * PHI1;
% 	rq = PHIinvSigmaQ * MuQ;
% 	zeta = Rq \ rq; %Can also be computed with c = lscov(Rq, rq)
% 	size(zeta)
% 	r(n).Data = reshape(zeta, model.nbVarPos, nbData); %Reshape data for plotting
% 	%Covariance Matrix computation of ordinary least squares estimate
% 	mse =  (MuQ'*inv(SigmaQ)*MuQ - rq'*inv(Rq)*rq) ./ ((model.nbVar-model.nbVarPos)*nbData);
% 	S = inv(Rq) * mse; 
	

% 	%Least squares computation method 3 (efficient computation using Cholesky and QR decompositions, inspired by lscov code)
% 	%%%%%%%%%%%%%%%%%%%
% 	T = chol(SigmaQ)'; %SigmaQ=T*T'
% 	TA = T \ PHI1;
% 	TMuQ = T \ MuQ;
% 	[Q, R, perm] = qr(TA,0); %PHI1(:,perm)=Q*R
% 	z = Q' * TMuQ;
% 	zeta = zeros(nbData*model.nbVarPos,1);
% 	zeta(perm,:) = R \ z; %zeta=(TA'*TA)\(TA'*TMuQ)
% 	r(n).Data = reshape(zeta, model.nbVarPos, nbData); %Reshape data for plotting
% 	%Covariance Matrix computation of ordinary least squares estimate
% 	err = TMuQ - Q*z;
% 	mse = err'*err ./ (model.nbVar*nbData - model.nbVarPos*nbData);
% 	Rinv = R \ eye(model.nbVarPos*nbData);
% 	S(perm,perm) = Rinv*Rinv' .* mse; 
	
	
	%Rebuild covariance by reshaping S
	for t=1:nbData
		id = (t-1)*model.nbVarPos+1:t*model.nbVarPos;
		r(n).expSigma(:,:,t) = S(id,id) * nbData;
	end
	
end %nbSamples


%% Plot timeline
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 600 600]);
for m=1:model.nbVarPos
	limAxes = [1, nbData, min(Data(m,:))-1E0, max(Data(m,:))+1E0];
	subplot(model.nbVarPos,1,m); hold on;
	for n=1:1 %nbSamples
		msh=[]; x0=[];
		for t=1:nbData-1
			if size(msh,2)==0
				msh(:,1) = [t; model.Mu(m,r(n).q(t))];
			end
			if t==nbData-1 || r(n).q(t+1)~=r(n).q(t)
				msh(:,2) = [t+1; model.Mu(m,r(n).q(t))];
				sTmp = model.Sigma(m,m,r(n).q(t))^.5;
				msh2 = [msh(:,1)+[0;sTmp], msh(:,2)+[0;sTmp], msh(:,2)-[0;sTmp], msh(:,1)-[0;sTmp], msh(:,1)+[0;sTmp]];
				patch(msh2(1,:), msh2(2,:), [.85 .85 .85],'edgecolor',[.7 .7 .7]);
				plot(msh(1,:), msh(2,:), '-','linewidth',3,'color',[.7 .7 .7]);
				plot([msh(1,1) msh(1,1)], limAxes(3:4), ':','linewidth',1,'color',[.7 .7 .7]);
				x0 = [x0 msh];
				msh=[];
			end
		end
		msh = [1:nbData, nbData:-1:1; r(n).Data(m,:)-squeeze(r(n).expSigma(m,m,:).^.5)'*1, fliplr(r(n).Data(m,:)+squeeze(r(n).expSigma(m,m,:).^.5)'*1)];
		patch(msh(1,:), msh(2,:), [1 .4 .4],'edgecolor',[1 .2 .2],'edgealpha',.8,'facealpha',.5);
	end
	for n=1:nbSamples
		plot(1:nbData, Data(m,(n-1)*nbData+1:n*nbData), '-','lineWidth',1,'color',[.2 .2 .2]);
	end
	for n=1:1
		plot(1:nbData, r(n).Data(m,:), '-','lineWidth',2.5,'color',[.8 0 0]);
	end
	
	set(gca,'xtick',[],'ytick',[]);
	xlabel('$t$','interpreter','latex','fontsize',18);
	ylabel(['$x_' num2str(m) '$'],'interpreter','latex','fontsize',18);
	axis(limAxes);
end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if model.nbVarPos>1
	figure('position',[620 10 600 600]); hold on;
	for n=1:1 %nbSamples
		plotGMM(r(n).Data([1,2],:), r(n).expSigma([1,2],[1,2],:), [1 .2 .2],.2);
	end
	plotGMM(model.Mu([1,2],:), model.Sigma([1,2],[1,2],:), [.5 .5 .5],.8);
	for n=1:nbSamples
		plot(Data(1,(n-1)*nbData+1:n*nbData), Data(2,(n-1)*nbData+1:n*nbData), '-','lineWidth',1,'color',[.2 .2 .2]); %-0.2+0.8*(n-1)/(nbSamples-1)
	end
	for n=1:1
		plot(r(n).Data(1,:), r(n).Data(2,:), '-','lineWidth',2.5,'color',[.8 0 0]);
	end
	set(gca,'xtick',[],'ytick',[]); axis equal; axis square;
	xlabel(['$x_1$'],'interpreter','latex','fontsize',18);
	ylabel(['$x_2$'],'interpreter','latex','fontsize',18);
end

%print('-dpng','graphs/demo_trajGMM01.png');
%pause;
%close all;


