function demo_batchLQR01
% Controller retrieval through a batch solution of linear quadratic optimal control (unconstrained linear MPC),
% by relying on a Gaussian mixture model (GMM) encoding of position and velocity data.
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbSamples = 5; %Number of demonstrations
nbRepros = 1; %Number of reproductions in new situations
nbNewRepros = 10; %Number of stochastically generated reproductions
nbData = 200; %Number of datapoints

model.nbStates = 4; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.dt = 0.01; %Time step duration
model.rfactor = model.dt^model.nbDeriv;	%Control cost in LQR


%% Dynamical System settings (discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %Integration with Euler method 
% Ac1d = diag(ones(model.nbDeriv-1,1),1); 
% Bc1d = [zeros(model.nbDeriv-1,1); 1];
% A = kron(eye(model.nbDeriv)+Ac1d*model.dt, eye(model.nbVarPos)); 
% B = kron(Bc1d*model.dt, eye(model.nbVarPos));

%Integration with higher order Taylor series expansion
A1d = zeros(model.nbDeriv);
for i=0:model.nbDeriv-1
	A1d = A1d + diag(ones(model.nbDeriv-i,1),i) * model.dt^i * 1/factorial(i); %Discrete 1D
end
B1d = zeros(model.nbDeriv,1); 
for i=1:model.nbDeriv
	B1d(model.nbDeriv-i+1) = model.dt^i * 1/factorial(i); %Discrete 1D
end
A = kron(A1d, eye(model.nbVarPos)); %Discrete nD
B = kron(B1d, eye(model.nbVarPos)); %Discrete nD

% %Conversion with control toolbox
% Ac1d = diag(ones(model.nbDeriv-1,1),1); %Continuous 1D
% Bc1d = [zeros(model.nbDeriv-1,1); 1]; %Continuous 1D
% Cc1d = [1, zeros(1,model.nbDeriv-1)]; %Continuous 1D
% sysd = c2d(ss(Ac1d,Bc1d,Cc1d,0), model.dt);
% A = kron(sysd.a, eye(model.nbVarPos)); %Discrete nD
% B = kron(sysd.b, eye(model.nbVarPos)); %Discrete nD

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbData-1),R);

%Build Sx and Su matrices for batch LQR, see Eq. (35)
Su = zeros(model.nbVar*nbData, model.nbVarPos*(nbData-1));
Sx = kron(ones(nbData,1),eye(model.nbVar));
M = B;
for n=2:nbData
	id1 = (n-1)*model.nbVar+1:nbData*model.nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	id1 = (n-1)*model.nbVar+1:n*model.nbVar; 
	id2 = 1:(n-1)*model.nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:model.nbVarPos), M]; %Also M = [A^(n-1)*B, M] or M = [Sx(id1,:)*B, M]
end

% figure('PaperPosition',[0 0 4 8],'position',[10 10 400 650],'name','Su'); 
% axes('Position',[0.01 0.01 .98 .98]); hold on; set(gca,'linewidth',2); 
% colormap(flipud(gray));
% pcolor([abs(Su) zeros(size(Su,1),1); zeros(1,size(Su,2)+1)]); %dummy values for correct display
% shading flat; axis ij; axis equal tight;
% set(gca,'xtick',[],'ytick',[]);
% pause;
% close all;
% return


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/2Dletters/A.mat');
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
fprintf('Parameters estimation...');
%model = init_GMM_kmeans(Data,model);
model = init_GMM_kbins(Data,model,nbSamples);

% %Initialization based on position data
% model0 = init_GMM_kmeans(Data(1:model.nbVarPos,:), model);
% [~, GAMMA2] = EM_GMM(Data(1:model.nbVarPos,:), model0);
% model.Priors = model0.Priors;
% for i=1:model.nbStates
% 	model.Mu(:,i) = Data * GAMMA2(i,:)';
% 	DataTmp = Data - repmat(model.Mu(:,i),1,nbData*nbSamples);
% 	model.Sigma(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp';
% end

%Refinement of parameters
[model, H] = EM_GMM(Data, model);

%Precomputation of inverse and eigencomponents (optional)
for i=1:model.nbStates
	[model.V(:,:,i), model.D(:,:,i)] = eigs(model.Sigma(:,:,i));
	model.invSigma(:,:,i) = inv(model.Sigma(:,:,i));
end

%Set list of states according to first demonstration (alternatively, an HSMM can be used)
[~,qList] = max(H(:,1:nbData),[],1); %works also for nbStates=1

%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q, see Eq. (27)
MuQ = reshape(model.Mu(:,qList), model.nbVar*nbData, 1); 
SigmaQ = zeros(model.nbVar*nbData);
for t=1:nbData
	id = (t-1)*model.nbVar+1:t*model.nbVar;
	%MuQ(id,1) = model.Mu(:,qList(t));
	SigmaQ(id,id) = model.Sigma(:,:,qList(t));
end	
%SigmaQ can alternatively be computed with: 
%SigmaQ = (kron(ones(nbData,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,qList), model.nbVar, model.nbVar*nbData)) .* kron(eye(nbData), ones(model.nbVar));

% %Create single Gaussian N(MuQ,SigmaQ) based on h	
% h = H(:,1:nbData);
% h = h ./ repmat(sum(h,1),model.nbStates,1);
% MuQ = zeros(model.nbVar*nbData,1);
% SigmaQ = zeros(model.nbVar*nbData);
% for t=1:nbData
% 	id = (t-1)*model.nbVar+1:t*model.nbVar;
% 	for i=1:model.nbStates
% 		MuQ(id) = MuQ(id) + model.Mu(:,i) * h(i,t);
% 		SigmaQ(id,id) = SigmaQ(id,id) + model.Sigma(:,:,i) * h(i,t);
% 	end
% end


%% Batch LQR reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set matrices to compute the damped weighted least squares estimate, see Eq. (37)
SuInvSigmaQ = Su' / SigmaQ;
Rq = SuInvSigmaQ * Su + R;
%M = (SuInvSigmaQ * Su + R) \ SuInvSigmaQ; 
%M = inv(Su' / SigmaQ * Su + R) * Su' /SigmaQ;

%Reproductions
for n=1:nbRepros
	X = Data(:,1) + [randn(model.nbVarPos,1)*0E0; zeros(model.nbVar-model.nbVarPos,1)];
	%X = Data(:,1); 
 	rq = SuInvSigmaQ * (MuQ-Sx*X);
 	u = Rq \ rq; %Can also be computed with u = lscov(Rq, rq);
	%[u,~,~,S] = lscov(Rq, rq);
	%u = M * (MuQ-Sx*X);
	r(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
	
% 	%Simulate plant
% 	r(n).u = reshape(u, model.nbVarPos, nbData-1);
% 	for t=1:nbData-1
% 		%id = (t-1)*model.nbVar+1:t*model.nbVar;
% 		%id2 = (t-1)*model.nbVarPos+1:t*model.nbVarPos;
% 		%r(n).u(:,t) = M(id2,id) * (MuQ(id) - X);
% 		X = A * X + B * r(n).u(:,t);
% 		r(n).Data(:,t) = X;
% 	end
end


% %% Stochastic sampling by exploiting the GMM representation
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nbEigs = 2; %Number of principal eigencomponents to keep
% X = Data(:,1);
% SuInvSigmaQ = Su' / SigmaQ;
% Rq = SuInvSigmaQ * Su + R;
% for n=1:nbNewRepros
% 	%N = randn(nbEigs,model.nbStates) * 2E0; %Noise on all components
% 	%N = [randn(nbEigs,1), zeros(nbEigs,model.nbStates-1)] * 2E0; %Noise on first component
% 	N = [zeros(nbEigs,model.nbStates-3), randn(nbEigs,1), zeros(nbEigs,2)] * 1E0; %Noise on second component
% 	for i=1:model.nbStates
% 		MuTmp(:,i) = model.Mu(:,i) + model.V(:,1:nbEigs,i) * model.D(1:nbEigs,1:nbEigs,i).^.5 * N(:,i);
% 	end
% 	MuQ2 = reshape(MuTmp(:,qList), model.nbVar*nbData, 1); 
% 	rq = SuInvSigmaQ * (MuQ2-Sx*X);
% 	u = Rq \ rq;
% 	rnew(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData); %Reshape data for plotting
% end


%% Stochastic sampling by exploiting distribution on x
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbEigs = 3; %Number of principal eigencomponents to keep
X = Data(:,1);
SuInvSigmaQ = Su' / SigmaQ;
Rq = SuInvSigmaQ * Su + R;
rq = SuInvSigmaQ * (MuQ-Sx*X);
u = Rq \ rq;
Mu = Sx * X + Su * u;
mse = 1; %abs(MuQ'/SigmaQ*MuQ - rq'/Rq*rq) ./ (model.nbVarPos*nbData);
Sigma = Su*(Rq\Su') * mse + eye(nbData*model.nbVar) * 0E-10;
[V,D] = eigs(Sigma);
%[V,D] = eig(Sigma);
for n=1:nbNewRepros
	%xtmp = Mu + V(:,1:nbEigs) * D(1:nbEigs,1:nbEigs).^.5 * randn(nbEigs,1);
	xtmp = real(Mu + V * D.^.5 * randn(size(D,1),1) * 8.9);
	rnew(n).Data = reshape(xtmp, model.nbVar, nbData); %Reshape data for plotting
end


% %% Stochastic sampling by exploiting distribution on u
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nbEigs = 3; %Number of principal eigencomponents to keep
% X = Data(:,1);
% SuInvSigmaQ = Su' / SigmaQ;
% Rq = SuInvSigmaQ * Su + R;
% rq = SuInvSigmaQ * (MuQ-Sx*X);
% Mu = Rq \ rq;
% mse =  abs(MuQ'/SigmaQ*MuQ - rq'/Rq*rq) ./ (model.nbVarPos*nbData);
% Sigma = inv(Rq) * mse + eye((nbData-1)*model.nbVarPos) * 1E-10;
% %[V,D] = eigs(Sigma);
% [V,D] = eig(Sigma);
% for n=1:nbNewRepros
% 	%utmp = Mu + V(:,1:nbEigs) * D(1:nbEigs,1:nbEigs).^.5 * randn(nbEigs,1);
% 	utmp = real(Mu + V * D.^.5 * randn(size(D,1),1) * 0.2);
% 	xtmp = Sx * X + Su * utmp;
% 	rnew(n).Data = reshape(xtmp, model.nbVar, nbData); %Reshape data for plotting
% end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 700 650],'color',[1 1 1]); hold on; axis off;
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [0.5 0.5 0.5]);
plot(Data(1,:), Data(2,:), 'k.');
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]);
end
for n=1:nbNewRepros
	plot(rnew(n).Data(1,:), rnew(n).Data(2,:), '-','linewidth',1,'color',max(min([0 .7+randn(1)*1E-1 0],1),0));
end
axis equal; 


%% Timeline plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
labList = {'$x_1$','$x_2$','$\dot{x}_1$','$\dot{x}_2$','$\ddot{x}_1$','$\ddot{x}_2$'};
figure('position',[720 10 600 650],'color',[1 1 1]); 
for j=1:model.nbVar
subplot(model.nbVar+1,1,j); hold on;
for n=1:nbSamples
	plot(Data(j,(n-1)*nbData+1:n*nbData), '-','linewidth',.5,'color',[0 0 0]);
end
for n=1:nbNewRepros
	plot(rnew(n).Data(j,:), '-','linewidth',1,'color',max(min([0 .7+randn(1)*1E-1 0],1),0));
end
for n=1:nbRepros
	plot(r(n).Data(j,:), '-','linewidth',1,'color',[.8 0 0]);
end
if j<7
	ylabel(labList{j},'fontsize',14,'interpreter','latex');
end
end

%Speed profile
if model.nbDeriv>1
subplot(model.nbVar+1,1,model.nbVar+1); hold on;
for n=1:nbSamples
	sp = sqrt(Data(3,(n-1)*nbData+1:n*nbData).^2 + Data(4,(n-1)*nbData+1:n*nbData).^2);
	plot(sp, '-','linewidth',.5,'color',[0 0 0]);
end
for n=1:nbNewRepros
	sp = sqrt(rnew(n).Data(3,:).^2 + rnew(n).Data(4,:).^2);
	plot(sp, '-','linewidth',1,'color',max(min([0 .7+randn(1)*1E-1 0],1),0));
end
for n=1:nbRepros
	sp = sqrt(r(n).Data(3,:).^2 + r(n).Data(4,:).^2);
	plot(sp, '-','linewidth',1,'color',[.8 0 0]);
end
ylabel('$|\dot{x}|$','fontsize',14,'interpreter','latex');
xlabel('$t$','fontsize',14,'interpreter','latex');
end

%print('-dpng','graphs/demo_batchLQR01.png');
%pause;
%close all;

