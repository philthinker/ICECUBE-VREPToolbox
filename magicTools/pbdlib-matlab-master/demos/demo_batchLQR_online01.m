function demo_batchLQR_online01
% Batch solution of linear quadratic optimal control (unconstrained linear MPC)  
% computed in an online manner, by relying on a GMM encoding of position and velocity data.
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
nbData = 100; %Number of datapoints
nbD = 20; %Time window for LQR computation

model.nbStates = 4; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.dt = 0.01; %Time step duration
model.rfactor = 1E-5;	%Control cost in LQR

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbD-1),R);


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
A = kron(A1d, eye(model.nbVarPos)); %Discrete nD
B = kron(B1d, eye(model.nbVarPos)); %Discrete nD

%Build Sx and Su matrices for batch LQR, see Eq. (35)
Su = zeros(model.nbVar*nbD, model.nbVarPos*(nbD-1));
Sx = kron(ones(nbD,1),eye(model.nbVar));
M = B;
for n=2:nbD
	id1 = (n-1)*model.nbVar+1:nbD*model.nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	id1 = (n-1)*model.nbVar+1:n*model.nbVar; 
	id2 = 1:(n-1)*model.nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:model.nbVarPos), M]; %Also M = [A^(n-1)*B, M] or M = [Sx(id1,:)*B, M]
end


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
%model = init_GMM_kmeans(Data,model);
model = init_GMM_kbins(Data,model,nbSamples);
[model, H] = EM_GMM(Data, model);


%% Batch LQR recomputed in an online manner 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:nbRepros
	X = Data(:,1); 
	for t=1:nbData
		%Set list of states according to first demonstration (alternatively, an HSMM can be used)
		id = [t:min(t+nbD-1,nbData), repmat(nbData,1,t-nbData+nbD-1)];
		[~,q] = max(H(:,id),[],1); %works also for nbStates=1
		%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q, see Eq. (27)
		MuQ = reshape(model.Mu(:,q), model.nbVar*nbD, 1); 
		SigmaQ = (kron(ones(nbD,1), eye(model.nbVar)) * reshape(model.Sigma(:,:,q), model.nbVar, model.nbVar*nbD)) .* kron(eye(nbD), ones(model.nbVar));
		%Set matrices to compute the damped weighted least squares estimate, see Eq. (37)
		SuInvSigmaQ = Su' / SigmaQ;
		Rq = SuInvSigmaQ * Su + R;
		rq = SuInvSigmaQ * (MuQ-Sx*X);
		u = Rq \ rq;
		%Update X with first control command
		X = A * X + B * u(1:model.nbVarPos);
		r(n).Data(:,t) = X;
	end
end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 700 650],'color',[1 1 1]); hold on; axis off;
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [0.5 0.5 0.5]);
plot(Data(1,:), Data(2,:), 'k.');
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]);
end
axis equal; 


%% Timeline plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
labList = {'$x_1$','$x_2$','$\dot{x}_1$','$\dot{x}_2$','$\ddot{x}_1$','$\ddot{x}_2$'};
figure('position',[720 10 600 650],'color',[1 1 1]); 
for j=1:model.nbVar
subplot(model.nbVar,1,j); hold on;
for n=1:nbSamples
	plot(Data(j,(n-1)*nbData+1:n*nbData), '-','linewidth',.5,'color',[0 0 0]);
end
for n=1:nbRepros
	plot(r(n).Data(j,:), '-','linewidth',1,'color',[.8 0 0]);
end
if j<7
	ylabel(labList{j},'fontsize',14,'interpreter','latex');
end
end
xlabel('$t$','fontsize',14,'interpreter','latex');


%print('-dpng','graphs/demo_batchLQR_online01.png');
%pause;
%close all;

