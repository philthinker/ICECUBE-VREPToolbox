function demo_batchLQR_viapoints01
% Keypoint-based motion through MPC, with a GMM encoding of position and velocity
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @inproceedings{Berio17GI,
%   author="Berio, D. and Calinon, S. and Fol Leymarie, F.",
%   title="Generating Calligraphic Trajectories with Model Predictive Control",
%   booktitle="Proc. 43rd Conf. on Graphics Interface",
%   year="2017",
%   month="May",
%   address="Edmonton, AL, Canada",
%   pages="132--139",
%   doi="10.20380/GI2017.17"
% }
% 
% Copyright (c) 2017 Idiap Research Institute, http://idiap.ch/
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
nbData = 200; %Number of datapoints

model.nbStates = 4; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.rfactor = 1E-8;	%Control cost in LQR
model.dt = 0.01; %Time step duration
model.params_diagRegFact = 1E-8; %Control cost in LQR

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbData-1),R);


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


%% Generate keypoints from handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/2Dletters/M.mat');
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
%model = init_GMM_kmeans(Data, model);
model = init_GMM_kbins(Data, model, nbSamples);

%Refinement of parameters
[model, H] = EM_GMM(Data, model);

%Precomputation of inverses
for i=1:model.nbStates
	model.invSigma(:,:,i) = inv(model.Sigma(:,:,i));
end

%Set list of states according to first demonstration (alternatively, an HSMM can be used)
[~,qList] = max(H(:,1:nbData),[],1); %works also for nbStates=1

%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q
MuQ = zeros(model.nbVar*nbData, 1); 
Q = zeros(model.nbVar*nbData);
qCurr = qList(1);
xDes = []; 
for t=1:nbData
	if qCurr~=qList(t) || t==nbData
		id = (t-1)*model.nbVar+1:t*model.nbVar;
		MuQ(id) = model.Mu(:,qCurr); 
		Q(id,id) = model.invSigma(:,:,qCurr);
		%List of viapoints with time and error information (for plots)
		xDes = [xDes, [t; model.Mu(:,qCurr); diag(model.Sigma(:,:,qCurr)).^.5]];	
		qCurr = qList(t);
	end
end	


%% Batch LQR reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Set matrices to compute the damped weighted least squares estimate, see Eq. (37)
SuQ = Su' * Q;
Rq = SuQ * Su + R;
%Reproductions
for n=1:nbRepros
	X = Data(:,1) + [randn(model.nbVarPos,1)*0E0; zeros(model.nbVarPos,1)];
	rq = SuQ * (MuQ-Sx*X);
 	u = Rq \ rq; %Can also be computed with u = lscov(Rq, rq);
	r(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 700 700],'color',[1 1 1]); hold on; axis off;
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [.5 .5 .5], .5);
plot(model.Mu(1,:), model.Mu(2,:), '.','markersize',15,'color',[.5 .5 .5]);
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]);
end
axis equal; 


%% Timeline plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
labList = {'$x_1$','$x_2$','$\dot{x}_1$','$\dot{x}_2$'};
figure('position',[710 10 600 700],'color',[1 1 1]); 
for j=1:model.nbVar
	subplot(model.nbVar,1,j); hold on;
	for n=1:nbRepros
		plot(r(n).Data(j,:), '-','linewidth',1,'color',[.8 0 0]);
	end
	for t=1:size(xDes,2)
		errorbar(xDes(1,t), xDes(1+j,t), xDes(1+model.nbVar+j,t), 'color',[.5 .5 .5]);
		plot(xDes(1,t), xDes(1+j,t), '.','markersize',15,'color',[.5 .5 .5]);
	end
	ylabel(labList{j},'fontsize',14,'interpreter','latex');
end
xlabel('$t$','fontsize',14,'interpreter','latex');


%print('-dpng','graphs/demo_batchLQR_viapoints01.png');
%pause;
%close all;

