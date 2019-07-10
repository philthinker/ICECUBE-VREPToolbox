function demo_LQR_infHor01
% Continuous infinite horizon linear quadratic tracking, 
% by relying on a GMM encoding of position and velocity data.
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
model.nbStates = 6; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.dt = 0.01; %Time step duration
model.rfactor = 1E-3;	%Control cost in LQR (to be set carefully because infinite horizon LQR can suffer mumerical instability)

nbSamples = 5; %Number of demonstrations
nbRepros = 1; %Number of reproductions
nbData = 200; %Number of datapoints

%Continuous dynamical System settings (Integration with Euler method) 
A = kron(diag(ones(model.nbDeriv-1,1),1), eye(model.nbVarPos)); 
B = kron([zeros(model.nbDeriv-1,1); 1], eye(model.nbVarPos));

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/2Dletters/S.mat');
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
%model.params_diagRegFact = 1E-3;
[model, H] = EM_GMM(Data, model);


%% Iterative continuous LQR with infinite horizon
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Set list of states according to first demonstration (alternatively, an HSMM can be used)
[~,qList] = max(H(:,1:nbData),[],1); %works also for nbStates=1

for n=1:nbRepros
	if n==1
		X = Data(:,1);
	else
		X = Data(:,1) + [randn(model.nbVarPos,1)*2E0; zeros(model.nbVar-model.nbVarPos,1)];
	end
	for t=1:nbData
		Q = inv(model.Sigma(:,:,qList(t)));
		
		P = solveAlgebraicRiccati_eig(A, B*(R\B'), (Q+Q')/2); 
		%P = solveAlgebraicRiccati_Schur(A, B*(R\B'), (Q+Q')/2); 
		%P = care(A, B, (Q+Q')/2, R); %[P,~,L]=...
		
		L = R\B'*P; %Feedback gain (for continuous systems)
		u =  -L * (X - model.Mu(:,qList(t))); %Compute acceleration (with feedback terms)
		DX = A*X + B*u;
		X = X + DX * model.dt;
		r(n).Data(:,t) = X;
	end
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1300 600],'color',[1 1 1]); 
%Plot position
subplot(1,2,1); hold on; 
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [0.5 0.5 0.5], .3);
for n=1:nbSamples
	plot(Data(1,(n-1)*nbData+1:n*nbData), Data(2,(n-1)*nbData+1:n*nbData), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',2,'color',[.8 0 0]); %Reproduction with iterative LQR
	plot(r(n).Data(1,1), r(n).Data(2,1), '.','markersize',18,'color',[.6 0 0]);
end
%quiver(xs(1,:),xs(2,:),xs(3,:),xs(4,:)); %Plot acceleration map
axis equal; 
xlabel('x_1'); ylabel('x_2');

%Plot velocity
subplot(1,2,2); hold on; 
plotGMM(model.Mu(3:4,:), model.Sigma(3:4,3:4,:), [0.5 0.5 0.5], .3);
for n=1:nbSamples
	plot(Data(3,(n-1)*nbData+1:n*nbData), Data(4,(n-1)*nbData+1:n*nbData), '-','color',[.7 .7 .7]);
end
for n=1:nbRepros
	plot(r(n).Data(3,:), r(n).Data(4,:), '-','linewidth',2,'color',[.8 0 0]); %Reproduction with iterative LQR
	plot(r(n).Data(3,1), r(n).Data(4,1), '.','markersize',18,'color',[.6 0 0]);
end
plot(0,0,'k+');
axis equal;
xlabel('dx_1'); ylabel('dx_2');

%print('-dpng','graphs/demo_LQR_infHor01.png');
%pause;
%close all;

