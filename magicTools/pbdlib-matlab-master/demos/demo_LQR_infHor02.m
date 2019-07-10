function demo_LQR_infHor02
% Discrete infinite horizon linear quadratic tracking, by relying on a GMM encoding of 
% position and velocity data. Compared to the continuous version, this discrete version 
% is less sensitive to numerical instability when defining the R cost.
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
model.nbStates = 6; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.dt = 0.01; %Time step duration
model.rfactor = 1E-5;	%Control cost in LQR (to be set carefully because infinite horizon LQR can suffer mumerical instability)
nbSamples = 5; %Number of demonstrations
nbRepros = 3; %Number of reproductions
nbData = 200; %Number of datapoints

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;


%% Discrete dynamical System settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %Integration with Euler method 
% Ac1d = diag(ones(model.nbDeriv-1,1),1); %Continuous 1D
% Bc1d = [zeros(model.nbDeriv-1,1); 1]; %Continuous 1D
% A = kron(eye(model.nbDeriv)+Ac1d*model.dt, eye(model.nbVarPos)); %Discrete
% B = kron(Bc1d*model.dt, eye(model.nbVarPos)); %Discrete

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
% A = kron(sysd.a, eye(model.nbVarPos));
% B = kron(sysd.b, eye(model.nbVarPos));


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
%model.params_diagRegFact = 1E-8;
[model, H] = EM_GMM(Data, model);


%% Iterative discrete LQR with infinite horizon
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
		
		%P = dare(A, B, (Q+Q')/2, R); %[P,~,L]=...
		P = solveAlgebraicRiccati_eig_discrete(A, B*(R\B'), (Q+Q')/2);
		
		L = (B'*P*B + R) \ B'*P*A; %Feedback gain (discrete version)
		u =  -L * (X - model.Mu(:,qList(t))); %Compute acceleration (with only feedback terms)
		X = A*X + B*u;
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

%print('-dpng','graphs/demo_LQR_infHor02.png');
%pause;
%close all;

