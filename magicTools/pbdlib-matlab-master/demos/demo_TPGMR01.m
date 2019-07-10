function demo_TPGMR01
% Task-parameterized Gaussian mixture model (TP-GMM), with time-based GMR used for reproduction.
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
model.nbVar = 3; %Dimension of the datapoints in the dataset (here: t,x1,x2)
model.dt = 1E-2; %Time step duration
model.params_diagRegFact = 1E-8; %Optional regularization term
nbData = 200; %Number of datapoints in a trajectory
nbRepros = 4; %Number of reproductions with new situations randomly generated


%% Load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% s(n).Data0 is the n-th demonstration of a trajectory of s(n).nbData datapoints, with s(n).p(m).b and 's(n).p(m).A describing
% the context in which this demonstration takes place (position and orientation of the m-th candidate coordinate system)
load('data/Data02.mat');


%% Observations from the perspective of each candidate coordinate system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Data' contains the observations in the different coordinate systems: it is a 3rd order tensor of dimension D x P x N, 
% with D=3 the dimension of a datapoint, P=2 the number of candidate frames, and N=TM the number of datapoints in a 
% trajectory (T=200) multiplied by the number of demonstrations (M=5)
Data = zeros(model.nbVar, model.nbFrames, nbSamples*nbData);
for n=1:nbSamples
	for m=1:model.nbFrames
		Data(:,m,(n-1)*nbData+1:n*nbData) = s(n).p(m).A \ (s(n).Data0 - repmat(s(n).p(m).b, 1, nbData));
	end
end


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM:');
%model = init_tensorGMM_kmeans(Data, model); 
model = init_tensorGMM_timeBased(Data, model); 
model = EM_tensorGMM(Data, model);

%Precomputation of covariance inverses
for m=1:model.nbFrames 
	for i=1:model.nbStates
		model.invSigma(:,:,m,i) = inv(model.Sigma(:,:,m,i));
	end
end


%% Reproductions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with GMR...');
DataIn(1,:) = s(1).Data(1,:); %1:nbData;
in = 1;
out = 2:model.nbVar;
MuGMR = zeros(length(out), nbData, model.nbFrames);
SigmaGMR = zeros(length(out), length(out), nbData, model.nbFrames);
		
%Gaussian mixture regression
for m=1:model.nbFrames 
	%Compute activation weights
	for i=1:model.nbStates
		H(i,:) = model.Priors(i) * gaussPDF(DataIn, model.Mu(in,m,i), model.Sigma(in,in,m,i));
	end
	H = H ./ (repmat(sum(H),model.nbStates,1)+realmin);

	for t=1:nbData
		%Compute conditional means
		for i=1:model.nbStates
			MuTmp(:,i) = model.Mu(out,m,i) + model.Sigma(out,in,m,i) / model.Sigma(in,in,m,i) * (DataIn(:,t) - model.Mu(in,m,i));
			MuGMR(:,t,m) = MuGMR(:,t,m) + H(i,t) * MuTmp(:,i);
		end
		%Compute conditional covariances
		for i=1:model.nbStates
			SigmaTmp = model.Sigma(out,out,m,i) - model.Sigma(out,in,m,i) / model.Sigma(in,in,m,i) * model.Sigma(in,out,m,i);
			SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
		end
		SigmaGMR(:,:,t,m) = SigmaGMR(:,:,t,m) - MuGMR(:,t,m) * MuGMR(:,t,m)' + eye(length(out)) * model.params_diagRegFact; 
	end
end

for n=1:nbSamples+nbRepros
	MuTmp = zeros(length(out), nbData, model.nbFrames);
	SigmaTmp = zeros(length(out), length(out), nbData, model.nbFrames);
	
	%Set context parameters
	if n<=nbSamples
		%Reproductions for the task parameters used to train the model
		pTmp = s(n).p;
	else
		%Reproductions for new random task parameters
		for m=1:model.nbFrames
			id = ceil(rand(2,1)*nbSamples);
			w = rand(2); 
			w = w / sum(w);
			pTmp(m).b = s(id(1)).p(m).b * w(1) + s(id(2)).p(m).b * w(2);
			pTmp(m).A = s(id(1)).p(m).A * w(1) + s(id(2)).p(m).A * w(2);
		end
	end
	r(n).p = pTmp;

	%Linear transformation of the retrieved Gaussians
	for m=1:model.nbFrames
		MuTmp(:,:,m) = pTmp(m).A(2:end,2:end) * MuGMR(:,:,m) + repmat(pTmp(m).b(2:end),1,nbData);
		for t=1:nbData
			SigmaTmp(:,:,t,m) = pTmp(m).A(2:end,2:end) * SigmaGMR(:,:,t,m) * pTmp(m).A(2:end,2:end)';
		end
	end
	
	%Product of Gaussians (fusion of information from the different coordinate systems)
	for t=1:nbData
		SigmaP = zeros(length(out));
		MuP = zeros(length(out), 1);
		for m=1:model.nbFrames
			SigmaP = SigmaP + inv(SigmaTmp(:,:,t,m));
			MuP = MuP + SigmaTmp(:,:,t,m) \ MuTmp(:,t,m);
		end
		r(n).Sigma(:,:,t) = inv(SigmaP);
		r(n).Data(:,t) = r(n).Sigma(:,:,t) * MuP;
	end
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,2300,900]);
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
		plot([s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,3)], [s(n).p(m).b(3) s(n).p(m).b(3)+s(n).p(m).A(3,3)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(2), s(n).p(m).b(3),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
	plot(s(n).Data(2,1), s(n).Data(3,1),'.','markersize',12,'color',clrmap(n,:));
	plot(s(n).Data(2,:), s(n).Data(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%REPROS
subplot(1,3,2); hold on; box on; title('Reproductions with GMR');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([r(n).p(m).b(2) r(n).p(m).b(2)+r(n).p(m).A(2,3)], [r(n).p(m).b(3) r(n).p(m).b(3)+r(n).p(m).A(3,3)], '-','linewidth',6,'color',colPegs(m,:));
		plot(r(n).p(m).b(2), r(n).p(m).b(3),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbSamples
	%Plot Gaussians
	plotGMM(r(n).Data(:,1:5:end), r(n).Sigma(:,:,1:5:end), clrmap(n,:), .05);
end
for n=1:nbSamples
	%Plot trajectories
	plot(r(n).Data(1,1), r(n).Data(2,1),'.','markersize',12,'color',clrmap(n,:));
	plot(r(n).Data(1,:), r(n).Data(2,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%NEW REPROS
subplot(1,3,3); hold on; box on; title('New reproductions with GMR');
for n=1:nbRepros
	%Plot frames
	for m=1:model.nbFrames
		plot([r(nbSamples+n).p(m).b(2) r(nbSamples+n).p(m).b(2)+r(nbSamples+n).p(m).A(2,3)], [r(nbSamples+n).p(m).b(3) r(nbSamples+n).p(m).b(3)+r(nbSamples+n).p(m).A(3,3)], '-','linewidth',6,'color',colPegs(m,:));
		plot(r(nbSamples+n).p(m).b(2), r(nbSamples+n).p(m).b(3), '.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbRepros
	%Plot trajectories
	plot(r(nbSamples+n).Data(1,1), r(nbSamples+n).Data(2,1),'.','markersize',12,'color',[.2 .2 .2]);
	plot(r(nbSamples+n).Data(1,:), r(nbSamples+n).Data(2,:),'-','linewidth',1.5,'color',[.2 .2 .2]);
end
for n=1:nbRepros
	%Plot Gaussians
	plotGMM(r(nbSamples+n).Data(:,1:5:end), r(nbSamples+n).Sigma(:,:,1:5:end), [.2 .2 .2], .05);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);


%print('-dpng','graphs/demo_TPGMR01.png');
pause;
close all;


