function demo_TPGMM01
% Task-parameterized Gaussian mixture model (TP-GMM) encoding.
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
model.nbVar = 2; %Dimension of the datapoints in the dataset (here: x1,x2)


%% Load 3rd order tensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% The MAT file contains a structure 's' with the multiple demonstrations. 's(n).Data' is a matrix data for
% sample n (with 's(n).nbData' datapoints). 's(n).p(m).b' and 's(n).p(m).A' contain the position and
% orientation of the m-th candidate coordinate system for this demonstration. 'Data' contains the observations
% in the different frames. It is a 3rd order tensor of dimension D x P x N, with D=2 the dimension of a
% datapoint, P=2 the number of candidate frames, and N=TM the number of datapoints in a trajectory (T=200)
% multiplied by the number of demonstrations (M=5).
load('data/Data01.mat');

% %Regenerate data
% Data = zeros(model.nbVar, model.nbFrames, nbSamples*nbData);
% for n=1:nbSamples
% 	s(n).Data0 = s(n).Data0(2:end,:); %Remove time
% 	for m=1:model.nbFrames
% 		Data(:,m,(n-1)*nbData+1:n*nbData) = s(n).p(m).A \ (s(n).Data0 - repmat(s(n).p(m).b, 1, nbData));
% 	end
% end


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM:');
model = init_tensorGMM_kmeans(Data, model); 
model = EM_tensorGMM(Data, model);

%Reconstruct GMM for each demonstration
for n=1:nbSamples
	[s(n).Mu, s(n).Sigma] = productTPGMM0(model, s(n).p);
end

% %Products of linearly transformed Gaussians
% for n=1:nbSamples
% 	for i=1:model.nbStates
% 		SigmaTmp = zeros(model.nbVar);
% 		MuTmp = zeros(model.nbVar,1);
% 		for m=1:model.nbFrames
% 			MuP = s(n).p(m).A * model.Mu(:,m,i) + s(n).p(m).b;
% 			SigmaP = s(n).p(m).A * model.Sigma(:,:,m,i) * s(n).p(m).A';
% 			SigmaTmp = SigmaTmp + inv(SigmaP);
% 			MuTmp = MuTmp + SigmaP\MuP;
% 		end
% 		s(n).Sigma(:,:,i) = inv(SigmaTmp);
% 		s(n).Mu(:,i) = s(n).Sigma(:,:,i) * MuTmp;
% 	end
% end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,1300,500]);
xx = round(linspace(1,64,nbSamples));
clrmap = colormap('jet');
clrmap = min(clrmap(xx,:),.95);
limAxes = [-1.2 0.8 -1.1 0.9];
colPegs = [[.9,.5,.9];[.5,.9,.5]];

%DEMOS
subplot(1,model.nbFrames+1,1); hold on; box on; title('Demonstrations');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(1) s(n).p(m).b(1)+s(n).p(m).A(1,2)], [s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,2)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(1), s(n).p(m).b(2),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
	plot(s(n).Data(2,1), s(n).Data(3,1),'.','markersize',15,'color',clrmap(n,:));
	plot(s(n).Data(2,:), s(n).Data(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
	%Plot Gaussians
	plotGMM(s(n).Mu, s(n).Sigma, [.5 .5 .5],.8);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%FRAMES
for m=1:model.nbFrames
	subplot(1,model.nbFrames+1,1+m); hold on; grid on; box on; title(['Frame ' num2str(m)]);
	for n=1:nbSamples
		plot(squeeze(Data(1,m,(n-1)*s(1).nbData+1)), ...
			squeeze(Data(2,m,(n-1)*s(1).nbData+1)), '.','markersize',15,'color',clrmap(n,:));
		plot(squeeze(Data(1,m,(n-1)*s(1).nbData+1:n*s(1).nbData)), ...
			squeeze(Data(2,m,(n-1)*s(1).nbData+1:n*s(1).nbData)), '-','linewidth',1.5,'color',clrmap(n,:));
	end
	plotGMM(squeeze(model.Mu(:,m,:)), squeeze(model.Sigma(:,:,m,:)), [.5 .5 .5],.8);
	axis square; set(gca,'xtick',[0],'ytick',[0]);
end


% %% Saving of data for C++ version of pbdlib
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %Save model parameters
% for m=1:model.nbFrames
% 	M = [];
% 	N = [];
% 	for i=1:model.nbStates
% 		M = [M model.Sigma(:,:,m,i)];
% 		N = [N model.Mu(:,m,i)];
% 	end
% 	save(['data/Sigma' num2str(m) '.txt'], 'M', '-ascii');
% 	save(['data/Mu' num2str(m) '.txt'], 'N', '-ascii');
% end
% save(['data/Priors.txt'], 'model.Priors', '-ascii');
% TPGMMparams = [model.nbVar; model.nbFrames; model.nbStates; model.dt];
% save(['data/Tpgmm.txt'], 'TPGMMparams', '-ascii');
% %Save task parameters
% for m=1:model.nbFrames
% 	Ab = [r(1).p(m).b'; r(1).p(m).A];
% 	save(['data/Params' num2str(m) '.txt'], 'Ab', '-ascii');
% end


%print('-dpng','graphs/demo_TPGMM01.png');
%pause;
%close all;


