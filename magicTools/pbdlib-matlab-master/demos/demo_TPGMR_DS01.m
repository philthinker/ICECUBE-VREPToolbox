function demo_TPGMR_DS01
% Task-parameterized probabilistic model encoding movements in the form of virtual spring-damper
% systems acting in multiple frames of reference. Each candidate coordinate system observes a set of
% demonstrations from its own perspective, by extracting an attractor path whose variations depend on the
% relevance of the frame through the task. This information is exploited to generate a new attractor path
% corresponding to new situations (new positions and orientation of the frames).
% This example presents the results for a time-based GMR reference retrieval process combined with a tracking system 
% with constant gains.
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
% @inproceedings{Calinon14,
%   author="Calinon, S. and Bruno, D. and Caldwell, D. G.",
%   title="A task-parameterized probabilistic model with minimal intervention control",
%   booktitle="Proc. {IEEE} Intl Conf. on Robotics and Automation ({ICRA})",
%   year="2014",
%   month="May-June",
%   address="Hong Kong, China",
%   pages="3339--3344"
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 3; %Number of Gaussians in the GMM
model.nbFrames = 2; %Number of candidate frames of reference
model.nbVar = 3; %Dimension of the datapoints in the dataset (here: t,x1,x2)
model.dt = 0.01; %Time step
model.kP = 100; %Stiffness gain
model.kV = (2*model.kP)^.5; %Damping gain (with ideal underdamped damping ratio)
nbRepros = 8; %Number of reproductions with new situations randomly generated


%% Load 3rd order tensor data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Load 3rd order tensor data...');
% The MAT file contains a structure 's' with the multiple demonstrations. 's(n).Data' is a matrix data for
% sample n (with 's(n).nbData' datapoints). 's(n).p(m).b' and 's(n).p(m).A' contain the position and
% orientation of the m-th candidate coordinate system for this demonstration. 'Data' contains the observations
% in the different frames. It is a 3rd order tensor of dimension D x P x N, with D=3 the dimension of a
% datapoint, P=2 the number of candidate frames, and N=TM the number of datapoints in a trajectory (T=200)
% multiplied by the number of demonstrations (M=5).
load('data/DataLQR01.mat');


%% Transformation of 'Data' to learn the path of the spring-damper system
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbD = s(1).nbData;
nbVarOut = model.nbVar - 1;
%Create transformation matrix to compute [X; DX; DDX]
D = (diag(ones(1,nbD-1),-1)-eye(nbD)) / model.dt;
D(end,end) = 0;
%Create transformation matrix to compute XHAT = X + DX*kV/kP + DDX/kP
K1d = [1, model.kV/model.kP, 1/model.kP];
K = kron(K1d,eye(nbVarOut));
%Create 3rd order tensor data with XHAT instead of X
for n=1:nbSamples
	DataTmp = s(n).Data0(2:end,:);
	DataTmp = [s(n).Data0(1,:); K * [DataTmp; DataTmp*D; DataTmp*D*D]];
	for m=1:model.nbFrames
		Data(:,m,(n-1)*nbD+1:n*nbD) = s(n).p(m).A \ (DataTmp - repmat(s(n).p(m).b, 1, nbD));
	end
end


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM:');
model = init_tensorGMM_timeBased(Data, model); %Initialization
model = EM_tensorGMM(Data, model);


%% Reproduction with DS-GMR for the task parameters used to train the model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with DS-GMR...');
DataIn = [1:s(1).nbData] * model.dt;
for n=1:nbSamples
	%Retrieval of attractor path through task-parameterized GMR
	a(n) = estimateAttractorPath(DataIn, model, s(n));
	r(n) = reproduction_DS(DataIn, model, a(n), a(n).currTar(:,1));
end


%% Reproduction with DS-GMR for new task parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('New reproductions with DS-GMR...');
for n=1:nbRepros
	for m=1:model.nbFrames
		%Random generation of new task parameters
		id=ceil(rand(2,1)*nbSamples);
		w=rand(2); w=w/sum(w);
		rTmp.p(m).b = s(id(1)).p(m).b * w(1) + s(id(2)).p(m).b * w(2);
		rTmp.p(m).A = s(id(1)).p(m).A * w(1) + s(id(2)).p(m).A * w(2);
	end
	%Retrieval of attractor path through task-parameterized GMR
	anew(n) = estimateAttractorPath(DataIn, model, rTmp);
	rnew(n) = reproduction_DS(DataIn, model, anew(n), anew(n).currTar(:,1));
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
		plot([s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,3)], [s(n).p(m).b(3) s(n).p(m).b(3)+s(n).p(m).A(3,3)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(2), s(n).p(m).b(3),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
	%Plot trajectories
	plot(s(n).Data0(2,1), s(n).Data0(3,1),'.','markersize',12,'color',clrmap(n,:));
	plot(s(n).Data0(2,:), s(n).Data0(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%REPROS
subplot(1,3,2); hold on; box on; title('Reproductions with DS-GMR');
for n=1:nbSamples
	%Plot frames
	for m=1:model.nbFrames
		plot([s(n).p(m).b(2) s(n).p(m).b(2)+s(n).p(m).A(2,3)], [s(n).p(m).b(3) s(n).p(m).b(3)+s(n).p(m).A(3,3)], '-','linewidth',6,'color',colPegs(m,:));
		plot(s(n).p(m).b(2), s(n).p(m).b(3),'.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbSamples
	%Plot trajectories
	plot(r(n).Data(2,1), r(n).Data(3,1),'.','markersize',12,'color',clrmap(n,:));
	plot(r(n).Data(2,:), r(n).Data(3,:),'-','linewidth',1.5,'color',clrmap(n,:));
end
for n=1:nbSamples
	%Plot Gaussians
	plotGMM(r(n).Mu(2:3,:,1), r(n).Sigma(2:3,2:3,:,1), [.5 .5 .5],.8);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%NEW REPROS
subplot(1,3,3); hold on; box on; title('New reproductions with DS-GMR');
for n=1:nbRepros
	%Plot frames
	for m=1:model.nbFrames
		plot([rnew(n).p(m).b(2) rnew(n).p(m).b(2)+rnew(n).p(m).A(2,3)], [rnew(n).p(m).b(3) rnew(n).p(m).b(3)+rnew(n).p(m).A(3,3)], '-','linewidth',6,'color',colPegs(m,:));
		plot(rnew(n).p(m).b(2), rnew(n).p(m).b(3), '.','markersize',30,'color',colPegs(m,:)-[.05,.05,.05]);
	end
end
for n=1:nbRepros
	%Plot trajectories
	plot(rnew(n).Data(2,1), rnew(n).Data(3,1),'.','markersize',12,'color',[.2 .2 .2]);
	plot(rnew(n).Data(2,:), rnew(n).Data(3,:),'-','linewidth',1.5,'color',[.2 .2 .2]);
end
for n=1:nbRepros
	%Plot Gaussians
	plotGMM(rnew(n).Mu(2:3,:,1), rnew(n).Sigma(2:3,2:3,:,1), [.5 .5 .5],.8);
end
axis(limAxes); axis square; set(gca,'xtick',[],'ytick',[]);

%print('-dpng','graphs/demo_TPGMR_DS01.png');


%% Plot additional information
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
%Plot norm of control commands
subplot(1,2,1); hold on;
for n=1:nbRepros
	plot(DataIn, rnew(n).ddxNorm, 'k-', 'linewidth', 2);
end
xlabel('t'); ylabel('|ddx|');
%Plot strength of the stiffness term
subplot(1,2,2); hold on;
for n=1:nbRepros
	plot(DataIn, rnew(n).kpDet, 'k-', 'linewidth', 2);
end
xlabel('t'); ylabel('|Kp|');

%pause;
%close all;


