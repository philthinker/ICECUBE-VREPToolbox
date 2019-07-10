function demo_HMM_Viterbi01
% Viterbi decoding in HMM to estimate best state sequence from observations.
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @article{Rozo16Frontiers,
%   author="Rozo, L. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
%   title="Learning Controllers for Reactive and Proactive Behaviors in Human-Robot Collaboration",
%   journal="Frontiers in Robotics and {AI}",
%   year="2016",
%   month="June",
%   volume="3",
%   number="30",
%   pages="1--11",
%   doi="10.3389/frobt.2016.00030"
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
model.nbStates = 3;
nbData = 6;
nbSamples = 1;


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/C.mat');
%nbSamples = length(demos);
Data=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	s(n).nbData = size(s(n).Data,2);
	Data = [Data s(n).Data]; 
end


%% Learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% model = init_GMM_kmeans(Data, model);
model = init_GMM_kbins(Data, model, nbSamples);

% %Random initialization
% model.Trans = rand(model.nbStates,model.nbStates);
% model.Trans = model.Trans ./ repmat(sum(model.Trans,2),1,model.nbStates);
% model.StatesPriors = rand(model.nbStates,1);
% model.StatesPriors = model.StatesPriors/sum(model.StatesPriors);

%Left-right model initialization
model.Trans = zeros(model.nbStates);
for i=1:model.nbStates-1
	model.Trans(i,i) = 1-(model.nbStates/nbData);
	model.Trans(i,i+1) = model.nbStates/nbData;
end
model.Trans(model.nbStates,model.nbStates) = 1.0;
model.StatesPriors = zeros(model.nbStates,1);
model.StatesPriors(1) = 1;

%Parameters refinement with EM
model = EM_HMM(s, model);

%MPE estimate of best path
H = computeGammaHMM(s(end), model);
[~,sMPE] = max(H);

%MAP estimate of best path (Viterbi)
sMAP = Viterbi_HMM(s(end).Data, model);

%Viterbi decoding illustration
gridTrans = zeros(model.nbStates,model.nbStates,6);
gridNode = zeros(model.nbStates,6);
gridInit = zeros(model.nbStates,1);
for t=1:nbData
	gridNode(sMAP(t),t) = 1;
	if t>1
		gridTrans(sMAP(t-1),sMAP(t),t-1) = 1;
	end
end
%Set colors
colTint = [0,.6,0];

%Plot HMM treillis representation
figure('PaperPosition',[0 0 15 4],'position',[10,10,1200,400],'color',[1 1 1]);
axes('Position',[0 0 1 1]); hold on; axis off;
plotHMMtrellis(model.Trans, model.StatesPriors, gridTrans, gridNode, gridInit, colTint);

%print('-dpng','graphs/demo_HMM_treillis05.png');
%pause;
%close all;
