function demo_GMR_polyFit01
% Polynomial fitting with multivariate GMR
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
model.nbStates = 3; %Number of states in the GMM
model.nbVarIn = 3; %Dimension of input vector
model.nbVarOut = 1; %Dimension of output vector
model.nbVar = model.nbVarIn + model.nbVarOut; %Number of variables (input+output)
nbData = 200; %Length of a trajectory


%% Load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/DataLS01.mat');
X = [];
for i=1:model.nbVarIn
	X = [X, x.^i]; %-> X=[x, x.^2, x.^3]
end
Data = [X'; Y'] + randn(model.nbVar,size(X,1))*1E-5;


%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kmeans(Data, model);
model = EM_GMM(Data, model);
model.Sigma0 = model.Sigma;
%Regularization term on the inputs
model.Sigma(1:model.nbVarIn,1:model.nbVarIn,:) = model.Sigma(1:model.nbVarIn,1:model.nbVarIn,:) + repmat(eye(model.nbVarIn)*1E3,[1,1,model.nbStates]);

%Regression
xr = linspace(min(x),max(x),nbData);
DataIn = [];
for i=1:model.nbVarIn
	DataIn = [DataIn; xr.^i]; %-> X=[x, x.^2, x.^3]
end
DataOut = GMR(model, DataIn, 1:model.nbVarIn, model.nbVarIn+1:model.nbVar); 


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,500]); hold on; %axis off; 
plotGMM(model.Mu([1,end],:), model.Sigma0([1,end],[1,end],:), [.8 .8 .8]);
plot(DataIn(1,:),DataOut(1,:),'-','linewidth',2,'color',[1 .6 .6]);
plot(DataIn(1,:),DataOut(1,:),'.','markersize',6,'color',[.8 0 0]);
plot(Data(1,:),Data(end,:),'.','markersize',16,'color',[.2 .2 .2]);
xlabel('x_1'); ylabel('y_1');
axis([min(DataIn(1,:))-0.1, max(DataIn(1,:))+0.1, min(DataOut(1,:))-0.1, max(DataOut(1,:))+0.1]);


%print('-dpng','graphs/demo_GMRpolyFit01.png');
%pause;
%close all;
