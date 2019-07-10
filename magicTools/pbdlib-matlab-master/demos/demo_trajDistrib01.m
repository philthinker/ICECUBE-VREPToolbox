function demo_trajDistrib01
% Stochastic sampling with Gaussian trajectory distribution 
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
nbVar = 2; %Number of variables [x1,x2]
nbData = 100; %Length of each trajectory
nbSamples = 8; %Number of demonstrations
nbRepros = 50; %Number of reproductions 
nbEigs = 4; %Number of principal eigencomponents to keep


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/A.mat');
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data(:,n) = reshape(s(n).Data, nbVar*nbData, 1); 
end


%% Compute normal trajectory distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.nbStates = 1;
model.Mu = mean(Data,2); 
model.Sigma = cov(Data');
%Keep only a few principal eigencomponents
for i=1:model.nbStates
	[V,D] = eig(model.Sigma(:,:,i));
	[d,id] = sort(diag(D),'descend');
	model.D(:,:,i) = diag(d(1:nbEigs));
	model.V(:,:,i) = V(:,id(1:nbEigs));
	model.Sigma(:,:,i) = model.V(:,:,i) * model.D(:,:,i) * model.V(:,:,i)' + eye(nbVar*nbData)*0E-4; 
end


%% Stochastic sampling from normal trajectory distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Data2 = model.V * model.D.^.5 * randn(nbEigs,nbRepros) + repmat(model.Mu,1,nbRepros);


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,700,500]); hold on; axis off;
for n=1:nbSamples
	plot(Data(1:2:end,n),Data(2:2:end,n),'-','linewidth',1,'color',[.5 .5 .5]);
end
for n=1:nbRepros
	plot(Data2(1:2:end,n),Data2(2:2:end,n),'-','linewidth',1,'color',[0 .7 0]);
end
plot(model.Mu(1:2:end),model.Mu(2:2:end),'-','linewidth',2,'color',[.8 0 0]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);


%print('-dpng','graphs/demo_trajDistrib01.png');
%pause;
%close all;
