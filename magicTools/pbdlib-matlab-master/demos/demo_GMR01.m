function demo_GMR01
% Gaussian mixture model (GMM) with time-based Gaussian mixture regression (GMR) 
% used for reproduction.
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
model.nbStates = 4; %Number of states in the GMM
model.nbVar = 3; %Number of variables [t,x1,x2]
model.dt = 0.001; %Time step duration
nbData = 200; %Length of each trajectory
nbSamples = 5; %Number of demonstrations


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/G.mat');
Data=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	Data = [Data [[1:nbData]*model.dt; s(n).Data]]; 
end


%% Learning and reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%model = init_GMM_kmeans(Data, model);
model = init_GMM_timeBased(Data, model);
model = EM_GMM(Data, model);
[DataOut, SigmaOut] = GMR(model, [1:nbData]*model.dt, 1, 2:model.nbVar); %see Eq. (17)-(19)


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,500]); 
%Plot GMM
subplot(1,2,1); hold on; axis off; title('GMM');
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.5 .5 .5]);
plotGMM(model.Mu(2:model.nbVar,:), model.Sigma(2:model.nbVar,2:model.nbVar,:), [.8 0 0], .5);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%Plot GMR
subplot(1,2,2); hold on; axis off; title('GMR');
plot(Data(2,:),Data(3,:),'.','markersize',8,'color',[.5 .5 .5]);
plotGMM(DataOut, SigmaOut, [0 .8 0], .03);
plot(DataOut(1,:),DataOut(2,:),'-','linewidth',2,'color',[0 .4 0]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

%print('-dpng','graphs/demo_GMR01.png');
%pause;
%close all;
