function demo_DTW01
% Trajectory realignment through dynamic time warping (DTW).
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
nbData = 200; %Length of each trajectory
wMax = 50; %Warping time window 
nbSamples = 5; %Number of demonstrations
nbVar = 2; %Number of dimensions (max 2 for AMARSI data)


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/G.mat');
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos(1:nbVar,:), linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
end


%% Dynamic time warping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r(1).Data = s(1).Data;
for n=2:nbSamples
	[r(1).Data, r(n).Data, r(n-1).wPath] = DTW(r(1).Data, s(n).Data, wMax);
	%Realign previous trajectories
	p = r(n-1).wPath(1,:);
	for m=2:n-1
		DataTmp = r(m).Data(:,p);
		r(m).Data = spline(1:size(DataTmp,2), DataTmp, linspace(1,size(DataTmp,2),nbData)); %Resampling
	end
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1000,500]); 
for k=1:nbVar
	subplot(2,nbVar,(k-1)*2+1); hold on; if k==1 title('Before DTW'); end;
	for n=1:nbSamples
		plot(s(n).Data(k,:), '-','linewidth',1,'color',[0 0 0]);
	end
	xlabel('t'); ylabel(['x_' num2str(k)]);
	axis tight; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
	subplot(2,nbVar,(k-1)*2+2); hold on; if k==1 title('After DTW'); end;
	for n=1:nbSamples
		plot(r(n).Data(k,:), '-','linewidth',1,'color',[0 0 0]);
	end
	xlabel('t'); ylabel(['x_' num2str(k)]);
	axis tight; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
end

figure; hold on;
for n=1:nbSamples-1
	plot(r(n).wPath(1,:),r(n).wPath(2,:),'-','color',[0 0 0]);
end
xlabel('w_1'); ylabel('w_2');

%print('-dpng','graphs/demo_DTW01.png');
%pause;
%close all;
