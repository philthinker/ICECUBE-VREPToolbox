function demo_testLQR01
% Test of the linear quadratic regulation with different variance in the data.
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
model.nbVar = 2; %Dimension of the datapoints in the dataset (here: t,x1)
model.dt = 0.01; %Time step
nbData = 500; %Number of datapoints
nbRepros = 3; %Number of reproductions with new situations randomly generated
rFactor = 1E-2; %Weighting term for the minimization of control commands in LQR


%% Reproduction with LQR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Reproductions with LQR...');
DataIn = [1:nbData] * model.dt;
a.currTar = ones(1,nbData);
for n=1:nbRepros
	a.currSigma = ones(1,1,nbData) * 10^(2-n);
	%r(n) = reproduction_LQR_finiteHorizon(model, a, 0, rFactor);
	r(n) = reproduction_LQR_infiniteHorizon(model, a, 0, rFactor);
	r(n).Data = [DataIn; r(n).Data];
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,50,1300,500]);
hold on; box on;
%Plot target
plot(r(1).Data(1,:), a.currTar, 'r-', 'linewidth', 2);
for n=1:nbRepros
	%Plot trajectories
	plot(r(n).Data(1,:), r(n).Data(2,:), '-', 'linewidth', 2, 'color', ones(3,1)*(n-1)/nbRepros);
end
xlabel('t'); ylabel('x_1');

figure;
%Plot norm of control commands
subplot(1,2,1); hold on;
for n=1:nbRepros
	plot(DataIn, r(n).ddxNorm, '-', 'linewidth', 2, 'color', ones(3,1)*(n-1)/nbRepros);
end
xlabel('t'); ylabel('|ddx|');
%Plot stiffness
subplot(1,2,2); hold on;
for n=1:nbRepros
	plot(DataIn, r(n).kpDet, '-', 'linewidth', 2, 'color', ones(3,1)*(n-1)/nbRepros);
end
xlabel('t'); ylabel('kp');

%print('-dpng','graphs/demo_testLQR01.png');
%pause;
%close all;

