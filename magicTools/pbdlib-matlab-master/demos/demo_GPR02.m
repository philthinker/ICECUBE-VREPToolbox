function demo_GPR02
% Gaussian process regression (GPR) with stochastic samples from the prior and the posterior.
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
nbVar = 2; %Dimension of datapoint (t,x1)
nbData = 4; %Number of datapoints
nbDataRepro = 100; %Number of datapoints for reproduction
nbRepros = 20; %Number of reproductions
p(1)=1E0; p(2)=1E-1; p(3)=1E-2; %GPR parameters


%% Generate data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Data = rand(2,nbData);
Data = [linspace(0,1,nbData); rand(1,nbData)-0.5];
%GPR precomputation
xIn = Data(1,:);
xOut = Data(2:end,:);
M = pdist2(xIn', xIn');
K = p(1) * exp(-p(2)^-1 * M.^2);
invK = pinv(K + p(3) * eye(size(K))); 


%% Reproduction with GPR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Mean trajectory computation
xInHat = linspace(0,1,nbDataRepro);
Md = pdist2(xInHat', xIn');
Kd = p(1) * exp(-p(2)^-1 * Md.^2);
r(1).Data = [xInHat; (Kd * invK * xOut')']; 
%Covariance computation
Mdd = pdist2(xInHat',xInHat');
Kdd = p(1) * exp(-p(2)^-1 * Mdd.^2);
%Kdd = Kdd + p(3) * eye(size(Kdd)); 
S = Kdd - Kd * invK * Kd';
r(1).SigmaOut = zeros(nbVar-1,nbVar-1,nbData);
for t=1:nbDataRepro
	r(1).SigmaOut(:,:,t) = eye(nbVar-1) * S(t,t); 
end

%Generate stochastic samples from the prior 
[V,D] = eig(Kdd);
for n=2:nbRepros/2
	DataOut = real(V*D^.5) * randn(nbDataRepro,1)*0.4;  
	r(n).Data = [xInHat; DataOut'];
end
%Generate stochastic samples from the posterior 
[V,D] = eig(S);
for n=nbRepros/2+1:nbRepros
	DataOut = real(V*D^.5) * randn(nbDataRepro,1)*0.5 + r(1).Data(2,:)';  
	r(n).Data = [xInHat; DataOut'];
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 12 4],'position',[10 10 1300 600]); 
%Prior samples
subplot(1,3,1); hold on; title('Samples from prior','fontsize',14);
for n=2:nbRepros/2
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','lineWidth',3.5,'color',[.9 .9 .9]*rand(1));
end
set(gca,'xtick',[],'ytick',[]); axis([0, 1, -1.2 1.2]);
xlabel('$x_1$','interpreter','latex','fontsize',18);
ylabel('$y_1$','interpreter','latex','fontsize',18);

%Posterior samples
subplot(1,3,2); hold on;  title('Samples from posterior','fontsize',14);
for n=nbRepros/2+1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','lineWidth',3.5,'color',[.9 .9 .9]*rand(1));
end
plot(Data(1,:), Data(2,:), '.','markersize',24,'color',[1 0 0]);
set(gca,'xtick',[],'ytick',[]); axis([0, 1, -1.2 1.2]);
xlabel('$x_1$','interpreter','latex','fontsize',18);
ylabel('$y_1$','interpreter','latex','fontsize',18);

%Trajectory distribution
subplot(1,3,3); hold on;  title('Trajectory distribution','fontsize',14);
patch([r(1).Data(1,:), r(1).Data(1,end:-1:1)], ...
	[r(1).Data(2,:)+squeeze(r(1).SigmaOut.^.5)', r(1).Data(2,end:-1:1)-squeeze(r(1).SigmaOut(:,:,end:-1:1).^.5)'], ...
	[.8 .8 .8],'edgecolor','none');
plot(r(1).Data(1,:), r(1).Data(2,:), '-','lineWidth',3.5,'color',[0 0 0]);
plot(Data(1,:), Data(2,:), '.','markersize',24,'color',[1 0 0]);
set(gca,'xtick',[],'ytick',[]); axis([0, 1, -1.2 1.2]);
xlabel('$x_1$','interpreter','latex','fontsize',18);
ylabel('$y_1$','interpreter','latex','fontsize',18);

%print('-dpng','graphs/GPR02.png');
%pause;
%close all;

