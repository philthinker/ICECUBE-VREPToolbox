function demo_Riemannian_sphere_GMM01
% GMM for data on a sphere by relying on Riemannian manifold. 
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please cite the related publications.
%
% @article{Zeestraten17RAL,
%   author="Zeestraten, M. J. A. and Havoutis, I. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
%   title="An Approach for Imitation Learning on Riemannian Manifolds",
%   journal="{IEEE} Robotics and Automation Letters ({RA-L})",
%   doi="",
%   year="2017",
%   month="",
%   volume="",
%   number="",
%   pages=""
% }
% 
% Copyright (c) 2017 Idiap Research Institute, http://idiap.ch/
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
nbData = 50; %Number of datapoints
nbSamples = 5; %Number of demonstrations
nbIter = 10; %Number of iteration for the Gauss Newton algorithm
nbIterEM = 30; %Number of iteration for the EM algorithm
nbDrawingSeg = 30; %Number of segments used to draw ellipsoids

model.nbStates = 5; %Number of states in the GMM
model.nbVar = 2; %Dimension of the tangent space
model.nbVarMan = 3; %Dimension of the manifold
model.params_diagRegFact = 1E-4; %Regularization of covariance


%% Generate data on a sphere from handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/S.mat');
u=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	u = [u [s(n).Data*1.3E-1]]; 
end
x = expmap(u, [0; -1; 0]);


%% GMM parameters estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kbins(u, model, nbSamples);
model.MuMan = expmap(model.Mu, [0; -1; 0]); %Center on the manifold
model.Mu = zeros(model.nbVar,model.nbStates); %Center in the tangent plane at point MuMan of the manifold

for nb=1:nbIterEM
	%E-step
	L = zeros(model.nbStates,size(x,2));
	for i=1:model.nbStates
		L(i,:) = model.Priors(i) * gaussPDF(logmap(x, model.MuMan(:,i)), model.Mu(:,i), model.Sigma(:,:,i));
	end
	GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
	H = GAMMA ./ repmat(sum(GAMMA,2),1,nbData*nbSamples);
	%M-step
	for i=1:model.nbStates
		%Update Priors
		model.Priors(i) = sum(GAMMA(i,:)) / (nbData*nbSamples);
		%Update MuMan
		for n=1:nbIter
			u(:,:,i) = logmap(x, model.MuMan(:,i));
			model.MuMan(:,i) = expmap(u(:,:,i)*H(i,:)', model.MuMan(:,i));
		end
		%Update Sigma
		model.Sigma(:,:,i) = u(:,:,i) * diag(H(i,:)) * u(:,:,i)' + eye(size(u,1)) * model.params_diagRegFact;
	end
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clrmap = lines(model.nbStates);

%Display of covariance contours on the sphere
tl = linspace(-pi, pi, nbDrawingSeg);
Gdisp = zeros(model.nbVarMan, nbDrawingSeg, model.nbStates);
for i=1:model.nbStates
	[V,D] = eig(model.Sigma(:,:,i));
	Gdisp(:,:,i) = expmap(V*D.^.5*[cos(tl); sin(tl)], model.MuMan(:,i));
end

%Manifold plot
figure('PaperPosition',[0 0 8 8],'position',[10,10,650,650]); hold on; axis off; grid off; rotate3d on; 
colormap([.9 .9 .9]);
[X,Y,Z] = sphere(20);
mesh(X,Y,Z);
%plot3(x(1,:), x(2,:), x(3,:), '--','markersize',12,'color',[0 0 0]);
for t=1:nbData*nbSamples
	plot3(x(1,t), x(2,t), x(3,t), '.','markersize',12,'color',GAMMA(:,t)'*clrmap);
end
for i=1:model.nbStates
	plot3(model.MuMan(1,i), model.MuMan(2,i), model.MuMan(3,i), '.','markersize',20,'color',clrmap(i,:));
	plot3(Gdisp(1,:,i), Gdisp(2,:,i), Gdisp(3,:,i), '-','linewidth',1,'color',clrmap(i,:));
% 	%Draw tangent plane
% 	msh = repmat(model.MuMan(:,i),1,5) + rotM(model.MuMan(:,i))' * [1 1 -1 -1 1; 1 -1 -1 1 1; 0 0 0 0 0] * 1E0;
% 	patch(msh(1,:),msh(2,:),msh(3,:), [.8 .8 .8],'edgecolor',[.6 .6 .6],'facealpha',.3,'edgealpha',.3);
end
view(-20,2); axis equal; axis tight; axis vis3d;  
%print('-dpng','graphs/demo_Riemannian_sphere_GMM01a.png');

%Tangent space plot
figure('PaperPosition',[0 0 6 8],'position',[670,10,650,650]); 
for i=1:model.nbStates
	subplot(ceil(model.nbStates/2),2,i); hold on; axis off; title(['k=' num2str(i)]);
	plot(0,0,'+','markersize',40,'linewidth',1,'color',[.7 .7 .7]);
	%plot(u(1,:,i), u(2,:,i), '.','markersize',6,'color',[0 0 0]);
	for t=1:nbData*nbSamples
		plot(u(1,t,i), u(2,t,i), '.','markersize',6,'color',GAMMA(:,t)'*clrmap); %w(1)*[1 0 0] + w(2)*[0 1 0]
	end
	plotGMM(model.Mu(:,i), model.Sigma(:,:,i)*3, clrmap(i,:), .3);
	axis equal; axis tight;
	
	%Plot contours of Gaussian j in tangent plane of Gaussian i (version 1)
	for j=1:model.nbStates
		if j~=i
			udisp = logmap(Gdisp(:,:,j), model.MuMan(:,i));
			patch(udisp(1,:), udisp(2,:), clrmap(j,:),'lineWidth',1,'EdgeColor',clrmap(j,:)*0.5,'facealpha',.1,'edgealpha',.1);	
		end
	end
end
%print('-dpng','graphs/demo_Riemannian_sphere_GMM01b.png');

% pause;
% close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = expmap(u, mu)
	x = rotM(mu)' * expfct(u);
end

function u = logmap(x, mu)
	if norm(mu-[0;0;-1])<1e-6
		R = [1 0 0; 0 -1 0; 0 0 -1];
	else
		R = rotM(mu);
	end
	u = logfct(R*x);
end

function Exp = expfct(u)
	normv = sqrt(u(1,:).^2+u(2,:).^2);
	Exp = real([u(1,:).*sin(normv)./normv; u(2,:).*sin(normv)./normv; cos(normv)]);
	Exp(:,normv < 1e-16) = repmat([0;0;1],1,sum(normv < 1e-16));	
end

function Log = logfct(x)
	scale = acos(x(3,:)) ./ sqrt(1-x(3,:).^2);
	scale(isnan(scale)) = 1;
	Log = [x(1,:).*scale; x(2,:).*scale];	
end
