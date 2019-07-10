function demo_Riemannian_sphere_GaussProd01
% Product of Gaussians on a sphere by relying on Riemannian manifold. 
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
nbDrawingSeg = 30; %Number of segments used to draw ellipsoids

model.nbStates = 2; %Number of states in the GMM
model.nbVar = 2; %Dimension of the tangent space
model.nbVarMan = 3; %Dimension of the manifold
model.params_diagRegFact = 1E-3; %Regularization of covariance


%% Setting GMM parameters explicitly
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.Priors = ones(1,model.nbStates) / model.nbStates;

model.MuMan(:,1) = [-1; .2; .6];
model.MuMan(:,2) = [.5; .5; .8];

for i=1:model.nbStates
	model.MuMan(:,i) = model.MuMan(:,i) / norm(model.MuMan(:,i));
end

model.Mu = zeros(model.nbVar,model.nbStates);

model.Sigma(:,:,1) = diag([.1,15]) * 1E-1;
model.Sigma(:,:,2) = diag([.1,15]) * 1E-1;


%% Sampling from GMM 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nb = round(nbData*nbSamples/model.nbStates);
x = [];
for i=1:model.nbStates
	if i==model.nbStates
		nb = nbData*nbSamples - (model.nbStates-1)*nb;
	end
	[V,D] = eig(model.Sigma(:,:,i));
	utmp = V*D.^.5 * randn(model.nbVar,nb);
	x = [x, expmap(utmp, model.MuMan(:,i))];
end
%Compute points on tangent spaces
for i=1:model.nbStates
	u(:,:,i) = logmap(x, model.MuMan(:,i));
end
%Compute likelihoods
L = zeros(model.nbStates,size(x,2));
for i=1:model.nbStates
	L(i,:) = model.Priors(i) * gaussPDF(logmap(x, model.MuMan(:,i)), model.Mu(:,i), model.Sigma(:,:,i));
end
GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
	
%Eigendecomposition of Sigma
for i=1:model.nbStates
	[V,D] = eig(model.Sigma(:,:,i));
	U0(:,:,i) = V * D.^.5;
end


%% Product of Gaussians (version transporting the covariances)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MuMan = [-1; -1; 0];
MuMan = MuMan / norm(MuMan);

%Compute MuMan incrementally
MuTmp = zeros(model.nbVar,model.nbStates);
SigmaTmp = zeros(model.nbVar,model.nbVar,model.nbStates);
for n=1:nbIter
	Mu = zeros(model.nbVar,1);
	SigmaSum = zeros(model.nbVar);
	for i=1:model.nbStates
		%Transportation of covariance from model.MuMan(:,i) to MuMan 
		Ac = transp(model.MuMan(:,i), MuMan);
		U1 = Ac * U0(:,:,i);
		SigmaTmp(:,:,i) = U1 * U1';
		%Tracking component for Gaussian i
		SigmaSum = SigmaSum + inv(SigmaTmp(:,:,i));
		MuTmp(:,i) = logmap(model.MuMan(:,i), MuMan);
		Mu = Mu + SigmaTmp(:,:,i) \ MuTmp(:,i);
	end
	Sigma = inv(SigmaSum);
	%Gradient computation
	Mu = Sigma * Mu;	
	%Keep an history for plotting
	hist(n).MuMan = MuMan;
	hist(n).Sigma = Sigma;
	%Update MuMan
	MuMan = expmap(Mu, MuMan);
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
for n=1:nbIter
	[V,D] = eig(hist(n).Sigma);
	hist(n).Gdisp2 = expmap(V*D.^.5*[cos(tl); sin(tl)], hist(n).MuMan);
end

%Manifold plot
figure('position',[10,10,650,650]); hold on; axis off; grid off; rotate3d on; 
colormap(repmat(linspace(1,.2,64),3,1)');

%colored sphere
nbp = 40;
[X,Y,Z] = sphere(nbp-1);
p = [reshape(X,1,nbp^2); reshape(Y,1,nbp^2); reshape(Z,1,nbp^2)];
c = zeros(nbp^2,1);
for i=1:model.nbStates
	dtmp = logmap(p,model.MuMan(:,i))';
	c = c + sum((dtmp/model.Sigma(:,:,i)).*dtmp, 2);
end
surf(X,Y,Z,reshape(c,nbp,nbp),'linestyle','none');

for n=1:nbData*nbSamples
	plot3(x(1,n), x(2,n), x(3,n), '.','markersize',12,'color',GAMMA(:,n)'*clrmap);
end
for i=1:model.nbStates
	plot3(model.MuMan(1,i), model.MuMan(2,i), model.MuMan(3,i), '.','markersize',12,'color',clrmap(i,:));
	plot3(Gdisp(1,:,i), Gdisp(2,:,i), Gdisp(3,:,i), '-','linewidth',3,'color',clrmap(i,:));
end

%Plot history
for n=1:nbIter
	coltmp = [.3 1 .3] * (nbIter-n)/nbIter;
	plot3(hist(n).MuMan(1), hist(n).MuMan(2), hist(n).MuMan(3), '.','markersize',12,'color',coltmp);
	plot3(hist(n).Gdisp2(1,:), hist(n).Gdisp2(2,:), hist(n).Gdisp2(3,:), '-','linewidth',1,'color',coltmp);
end
view(-150,20); axis equal; axis vis3d;  
%print('-dpng','graphs/demo_Riemannian_sphere_GaussProd01.png');

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

function Ac = transp(g,h)
	E = [eye(2); zeros(1,2)];
	vm = rotM(g)' * [logmap(h,g); 0];
	mn = norm(vm);
	uv = vm / (mn+realmin);
	Rpar = eye(3) - sin(mn)*(g*uv') - (1-cos(mn))*(uv*uv');	
	Ac = E' * rotM(h) * Rpar * rotM(g)' * E; %Transportation operator from g to h 
end

