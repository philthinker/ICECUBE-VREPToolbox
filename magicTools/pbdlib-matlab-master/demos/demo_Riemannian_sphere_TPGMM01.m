function demo_Riemannian_sphere_TPGMM01
% TP-GMM for data on a sphere by relying on Riemannian manifold (with single frame).
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
nbSamples = 4; %Number of demonstrations
nbIter = 10; %Number of iteration for the Gauss Newton algorithm
nbIterEM = 10; %Number of iteration for the EM algorithm
nbDrawingSeg = 15; %Number of segments used to draw ellipsoids

model.nbStates = 5; %Number of states in the GMM
model.nbFrames = 1; %Number of candidate frames of reference
model.nbVar = 3; %Dimension of the tangent space (incl. time)
model.nbVarMan = 4; %Dimension of the manifold (incl. time)
model.dt = 0.01; %Time step duration
model.params_diagRegFact = 1E-3; %Regularization of covariance
e0 = [0; 0; 1]; %Origin on the manifold


%% Generate dataset on a sphere from handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/S.mat');
uIn=[]; uOut=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	uOut = [uOut, s(n).Data*0.5E-1];
	uIn = [uIn, [1:nbData]*model.dt];
	for m=1:model.nbFrames
		s(n).p(m).b = (rand(model.nbVarMan-1,1)-0.5)*2;
		s(n).p(m).b = s(n).p(m).b / norm(s(n).p(m).b);
		[s(n).p(m).A,~] = qr(randn(model.nbVar-1));
	end
end
xOut = expmap(uOut, e0);
xIn = uIn;
u = [uIn; uOut];
x = [xIn; xOut];


%% GMM parameters estimation (joint distribution with time as input, sphere as output)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kbins(u, model, nbSamples);
model.MuMan = [model.Mu(1,:); expmap(model.Mu(2:3,:), e0)]; %Center on the manifold
model.Mu = zeros(model.nbVar,model.nbStates); %Center in the tangent plane at point MuMan of the manifold

uTmp = zeros(model.nbVar,nbData*nbSamples,model.nbStates);
for nb=1:nbIterEM
	%E-step
	L = zeros(model.nbStates,size(x,2));
	for i=1:model.nbStates
		L(i,:) = model.Priors(i) * gaussPDF([xIn-model.MuMan(1,i); logmap(xOut, model.MuMan(2:4,i))], model.Mu(:,i), model.Sigma(:,:,i));
	end
	GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData*nbSamples);
	%M-step
	for i=1:model.nbStates
		%Update Priors
		model.Priors(i) = sum(GAMMA(i,:)) / (nbData*nbSamples);
		%Update MuMan
		for n=1:nbIter
			uTmp(:,:,i) = [xIn-model.MuMan(1,i); logmap(xOut, model.MuMan(2:4,i))];
			model.MuMan(:,i) = [(model.MuMan(1,i)+uTmp(1,:,i))*GAMMA2(i,:)'; expmap(uTmp(2:3,:,i)*GAMMA2(i,:)', model.MuMan(2:4,i))];
		end
		%Update Sigma
		model.Sigma(:,:,i) = uTmp(:,:,i) * diag(GAMMA2(i,:)) * uTmp(:,:,i)' + eye(size(u,1)) * model.params_diagRegFact;
	end
end

%Eigendecomposition of Sigma
for i=1:model.nbStates
	[V,D] = eig(model.Sigma(:,:,i));
	U0(:,:,i) = V * D.^.5;
end


%% GMR in each frame
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
in=1; out=2:3; outMan=2:4;
nbVarOut = length(out);
nbVarOutMan = length(outMan);

uhat = zeros(nbVarOut,nbData);
xhat = zeros(nbVarOutMan,nbData);
uOutTmp = zeros(nbVarOut,model.nbStates,nbData);
SigmaTmp = zeros(model.nbVar,model.nbVar,model.nbStates);
expSigma = zeros(nbVarOut,nbVarOut,nbData);

%Version with single optimization loop
for t=1:nbData
	%Compute activation weight
	for i=1:model.nbStates
		H(i,t) = model.Priors(i) * gaussPDF(xIn(:,t)-model.MuMan(in,i), model.Mu(in,i), model.Sigma(in,in,i));
	end
	H(:,t) = H(:,t) / sum(H(:,t)+realmin);
	
	%Compute conditional mean (with covariance transportation)
	if t==1
		[~,id] = max(H(:,t));
		xhat(:,t) = model.MuMan(outMan,id); %Initial point
	else
		xhat(:,t) = xhat(:,t-1);
	end
	for n=1:nbIter
		for i=1:model.nbStates
			%Transportation of covariance from model.MuMan(outMan,i) to xhat(:,t) 
			Ac = transp(model.MuMan(outMan,i), xhat(:,t));
			U1 = blkdiag(1, Ac) * U0(:,:,i); %First variable in Euclidean space
			SigmaTmp(:,:,i) = U1 * U1';
			%Gaussian conditioning on the tangent space
			uOutTmp(:,i,t) = logmap(model.MuMan(outMan,i), xhat(:,t)) + SigmaTmp(out,in,i)/SigmaTmp(in,in,i) * (xIn(:,t)-model.MuMan(in,i)); 
		end
		uhat(:,t) = uOutTmp(:,:,t) * H(:,t);
		xhat(:,t) = expmap(uhat(:,t), xhat(:,t));
	end
	
	%Compute conditional covariances
	for i=1:model.nbStates
		SigmaOutTmp = SigmaTmp(out,out,i) - SigmaTmp(out,in,i)/SigmaTmp(in,in,i) * SigmaTmp(in,out,i);
		expSigma(:,:,t) = expSigma(:,:,t) + H(i,t) * (SigmaOutTmp + uOutTmp(:,i,t)*uOutTmp(:,i,t)');
	end
	expSigma(:,:,t) = expSigma(:,:,t) - uhat(:,t)*uhat(:,t)' + eye(nbVarOut) * model.params_diagRegFact; 
end


%% Transportation of GMR results from e0 to s(n).p(1).b 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:nbSamples
	s(n).x = expmap(s(n).p(1).A * uOut, s(n).p(1).b);	
	uTmp = s(n).p(1).A * logmap(xhat, e0);
	s(n).xhat = expmap(uTmp, s(n).p(1).b);
	for t=1:nbData
		s(n).expSigma(:,:,t) = s(n).p(1).A * expSigma(:,:,t) * s(n).p(1).A';
	end
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clrmap = lines(model.nbStates);

%Display of covariance contours on the sphere
tl = linspace(-pi, pi, nbDrawingSeg);
Gdisp = zeros(nbVarOutMan, nbDrawingSeg, model.nbStates);
for i=1:model.nbStates
	[V,D] = eig(model.Sigma(2:3,2:3,i));
	Gdisp(:,:,i) = expmap(V*D.^.5*[cos(tl); sin(tl)], model.MuMan(2:4,i));
end
Gregr = zeros(nbVarOutMan, nbDrawingSeg, nbData);
for t=1:nbData
	[V,D] = eig(expSigma(:,:,t));
	Gregr(:,:,t) = expmap(V*D.^.5*[cos(tl); sin(tl)], xhat(:,t));
end
for n=1:nbSamples
	for t=1:nbData
		[V,D] = eig(s(n).expSigma(:,:,t));
		utmp = logmap(s(n).xhat(:,t), s(n).p(1).b);
		s(n).Gdisp(:,:,t) = expmap(V*D.^.5*[cos(tl); sin(tl)] + repmat(utmp,1,nbDrawingSeg), s(n).p(1).b);
	end
end
	
%Manifold plot
figure('PaperPosition',[0 0 8 8],'position',[10,10,650,650]); hold on; axis off; grid off; rotate3d on; 
colormap([.9 .9 .9]);
[X,Y,Z] = sphere(20);
mesh(X,Y,Z);
plot3(xOut(1,:), xOut(2,:), xOut(3,:), '.','markersize',6,'color',[.6 .6 .6]);
plot3(xhat(1,:), xhat(2,:), xhat(3,:), '.','markersize',12,'color',[.8 0 0]);
for t=1:nbData
	plot3(Gregr(1,:,t), Gregr(2,:,t), Gregr(3,:,t), '-','linewidth',1,'color',[.8 0 0]);
end

for n=1:nbSamples
	plot3(s(n).p(1).b(1), s(n).p(1).b(2), s(n).p(1).b(3), '+','markersize',12,'color',[0 0 0]);
	plot3(s(n).x(1,:), s(n).x(2,:), s(n).x(3,:), '.','markersize',6,'color',[.6 .6 .6]);
	plot3(s(n).xhat(1,:), s(n).xhat(2,:), s(n).xhat(3,:), '.','markersize',12,'color',[0 0 0]);
	for t=1:nbData
		plot3(s(n).Gdisp(1,:,t), s(n).Gdisp(2,:,t), s(n).Gdisp(3,:,t), '-','linewidth',1,'color',[0 0 0]);
	end
end
for i=1:model.nbStates
	plot3(model.MuMan(1,i), model.MuMan(2,i), model.MuMan(3,i), '.','markersize',12,'color',clrmap(i,:));
	plot3(Gdisp(1,:,i), Gdisp(2,:,i), Gdisp(3,:,i), '-','linewidth',1,'color',clrmap(i,:));
end
view(-20,70); axis equal; axis tight; axis vis3d;  
%print('-dpng','graphs/demo_Riemannian_sphere_TPGMM01.png');

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
