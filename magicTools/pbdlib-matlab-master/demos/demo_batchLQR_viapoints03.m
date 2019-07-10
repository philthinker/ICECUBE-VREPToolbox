function demo_batchLQR_viapoints03
% Equivalence between cubic Bezier curve and batch LQR with double integrator
% (formulation with position and velocity)
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @inproceedings{Berio17GI,
%   author="Berio, D. and Calinon, S. and Fol Leymarie, F.",
%   title="Generating Calligraphic Trajectories with Model Predictive Control",
%   booktitle="Proc. 43rd Conf. on Graphics Interface",
%   year="2017",
%   month="May",
%   address="Edmonton, AL, Canada",
%   pages="132--139",
%   doi="10.20380/GI2017.17"
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
nbData = 100; %Number of datapoints
nbRepros = 1; %Number of reproductions

model.nbStates = 2; %Number of Gaussians in the GMM
model.nbVarPos = 2; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector
model.dt = 1/nbData; %Time step duration
model.rfactor = 1E-10;	%Control cost in LQR

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbData-1),R);

%Setting cubic Bezier curve parameters
model.P = rand(model.nbVarPos, model.nbStates*2);
model.Mu = [model.P(:,[1,end]); zeros(model.nbVarPos, model.nbStates)];


%% Dynamical System settings (discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Integration with higher order Taylor series expansion
A1d = zeros(model.nbDeriv);
for i=0:model.nbDeriv-1
	A1d = A1d + diag(ones(model.nbDeriv-i,1),i) * model.dt^i * 1/factorial(i); %Discrete 1D
end
B1d = zeros(model.nbDeriv,1); 
for i=1:model.nbDeriv
	B1d(model.nbDeriv-i+1) = model.dt^i * 1/factorial(i); %Discrete 1D
end
A = kron(A1d, eye(model.nbVarPos)); %Discrete nD
B = kron(B1d, eye(model.nbVarPos)); %Discrete nD

%Build Sx and Su matrices for batch LQR, see Eq. (35)
Su = zeros(model.nbVar*nbData, model.nbVarPos*(nbData-1));
Sx = kron(ones(nbData,1),eye(model.nbVar));
M = B;
for n=2:nbData
	id1 = (n-1)*model.nbVar+1:nbData*model.nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	id1 = (n-1)*model.nbVar+1:n*model.nbVar; 
	id2 = 1:(n-1)*model.nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:model.nbVarPos), M]; %Also M = [A^(n-1)*B, M] or M = [Sx(id1,:)*B, M]
end


%% Cubic Bezier curve
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Cubic Bezier curve plot from Bernstein polynomials
%See e.g. http://blogs.mathworks.com/graphics/2014/10/13/bezier-curves/ or 
%http://www.ams.org/samplings/feature-column/fcarc-bezier#2
t = linspace(0,1,nbData);
x = kron((1-t).^3, model.P(:,1)) + kron(3*(1-t).^2.*t, model.P(:,2)) + kron(3*(1-t).*t.^2, model.P(:,3)) + kron(t.^3, model.P(:,4));
%dx = kron(3*(1-t).^2, model.P(:,2)-model.P(:,1)) + kron(6*(1-t).*t, model.P(:,3)-model.P(:,2)) + kron(3*t.^2, model.P(:,4)-model.P(:,3));
% model.Mu(3:4,1) = dx(1:2,1);
% model.Mu(3:4,2) = dx(1:2,end);
model.Mu(3:4,2) = (model.P(1:2,4) - model.P(1:2,3)) * 3;
model.Mu(3:4,1) = (model.P(1:2,2) - model.P(1:2,1)) * 3;


%% MPC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.invSigma = repmat(eye(model.nbVar)*1E0, [1,1,model.nbStates]);
% model.invSigma = repmat(blkdiag(eye(model.nbVarPos),ones(model.nbVarPos)*1E-18), [1,1,model.nbStates]);
% xt = rand(2,1);
% xt = xt /norm(xt);
% model.invSigma = repmat(blkdiag(xt*xt'+eye(model.nbVarPos)*1E-4, eye(model.nbVarPos)*1E-18), [1,1,model.nbStates]);

%Create single Gaussian N(MuQ,SigmaQ) 
qList = round(linspace(1,model.nbStates,nbData));
MuQ = reshape(model.Mu(:,qList), model.nbVar*nbData, 1); 
%Set cost for two viapoints at the beginning and at the end
Q = blkdiag(model.invSigma(:,:,1), zeros(model.nbVar*(nbData-2)), model.invSigma(:,:,2));

%Batch LQR reproduction
X = model.Mu(:,1); 
%X = [model.Mu(1:2,1); zeros(2,1)]; 
for n=1:nbRepros	
	SuInvSigmaQ = Su' * Q;
	Rq = SuInvSigmaQ * Su + R;
	rq = SuInvSigmaQ * (MuQ-Sx*X);
	u = Rq \ rq; 
	r(n).Data = reshape(Sx*X+Su*u, model.nbVar, nbData);
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 700 700],'color',[1 1 1]); hold on; axis off;
plotGMM(model.Mu(1:2,:), model.invSigma(1:2,1:2,:)*1E-4, [0 .7 0], .3);
plotGMM(model.P(1:2,2:3), model.invSigma(3:4,3:4,:)*1E-4, [.7 0 0], .3);
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',1,'color',[1-n/nbRepros 0 0]);
	plot(r(n).Data(1,:), r(n).Data(2,:), '.','markersize',6,'color',[.8 0 0]);
end
plot(model.P(1,:), model.P(2,:), '.','color',[.7 .7 .7]);
plot(model.P(1,1:2), model.P(2,1:2), '-','color',[.7 .7 .7]);
plot(model.P(1,3:4), model.P(2,3:4), '-','color',[.7 .7 .7]);
plot(x(1,:),x(2,:),'-','color',[0 .7 0]);
plot(x(1,:),x(2,:),'.','markersize',6,'color',[0 .7 0]);
axis equal; 

%print('-dpng','graphs/demo_batchLQR_viapoints03.png');
%pause;
%close all;

