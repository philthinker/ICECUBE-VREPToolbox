function demo_Riemannian_cov_interp01
% Covariance interpolation on Riemannian manifold
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please cite the related publications.
%
% @article{Jaquier17IROS,
%   author="Jaquier, N. and Calinon, S.",
%   title="Gaussian Mixture Regression on Symmetric Positive Definite Matrices Manifolds: 
%	    Application to Wrist Motion Estimation with s{EMG}",
%   year="2017",
%	  booktitle = "{IEEE/RSJ} Intl. Conf. on Intelligent Robots and Systems ({IROS})",
%	  address = "Vancouver, Canada"
% }
% 
% Copyright (c) 2017 Sylvain Calinon, Idiap Research Institute, http://idiap.ch/
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
model.nbVar = 2; %Number of variables
model.nbStates = 2; %Number of states
nbData = 20; %Number of interpolations
% nbIter = 5; %Number of iteration for the Gauss Newton algorithm


%% Gaussians parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.Mu(:,1) = [0; 0];
d1 = [2; 3];
model.Sigma(:,:,1) = d1*d1' + eye(model.nbVar)*1;

model.Mu(:,2) = [10; 0];
% d2 = [4; 1];
% model.Sigma(:,:,2) = d2*d2' + eye(model.nbVar)*1E-1;
[R,~] = qr(randn(model.nbVar));
model.Sigma(:,:,2) = R * model.Sigma(:,:,1) * R';


%% Linear interpolation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w = [linspace(1,0,nbData); linspace(0,1,nbData)];
Mu0 = interp1([1,0], model.Mu', w(1,:))';
Sigma0 = zeros(model.nbVar,model.nbVar,nbData);
for t=1:nbData
	for i=1:model.nbStates
		Sigma0(:,:,t) = Sigma0(:,:,t) + w(i,t) * model.Sigma(:,:,i);
	end
	[V0(:,:,t), D0(:,:,t)] = eigs(Sigma0(:,:,t));
	S0det(t) = det(Sigma0(:,:,t)); 
	S0tra(t) = trace(Sigma0(:,:,t));
end


%% Geodesic interpolation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w = [linspace(1,0,nbData); linspace(0,1,nbData)];
Mu = interp1([1,0], model.Mu', w(1,:))';
Sigma = zeros(model.nbVar,model.nbVar,nbData);

% S = model.Sigma(:,:,1);
for t=1:nbData
	
% 	%Interpolation between more than 2 covariances can be computed in an iterative form
% 	for n=1:nbIter
% 		W = zeros(model.nbVar);
% 		for i=1:model.nbStates
% 			W = W + w(i,t) * logmap(model.Sigma(:,:,i), S);
% 		end
% 		S = expmap(W,S);
% 	end
% 	Sigma(:,:,t) = S;

	%Interpolation between two covariances can be computed in closed form
	Sigma(:,:,t) = expmap(w(2,t)*logmap(model.Sigma(:,:,2), model.Sigma(:,:,1)), model.Sigma(:,:,1));

	[V(:,:,t), D(:,:,t)] = eigs(Sigma(:,:,t));
	Sdet(t) = det(Sigma(:,:,t));
	Stra(t) = trace(Sigma(:,:,t));
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1300,800]); 

%Plot determinant for linear interpolation
subplot(2,2,1); hold on; title('Linear interpolation');
plot(S0det,'k-');
axis([1, nbData, min(S0det)-1, max(S0det)+1]);
xlabel('t'); ylabel('det(S)');
%Plot determinant for geodesic interpolation
subplot(2,2,2); hold on; title('Geodesic interpolation');
plot(Sdet,'k-');
axis([1, nbData, min(Sdet)-1, max(Sdet)+1]);
xlabel('t'); ylabel('det(S)');

%Plot linear interpolation
subplot(2,2,3); hold on; axis off; 
plotGMM(model.Mu, model.Sigma, [0 0 0]);
plotGMM(Mu0, Sigma0, [0 .8 0], .1);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%Plot geodesic interpolation
subplot(2,2,4); hold on; axis off; 
plotGMM(model.Mu, model.Sigma, [0 0 0]);
plotGMM(Mu, Sigma, [.8 0 0], .1);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);

% pause;
% close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function S = expmap(W,S)
	S = S^.5 * expm(S^-.5 * W * S^-.5) * S^.5;
end

function S = logmap(W,S)
	S = S^.5 * logm(S^-.5 * W * S^-.5) * S^.5;
end




