function demo_Riemannian_cov_search01
% EM-based stochastic optimization of covariance on Riemannian manifold 
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
nbVar = 2; %Number of variables [t,x1,x2]
nbVarCov = nbVar + nbVar * (nbVar-1)/2; %Dimension of the vecotrized form of covariance
nbData = 10; %Number of initial datapoints
nbEpisods = 20; %Number of iterations for the stochastic search
nbIter = 5; %Number of iteration for the Gauss Newton algorithm
minSigma = eye(nbVarCov) * 1E-4; %Minimal exploration noise 


%% Generate random covariances
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Desired point 
xtmp = randn(nbVar,5) * 1E-1;
xvecHat = symMat2vec(xtmp*xtmp' + eye(nbVar)*1E-2); 

%Initial guess and exploration noise
MuMan = symMat2vec(eye(nbVar) * 1E-1);
Sigma = eye(nbVarCov) * 1E-2;

xvec = [];
for n=1:nbEpisods
	%Sampling of new point(s)
	[V,D] = eig(Sigma);
	if n==1
		uvec = V*D.^.5 * randn(nbVarCov,nbData);
	else
		uvec = V*D.^.5 * randn(nbVarCov,1);
	end
	xvec = [xvec, expmap_vec(uvec,MuMan)];
	
	%Evaluate return
	%utmp = logmap_vec(xvec, xvecHat) + realmin;
	%H = sum(utmp.*utmp)'.^-1;
	%H = gaussPDF(utmp, zeros(nbVarCov,1), eye(nbVarCov)*1E-5);
	H = gaussPDF(xvec, xvecHat, eye(nbVarCov)*1E-5);
	H = (H+realmin) ./ sum(H+realmin); 	

	%Update MuMan
	for t=1:nbIter
		uvec = logmap_vec(xvec, MuMan);
		MuMan = expmap_vec(uvec * H, MuMan);
	end
	%Update Sigma
	Sigma = uvec * diag(H) * uvec' + minSigma;
	
	%Log data
	r(n).MuMan = MuMan;
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,650,650]); hold on; axis off;
% for t=1:size(xvec,2)
% 	plotGMM(zeros(2,1), reshape(xvec(:,t),2,2), [.5 .5 .5], .1);
% end
plotGMM(zeros(2,1), vec2symMat(xvecHat), [.7 .7 1], 1);
for t=1:nbData
	plotGMM(zeros(2,1), vec2symMat(xvec(:,t)), [.7 .7 .7], .1);
end
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%pause
for n=1:nbEpisods
	plotGMM(zeros(2,1), vec2symMat(r(n).MuMan), [.8*n/nbEpisods .8*(1-n/nbEpisods) 0], .05);
	pause(0.01);
end
%print('-dpng','graphs/demo_Riemannian_cov_search01.png');

% pause;
% close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X = expmap(U,S)
% Exponential map (SPD manifold)
N = size(U,3);
for n=1:N
	X(:,:,n) = S^.5 * expm(S^-.5 * U(:,:,n) * S^-.5) * S^.5;
end
end

function U = logmap(X,S)
% Logarithm map (SPD manifold)
N = size(X,3);
for n=1:N
	U(:,:,n) = S^.5 * logm(S^-.5 * X(:,:,n) * S^-.5) * S^.5;
end
end

function x = expmap_vec(u,s)
% Exponential map for the vector form (SPD manifold)
	U = vec2symMat(u);
	S = vec2symMat(s);
	x = symMat2vec(expmap(U,S));
end

function u = logmap_vec(x,s)
% Logarithm map for the vector form (SPD manifold)
	X = vec2symMat(x);
	S = vec2symMat(s);
	u = symMat2vec(logmap(X,S));
end

function v = symMat2vec(S)
% Reduced vectorisation of a symmetric matrix
[d,~,N] = size(S);
v = zeros(d+d*(d-1)/2,N);
for n = 1:N
	v(1:d,n) = diag(S(:,:,n));
	row = d+1;
	for i = 1:d-1
		v(row:row + d-1-i,n) = sqrt(2).*S(i+1:end,i,n);
		row = row + d-i;
	end
end
end

function S = vec2symMat(v)
% Matricisation of a vector to a symmetric matrix
[t, N] = size(v);
d = (-1 + sqrt(1+8*t))/2;
S = zeros(d,d,N);
for n= 1:N
	% Side elements
	i = d+1;
	for row = 1:d-1
		S(row,row+1:d,n) = v(i:i+d-1-row,n)./sqrt(2);
		i = i+d-row;
	end
	S(:,:,n) = S(:,:,n) + S(:,:,n)';
	% Diagonal elements
	S(:,:,n) = S(:,:,n) + diag(v(1:d,n));
end
end

function [Sred, V, D] = covOrder4to2(S)
% Reduction of a 4th-order covariance to a 2nd-order covariance. Return the
% reduced covariance, the eigentensors and eigenvalues.
d = size(S,1);
% Reduction to 2nd-order
Sred = zeros(d+d*(d-1)/2);
% left-up part
for k = 1:d
	for m = 1:d
		Sred(k,m) = S(k,k,m,m);
	end
end
% right-down part
row = d+1; col = d+1;
for k = 1:d-1
	for m = k+1:d
		for p = 1:d-1
			for q = p+1:d
				Sred(row,col) = 2*S(k,m,p,q);
				col = col+1;
			end
		end
		row = row + 1;
		col = d+1;
	end
end
% side-parts
for k = 1:d
	col = d+1;
	for p = 1:d-1
		for q = p+1:d
			Sred(k,col) = sqrt(2)*S(k,k,p,q);
			col = col+1;
		end
	end
end
Sred(d+1:end,1:d) = Sred(1:d,d+1:end)';
% Find eigenvalues and eigenvectors
[v,D] = eig(Sred);
V = zeros(d,d,d+d*(d-1)/2);
for i = 1:d+d*(d-1)/2
	j = d+1;
	for k = 1:d-1
		for m = k+1:d
			V(k,m,i) = v(j,i)/sqrt(2);
			j = j+1;
		end
	end
	V(:,:,i) = V(:,:,i) + V(:,:,i)' + diag(v(1:d,i));
end
end

function [S, V, D] = covOrder2to4(Sred)
% Augmentation of a 2nd-order (matrix) covariance to a 4th-order covariance
t = size(Sred,1);
d = (-1 + sqrt(1+8*t))/2;
% Find eigenvalues and eigenvectors
[v,D] = eig(Sred);
V = zeros(d,d,d+d*(d-1)/2);
for i = 1:d+d*(d-1)/2
	j = d+1;
	for k = 1:d-1
		for m = k+1:d
			V(k,m,i) = v(j,i)/sqrt(2);
			j = j+1;
		end
	end
	V(:,:,i) = V(:,:,i) + V(:,:,i)' + diag(v(1:d,i));
end
S = zeros(d,d,d,d);
for j = 1:size(V,3)
	S = S + D(j,j).*outerprod(V(:,:,j),V(:,:,j));
end
end
