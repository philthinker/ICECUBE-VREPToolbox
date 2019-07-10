function demo_Riemannian_cov_vecTransp01
% Verification of angle conservation in parallel transport on the symmetric positive definite (SPD) manifold S²+
% (covariance matrices of dimension 2x2)
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
% Copyright (c) 2017 Idiap Research Institute, http://idiap.ch/
% Written by Noémie Jaquier and Sylvain Calinon
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

addpath('./m_fcts');


%% Generate SPD data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = randn(2,10);
h = x*x';
x = randn(2,10);
g = x*x';
x = randn(2,10);
U0 = x*x';


%% Parallel transport of U0 from g to h
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tl = linspace(0,1,20);

for n=1:20
	t = tl(n);
	
	hist(n).MuMan = expmap(logmap(h,g)*t, g);
	
	Ac = transp(g, hist(n).MuMan);
	hist(n).U = Ac * U0 * Ac';
	  
	%Direction of the geodesic
	hist(n).dirG = logmap(h, hist(n).MuMan);
	if norm(hist(n).dirG) > 1E-5
		
		%Normalise the direction
		%hist(n).dirG = hist(n).dirG ./ norm(hist(n).dirG); %for S^3 manifold
		innormdir = trace(hist(n).MuMan^-.5 * hist(n).dirG * hist(n).MuMan^-1 * hist(n).dirG * hist(n).MuMan^-.5);
		hist(n).dirG = hist(n).dirG ./ sqrt(innormdir);

		%Compute the inner product with the first eigenvector
		%inprod(n) = hist(n).dirG' * hist(n).U(:,1); %for S^3 manifold
		inprod(n) = trace(hist(n).MuMan^-.5 * hist(n).dirG * hist(n).MuMan^-1 * hist(n).U * hist(n).MuMan^-.5);
	end
    
end
inprod

end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X = expmap(U,S)
	X = S^.5 * expm(S^-.5 * U * S^-.5) * S^.5;
end

function U = logmap(X,S)
	U = S^.5 * logm(S^-.5 * X * S^-.5) * S^.5;
end

function x = expmap_vec(u,s)
	nbData = size(u,2);
	d = size(u,1)^.5;
	U = reshape(u, [d, d, nbData]);
	S = reshape(s, [d, d]);
	x = zeros(d^2, nbData);
	for t=1:nbData
		x(:,t) = reshape(expmap(U(:,:,t),S), [d^2, 1]);
	end
end

function u = logmap_vec(x,s)
	nbData = size(x,2);
	d = size(x,1)^.5;
	X = reshape(x, [d, d, nbData]);
	S = reshape(s, [d, d]);
	u = zeros(d^2, nbData);
	for t=1:nbData
		u(:,t) = reshape(logmap(X(:,:,t),S), [d^2, 1]);
	end
end

function Ac = transp(S1,S2)
	t = 1;
	U = logmap(S2,S1);
	Ac = S1^.5 * expm(0.5 .* t .* S1^-.5 * U * S1^-.5) * S1^-.5;
	%Computationally economic way: Ac = (S2/S1)^.5
end



