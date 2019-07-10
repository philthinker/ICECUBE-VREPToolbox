function [r,P] = reproduction_LQR_infiniteHorizon(model, r, currPos, rFactor)
% Reproduction with a linear quadratic regulator of infinite horizon (continuous version).
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please reward the authors by citing the related publications, 
% and consider making your own research available in this way.
%
% @inproceedings{Calinon14ICRA,
%   author="Calinon, S. and Bruno, D. and Caldwell, D. G.",
%   title="A task-parameterized probabilistic model with minimal intervention control",
%   booktitle="Proc. {IEEE} Intl Conf. on Robotics and Automation ({ICRA})",
%   year="2014",
%   month="May-June",
%   address="Hong Kong, China",
%   pages="3339--3344"
% }
%
% @article{Calinon15,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%   year="2015"
% }
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon (http://calinon.ch/), Danilo Bruno (danilo.bruno@iit.it)
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


[nbVarOut,nbData] = size(r.currTar);

%% LQR with cost = sum_t X(t)' Q(t) X(t) + u(t)' R u(t)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Definition of a double integrator system (DX = A X + B u with X = [x; dx])
A = kron([0 1; 0 0], eye(nbVarOut)); 
B = kron([0; 1], eye(nbVarOut)); 
%Initialize Q and R weighting matrices
Q = zeros(nbVarOut*2,nbVarOut*2);
R = eye(nbVarOut) * rFactor;

%Variables for feedforward term computation (optional for movements with low dynamics)
%tar = [r.currTar; gradient(r.currTar,1,2)/model.dt];
%dtar = gradient(tar,1,2)/model.dt;
tar = [r.currTar; zeros(nbVarOut,nbData)];
dtar = gradient(tar,1,2)/model.dt;


%% Reproduction with varying impedance parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = currPos;
dx = zeros(nbVarOut,1);
for t=1:nbData
	%Current weighting term
	Q(1:nbVarOut,1:nbVarOut) = inv(r.currSigma(:,:,t));
	
	%care() is a function from the Matlab Control Toolbox to solve the algebraic Riccati equation,
	%also available with the control toolbox of GNU Octave
	%P = care(A, B, (Q+Q')/2, R); %(Q+Q')/2 is used instead of Q to avoid warnings for the symmetry of Q
	
	%Alternatively, the function below can be used for an implementation based on Schur decomposition
	%P = solveAlgebraicRiccati_Schur(A, B/R*B', (Q+Q')/2);
	
	%Alternatively, the function below can be used for an implementation based on Eigendecomposition
	%-> the only operator is eig([A -B/R*B'; -Q -A'])
	P = solveAlgebraicRiccati_eig(A, B/R*B', (Q+Q')/2); %See Eq. (5.1.27) and Sec. (5.2) in doc/TechnicalReport.pdf
	
	%Variable for feedforward term computation (optional for movements with low dynamics)
	d = (P*B*(R\B')-A') \ (P*dtar(:,t) - P*A*tar(:,t)); %See Eq. (5.1.28) with d_dot=0 in doc/TechnicalReport.pdf
	
	%Feedback term
	L = R\B'*P; 
	
	%Feedforward term
	M = R\B'*d; 
	
	%Compute acceleration (with only feedback terms)
	ddx =  -L * [x-r.currTar(:,t); dx];
	
	%Compute acceleration (with feedback and feedforward terms)
	%ddx =  -L * [x-r.currTar(:,t); dx] + M; 
	
	%Update velocity and position
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	
	%Log data (with additional variables collected for analysis purpose)
	r.Data(:,t) = x;
	r.ddxNorm(t) = norm(ddx);
	%r.FB(:,t) = -L * [x-r.currTar(:,t); dx];
	%r.FF(:,t) = M;
	%r.Kp(:,:,t) = L(:,1:nbVarOut);
	%r.Kv(:,:,t) = L(:,nbVarOut+1:end);
	r.kpDet(t) = det(L(:,1:nbVarOut));
	r.kvDet(t) = det(L(:,nbVarOut+1:end));
	%Note that if [V,D] = eigs(L(:,1:nbVarOut)), we have L(:,nbVarOut+1:end) = V * (2*D).^.5 * V'
end
