function [centers] = kMeans(Data,nKernel)
%kMeans The K-Means clustering algorithm
%   Data:N x M, data vectors
%   nKernel: Integer, number of centers

%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
%

% In order not to be disturbed by nomination, we do not recommend it added
% into your working path directly.

% Criterion to stop the EM iterative update
cumdist_threshold = 1e-10;
maxIter = 100;

% Initialization of the parameters
Data = Data';   % For S. Calinon's habit
[~, nbData] = size(Data);
nbStates = nKernel;
cumdist_old = -realmax;
nbStep = 0;

idTmp = randperm(nbData);
Mu = Data(:,idTmp(1:nbStates));

%k-means iterations
while 1
	%E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for i=1:nbStates
		%Compute distances
		distTmp(:,i) = sum((Data-repmat(Mu(:,i),1,nbData)).^2, 1);
	end
	[vTmp,idList] = min(distTmp,[],2);
	cumdist = sum(vTmp);
	%M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	for i=1:nbStates
		%Update the centers
		Mu(:,i) = mean(Data(:,idList==i),2);
	end
	%Stopping criterion %%%%%%%%%%%%%%%%%%%%
	if abs(cumdist-cumdist_old) < cumdist_threshold
		break;
	end
	cumdist_old = cumdist;
	nbStep = nbStep+1;
	if nbStep>maxIter
		disp(['Maximum number of kmeans iterations, ' num2str(maxIter) 'is reached']);
		break;
	end
end
centers = Mu';  % For S. Calinon's habit
end

