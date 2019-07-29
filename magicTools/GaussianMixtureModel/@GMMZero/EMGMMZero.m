function [obj,GAMMA2,LL] = EMGMMZero(obj,Data)
%EMGMMZero Expectation Maximization for GMMZero
%   Data: 1 x D cells, demonstration data. We assume that those
%   GAMMA2:regulized gamma
%   LL:the log-likelihood
%   @GMMZero

% 
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

Data = Data';   % For S. Calinon's habit

nbData = size(Data,2);
for nbIter=1:obj.params_nbMaxSteps
	fprintf('.');   % Animation
	
	%E-step
	[L, GAMMA] = computeGamma(Data, model); %See 'computeGamma' function below
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
	
	%M-step
	for i=1:model.nbStates
		%Update Priors
		if model.params_updateComp(1)
			model.Priors(i) = sum(GAMMA(i,:)) / nbData;
		end
		%Update Mu
		if model.params_updateComp(2)
			model.Mu(:,i) = Data * GAMMA2(i,:)';
		end
		%Update Sigma
		if model.params_updateComp(3)
			DataTmp = Data - repmat(model.Mu(:,i),1,nbData);
			model.Sigma(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(size(Data,1)) * model.params_diagRegFact;
		end
	end
	
	%Compute average log-likelihood
	LL(nbIter) = sum(log(sum(L,1))) / nbData;
	%Stop the algorithm if EM converged (small change of LL)
	if nbIter>model.params_nbMinSteps
		if LL(nbIter)-LL(nbIter-1)<model.params_maxDiffLL || nbIter==model.params_nbMaxSteps-1
			disp(['EM converged after ' num2str(nbIter) ' iterations.']);
			return;
		end
	end
end
disp(['The maximum number of ' num2str(model.params_nbMaxSteps) ' EM iterations has been reached.']);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [L, GAMMA] = computeGamma(Data, model)
L = zeros(model.nbStates,size(Data,2));
for i=1:model.nbStates
	L(i,:) = model.Priors(i) * gaussPDF(Data, model.Mu(:,i), model.Sigma(:,:,i));
end
GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
end
