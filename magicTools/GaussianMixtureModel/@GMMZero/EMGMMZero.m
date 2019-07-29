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
	[L, GAMMA] = obj.computeGamma(Data);
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,nbData);
	
	%M-step
	for i=1:obj.nKernel
		%Update Priors
		if obj.params_updateComp(1)
			obj.Prior(i) = sum(GAMMA(i,:)) / nbData;
		end
		%Update Mu
		if obj.params_updateComp(2)
% 			model.Mu(:,i) = Data * GAMMA2(i,:)';
            MuTmp = Data * GAMMA2(i,:)';
            obj.Mu(i,:) = MuTmp';
		end
		%Update Sigma
		if obj.params_updateComp(3)
% 			DataTmp = Data - repmat(model.Mu(:,i),1,nbData);
% 			model.Sigma(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(size(Data,1)) * model.params_diagRegFact;
            DataTmp = Data - repmat(MuTmp,1,nbData);
			obj.Sigma(:,:,i) = DataTmp * diag(GAMMA2(i,:)) * DataTmp' + eye(size(Data,1)) * obj.params_diagRegFact;
		end
	end
	
	%Compute average log-likelihood
	LL(nbIter) = sum(log(sum(L,1))) / nbData;
	%Stop the algorithm if EM converged (small change of LL)
	if nbIter>obj.params_nbMinSteps
		if LL(nbIter)-LL(nbIter-1)<obj.params_maxDiffLL || nbIter==obj.params_nbMaxSteps-1
			disp(['EM converged after ' num2str(nbIter) ' iterations.']);
			return;
		end
	end
end
disp(['The maximum number of ' num2str(model.params_nbMaxSteps) ' EM iterations has been reached.']);

end

