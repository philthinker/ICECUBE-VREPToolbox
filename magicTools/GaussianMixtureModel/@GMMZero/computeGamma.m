function [L,GAMMA] = computeGamma(obj,Data)
%computeGamma Used in EM algorithm
%   @GMMZero

L = zeros(obj.nKernel,size(Data,2));
for i=1:obj.nKernel
	L(i,:) = obj.Prior(i) * obj.GaussianPD(Data, (obj.Mu(i,:))', obj.Sigma(:,:,i));
end
GAMMA = L ./ repmat(sum(L,1)+realmin, obj.nKernel, 1);

end

