function [expData, expSigma, H] = GMRZero(obj, DataIn, in, out)
%GMRZero Gaussian mixture regression (GMR)
%   DataIn: 1 x N, Query data
%   in: Integer, the position of DataIn in demo data
%   out: 1 x D, the position of the generated data in demo data
%   expData: D x N, expection data
%   expSigma: D x D x N, expection sigma for each generated data
%   @GMMZero
%
% Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 

nbData = size(DataIn,2);
nbVarOut = length(out);
diagRegularizationFactor = 1E-8; %Optional regularization term

MuTmp = zeros(nbVarOut,obj.nKernel);
expData = zeros(nbVarOut,nbData);
expSigma = zeros(nbVarOut,nbVarOut,nbData);

nbState = obj.nKernel;
Prior = obj.Prior;
Mu = (obj.Mu)';
Sigma = obj.Sigma;
H = zeros(nbState,nbData);

for t=1:nbData
    %Compute activation weight
    for i=1:nbState
%         H(i,t) = Prior(i) * gaussPDF(DataIn(:,t), model.Mu(in,i), model.Sigma(in,in,i));
        H(i,t) = Prior(i) * obj.GaussianPD(DataIn(:,t), Mu(in,i), Sigma(in,in,i));
    end
    H(:,t) = H(:,t) / sum(H(:,t)+realmin);
    %Compute conditional means
    for i=1:nbState
        MuTmp(:,i) = Mu(out,i) + Sigma(out,in,i)/Sigma(in,in,i) * (DataIn(:,t)-Mu(in,i));
        expData(:,t) = expData(:,t) + H(i,t) * MuTmp(:,i);
    end
    %Compute conditional covariances
    for i=1:nbState
        SigmaTmp = Sigma(out,out,i) - Sigma(out,in,i)/Sigma(in,in,i) * Sigma(in,out,i);
        expSigma(:,:,t) = expSigma(:,:,t) + H(i,t) * (SigmaTmp + MuTmp(:,i)*MuTmp(:,i)');
    end
    expSigma(:,:,t) = expSigma(:,:,t) - expData(:,t)*expData(:,t)' + eye(nbVarOut) * diagRegularizationFactor;
end

end

