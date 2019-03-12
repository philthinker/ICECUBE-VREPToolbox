function theta = sample_theta_extra(theta,obsModel,Kextra)

prior_params = obsModel.params;

if ~strcmp(obsModel.priorType,'MNIW')
    error('Only coded for MNIW prior')
end

nu = prior_params.nu;
nu_delta = prior_params.nu_delta;

invSigma = theta.invSigma;
A = theta.A;

[tmp1 tmp2 Kz Ks] = size(invSigma);

K = prior_params.K;
M = prior_params.M;

for kz=Kz+1:Kz+Kextra
    for ks=1:Ks
        
        Sxx = K;
        SyxSxxInv = M;
        Sygx = 0;
        
        % Sample Sigma given s.stats
        [sqrtSigma sqrtinvSigma] = randiwishart(Sygx + nu_delta,nu);
        invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
        
        % Sample A given Sigma and s.stats
        cholinvSxx = chol(inv(Sxx));
        A(:,:,kz,ks) = sampleFromMatrixNormal(SyxSxxInv,sqrtSigma,cholinvSxx);
        
    end
end

theta.invSigma = invSigma;
theta.A =  A;

return;