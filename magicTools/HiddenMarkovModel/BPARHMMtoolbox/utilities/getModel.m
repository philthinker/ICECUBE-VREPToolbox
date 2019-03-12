%%
% Set Hyperparameters

clear model

% Type of dynamical system:
model.obsModel.type = obsModelType;

if strcmp(obsModelType,'AR')
    % Order of AR process:
    model.obsModel.r = r;
    m = d*r;
else
    m = d;
end

% Type of prior on dynamic parameters. Choices include matrix normal
% inverse Wishart on (A,Sigma) and normal on mu ('MNIW-N'), matrix normal
% inverse Wishart on (A,Sigma) with mean forced to 0 ('MNIW'), normal on A,
% inverse Wishart on Sigma, and normal on mu ('N-IW-N'), and fixed A,
% inverse Wishart on Sigma, and normal on mu ('Afixed-IW-N').  NOTE: right
% now, the 'N-IW-N' option is only coded for shared A!!!
model.obsModel.priorType = priorType;

switch model.obsModel.priorType
    case 'NIW'

        model.obsModel.params.M  = zeros([d 1]);
        model.obsModel.params.K =  kappa;
        
    case 'IW-N'
        % Mean and covariance for Gaussian prior on mean:
        model.obsModel.params.mu0 = zeros(d,1);
        model.obsModel.params.cholSigma0 = chol(sig0*eye(d));
    
    case 'MNIW'
        % Mean and covariance for A matrix:
        model.obsModel.params.M  = zeros([d m]);

        % Inverse covariance along rows of A (sampled Sigma acts as
        % covariance along columns):
        model.obsModel.params.K =  K(1:m,1:m);
        
    case 'MNIW-N'
        % Mean and covariance for A matrix:
        model.obsModel.params.M  = zeros([d m]);

        % Inverse covariance along rows of A (sampled Sigma acts as
        % covariance along columns):
        model.obsModel.params.K =  K(1:m,1:m);

        % Mean and covariance for mean of process noise:
        model.obsModel.params.mu0 = zeros(d,1);
        model.obsModel.params.cholSigma0 = chol(sig0*eye(d));

    case 'N-IW-N'
        % Mean and covariance for A matrix:
        model.obsModel.params.M  = zeros([d m]);
        model.obsModel.params.Lambda0_A = inv(kron(inv(K),meanSigma));

        % Mean and covariance for mean of process noise:
        model.obsModel.params.mu0 = zeros(d,1);
        model.obsModel.params.cholSigma0 = chol(sig0*eye(d));
        
    case 'Afixed-IW-N'
        % Set fixed A matrix:
        model.obsModel.params.A = A_shared;
        
        % Mean and covariance for mean of process noise:
        model.obsModel.params.mu0 = zeros(d,1);
        model.obsModel.params.cholSigma0 = chol(sig0*eye(d));
        
    case 'ARD'        % Gamma hyperprior parameters for prior on precision parameter:
        model.obsModel.params.a_ARD = a_ARD;
        model.obsModel.params.b_ARD = b_ARD;
        
        % Placeholder for initializeStructs. Can I get rid of this?
        model.obsModel.params.M  = zeros([d m]);

        % Mean and covariance for mean of process noise:
        model.obsModel.params.zeroMean = 1;
end
        
% Degrees of freedom and scale matrix for covariance of process noise:
model.obsModel.params.nu = nu; %d + 2;
model.obsModel.params.nu_delta = (model.obsModel.params.nu-d-1)*meanSigma;

if strcmp(obsModelType,'SLDS')
    % Degrees of freedom and scale matrix for covariance of measurement noise:
    model.obsModel.y_params.nu = nu_y; %dy + 2;
    model.obsModel.y_params.nu_delta = (model.obsModel.y_params.nu-dy-1)*meanR;
    
    model.obsModel.y_priorType = y_priorType;
    
    switch model.obsModel.y_priorType
        case 'NIW'
            
            model.obsModel.y_params.M  = zeros([dy 1]);
            model.obsModel.y_params.K =  kappa_y;
            
        case 'IW-N'
            % Mean and covariance for Gaussian prior on mean:
            model.obsModel.y_params.mu0 = mu0_y; %zeros(dy,1);
            model.obsModel.y_params.cholSigma0 = chol(sig0_y*eye(dy));
    end
    
    % Fixed measurement matrix:
    model.obsModel.params.C = [eye(dy) zeros(dy,d-dy)];
    
    % Initial state covariance:
    model.obsModel.params.P0 = P0*eye(d);
end

% Always using DP mixtures emissions, with single Gaussian forced by
% Ks=1...Need to fix.
model.obsModel.mixtureType = 'infinite';

% Sticky HDP-HMM parameter settings:
model.HMMmodel.params.a_alpha=a_alpha;  % affects \pi_z
model.HMMmodel.params.b_alpha=b_alpha;
model.HMMmodel.params.var_alpha=var_alpha;
model.HMMmodel.params.a_kappa=a_kappa;  % affects \pi_z
model.HMMmodel.params.b_kappa=b_kappa;
model.HMMmodel.params.var_kappa=var_kappa;
model.HMMmodel.params.a_gamma=a_gamma;  % global expected # of HMM states (affects \beta)
model.HMMmodel.params.b_gamma=b_gamma;

numObj = length(data_struct);
harmonic = 0;
for n=1:length(data_struct);
    harmonic = harmonic + 1/n;
end
model.HMMmodel.params.harmonic = harmonic;
if exist('Ks')
    if Ks>1
        model.HMMmodel.params.a_sigma = 1;
        model.HMMmodel.params.b_sigma = 0.01;
    end
else
    Ks = 1;
end
if exist('Kr')
    if Kr > 1
        model.HMMmodel.params.a_eta = 1;
        model.HMMmodel.params.b_eta = 0.01;
    end
end
