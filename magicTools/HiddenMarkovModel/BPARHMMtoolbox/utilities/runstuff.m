
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%         Generate Data          %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath utilities
addpath relabeler

d = 1;  % dimension of each time series
r = 1;  % autoregressive order for each time series
T = 1000; % length of each time-series.  Note, each could be a different length.
m = d*r;
K = inv(diag([10*ones(1,m)]));  % matrix normal hyperparameter (affects covariance of matrix)
M = zeros(d,m); % matrix normal hyperparameter (mean matrix)

nu = d+2;   % inverse Wishart degrees of freedom
meanSigma = 0.5*eye(d); % inverse Wishart mean covariance matrix
nu_delta = (nu-d-1)*meanSigma;

numObj = 5; % number of time series
numStates = 9;  % number of behaviors
for k=1:numStates
    Sigma{k} = iwishrnd(nu_delta,nu);   % sample a covariance matrix
    
    if r==1 % if autoregressive order is 1, use some predefined dynamic matrices that cover the range of stable dynamics
        A{k}     = (-1 +0.2*k)*eye(d);
    else
        A{k} = sampleFromMatrixNormal(M,Sigma{k},K);  % otherwise, sample a random set of lag matrices (each behavior might not be very distinguishable!)
    end
end

% Define feature matrix by sampling from truncated IBP:
F = sample_truncated_features_init(numObj,numStates,10);

% Define transition distributions:
p_self = 0.95;
pi_z = ((1-p_self)/(numStates-1))*ones(numStates,numStates);
for ii=1:numStates
    pi_z(ii,ii) = p_self;
end
pi_init = ones(1,numStates);
pi_init = pi_init./sum(pi_init);
pi_s = ones(numStates,1);
dist_struct_tmp.pi_z = pi_z;
dist_struct_tmp.pi_init = pi_init;
dist_struct_tmp.pi_s = pi_s;

for nn=1:numObj
    
    Kz_inds = F(nn,:)>0;
    
    [pi_z_nn pi_init_nn] = transformDistStruct(dist_struct_tmp,Kz_inds);
    
    clear Y X;
    labels = zeros(1,T);
    P = cumsum(pi_init_nn);
    labels_temp = 1+sum(P(end)*rand(1) > P);
    labels(1) = Kz_inds(labels_temp);
    tmp = mvnrnd(zeros(d,1)',Sigma{labels(1)},r)';
    x0 = tmp(:);
    x = x0;
    
    for k=1:T
        if k>1
            P = cumsum(pi_z_nn(labels(k-1),:));
            labels(k) = 1+sum(P(end)*rand(1) > P);
        end
        Y(:,k) = A{labels(k)}*x + mvnrnd(zeros(d,1)',Sigma{labels(k)},1)';
        X(:,k) = x;
        x = [Y(:,k);x(1:(end-d),:)];
    end
    
    figure(1);
    plot(Y'); hold on; plot(labels,'m'); hold off;
    waitforbuttonpress;
    
    data_struct(nn).obs = Y;
    data_struct(nn).true_labels = labels;
    
end

%%
clear model settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%      Set Model Params     %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set mean covariance matrix from data (instead of assuming knowledge of
% ground truth):

Ybig2 = [];
for ii=1:length(data_struct)
    Ybig2 = [Ybig2 data_struct(ii).obs];
end
nu = d+2;
meanSigma = 0.75*cov(diff(Ybig2'));

obsModelType = 'AR';
priorType = 'MNIW';

% Set hyperprior settings for Dirichlet and IBP
a_alpha = 1;
b_alpha = 1;
var_alpha = 1;
a_kappa = 100;
b_kappa = 1;
var_kappa = 100;
a_gamma = 0.1;
b_gamma = 1;

% The 'getModel' function takes the settings above and creates the
% necessary 'model' structure.
getModel

% Setting for inference:
settings.Ks = 1;  % legacy parameter setting from previous code.  Do not change.
settings.Niter = 1000;  % Number of iterations of the MCMC sampler  
settings.storeEvery = 1;  % How often to store MCMC statistics
settings.saveEvery = 100;  % How often to save (to disk) structure containing MCMC sample statistics
settings.ploton = 1;  % Whether or not to plot the mode sequences and feature matrix while running sampler
settings.plotEvery = 10;  % How frequently plots are displayed
settings.plotpause = 0;

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Run IBP Inference %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
% Directory to which you want statistics stored.  This directory will be
% created if it does not already exist:
settings.saveDir = '../savedStats/BPARHMM/'; 

settings.formZInit = 1; % whether or not the sampler should be initialized with specified mode sequence.  (Experimentally, this seems to work well.)
settings.ploton = 1;

% Number of initializations/chains of the MCMC sampler:
trial_vec = [1:10];

for t=trial_vec;
    
    z_max = 0;
    for seq = 1:length(data_struct)
        
        % Form initial mode sequences to simply block partition each
        % time series into 'Ninit' features.  Time series are given
        % non-overlapping feature labels:
        T = size(data_struct(seq).obs,2);
        Ninit = 5;
        init_blocksize = floor(T/Ninit);
        z_init = [];
        for i=1:Ninit
            z_init = [z_init i*ones(1,init_blocksize)];
        end
        z_init(Ninit*init_blocksize+1:T) = Ninit;
        data_struct(seq).z_init = z_init + z_max;
        
        z_max = max(data_struct(seq).z_init);
    end

    settings.trial = t;
    
    % Call to main function:
    IBPHMMinference(data_struct,model,settings);
end