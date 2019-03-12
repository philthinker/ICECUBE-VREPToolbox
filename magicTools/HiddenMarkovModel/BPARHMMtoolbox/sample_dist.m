function dist_struct = sample_dist(stateCounts,hyperparams,Kextra)

numObj = size(stateCounts.Ns,3);
Kz_prev = size(stateCounts.Ns,1);
Kz = Kz_prev + Kextra;
Ks = size(stateCounts.Ns,2);

% Define alpha0 and kappa0 in terms of alpha0+kappa0 and rho0:
alpha0 = hyperparams.alpha0;
kappa0 = hyperparams.kappa0;
sigma0 = hyperparams.sigma0;

N = stateCounts.N;  % N(i,j) = # z_t = i to z_{t+1}=j transitions. N(Kz+1,i) = 1 for i=z_1.
Ns = stateCounts.Ns;  % Ns(i,j) = # s_t = j given z_t=i

dist_struct(1:numObj) = struct('pi_z',zeros(Kz,Kz),'pi_init',zeros(1,Kz),'pi_s',zeros(Kz,Ks));

beta_vec = ones(1,Kz);

Ntemp = zeros(Kz+1,Kz);
Nstemp = zeros(Kz,Ks);

for ii=1:numObj
    
    Ntemp(1:Kz_prev,1:Kz_prev) = N(1:Kz_prev,:,ii);
    Ntemp(end,1:Kz_prev) = N(Kz_prev+1,:,ii);
    Nstemp(1:Kz_prev,:) = Ns(:,:,ii);
    
    if Ks>1
        % Sample HMM-state-specific mixture weights \psi_j's with truncation
        % level Ks given sampled s stats Ns:
        sigma_vec = (sigma0/Ks)*ones(1,Ks);
    else
        sigma_vec = sigma0;
    end
    
    pi_z = zeros(Kz,Kz);
    pi_s = zeros(Kz,Ks);
    for j=1:Kz
        kappa_vec = zeros(1,Kz);
        % Add an amount \kappa to Dirichlet parameter corresponding to a
        % self-transition:
        kappa_vec(j) = kappa0;
        % Sample \pi_j's given sampled \beta_vec and counts N, where
        % DP(\alpha+\kappa,(\alpha\beta+\kappa\delta(j))/(\alpha+\kappa)) is
        % Dirichlet distributed over the finite partition defined by beta_vec:
        pi_z(j,:) = randdirichlet_unnorm([alpha0*beta_vec + kappa_vec + Ntemp(j,:)]')';
        % Sample HMM-state-specific mixture weights \psi_j's with truncation
        % level Ks given sampled s stats Ns:
        pi_s(j,:) = randdirichlet([Nstemp(j,:) + sigma_vec]')';
    end
    pi_init = randdirichlet_unnorm([alpha0*beta_vec + Ntemp(Kz+1,:)]')';
    
    if isfield(stateCounts,'Nr')
        Nr = stateCounts.Nr(ii,:);  % Nr(i) = # r_t = i
        Kr = length(Nr);
        eta0 = hyperparams.eta0;
        dist_struct(ii).pi_r = randdirichlet([Nr + eta0/Kr]')';
    end
    
    dist_struct(ii).pi_z = pi_z;
    dist_struct(ii).pi_init = pi_init;
    dist_struct(ii).pi_s = pi_s;
    
end
