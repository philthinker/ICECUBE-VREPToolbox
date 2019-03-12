%%%%%%%%%%%%%%%%%%%%%%%%%%% IBPHMMinference.m %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Inputs:
%%%% data_struct - structure of observations and any associated blockings
%%%% prior_params - structure of hyperparameters for the prior on the model parameters
%%%% hyperhyperparams - structure of hyperparameters on concentration parameters
%%%% settings - structure of settings including truncation levels, number of Gibbs iterations, etc.
%%
%% Outputs:
%%%% various statistics saved at each iteration as 'stats(Niter).mat'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [num_prop num_accept log_prob_tracker] = IBPHMMinference_PoissonProp(data_struct,model,settings,restart,init_params)

trial = settings.trial;
if ~isfield(settings,'saveMin')
    settings.saveMin = 1;
end
Niter = settings.Niter;

display(strcat('Trial:',num2str(trial)))

%%%%%%%%%% Generate observations (if not provided) %%%%%%%%%%
%%%                       and                             %%%
%%%%%%%%%%        Initialize variables             %%%%%%%%%%

if exist('restart')
    if restart==1
        n = settings.lastSave;
      
        % Save stats:
        if isfield(settings,'filename')
            filename = strcat(settings.saveDir,'/',settings.filename,'iter',num2str(n),'trial',num2str(settings.trial));    % create filename for current iteration
        else
            filename = strcat(settings.saveDir,'/IBPHMMstats','iter',num2str(n),'trial',num2str(settings.trial));    % create filename for current iteration
        end
        
        load(filename)
        
        F = S(end).F;
        % Build initial structures for parameters and sufficient statistics:
        [theta Ustats stateCounts data_struct model S] = initializeStructs(F,model,data_struct,settings);
        clear theta Ustats stateCounts S
        
        load(filename)
        
        obsModel = model.obsModel;  % structure containing the observation model parameters
        obsModelType = obsModel.type;   % type of emissions including Gaussian, multinomial, AR, and SLDS.
        HMMhyperparams = model.HMMmodel.params; % hyperparameter structure for the HMM parameters
        numObj = length(data_struct);
        
        dist_init_flag = 0;
        theta_init_flag = 0;
        hyperparams_init_flag = 0;

        theta = S(end).theta;
        dist_struct = S(end).dist_struct;
        hyperparams = S(end).hyperparams;
        
        n_start = n + 1;
        
        if isfield('Kstar',settings)
            Kstar = settings.Kstar;
        else
            Kstar = numObj;
        end
        
    end
else
    if settings.ploton
        H1 = figure;
        H2 = figure; A2 = gca();
    end
    
    n_start = 1;
    
    %Kstar = settings.Kstar;
    
    obsModel = model.obsModel;  % structure containing the observation model parameters
    obsModelType = obsModel.type;   % type of emissions including Gaussian, multinomial, AR, and SLDS.
    HMMhyperparams = model.HMMmodel.params; % hyperparameter structure for the HMM parameters
    numObj = length(data_struct);
    
    if isfield(settings,'Kstar')
        Kstar = settings.Kstar;
    else
        Kstar = numObj;
    end
    
    % Resample concentration parameters:
    %hyperparams = sample_hyperparams_init(stateCounts,hyperparams,HMMhyperparams,HMMmodelType,resample_kappa);
    hyperparams_init_flag = 0;
    if exist('init_params','var')
        if isfield(init_params,'hyperparams')           
            hyperparams = init_params.hyperparams;
            %S = rmfield(S,'hyperparams');
            %S.hyperparams(1:settings.saveEvery/settings.storeEvery) = hyperparams;
            hyperparams_init_flag = 1;
        end
    end
    
    if ~hyperparams_init_flag   
        hyperparams.alpha0 = HMMhyperparams.a_alpha/HMMhyperparams.b_alpha;
        hyperparams.kappa0 = HMMhyperparams.a_kappa/HMMhyperparams.b_kappa;
        hyperparams.sigma0 = 1;
        hyperparams.gamma0 = HMMhyperparams.a_gamma/HMMhyperparams.b_gamma;
    end
    
    
    F_init_flag = 0;
    if exist('init_params','var')
        if isfield(init_params,'F')
            F = init_params.F;
            F_init_flag = 1;
        end
    end
    
    if ~F_init_flag
        
        %         %F = eye(2);
        %         cd /csail/fisher2/users/ebfox/HDPHMMtoolbox_tmp/
        %
        %         Kplus = 0;
        %
        %         for ii=1:length(data_struct)
        %
        %             settingsHDPHMM.trial = ii;
        %
        %             HDPHMMDPinference(data_struct(ii),modelHDPHMM,settingsHDPHMM)
        %
        %             load([settingsHDPHMM.saveDir 'HDPHMMDPstatsiter' num2str(settingsHDPHMM.Niter) 'trial' num2str(settingsHDPHMM.trial)]);
        %
        %             F(ii,unique(S.stateSeq(end).z)+Kplus) = 1;
        %             posInds = find(sum(F,1)>0);
        %             Kplus = posInds(end);
        %
        %         end
        %
        %         cd /csail/fisher2/users/ebfox/IBPHMMcondensed/
        
        if isfield(settings,'formZInit')
            for jj=1:length(data_struct)
                F(jj,unique(data_struct(jj).z_init)) = 1;
            end
        else
            F = ones(numObj,20);
        end
        %F = sample_features_init(numObj,hyperparams.gamma0);
        
        if settings.ploton
            imagesc(F,'Parent',A2); title(A2,['Featuer Matrix, Iter: ' num2str(n_start)]);
            drawnow;
        end
    end
    
    
    
    % Build initial structures for parameters and sufficient statistics:
    [theta Ustats stateCounts data_struct model S] = initializeStructs(F,model,data_struct,settings);
    
    % Sample the transition distributions pi_z, initial distribution
    % pi_init, emission weights pi_s, and global transition distribution beta
    % (only if HDP-HMM) from the priors on these distributions:
    dist_init_flag = 0;
    if exist('init_params','var')
        if isfield(init_params,'dist_struct')
            dist_struct = init_params.dist_struct;
            dist_init_flag = 1;
        end
    end
    
    if ~dist_init_flag
        %dist_struct = sample_dist(stateCounts,hyperparams,Kstar);
        dist_struct = sample_dist(stateCounts,hyperparams,Kstar);
    end
    
    if isfield(settings,'formZInit')
        Ustats_temp = Ustats;
        [stateSeq INDS stateCounts] = sample_zs_init(data_struct,dist_struct,obsModelType);
        Ustats = update_Ustats(data_struct,INDS,stateCounts,obsModelType);
        if strcmp(obsModelType,'SLDS')
            Ustats.Ustats_r = Ustats_temp.Ustats_r;
        end
        numInitThetaSamples = 1;
        display('Forming initial z using specified z_init or sampling from the prior using whatever fixed data is available')
    else
        numInitThetaSamples = 1;
    end
        
    % Sample emission params theta_{z,s}'s initially from prior (sometimes bad
    % choice):
    theta_init_flag = 0; 
    if exist('init_params','var')
        if isfield(init_params,'theta')
            theta = init_params.theta;
            theta_init_flag = 1;
        end
    end
    
    if ~theta_init_flag
        %theta = sample_theta(theta,Ustats,obsModel,Kstar);
        theta = sample_theta(theta,Ustats,obsModel,Kstar);
        for ii=1:numInitThetaSamples
            %theta = sample_theta(theta,Ustats,obsModel,Kstar);
            theta = sample_theta(theta,Ustats,obsModel,0);
        end
    end
    
    if ~exist(settings.saveDir,'file')
        mkdir(settings.saveDir);
    end
    
    % Save initial statistics and settings for this trial:
    if isfield(settings,'filename')
        settings_filename = strcat(settings.saveDir,'/',settings.filename,'_info4trial',num2str(trial));    % create filename for current iteration
        init_stats_filename = strcat(settings.saveDir,'/',settings.filename,'initialStats_trial',num2str(trial));    % create filename for current iteration
    else
        settings_filename = strcat(settings.saveDir,'/info4trial',num2str(trial));    % create filename for current iteration
        init_stats_filename = strcat(settings.saveDir,'/initialStats_trial',num2str(trial));    % create filename for current iteration
    end
    save(settings_filename,'data_struct','settings','model') % save current statistics
    save(init_stats_filename,'dist_struct','theta','hyperparams') % save current statistics
    
end

total_length = 0;
length_ii = zeros(1,length(data_struct));
for ii=1:length(data_struct)
    length_ii(ii) = length(data_struct(ii).true_labels);
    total_length = total_length + length_ii(ii);
end
cummlength = cumsum(length_ii);
z_tot = zeros(1,cummlength(end));
true_labels_tot = zeros(1,cummlength(end));
true_labels_tot(1:length_ii(1)) = data_struct(1).true_labels;
for ii=2:length(data_struct) 
    true_labels_tot(cummlength(ii-1)+1:cummlength(ii)) = data_struct(ii).true_labels;
end

if ~exist('track_joint_prob','var')
    display('Not tracking joint probability')
end

%%%%%%%%%% Run Sampler %%%%%%%%%%
log_prob_tracker = zeros(1,Niter);

num_prop = zeros(Niter,Kstar);
num_accept = zeros(Niter,Kstar);

for n=n_start:Niter
    
    %if ~F_init_flag
        %%[F dist_struct theta config_log_likelihood] = sample_features(F,hyperparams.gamma0,data_struct,dist_struct,theta,obsModel,Kstar);
        %[F dist_struct theta config_log_likelihood] = sample_features(F,hyperparams.gamma0,data_struct,dist_struct,theta,obsModel);
        [F dist_struct theta config_log_likelihood num_prop(n,:) num_accept(n,:)] = sample_features_PoissonProp(F,hyperparams.gamma0,data_struct,dist_struct,theta,obsModel,...
            hyperparams,Kstar);
    %end
    
    % Sample z and s sequences given data, transition distributions,
    % HMM-state-specific mixture weights, and emission parameters:
    % Block sample z_{1:T}|y_{1:T}
    [stateSeq INDS stateCounts] = sample_zs_old(data_struct,dist_struct,F,theta,obsModelType);
    % Create sufficient statistics:
    Ustats = update_Ustats(data_struct,INDS,stateCounts,obsModelType);
 
    % Sample the transition distributions pi_z, initial distribution
    % pi_init, emission weights pi_s, and avg transition distribution beta
    % (only if HDP-HMM):
    %
    if ~dist_init_flag
        %dist_struct = sample_dist(stateCounts,hyperparams,Kstar);
        dist_struct = sample_dist(stateCounts,hyperparams,Kstar);
    end
    
    % Sample theta_{z,s}'s conditioned on z and s sequences and data suff.
    % stats. Ustats:
    if ~theta_init_flag
        %theta = sample_theta(theta,Ustats,obsModel,Kstar);
        theta = sample_theta(theta,Ustats,obsModel,Kstar);
    end
    
    [hyperparams] = sample_IBPparam(F,hyperparams,HMMhyperparams);
    
    % Resample concentration parameters:
    if ~hyperparams_init_flag  
        hyperparams = sample_distparams(F,dist_struct,hyperparams,HMMhyperparams,50);
    end
    
    % Build and save stats structure:
    S = store_stats(S,n,settings,F,config_log_likelihood,stateSeq,dist_struct,theta,hyperparams);
    
    % Plot stats:
    if isfield(data_struct,'true_labels') & settings.ploton
                
        if rem(n,settings.plotEvery)==0
                        
            F_used = zeros(size(F));
            Nsets = length(data_struct);
            sub_x = floor(sqrt(Nsets));
            sub_y = ceil(Nsets/sub_x);
            
            z_tot(1:length_ii(1)) = stateSeq(1).z;            
            for ii=2:Nsets
                z_tot(cummlength(ii-1)+1:cummlength(ii)) = stateSeq(ii).z;
            end
            
            [relabeled_z Hamm assignment relabeled_true_labels] = mapSequence2Truth(true_labels_tot,z_tot);
            
            F_used(1,unique(stateSeq(1).z)) = 1;
            A1 = subplot(sub_x,sub_y,1,'Parent',H1);
            imagesc([relabeled_z(1:cummlength(1)); relabeled_true_labels(1:cummlength(1))],'Parent',A1,[1 max(union(relabeled_z,relabeled_true_labels))]); colorbar('peer',A1); title(A1,['Iter: ' num2str(n)]);
            for ii=2:Nsets
                F_used(ii,unique(stateSeq(ii).z)) = 1;
                A1 = subplot(sub_x,sub_y,ii,'Parent',H1);
                imagesc([relabeled_z(cummlength(ii-1)+1:cummlength(ii)); relabeled_true_labels(cummlength(ii-1)+1:cummlength(ii))],'Parent',A1,[1 max(union(relabeled_z,relabeled_true_labels))]); colorbar('peer',A1);  title(A1,['Iter: ' num2str(n)]); 
            end
            drawnow;
            
            imagesc(F+F_used,'Parent',A2); title(A2,['Featuer Matrix, Iter: ' num2str(n)]);
            drawnow;
            
            if isfield(settings,'plotpause') && settings.plotpause
                if isnan(settings.plotpause), waitforbuttonpress; else pause(settings.plotpause); end
            end
   
        end
    end
    
end

fname = [settings.saveDir '/num_accept_prop_trial' num2str(trial)];

save(fname,'num_accept','num_prop')

