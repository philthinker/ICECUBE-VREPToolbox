%%%%%%%%%%%%%%%%%%%%%%%%%%% IBPHMMinference.m %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%           ****SEE 'utilities/runstuff.m' FOR EXAMPLE INPUTS *****
%%
%% Inputs:
%%%% data_struct - structure of observations, initial segmentation of data, etc.
%%%% model - structure containing hyperparameters for priors on feature matrix, transition distributions, and dynamic parameters 
%%%% settings - structure of settings including number of Gibbs iterations, directory to save statistics to, how often to save, etc.
%%
%% Outputs
%%%% various statistics saved at preset frequency to
%%%% settings.saveDir/IBPHMMstatsiter[insert iter #]trial[insert trial #].mat
%%%% in a structure of the form S(store_count).field(time_series).subfield
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function IBPHMMinference(data_struct,model,settings)

trial = settings.trial;
if ~isfield(settings,'saveMin')
    settings.saveMin = 1;
end
Niter = settings.Niter;

display(strcat('Trial:',num2str(trial)))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%               Initialize variables                       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if settings.ploton
    H1 = figure;
    H2 = figure; A2 = gca();
end

obsModel = model.obsModel;  % structure containing the observation model parameters
obsModelType = obsModel.type;   % type of emissions including Gaussian, multinomial, AR, and SLDS.  CODE CURRENTLY DOES NOT SUPPORT SLDS
HMMhyperparams = model.HMMmodel.params; % hyperparameter structure for the HMM parameters
numObj = length(data_struct);

% Initialize concentration parameters:
hyperparams.alpha0 = HMMhyperparams.a_alpha/HMMhyperparams.b_alpha;
hyperparams.kappa0 = HMMhyperparams.a_kappa/HMMhyperparams.b_kappa;
hyperparams.sigma0 = 1;
hyperparams.gamma0 = HMMhyperparams.a_gamma/HMMhyperparams.b_gamma;

% Initialize feature matrix:    
if isfield(settings,'formZInit')
    for jj=1:length(data_struct)
        F(jj,unique(data_struct(jj).z_init)) = 1;
    end
else
    F = ones(numObj,20);
end

if settings.ploton
    imagesc(F,'Parent',A2); title(A2,['Featuer Matrix, Iter: 1']);
    drawnow;
end

% Build initial structures for parameters and sufficient statistics:
[theta Ustats stateCounts data_struct model S] = initializeStructs(F,model,data_struct,settings);

% Sample the transition distributions pi_z and initial distribution
% pi_init from the priors on these distributions:
dist_struct = sample_dist(stateCounts,hyperparams,numObj);

% Initialize dynamic parameters either from prior or using specified
% initial mode sequences:
if isfield(settings,'formZInit')
    Ustats_temp = Ustats;
    [stateSeq INDS stateCounts] = sample_zs_init(data_struct,dist_struct,obsModelType);
    Ustats = update_Ustats(data_struct,INDS,stateCounts,obsModelType);
    numInitThetaSamples = 1;
    display('Forming initial z using specified z_init or sampling from the prior using whatever fixed data is available')
else
    numInitThetaSamples = 1;
end

theta = sample_theta(theta,Ustats,obsModel,numObj);
for ii=1:numInitThetaSamples
    theta = sample_theta(theta,Ustats,obsModel,0);
end

% Create directory where statistics will be stored:
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                   Run Sampler                            %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

num_accept = zeros(Niter,2);
num_prop = zeros(Niter,2);

for n=1:Niter
    
    [F dist_struct theta config_log_likelihood num_accept(n,:) num_prop(n,:)] = sample_features(F,hyperparams.gamma0,data_struct,dist_struct,theta,obsModel);
    
    % Sample z and s sequences given data, transition distributions,
    % HMM-state-specific mixture weights, and emission parameters:
    % Block sample z_{1:T}|y_{1:T}
    [stateSeq INDS stateCounts] = sample_zs(data_struct,dist_struct,F,theta,obsModelType);
    % Create sufficient statistics:
    Ustats = update_Ustats(data_struct,INDS,stateCounts,obsModelType);
 
    % Sample the transition distributions pi_z and initial distribution
    % pi_init:
    dist_struct = sample_dist(stateCounts,hyperparams,numObj);
    
    % Sample theta_{z,s}'s conditioned on z and s sequences and data suff.
    % stats. Ustats:
    theta = sample_theta(theta,Ustats,obsModel,numObj);
    
    % Resample IBP hyperparameter:
    [hyperparams] = sample_IBPparam(F,hyperparams,HMMhyperparams);
    
    % Resample Dirichlet concentration parameters: 
    hyperparams = sample_distparams(F,dist_struct,hyperparams,HMMhyperparams,50);
    
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
            imagesc([relabeled_z(1:cummlength(1)); relabeled_true_labels(1:cummlength(1))],'Parent',A1,[1 max(union(relabeled_z,relabeled_true_labels))]); title(A1,['Iter: ' num2str(n)]);
            for ii=2:Nsets
                F_used(ii,unique(stateSeq(ii).z)) = 1;
                A1 = subplot(sub_x,sub_y,ii,'Parent',H1);
                imagesc([relabeled_z(cummlength(ii-1)+1:cummlength(ii)); relabeled_true_labels(cummlength(ii-1)+1:cummlength(ii))],'Parent',A1,[1 max(union(relabeled_z,relabeled_true_labels))]); title(A1,['Iter: ' num2str(n)]); 
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