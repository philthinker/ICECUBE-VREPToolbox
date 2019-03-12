function [hamming_dist meanF config_log_likelihood alpha_hist gamma_hist kappa_hist] = analyze_results(trial_vec,settings,object_ind,saveMin)

figure;

if ~exist('saveMin','var')
    saveMin = 1;
end
Ntrial = length(trial_vec);
saveDir = settings.saveDir;
saveEvery = settings.saveEvery;
storeEvery = settings.storeEvery;
Niter = settings.Niter;
hamming_dist = zeros(Ntrial,Niter/storeEvery);
config_log_likelihood = zeros(Ntrial,Niter/storeEvery);

load([saveDir 'info4trial' num2str(trial_vec(1))])

alpha_hist = zeros(size(hamming_dist));
gamma_hist = zeros(size(hamming_dist));
kappa_hist = zeros(size(hamming_dist));

numObj = length(data_struct);
meanF = zeros(numObj,100,length(trial_vec));

total_length = 0;
length_ii = zeros(1,length(data_struct));
for ii=1:length(data_struct)
    length_ii(ii) = length(data_struct(ii).true_labels);
    total_length = total_length + length_ii(ii);
    trueF(ii,unique(data_struct(ii).true_labels))=1;
end
cummlength = cumsum(length_ii);
z = zeros(1,cummlength(end));
true_labels = zeros(1,cummlength(end));

trial_count = 1;
for trial = trial_vec
    iter_count = 1;
    for iter=saveMin:storeEvery:Niter
        n = iter+saveEvery-1;
        if rem(n,saveEvery)==0 && n<=Niter
            filename = [saveDir 'IBPHMMstatsiter' num2str(n) 'trial' num2str(trial) '.mat'];
            load(filename)
            store_count = 1;
        end

        z(1:length_ii(1)) = S(store_count).stateSeq(1).z;
        true_labels(1:length_ii(1)) = data_struct(1).true_labels;
        for ii=2:length(data_struct)
            z(cummlength(ii-1)+1:cummlength(ii)) = S(store_count).stateSeq(ii).z;
            true_labels(cummlength(ii-1)+1:cummlength(ii)) = data_struct(ii).true_labels;
        end
        
        [relabeled_z hamming_dist(trial_count,iter_count) assignment relabeled_true_labels] = mapSequence2Truth(true_labels,z);
        
        if exist('object_ind','var')
            hamm_inds = cummlength(object_ind-1):1:cummlength(object_ind);
            hamming_dist(trial_count,iter_count) = sum(relabeled_z(hamm_inds)~=relabeled_true_labels(hamm_inds))/length(hamm_inds);         
        end
        
        config_log_likelihood(trial_count,iter_count) = S(store_count).config_log_likelihood;
        
        F = S(store_count).F;
        Kz = size(F,2);
        
        Kz_tmp = max(Kz,max(relabeled_true_labels));
        if Kz_tmp==1;
            Kz_tmp = 10;
        end
        
        if iter==Niter
            %figure;
            sub_x = floor(sqrt(numObj));
            sub_y = ceil(numObj/sub_x);
            A1 = subplot(sub_x,sub_y,1);
            imagesc([relabeled_z(1:cummlength(1)); relabeled_true_labels(1:cummlength(1))],'Parent',A1,[1 Kz_tmp]);
            for ii=2:numObj
                A1 = subplot(sub_x,sub_y,ii);
                imagesc([relabeled_z(cummlength(ii-1)+1:cummlength(ii)); relabeled_true_labels(cummlength(ii-1)+1:cummlength(ii))],'Parent',A1,[1 Kz_tmp]); 
            end
            drawnow;
        end
            
        relabeled_est_mat = mapMatrix2Truth(trueF,F);
        Kz = size(relabeled_est_mat,2);
        
        meanF(:,1:Kz,trial_count) = meanF(:,1:Kz,trial_count) + relabeled_est_mat; %lof(F);
        
        alpha_hist(trial_count,iter_count) = S(store_count).hyperparams.alpha0;
        gamma_hist(trial_count,iter_count) = S(store_count).hyperparams.gamma0;
        kappa_hist(trial_count,iter_count) = S(store_count).hyperparams.kappa0;

        store_count = store_count + 1;
        iter_count = iter_count + 1;
    end
    meanF(:,:,trial_count) = meanF(:,:,trial_count)./(iter_count-1);
    trial_count = trial_count + 1;
end
