function [F dist_struct theta config_log_likelihood num_accept num_prop] = sample_features(F_prev,gamma0,data_struct,dist_struct,theta,obsModel)

num_accept = zeros(1,2);
num_prop = zeros(1,2);

obsModelType = obsModel.type;
priorType = obsModel.priorType;

[numObj Kz_prev] = size(F_prev);
Kz_max = Kz_prev+numObj;
F = zeros(numObj,Kz_max);
F(:,1:Kz_prev) = F_prev;
F = (F > 0);
featureCounts = sum(F,1);
stored_log_likelihood = zeros(1,numObj);

Ks = size(dist_struct(1).pi_s,2);

log_likelihood_ii_kk = [0 0];

seq_order = randperm(numObj);

feature_inds = [1:Kz_max];

config_log_likelihood = 0;

for ii=seq_order
    
    % Reset vector indicating the previous set of unique features to object i:
    unique_features_ii = zeros(1,Kz_max);
    unique_features_ii = (unique_features_ii > 0);
    
    % Compute likelihood under all possible parameters (including ones we
    % have not yet seen in the data):
    log_likelihood = compute_likelihood_unnorm(data_struct(ii),theta,obsModelType,[1:Kz_max],Kz_max,Ks);
    dimu = size(data_struct(ii).obs,1);
    
    % Compute current likelihood of the current feature assignments:
    if sum(F(ii,:)) == 0
        stored_log_likelihood(ii) = -inf;
    else
        
        pi_init = dist_struct(ii).pi_init(F(ii,:));
        pi_init = pi_init./sum(pi_init);
        pi_z = dist_struct(ii).pi_z(F(ii,:),F(ii,:));
        pi_z = pi_z./repmat(sum(pi_z,2),[1,size(pi_z,2)]);
        pi_s = dist_struct(ii).pi_s(F(ii,:));
        pi_s = pi_s./repmat(sum(pi_s,2),[1,size(pi_s,2)]);
        
        % Pass messages forward to integrate over the mode/state sequence:
        log_likelihood_ii = log_likelihood(F(ii,:),:,:);
        log_normalizer_ii = max(max(log_likelihood_ii,[],1),[],2);
        log_likelihood_ii = log_likelihood_ii - log_normalizer_ii(ones(sum(F(ii,:)),1),ones(Ks,1),:);
        likelihood_ii = exp(log_likelihood_ii);
        log_normalizer_ii = log_normalizer_ii - (dimu/2)*log(2*pi);
        
        [fwd_msg neglog_c] = forward_message_vec(likelihood_ii,log_normalizer_ii,data_struct(ii).blockEnd,pi_z,pi_s,pi_init);
        
        if isnan(sum(neglog_c))
            stored_log_likelihood(ii) = -inf;
        else
            stored_log_likelihood(ii) = sum(neglog_c); %observation_likelihood(F(ii,:),data_struct(ii),obsModelType,dist_struct(ii),theta);
        end
    end

    
    % For each of the currently instantiated features (this vector will
    % change after sampling each object ii):
    for kk=feature_inds((featureCounts>0))
        
        % Store previous feature value:
        Fik_prev = F(ii,kk);
        % Remove object i's count from the kth feature count:
        featureCounts(kk) = featureCounts(kk)-F(ii,kk);
        
        % If other objects are using this feature:
        if featureCounts(kk)>0
                      
            % Grab out previous likelihood of data under this feature
            % assignment:
            log_likelihood_ii_kk(Fik_prev+1) = stored_log_likelihood(ii);
            
            % Try out other value for f_{ik}:
            F(ii,kk) = ~Fik_prev; 
            
            if sum(F(ii,:)) == 0
                log_likelihood_ii_kk(~Fik_prev+1) = -inf;
            else                
                pi_init = dist_struct(ii).pi_init(F(ii,:));
                pi_init = pi_init./sum(pi_init);
                pi_z = dist_struct(ii).pi_z(F(ii,:),F(ii,:));
                pi_z = pi_z./repmat(sum(pi_z,2),[1,size(pi_z,2)]);
                pi_s = dist_struct(ii).pi_s(F(ii,:));
                pi_s = pi_s./repmat(sum(pi_s,2),[1,size(pi_s,2)]);
                
                % Pass messages forward to integrate over the mode/state sequence:
                log_likelihood_ii = log_likelihood(F(ii,:),:,:);
                log_normalizer_ii = max(max(log_likelihood_ii,[],1),[],2);
                log_likelihood_ii = log_likelihood_ii - log_normalizer_ii(ones(sum(F(ii,:)),1),ones(Ks,1),:);
                likelihood_ii = exp(log_likelihood_ii);
                log_normalizer_ii = log_normalizer_ii - (dimu/2)*log(2*pi);
                
                [fwd_msg neglog_c] = forward_message_vec(likelihood_ii,log_normalizer_ii,data_struct(ii).blockEnd,pi_z,pi_s,pi_init);
                
                if isnan(sum(neglog_c))
                    log_likelihood_ii_kk(~Fik_prev+1) = -inf;
                else
                    log_likelihood_ii_kk(~Fik_prev+1) = sum(neglog_c); %observation_likelihood(F(ii,:),data_struct(ii),obsModelType,dist_struct(ii),theta);
                end
            end
            
            % Compute accept-reject ratio:
            log_rho_star = log(numObj - featureCounts(kk)) + log_likelihood_ii_kk(1)-log(featureCounts(kk)) - log_likelihood_ii_kk(2);
            rho = exp(sign(Fik_prev-0.5)*log_rho_star);
            
            % Sample new feature value:
            if isnan(rho)
                F(ii,kk)=0;
            else
                
                if rho>1
                    F(ii,kk) = ~Fik_prev;
                else
                    sample_set = [Fik_prev ~Fik_prev];
                    ind = 1+(rand(1)>(1-rho));
                    F(ii,kk) = sample_set(ind);
                end
            end
            
            % Store likelihood of current assignment:
            stored_log_likelihood(ii) = log_likelihood_ii_kk(F(ii,kk)+1);
        
            % Add new assignment of f_{ik} to feature count of kth feature:
            featureCounts(kk) = featureCounts(kk)+F(ii,kk);
            
        else
            
            % If kth feature is specific to object i, place it in the
            % indicator vector:
            unique_features_ii(kk) = 1;
        
        end
    end
    
    
    % deal with unique features
    
    num_unique_features = sum(unique_features_ii);
    % Form proposal distribution that's uniform over "birth" and
    % each possible feature "death":
    %q = ones(1,num_unique_features+1);
    p_birth = 1-poisscdf(num_unique_features,gamma0/numObj); %0.5; poisspdf(0,gamma0/numObj);
    q = ((1-p_birth)/num_unique_features)*ones(1,num_unique_features+1);
    q(1) = p_birth;
    q = q./sum(q);
    
    Q   = cumsum(q);
    transition_case = 1 + sum(Q(end)*rand(1) > Q);
   
   if transition_case==1
       % Birth:
       f_ii = F(ii,:);
       f_ii(Kz_prev + ii) = 1;
       num_new_unique_features = num_unique_features + 1;
       
       %q_rev = ones(1,num_new_unique_features+1);
       p_birth_rev = 1-poisscdf(num_new_unique_features,gamma0/numObj); %0.5; poisspdf(0,gamma0/numObj);
       q_rev = ((1-p_birth_rev)/num_new_unique_features)*ones(1,num_new_unique_features+1);
       q_rev(1) = p_birth_rev;
       q_rev = q_rev./sum(q_rev);
       
       log_prob_proposal = log(q(1));  % probability of birth
       log_prob_reverse_proposal = log(q_rev(end)); % probability of killing the last feature
       
       num_prop(2) = num_prop(2)+1;
       
       %display('propose birth')
   else
       unique_feature_inds = feature_inds(unique_features_ii);
       death_ind = transition_case-1;
       f_ii = F(ii,:);
       f_ii(unique_feature_inds(death_ind)) = 0;       
       num_new_unique_features = num_unique_features - 1;
       
       %q_rev = ones(1,num_new_unique_features+1);
       p_birth_rev = 1-poisscdf(num_new_unique_features,gamma0/numObj);
       q_rev = ((1-p_birth_rev)/num_new_unique_features)*ones(1,num_new_unique_features+1);
       q_rev(1) = p_birth_rev;
       q_rev = q_rev./sum(q_rev);
       
       log_prob_proposal = log(q(transition_case));  % probability of killing that feature
       log_prob_reverse_proposal = log(q_rev(1)); % probability of birth step
       
       num_prop(1) = num_prop(1) + 1;
       %display('propose death')
   end
   
   % Grab likelihood under the previous assignment:
   log_likelihood_ii_kk(1) = stored_log_likelihood(ii);
   
   % Compute likelihood under the proposed change:
   if sum(f_ii) == 0
       log_likelihood_ii_kk(2) = -inf;
   else
       pi_init = dist_struct(ii).pi_init(f_ii);
       pi_init = pi_init./sum(pi_init);
       pi_z = dist_struct(ii).pi_z(f_ii,f_ii);
       pi_z = pi_z./repmat(sum(pi_z,2),[1,size(pi_z,2)]);
       pi_s = dist_struct(ii).pi_s(f_ii);
       pi_s = pi_s./repmat(sum(pi_s,2),[1,size(pi_s,2)]);
       
       % Pass messages forward to integrate over the mode/state sequence:
       log_likelihood_ii = log_likelihood(f_ii,:,:);
       log_normalizer_ii = max(max(log_likelihood_ii,[],1),[],2);
       log_likelihood_ii = log_likelihood_ii - log_normalizer_ii(ones(sum(f_ii),1),ones(Ks,1),:);
       likelihood_ii = exp(log_likelihood_ii);
       log_normalizer_ii = log_normalizer_ii - (dimu/2)*log(2*pi);
       
       [fwd_msg neglog_c] = forward_message_vec(likelihood_ii,log_normalizer_ii,data_struct(ii).blockEnd,pi_z,pi_s,pi_init);
       
       if isnan(sum(neglog_c))
           log_likelihood_ii_kk(2) = -inf;
       else
           log_likelihood_ii_kk(2) = sum(neglog_c); %observation_likelihood(F(ii,:),data_struct(ii),obsModelType,dist_struct(ii),theta);
       end
   end
   
   % Compute accept-reject ratio:
   log_rho_star = (log_likelihood_ii_kk(2) - log_likelihood_ii_kk(1))...
       + (log(poisspdf(num_new_unique_features,gamma0/numObj)) - log(poisspdf(num_unique_features,gamma0/numObj)))...
       + (log_prob_reverse_proposal - log_prob_proposal);
   rho = exp(log_rho_star);
   
   % Sample new feature value:
   if isnan(rho)
       error('NaN rho')
   else
       
       if rho>1
           F(ii,:) = f_ii;
           ind = 1;
       else
           ind = (rand(1)>(1-rho));
           F(ii,:) = (1-ind)*F(ii,:) + (ind-0)*f_ii;
       end
       
       prop_ind = (transition_case == 1)+1;
       num_accept(prop_ind) = num_accept(prop_ind) + ind;
       
   end
   
   %display(num2str((ind-0)*['accept proposal'] + (1-ind)*['reject proposal']))
   %
   %    if (ind==1) && (transition_case>1)
   %        removed_features(unique_feature_inds(death_ind)) = 1;
   %    end
   
%    if log_likelihood_ii_kk(ind+1)<stored_log_likelihood(ii)
%        display('accepted lower likelihood move')
%    else
%        display('moved to higher likelihood')
%    end
   
   stored_log_likelihood(ii) = log_likelihood_ii_kk(ind+1);
   
   config_log_likelihood = config_log_likelihood + stored_log_likelihood(ii);
   
   featureCounts = sum(F,1);
   
end

[F dist_struct theta] = reallocate_states(F,dist_struct,theta,priorType);
    
