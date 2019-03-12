function S = store_stats(S,n,settings,F_n,config_log_likelihood_n,stateSeq_n,dist_struct_n,theta_n,hyperparams_n)   


if rem(n,settings.storeEvery)==0 & n>=settings.saveMin 
    
    storeCount = rem(n,settings.saveEvery);
    if ~storeCount
        storeCount = settings.saveEvery;
    end
    
    S(storeCount).stateSeq = stateSeq_n;
    S(storeCount).dist_struct = dist_struct_n;
    S(storeCount).F = F_n;
    S(storeCount).config_log_likelihood = config_log_likelihood_n;
    S(storeCount).theta = theta_n;
    S(storeCount).hyperparams = hyperparams_n;   
end
    
if rem(n,settings.saveEvery)==0

    % Save stats:
    if isfield(settings,'filename')
        filename = strcat(settings.saveDir,'/',settings.filename,'iter',num2str(n),'trial',num2str(settings.trial));    % create filename for current iteration
    else
        filename = strcat(settings.saveDir,'/IBPHMMstats','iter',num2str(n),'trial',num2str(settings.trial));    % create filename for current iteration
    end

    save(filename,'S') % save current statistics
    
    display(strcat('Iteration: ',num2str(n)))
     
end