function log_likelihood = compute_likelihood_unnorm(data_struct,theta,obsModelType,Kz_inds,Kz,Ks)

switch obsModelType
   
    case 'Gaussian'
        
        invSigma = theta.invSigma;
        mu = theta.mu;
        
        T = size(data_struct.obs,2);
        dimu = size(data_struct.obs,1);
        
        log_likelihood = -inf*ones(Kz,Ks,T);
        for kz=Kz_inds
            for ks=1:Ks
                
                cholinvSigma = chol(invSigma(:,:,kz,ks));
                dcholinvSigma = diag(cholinvSigma);
                
                u = cholinvSigma*(data_struct.obs - mu(:,kz*ones(1,T),ks));
                
                log_likelihood(kz,ks,:) = -0.5*sum(u.^2,1) + sum(log(dcholinvSigma));
            end
        end
%         normalizer = max(max(log_likelihood,[],1),[],2);
%         log_likelihood = log_likelihood - normalizer(ones(Kz,1),ones(Ks,1),:);
%         likelihood = exp(log_likelihood);
%         
%         normalizer = normalizer - (dimu/2)*log(2*pi);
        
    case {'AR','SLDS'}
        
        invSigma = theta.invSigma;
        A = theta.A;
        X = data_struct.X;
        
        T = size(data_struct.obs,2);
        dimu = size(data_struct.obs,1);
        
        log_likelihood = -inf*ones(Kz,Ks,T);
        if isfield(theta,'mu')
            
            mu = theta.mu;
            
            for kz=Kz_inds
                for ks=1:Ks
                    
                    cholinvSigma = chol(invSigma(:,:,kz,ks));
                    dcholinvSigma = diag(cholinvSigma);
                    
                    u = cholinvSigma*(data_struct.obs - A(:,:,kz,ks)*X-mu(:,kz*ones(1,T),ks));
                    
                    log_likelihood(kz,ks,:) = -0.5*sum(u.^2,1) + sum(log(dcholinvSigma));
                    
                end
            end
        else
            
            for kz=Kz_inds
                for ks=1:Ks
                    
                    cholinvSigma = chol(invSigma(:,:,kz,ks));
                    dcholinvSigma = diag(cholinvSigma);
                    
                    u = cholinvSigma*(data_struct.obs - A(:,:,kz,ks)*X);
                    
                    log_likelihood(kz,ks,:) = -0.5*sum(u.^2,1) + sum(log(dcholinvSigma));
                    
                end
            end
            
        end
        
       
%         normalizer = max(max(log_likelihood,[],1),[],2);
%         log_likelihood = log_likelihood - normalizer(ones(Kz,1),ones(Ks,1),:);
%         likelihood = exp(log_likelihood);
%        
%         normalizer = normalizer - (dimu/2)*log(2*pi);
       
    case 'Multinomial'

        log_likelihood = log(theta.p(:,:,data_struct.obs));
        %normalizer = zeros(1,size(data_struct.obs,2));
       
end