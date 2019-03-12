function [F dist_struct theta] = reallocate_states(F,dist_struct,theta,priorType)

[numObj Kz] = size(F);

[F sort_ind] = lof(F);
posInds = find(sum(F)>0);
sort_ind = sort_ind(1:posInds(end));
F = F(:,1:posInds(end));

for ii=1:numObj;
    dist_struct(ii).pi_z = dist_struct(ii).pi_z(sort_ind,sort_ind);
    dist_struct(ii).pi_s = dist_struct(ii).pi_s(sort_ind,:);
    dist_struct(ii).pi_init = dist_struct(ii).pi_init(sort_ind);
end

switch priorType
    
    case 'MNIW'
                
        theta.invSigma = theta.invSigma(:,:,sort_ind,:);
        theta.A =  theta.A(:,:,sort_ind,:);
        
    case {'NIW','IW-N','IW-N-tiedwithin'}
      
        theta.invSigma = theta.invSigma(:,:,sort_ind,:);
        theta.mu =  theta.mu(:,1:Kz,:);
        
    case {'MNIW-N','N-IW-N','Afixed-IW-N'}
        
        theta.invSigma = theta.invSigma(:,:,sort_ind,:);
        theta.A = theta.A(:,:,1:Kz,:);
        theta.mu =  theta.mu(:,1:Kz,:);     
   
    case {'ARD'}
      
        theta.invSigma = theta.invSigma(:,:,sort_ind,:);
        theta.A = theta.A(:,:,sort_ind,:);
        theta.mu =  theta.mu(:,sort_ind,:);   
        theta.ARDhypers = theta.ARDhypers(:,sort_ind,:); 
        
    case 'IW'
        
        theta.invSigma = theta.invSigma(:,:,sort_ind,:);
        
end
