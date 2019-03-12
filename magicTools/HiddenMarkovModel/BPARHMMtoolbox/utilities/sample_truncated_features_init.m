function F = sample_truncated_features_init(numObj,Kz,gamma0)

F = zeros(numObj,Kz);
featureCounts = sum(F,1);
Fik_prev = 0;

for ii=1:numObj

    for kk=1:Kz
        
        rho = (featureCounts(kk) + gamma0/Kz)/(ii + gamma0/Kz);
        
        if rho>1
            F(ii,kk) = ~Fik_prev;
        else
            sample_set = [Fik_prev ~Fik_prev];
            ind = 1+rand(1)>(1-rho);
            F(ii,kk) = sample_set(ind);
        end
        
        F(ii,kk) = rand(1)>(1-rho);
        
        featureCounts(kk) = featureCounts(kk)+F(ii,kk);
    end
    
%     if sum(F(ii,:))==0
%         F(ii,1) = 1;
%         display('initial matrix chose no features for an object')
%     end
end