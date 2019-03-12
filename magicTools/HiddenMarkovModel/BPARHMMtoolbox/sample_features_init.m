function F = sample_features_init(numObj,gamma0)


F(1,1:poissrnd(gamma0))=1;
if sum(F(1,:))==0;
    F(1,1) = 1;
end

featureCounts = sum(F,1);

posInds = find(sum(F,1)>0);
Kz = posInds(end);

for ii=2:numObj

    for kk=1:Kz
        
        rho = featureCounts(kk)/ii;
        
        F(ii,kk) = rand(1)>(1-rho);

    end
    
    F(ii,Kz+1:Kz+poissrnd(gamma0/ii)) = 1;
    
    if sum(F(ii,:))==0
        F(ii,Kz+1)=1;
    end
    
    featureCounts = sum(F,1);
    
    posInds = find(featureCounts>0);
    Kz = posInds(end);
    
end

F = F(:,1:Kz);