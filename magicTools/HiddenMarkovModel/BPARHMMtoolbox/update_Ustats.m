function Ustats = update_Ustats(data_struct,INDS,stateCounts,obsModelType)

Ns = stateCounts.Ns;

Kz = size(Ns,1);
Ks = size(Ns,2);

switch obsModelType

    case {'Gaussian'}
        
        dimu = size(data_struct(1).obs,1);

        store_YY = zeros(dimu,dimu,Kz,Ks);
        store_sumY = zeros(dimu,Kz,Ks);
        store_card = zeros(Kz,Ks);
        
        for ii=1:length(data_struct)
            
            unique_z = find(sum(Ns(:,:,ii),2))';

            u = data_struct(ii).obs;
            
            for kz=unique_z
                unique_s_for_z = find(Ns(kz,:,ii));
                for ks=unique_s_for_z
                    obsInd = INDS(ii).obsIndzs(kz,ks).inds(1:INDS(ii).obsIndzs(kz,ks).tot);
                    store_YY(:,:,kz,ks) = store_YY(:,:,kz,ks) + u(:,obsInd)*u(:,obsInd)';
                    store_sumY(:,kz,ks) = store_sumY(:,kz,ks) + sum(u(:,obsInd),2);
                end
            end
            store_card = store_card + Ns(:,:,ii);
        end
        
      
        Ustats.card = store_card;
        Ustats.YY = store_YY;
        Ustats.sumY = store_sumY;
    
    case {'AR','SLDS'}

        dimu = size(data_struct(1).obs,1);
        dimX = size(data_struct(1).X,1);

        store_XX = zeros(dimX,dimX,Kz,Ks);
        store_YX = zeros(dimu,dimX,Kz,Ks);
        store_YY = zeros(dimu,dimu,Kz,Ks);
        store_sumY = zeros(dimu,Kz,Ks);
        store_sumX = zeros(dimX,Kz,Ks);
        store_card = zeros(Kz,Ks);

        for ii=1:length(data_struct)
            
            unique_z = find(sum(Ns(:,:,ii),2))';

            u = data_struct(ii).obs;
            X = data_struct(ii).X;

            for kz=unique_z
                unique_s_for_z = find(Ns(kz,:,ii));
                for ks=unique_s_for_z
                    obsInd = INDS(ii).obsIndzs(kz,ks).inds(1:INDS(ii).obsIndzs(kz,ks).tot);
                    store_XX(:,:,kz,ks) = store_XX(:,:,kz,ks) + X(:,obsInd)*X(:,obsInd)';
                    store_YX(:,:,kz,ks) = store_YX(:,:,kz,ks) + u(:,obsInd)*X(:,obsInd)';
                    store_YY(:,:,kz,ks) = store_YY(:,:,kz,ks) + u(:,obsInd)*u(:,obsInd)';
                    store_sumY(:,kz,ks) = store_sumY(:,kz,ks) + sum(u(:,obsInd),2);
                    store_sumX(:,kz,ks) = store_sumX(:,kz,ks) + sum(X(:,obsInd),2);
                end
            end
            store_card = store_card + Ns(:,:,ii);
            
        end
               
        Ustats.card = store_card;
        Ustats.XX = store_XX;
        Ustats.YX = store_YX;
        Ustats.YY = store_YY;
        Ustats.sumY = store_sumY;
        Ustats.sumX = store_sumX;
        
        if strcmp(obsModelType,'SLDS') && isfield(stateCounts,'Nr')  % Don't update if just using z_init
            
            Nr = stateCounts.Nr;
            Kr = length(Nr);
            unique_r = find(Nr);
            
            dimy = size(data_struct(1).tildeY,1);
            
            store_tildeYtildeY = zeros(dimy,dimy,Kr);
            store_sumtildeY = zeros(dimy,Kr);
            store_card = zeros(1,Kr);
            
            for ii=1:length(data_struct)
                
                tildeY = data_struct(ii).tildeY;
                for kr = unique_r
                    obsInd_r = INDS(ii).obsIndr(kr).inds(1:INDS(ii).obsIndr(kr).tot);
                    store_tildeYtildeY(:,:,kr) = store_tildeYtildeY(:,:,kr) + tildeY(:,obsInd_r)*tildeY(:,obsInd_r)';
                    store_sumtildeY(:,kr) = store_sumtildeY(:,kr) + sum(tildeY(:,obsInd_r),2);
                end
                store_card = store_card + Nr(ii,:);
                
            end
            
            Ustats.Ustats_r.YY = store_tildeYtildeY;
            Ustats.Ustats_r.sumY = store_sumtildeY;
            Ustats.Ustats_r.card = store_card;
        end
        
    case 'Multinomial'

        numVocab = data_struct(1).numVocab;
        
        store_counts = zeros(numVocab,Kz,Ks);

        for ii=1:length(data_struct)
            u = data_struct(ii).obs;

            unique_z = find(sum(Ns(:,:,ii),2))';

            for kz = unique_z
                unique_s_for_z = find(Ns(kz,:,ii));
                for ks = unique_s_for_z
                    obsInd = INDS(ii).obsIndzs(kz,ks).inds(1:INDS(ii).obsIndzs(kz,ks).tot);
                    store_counts(:,kz,ks) = store_counts(:,kz,ks) + histc(u(obsInd),[1:numVocab])';
                end
            end
        end
        
        Ustats.card = store_counts;

end