function theta = sample_theta(theta,Ustats,obsModel,Kextra)

prior_params = obsModel.params;

switch obsModel.type
    
    case 'Multinomial'
        
        p = theta.p;
        store_counts = Ustats.card;
        alpha_vec = prior_params.alpha;
        
        [Kz Ks] = size(store_counts);
        
        for kz=1:Kz
            for ks = 1:Ks
                p(kz,ks,:) = randdirichlet([alpha_vec'+store_counts(:,kz,ks)])';
            end
        end
        
        theta.p = p;
        
    case {'Gaussian','AR','SLDS'}
        
        theta = sample_theta_submodule(theta,Ustats,obsModel.priorType,prior_params,Kextra);
        
        if strcmp(obsModel.type,'SLDS')
            y_prior_params = obsModel.y_params;
            theta.theta_r = sample_theta_submodule(theta.theta_r,Ustats.Ustats_r,obsModel.y_priorType,y_prior_params,[]);
        end
        
end

return;


function theta = sample_theta_submodule(theta,Ustats,priorType,prior_params,Kextra)

nu = prior_params.nu;
nu_delta = prior_params.nu_delta;
store_card = Ustats.card;

if size(store_card,1)==1
    store_card = store_card';
end
store_card = [store_card; zeros(Kextra,size(store_card,2))];
[Kz Ks] = size(store_card);

switch priorType
    
    case 'MNIW'
        
        invSigma = theta.invSigma;
        A = theta.A;
        
        store_XX = Ustats.XX;
        store_YX = Ustats.YX;
        store_YY = Ustats.YY;
        store_sumY = Ustats.sumY;
        store_sumX = Ustats.sumX;
        
        K = prior_params.K;
        M = prior_params.M;
        MK = prior_params.M*prior_params.K;
        MKM = MK*prior_params.M';
        
        for kz=1:Kz
            for ks=1:Ks
                
                if store_card(kz,ks)>0
                    
                    %% Given X, Y get sufficient statistics
                    Sxx       = store_XX(:,:,kz,ks) + K;
                    Syx       = store_YX(:,:,kz,ks) + MK;
                    Syy       = store_YY(:,:,kz,ks) + MKM;
                    SyxSxxInv = Syx/Sxx;
                    Sygx      = Syy - SyxSxxInv*Syx';
                    Sygx = (Sygx + Sygx')/2;
                    
                else
                    Sxx = K;
                    SyxSxxInv = M;
                    Sygx = 0;
                    
                end
                
                % Sample Sigma given s.stats
                [sqrtSigma sqrtinvSigma] = randiwishart(Sygx + nu_delta,nu+store_card(kz,ks));
                invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                
                % Sample A given Sigma and s.stats
                cholinvSxx = chol(inv(Sxx));
                A(:,:,kz,ks) = sampleFromMatrixNormal(SyxSxxInv,sqrtSigma,cholinvSxx);
                
            end
        end
        
        theta.invSigma = invSigma;
        theta.A =  A;
        
    case 'NIW'
        
        invSigma = theta.invSigma;
        mu = theta.mu;
        
        store_YY = Ustats.YY;
        store_sumY = Ustats.sumY;
        
        K = prior_params.K;
        M = prior_params.M;
        MK = prior_params.M*prior_params.K;
        MKM = MK*prior_params.M';
        
        for kz=1:Kz
            for ks=1:Ks
                
                if store_card(kz,ks)>0
                    
                    %% Given X, Y get sufficient statistics
                    Sxx       = store_card(kz,ks) + K;
                    Syx       = store_sumY(:,kz,ks) + MK;
                    Syy       = store_YY(:,:,kz,ks) + MKM;
                    SyxSxxInv = Syx/Sxx;
                    Sygx      = Syy - SyxSxxInv*Syx';
                    Sygx = (Sygx + Sygx')/2;
                    
                else
                    Sxx = K;
                    SyxSxxInv = M;
                    Sygx = 0;
                    
                end
                
                % Sample Sigma given s.stats
                [sqrtSigma sqrtinvSigma] = randiwishart(Sygx + nu_delta,nu+store_card(kz,ks));
                invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                
                % Sample A given Sigma and s.stats
                cholinvSxx = chol(inv(Sxx));
                mu(:,kz,ks) = sampleFromMatrixNormal(SyxSxxInv,sqrtSigma,cholinvSxx);
                
            end
        end
        
        theta.invSigma = invSigma;
        theta.mu =  mu;
        
    case {'MNIW-N'}
        
        invSigma = theta.invSigma;
        A = theta.A;
        mu = theta.mu;
        
        store_XX = Ustats.XX;
        store_YX = Ustats.YX;
        store_YY = Ustats.YY;
        store_sumY = Ustats.sumY;
        store_sumX = Ustats.sumX;
        
        % If MNIW-N, K and M are as in MNIW. If IW-N, K=1 and M=0.
        K = prior_params.K;
        M = prior_params.M;
        MK = prior_params.M*prior_params.K;
        MKM = MK*prior_params.M';
        
        if ~isfield(prior_params,'numIter'), prior_params.numIter = 50; end
        
        numIter = prior_params.numIter;
        
        mu0 = prior_params.mu0;
        cholSigma0 = prior_params.cholSigma0;
        Lambda0 = inv(prior_params.cholSigma0'*prior_params.cholSigma0);
        theta0 = Lambda0*prior_params.mu0;
        
        dimu = size(nu_delta,1);
        
        for kz=1:Kz
            for ks=1:Ks
                
                if store_card(kz,ks)>0  %**
                    for n=1:numIter
                        
                        %% Given X, Y get sufficient statistics
                        Sxx       = store_XX(:,:,kz,ks) + K;
                        Syx       = store_YX(:,:,kz,ks) + MK - mu(:,kz,ks)*store_sumX(:,kz,ks)';
                        Syy       = store_YY(:,:,kz,ks) + MKM ...
                            - mu(:,kz,ks)*store_sumY(:,kz,ks)' - store_sumY(:,kz,ks)*mu(:,kz,ks)' + store_card(kz,ks)*mu(:,kz,ks)*mu(:,kz,ks)';
                        SyxSxxInv = Syx/Sxx;
                        Sygx      = Syy - SyxSxxInv*Syx';
                        Sygx = (Sygx + Sygx')/2;
                        
                        % Sample Sigma given s.stats
                        [sqrtSigma sqrtinvSigma] = randiwishart(Sygx + nu_delta,nu+store_card(kz,ks));
                        invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                        
                        % Sample A given Sigma and s.stats
                        cholinvSxx = chol(inv(Sxx));
                        A(:,:,kz,ks) = sampleFromMatrixNormal(SyxSxxInv,sqrtSigma,cholinvSxx);
                        
                        % Sample mu given A and Sigma
                        Sigma_n = inv(Lambda0 + store_card(kz,ks)*invSigma(:,:,kz,ks));
                        mu_n = Sigma_n*(theta0 + invSigma(:,:,kz,ks)*(store_sumY(:,kz,ks)-A(:,:,kz,ks)*store_sumX(:,kz,ks)));
                        
                        mu(:,kz,ks) = mu_n + chol(Sigma_n)'*randn(dimu,1);
                        
                    end
                else
                    Sxx = K;
                    SyxSxxInv = M;
                    Sygx = 0;
                    
                    [sqrtSigma sqrtinvSigma] = randiwishart(nu_delta,nu);
                    invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                    
                    cholinvK = chol(inv(K));
                    A(:,:,kz,ks) = sampleFromMatrixNormal(M,sqrtSigma,cholinvK);
                    
                    mu(:,kz,ks) = mu0 + cholSigma0'*randn(dimu,1);
                    
                end
                
            end
        end
        
        theta.invSigma = invSigma;
        theta.A = A;
        theta.mu =  mu;
        
    case {'IW-N'}
        
        invSigma = theta.invSigma;
        mu = theta.mu;
        
        store_YY = Ustats.YY;
        store_sumY = Ustats.sumY;
        
        if ~isfield(prior_params,'numIter'), prior_params.numIter = 50; end
        
        numIter = prior_params.numIter;
        
        mu0 = prior_params.mu0;
        cholSigma0 = prior_params.cholSigma0;
        Lambda0 = inv(prior_params.cholSigma0'*prior_params.cholSigma0);
        theta0 = Lambda0*prior_params.mu0;
        
        dimu = size(nu_delta,1);
        
        for kz=1:Kz
            for ks=1:Ks
                
                if store_card(kz,ks)>0  %**
                    for n=1:numIter
                        
                        %% Given X, Y get sufficient statistics
                        Syy       = store_YY(:,:,kz,ks) ...
                            - mu(:,kz,ks)*store_sumY(:,kz,ks)' - store_sumY(:,kz,ks)*mu(:,kz,ks)' + store_card(kz,ks)*mu(:,kz,ks)*mu(:,kz,ks)';
                        Sygx = (Syy + Syy')/2;
                        
                        % Sample Sigma given s.stats
                        [sqrtSigma sqrtinvSigma] = randiwishart(Sygx + nu_delta,nu+store_card(kz,ks));
                        invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                        
                        % Sample mu given A and Sigma
                        Sigma_n = inv(Lambda0 + store_card(kz,ks)*invSigma(:,:,kz,ks));
                        mu_n = Sigma_n*(theta0 + invSigma(:,:,kz,ks)*store_sumY(:,kz,ks));
                        
                        mu(:,kz,ks) = mu_n + chol(Sigma_n)'*randn(dimu,1);
                        
                    end
                else
                    
                    [sqrtSigma sqrtinvSigma] = randiwishart(nu_delta,nu);
                    invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                    
                    mu(:,kz,ks) = mu0 + cholSigma0'*randn(dimu,1);
                    
                end
                
            end
        end
        
        theta.invSigma = invSigma;
        theta.mu =  mu;
        
    case {'IW-N-tiedwithin'}
        
        invSigma = theta.invSigma;
        mu = theta.mu;
        
        store_YY = Ustats.YY;
        store_sumY = Ustats.sumY;
        
        if ~isfield(prior_params,'numIter'), prior_params.numIter = 50; end
        
        numIter = prior_params.numIter;
        
        mu0 = prior_params.mu0;
        cholSigma0 = prior_params.cholSigma0;
        Lambda0 = inv(prior_params.cholSigma0'*prior_params.cholSigma0);
        theta0 = Lambda0*prior_params.mu0;
        
        dimu = size(nu_delta,1);
        
        for kz=1:Kz
            
            store_invSigma = invSigma(:,:,kz,1);
            
            for n=1:numIter
                
                % muY = zeros(dimu,dimu);
                % muN = zeros(dimu,dimu);
                
                for ks=1:Ks
                    if store_card(kz,ks)>0
                        % Sample mu given A and Sigma
                        Sigma_n = inv(Lambda0 + store_card(kz,ks)*store_invSigma);
                        mu_n = Sigma_n*(theta0 + store_invSigma*store_sumY(:,kz,ks));
                        
                        mu(:,kz,ks) = mu_n + chol(Sigma_n)'*randn(dimu,1);
                        % muY = muY + mu(:,kz,ks)*store_sumY(:,kz,ks)';
                        % muN = muN + store_card(kz,ks)*mu(:,kz,ks)*mu(:,kz,ks)';
                    else
                        mu(:,kz,ks) = mu0 + cholSigma0'*randn(dimu,1);
                    end
                end
                
                %% Given Y get sufficient statistics   %**
                store_card_kz = store_card(kz,:);
                squeeze_mu_kz = squeeze(mu(:,kz,:));
                muY = squeeze_mu_kz*squeeze(store_sumY(:,kz,:))';
                muN = (repmat(store_card_kz,[dimu 1]).*squeeze_mu_kz)*squeeze_mu_kz';
                Syy       = sum(store_YY(:,:,kz,:),4) - muY - muY' + muN;
                Syy = (Syy + Syy')/2;
                
                % Sample Sigma given s.stats
                [sqrtSigma sqrtinvSigma] = randiwishart(Syy + nu_delta,nu+sum(store_card_kz));
                store_invSigma = sqrtinvSigma'*sqrtinvSigma;
                
            end
            
            invSigma(:,:,kz,1:Ks) = repmat(store_invSigma,[1 1 1 Ks]);
            
        end
        
        theta.invSigma = invSigma;
        theta.mu =  mu;
        
        
    case {'N-IW-N','Afixed-IW-N','ARD'}
        
        invSigma = theta.invSigma;
        A = theta.A;
        mu = theta.mu;
        
        store_XX = Ustats.XX;
        store_YX = Ustats.YX;
        store_YY = Ustats.YY;
        store_sumY = Ustats.sumY;
        store_sumX = Ustats.sumX;
        
        if ~isfield(prior_params,'numIter'), prior_params.numIter = 50; end
        
        numIter = prior_params.numIter;
        
        if ~isfield(prior_params,'zeroMean')
            mu0 = prior_params.mu0;
            cholSigma0 = prior_params.cholSigma0;
            Lambda0 = inv(prior_params.cholSigma0'*prior_params.cholSigma0);
            theta0 = Lambda0*prior_params.mu0;
        end
        
        r = size(A,2)/size(A,1);
        
        if strcmp(priorType,'N-IW-N')
            M = prior_params.M;
            Lambda0_A = prior_params.Lambda0_A;
            theta0_A = Lambda0_A*M(:);
            dim_vecA = numel(M);
            
            XinvSigmaX = zeros(dim_vecA,dim_vecA);
            XinvSigmay = zeros(dim_vecA,1);
        elseif strcmp(priorType,'ARD')
            M = zeros(size(A(:,:,1,1)));
            theta0_A = zeros(numel(M),1);
            [numRow numCol] = size(M);
            dim_vecA = numRow*numCol;
            
            a_ARD = prior_params.a_ARD;
            b_ARD = prior_params.b_ARD;
            
            if r == 1
                % One hyperparameter per column of SLDS matrix:
                numHypers = numCol;
            else
                % One hyperparameter per lag matrix in VAR matrix:
                numHypers = r;
            end
            
            %numHypers = numCol;
            
            ARDhypers = ones(numHypers,Kz,Ks);
            posInds = find(store_card>0);
            if ~isempty(posInds)
                ARDhypers(:,1:size(mu,2)) = theta.ARDhypers;
            end
            
        end
        
        dimu = size(nu_delta,1);
        
        for n=1:numIter
            
            for kz=1:Kz
                for ks=1:Ks
                    
                    if store_card(kz,ks)>0  %**
                        
                        % Sample Sigma given A, mu, and s.stats
                        S = store_YY(:,:,kz,ks) + A(:,:,kz,ks)*store_XX(:,:,kz,ks)*A(:,:,kz,ks)'...
                            -A(:,:,kz,ks)*store_YX(:,:,kz,ks)' - store_YX(:,:,kz,ks)*A(:,:,kz,ks)'...
                            - mu(:,kz,ks)*(store_sumY(:,kz,ks)-A(:,:,kz,ks)*store_sumX(:,kz,ks))'- (store_sumY(:,kz,ks)-A(:,:,kz,ks)*store_sumX(:,kz,ks))*mu(:,kz,ks)'...
                            + store_card(kz,ks)*mu(:,kz,ks)*mu(:,kz,ks)';
                        S = 0.5*(S+S');
                        
                        [sqrtSigma sqrtinvSigma] = randiwishart(S + nu_delta,nu+store_card(kz,ks));
                        invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                        
                        if isfield(prior_params,'zeroMean')
                            mu(:,kz,ks) = zeros(dimu,1);
                        else
                            % Sample mu given A, Sigma, and s.stats
                            %  %**
                            Sigma_n = inv(Lambda0 + store_card(kz,ks)*invSigma(:,:,kz,ks));
                            mu_n = Sigma_n*(theta0 + invSigma(:,:,kz,ks)*(store_sumY(:,kz,ks)-A(:,:,kz,ks)*store_sumX(:,kz,ks)));
                            mu(:,kz,ks) = mu_n + chol(Sigma_n)'*randn(dimu,1);
                        end
                        
                        if strcmp(priorType,'N-IW-N')  %**
                            XinvSigmaX = XinvSigmaX + kron(store_XX(:,:,kz,ks),invSigma(:,:,kz,ks));  % since A is shared, grow for all data, not just kz,ks
                            temp = invSigma(:,:,kz,ks)*(store_YX(:,:,kz,ks)-mu(:,kz,ks)*store_sumX(:,kz,ks)');
                            XinvSigmay = XinvSigmay + temp(:);  % since A is shared, grow for all data, not just kz,ks
                        elseif strcmp(priorType,'ARD')  %**
                            XinvSigmaX = kron(store_XX(:,:,kz,ks),invSigma(:,:,kz,ks));
                            XinvSigmay = invSigma(:,:,kz,ks)*(store_YX(:,:,kz,ks)-mu(:,kz,ks)*store_sumX(:,kz,ks)');
                            XinvSigmay = XinvSigmay(:);
                            
                            ARDhypers_kzks = ARDhypers(:,kz,ks);
                            if r == 1
                                numObsPerHyper = numRow;
                            else
                                numObsPerHyper = numRow*(numCol/r);
                            end
                            ARDhypers_kzks = ARDhypers_kzks(:,ones(1,numObsPerHyper))';
                            ARDhypers_kzks = ARDhypers_kzks(:);
                            Lambda0_A = diag(ARDhypers_kzks);
                            
                            Sigma_A = inv(Lambda0_A + XinvSigmaX);
                            mu_A = Sigma_A*(theta0_A + XinvSigmay);
                            vecA = mu_A + chol(Sigma_A)'*randn(dim_vecA,1);
                            A(:,:,kz,ks) = reshape(vecA,size(M));
                           
                            AA = sum(A(:,:,kz,ks).*A(:,:,kz,ks),1);
                            if r>1
                                AA = reshape(AA,[numCol/r r]);
                                AA = sum(AA,1);
                            end
                            aa = zeros(1,numHypers);
                            for ii=1:numHypers;
                                aa(ii) = randgamma(a_ARD + numObsPerHyper/2);
                                %aa(ii) = randgamma(a_ARD + numRow/2);
                            end
                            ARDhypers(:,kz,ks) =  aa ./ (b_ARD + AA/2);
                        end
                        
                    else
                        [sqrtSigma sqrtinvSigma] = randiwishart(nu_delta,nu);
                        
                        invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                        
                        if isfield(prior_params,'zeroMean')
                            mu(:,kz,ks) = zeros(dimu,1);
                        else
                            mu(:,kz,ks) = mu0 + cholSigma0'*randn(dimu,1);
                        end
                        
                        if strcmp(priorType,'ARD')
                            ARDhypers_kzks = ARDhypers(:,kz,ks);
                            if r == 1
                                numObsPerHyper = numRow;
                            else
                                numObsPerHyper = numRow*(numCol/r);
                            end
                            ARDhypers_kzks = ARDhypers_kzks(:,ones(1,numObsPerHyper))';
                            ARDhypers_kzks = ARDhypers_kzks(:);
                            
                            cholSigma_A = diag(1./sqrt(ARDhypers_kzks));
                            vecA = cholSigma_A'*randn(dim_vecA,1);
                            A(:,:,kz,ks) = reshape(vecA,size(M));
                            
                            AA = sum(A(:,:,kz,ks).*A(:,:,kz,ks),1);
                            if r>1
                                AA = reshape(AA,[numCol/r r]);
                                AA = sum(AA,1);
                            end
                            aa = zeros(1,numHypers);
                            for ii=1:numHypers;
                                aa(ii) = randgamma(a_ARD + numObsPerHyper/2);
                                %aa(ii) = randgamma(a_ARD + numRow/2);
                            end
                            ARDhypers(:,kz,ks) =  aa ./ (b_ARD + AA/2);
                        end
                        
                    end
                    
                end
            end
            
            % Possible code speedup: if using fixed A matrix one could put the
            % n=1:numIter inside the Kz,Ks loop and precompute
            % A*store_XX*A', etc.
            if strcmp(priorType,'N-IW-N')
                % Code currently assumes shared A if N-IW-N is called,
                % but is written below so as to allow easy change to
                % Gaussian prior on individual A(:,:,kz,ks) as well
                Sigma_A = inv(Lambda0_A + XinvSigmaX);
                mu_A = Sigma_A*(theta0_A + XinvSigmay);
                vecA = mu_A + chol(Sigma_A)'*randn(dim_vecA,1);
                A = repmat(reshape(vecA,size(M)),[1 1 Kz Ks]);
                
                % Reset stored stats:
                XinvSigmaX = zeros(dim_vecA,dim_vecA);
                XinvSigmay = zeros(dim_vecA,1);
            end
            
        end
        
        theta.invSigma = invSigma;
        theta.A = A;
        theta.mu = mu;
        
        if strcmp(priorType,'ARD')
            theta.ARDhypers = ARDhypers;
        end
        
    case 'IW'
        
        invSigma = theta.invSigma;
        
        store_YY = Ustats.YY;
        store_sumY = Ustats.sumY;
        
        dimu = size(nu_delta,1);
        
        for kz=1:Kz
            for ks=1:Ks
                
                if store_card(kz,ks)>0  %**
                    
                    %% Given X, Y get sufficient statistics
                    Syy  = store_YY(:,:,kz,ks);
                    Sygx = (Syy + Syy')/2;
                    
                    % Sample Sigma given s.stats
                    [sqrtSigma sqrtinvSigma] = randiwishart(Sygx + nu_delta,nu+store_card(kz,ks));
                    invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                    
                else
                    [sqrtSigma sqrtinvSigma] = randiwishart(nu_delta,nu);
                    invSigma(:,:,kz,ks) = sqrtinvSigma'*sqrtinvSigma;
                end
            end
        end
        
        theta.invSigma = invSigma;
        
end


return;