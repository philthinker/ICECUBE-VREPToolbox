function [theta Ustats stateCounts data_struct model S] = initializeStructs(F,model,data_struct,settings)

Kz = size(F,2);
Ks = settings.Ks;

prior_params = model.obsModel.params;
if ~isfield(data_struct(1),'blockSize')
    data_struct(1).blockSize = [];
end

switch model.obsModel.type

    case 'Gaussian'

        dimu = size(data_struct(1).obs,1);
        
        for ii=1:length(data_struct)
            if isempty(data_struct(ii).blockSize)
                data_struct(ii).blockSize = ones(1,size(data_struct(ii).obs,2));
            end
            data_struct(ii).blockEnd = cumsum(data_struct(ii).blockSize);
        end

        theta = struct('invSigma',zeros(dimu,dimu,Kz,Ks),'mu',zeros(dimu,Kz,Ks));
        
        Ustats = struct('card',zeros(Kz,Ks),'YY',zeros(dimu,dimu,Kz,Ks),'sumY',zeros(dimu,Kz,Ks));
        
    case 'Multinomial'

        for ii=1:length(data_struct)
            if size(data_struct(ii).obs,1)>1
                error('not multinomial obs')
            end
            if isempty(data_struct(ii).blockSize)
                data_struct(ii).blockSize = ones(1,size(data_struct(ii).obs,2));
            end
            data_struct(ii).blockEnd = cumsum(data_struct(ii).blockSize);
        end
        
        data_struct(1).numVocab = length(prior_params.alpha);
        
        theta = struct('p',zeros(Kz,Ks,data_struct(1).numVocab));

        Ustats = struct('card',zeros(data_struct(1).numVocab,Kz,Ks));

    case {'AR','SLDS'}

        if settings.Ks~=1
            error('Switching linear dynamical models only defined for Gaussian process noise, not MoG')
        end
        
        switch model.obsModel.priorType
            case 'MNIW'
                
                dimu = size(prior_params.M,1);
                dimX = size(prior_params.M,2);
                
                theta = struct('invSigma',zeros(dimu,dimu,Kz,Ks),'A',zeros(dimu,dimX,Kz,Ks));
                
            case {'MNIW-N','N-IW-N'}
                
                dimu = size(prior_params.M,1);
                dimX = size(prior_params.M,2);
                
                theta = struct('invSigma',zeros(dimu,dimu,Kz,Ks),'A',zeros(dimu,dimX,Kz,Ks),'mu',zeros(dimu,Kz,Ks));
                
            case {'ARD'}
                
                dimu = size(prior_params.M,1);
                dimX = size(prior_params.M,2);
                
                theta = struct('invSigma',zeros(dimu,dimu,Kz,Ks),'A',zeros(dimu,dimX,Kz,Ks),'mu',zeros(dimu,Kz,Ks),'ARDhypers',zeros(dimX,Kz,Ks));
                              
            case {'Afixed-IW-N'}
                
                dimu = size(prior_params.A,1);
                dimX = size(prior_params.A,2);
                
                theta = struct('invSigma',zeros(dimu,dimu,Kz,Ks),'A',repmat(prior_params.A,[1 1 Kz Ks]),'mu',zeros(dimu,Kz,Ks));
                
            otherwise
                error('no known prior type')
        end
        
        Ustats = struct('card',zeros(Kz,Ks),'XX',zeros(dimX,dimX,Kz,Ks),'YX',zeros(dimu,dimX,Kz,Ks),'YY',zeros(dimu,dimu,Kz,Ks),'sumY',zeros(dimu,Kz,Ks),'sumX',zeros(dimX,Kz,Ks));
        
        if strcmp(model.obsModel.type,'SLDS')
            
            model.obsModel.r = 1;
            
            if ~isfield(settings,'Kr')
                Kr = 1;
                model.HMMmodel.params.a_eta = 1;
                model.HMMmodel.params.b_eta = 1;
                display('Using single Gaussian measurement noise model')
            else
                Kr = settings.Kr;
                display('Using mixture of Gaussian measurement noise model')
            end
            
            dimy = size(prior_params.C,1);
            
            switch model.obsModel.y_priorType
                
                case 'IW'
                    theta.theta_r = struct('invSigma',zeros(dimy,dimy,Kr));
                case {'NIW','IW-N'}
                    theta.theta_r = struct('invSigma',zeros(dimy,dimy,Kr),'mu',zeros(dimy,Kr));
                otherwise
                    error('no known prior type for measurement noise')
            end
            
            Ustats.Ustats_r = struct('card',zeros(1,Kr),'YY',zeros(dimy,dimy,Kr),'sumY',zeros(dimy,Kr));
            
            hyperparams.eta0 = 0;
            
            stateCounts.Nr = zeros(1,Kr);
                        
        end
        
        for ii=1:length(data_struct)
            if ~isfield(data_struct(ii),'X') || isempty(data_struct(ii).X)
                
                [X,valid] = makeDesignMatrix(data_struct(ii).obs,model.obsModel.r);
                
                data_struct(ii).obs = data_struct(ii).obs(:,find(valid));
                data_struct(ii).X = X(:,find(valid));
                if isempty(data_struct(ii).blockSize)
                    data_struct(ii).blockSize = ones(1,size(data_struct(ii).obs,2));
                end
                data_struct(ii).blockEnd = cumsum(data_struct(ii).blockSize);
                if isfield(data_struct(ii),'true_labels')
                    data_struct(ii).true_labels = data_struct(ii).true_labels(find(valid));
                end
            end
        end
end

numObj = length(data_struct);

stateCounts.N = zeros(Kz+1,Kz,numObj);
stateCounts.Ns = zeros(Kz,Ks,numObj);

hyperparams.gamma0 = 0;
hyperparams.alpha0 = 0;
hyperparams.kappa0 = 0;          
hyperparams.sigma0 = 0;

numSaves = settings.saveEvery/settings.storeEvery;
S(1:numSaves) = struct('F',[],'config_log_likelihood',[],'theta',[],'dist_struct',[],'hyperparams',[],'stateSeq',[]);
