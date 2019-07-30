classdef GMMZero
    %GMMZero Number Zero Gaussian Mixture Model designed by Haopeng Hu.
    %   GMM designed for LfD problem specifically.
    %   You must assign num. of kernels initially. Demonstration data must
    %   be aligned to vectors in cells.
    
    %   Haopeng Hu
    %   2019.07.29
    %   All rights reserved
    
    properties (Access = public)
        nKernel;        % Number of Guassian kernels (states)
        nVar;           % Number/Dimension of variables
        Mu;             % Mean values
        Sigma;          % Variances
        Prior;          % Priors
        dt;             % Minimum time step
    end
    
    properties (Access = protected)
        params_nbMinSteps = 5;          %Minimum number of iterations allowed
        params_nbMaxSteps = 100;        %Maximum number of iterations allowed
        params_maxDiffLL = 1E-4;        %Likelihood increase threshold to stop the algorithm
        params_diagRegFact = 1E-4;      %Regularization term is optional
        params_updateComp = ones(3,1);  %pi,Mu,Sigma
    end
    
    methods (Access = public)
        function obj = GMMZero(nKernel,nVar,dt)
            %GMMZero Assign num. of kernels and Demonstration data
            %   nKernel: Integer,  num. of Gaussian kernels
            %   nVar: Integer, dimension/number of variables
            %   dt: Float, minimum time step (optional)
            obj.nKernel = nKernel;
            obj.nVar = nVar;    % We assume data are vectors
            obj.Mu = zeros(obj.nKernel,obj.nVar);
            obj.Sigma = zeros(obj.nVar,obj.nVar,obj.nKernel);
            obj.Prior = ones(obj.nKernel,1)/obj.nKernel;
            obj.dt = 1e-3;
            if nargin > 2
                obj.dt = dt;
            end
        end
        
        function obj = initGMMKMeans(obj,Demos)
            %initGMM Initialize the GMM before EM
            %   Demos: 1 x D cells, demonstration data.
            diagRegularizationFactor = 1E-2; %Optional regularization term
            Data = obj.dataRegulate(Demos);
            [idList,obj.Mu] = obj.kMeans(Data);  % K-Means clustering
            for i = 1:obj.nKernel
                idtmp = find(idList == i);
                obj.Prior(i) = length(idtmp);
                obj.Sigma(:,:,i) = cov([Data(idtmp,:);Data(idtmp,:)]);
                % Optional regularization term to avoid numerical
                % instability
                obj.Sigma(:,:,i) = obj.Sigma(:,:,i) + eye(obj.nVar)*diagRegularizationFactor;
            end
            obj.Prior = obj.Prior/sum(obj.Prior);
        end
        
        function obj = learnGMM(obj,Demos)
            %learnGMM Learn the GMM by EM
            %   Demos: 1 x D cells, demonstration data.
            Data = obj.dataRegulate(Demos);
            obj = obj.EMGMMZero(Data);
        end
        
        function [eData,eSigma] = GMR(obj,queryData)
            %GMR Gaussian Mixture Regression
            %   queryData: N x 1, the query points (e.g. time)
            %   eData: N x D, the expected data
            %   eSigma: D x D x N, the expected covariances
            [eData, eSigma] = obj.GMRZero(queryData',1,(2:obj.nVar));
            eData = eData'; % For S. Calinon's habit
        end
    end
    
    methods (Access = public)
        % Figures
        h = plotGMM2SC(obj, color, valAlpha);
        h = plotGMM2GMR(obj,data,sigma,color,valAlpha);
    end
    
    methods (Access = protected)
        [Data] = dataRegulate(obj,Demos);
        [idList, Mu] = kMeans(obj,Data);
        [L,GAMMA] = computeGamma(obj,Data);
        [prob] = GaussianPD(obj, Data, Mu, Sigma);
        [obj,GAMMA2,LL] = EMGMMZero(obj,Data);
        [expData, expSigma, H] = GMRZero(model, DataIn, in, out);
    end
end

