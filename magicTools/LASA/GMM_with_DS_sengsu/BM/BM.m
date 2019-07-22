classdef BM
% Binary Merging (BM): Learning a point to point motion from a set of demonstrations.
%
% BM assumes that the demonstrated task can be represented as a first order
% ODE function xi_d=f(xi). The underlying function f is estimated based on
% a set of demonstrations using a probabilistic framework (Mixture of Gaussians).
% The main feature of BM is that it constructs f such that it becomes 
% globally stable (i.e. starting from any point xi_0, it asymptotically
% converges to the target). BM is an iterative procedure. It starts from
% the maximum number of Gaussians, and then merges each pair of randomly
% selected adjacent Gaussian by considering two criteria 1) stability condition
% 2) accuracy condition. If the resultant model fails to satisfy either of
% these conditions, then the algorithm keeps them unmerged and select
% another pair of Gaussians. The algorithm converges within a finite
% iteration steps (in fact it is T(T-1)/2 where T is the number of datapoints
% in the demonstrations).
%
% This files contains the class of BM, and includes the following
% functions and Variables:
%
% Variables ---------------------------------------------------------------
%   o i_in:          Indices corresponding to input data xi
%   o i_out:         Indices corresponding to output data xi_d
%   o K:             Number of Gaussians
%   o nbData:        Number of data in each demonstration
%   o nbVar:         Dimension of [xi;xi_d]
%   o nbSamples:     Number of demonstration
%   o order:         Order of the model. Here, we only consider first and
%                    second order models.
%   o Mu:            A 2D matrix representing center of Gaussians
%   o Sigma:         A 3D matrix representing the covariance of Gaussians
%   o delta:         A vector is used to define the D domain
%   o R:             An scalar weighting the error in estimating the
%                    direction of xi_d [default=0.6] 
%   o Q:             An scalar weighting the error in estimating the
%                    magnitude of xi_d [default= 0.4]   
%   o e_max:         Maximum allowed error in estimation [default= 0.15]
%   o Gap_value:     A scalar defining the cost in having a gap between
%                    datapoints [default= 0.2] 
%   o tol:           Tolerance is used to avoid singularity in Sigma [default=10^-6]
%   o x0:            The avarage of all demonstration's starting point
%   o dx_shift:      Transformation vector to shift the target to the origin
%   o Original_Data: The original datapoints provided by user
%
% Functions ---------------------------------------------------------------
%   o BM:              Constructing an object with the class of BM
%   o Learning:        Learning a point to point motion from demonstrations
%   o Simulation:      Simulating a motion from the given starting point
%   o GMR:             Retrieve xi_d for the given xi
%   o plotModel:       Plot the learned model
%   o plotD:           Plot the 
%   o plotStreamLines: Plot stream lines to represent the motion flow in
%                      the task space
%   o plotPhaseplot:   Plotting the phase plot of the model
%   o plotHyperplanes: Plot hyperplanes and their norms in the task space
%   o Reset:           Reset the model to its original condition.
%
% For more information on a parameter or function type
% doc BM.(parameter or function's name). For example: doc BM.Learning
%
% Copyright(c) 2009 S. Mohammad Khansari Zadeh, LASA Lab, EPFL, Lausanne,
%               Switzerland, http://lasa.epfl.ch/khansari
%
% The program is free for non-commercial academic use. The software must
% not be modified or distributed without prior permission of the authors.


    properties (Hidden)
        b_ind %b_ind is a variable like b_ind that is used to keep the indices of data corresponding to each Gaussian
        i_Omega %This defines the region that each data points belong to
        V %A matrix containing norms of all hyperplanes
        T % A [i_in nbData nbSamples] matrix corresponding to demonstration's time.
        Data % A [nbVar nbData nbSamples] matrix corresponding to demonstration's datapoints.
        IsInitialized=false; %This parameter is used to avoid initialization for datapoints that are already initialized
        iK_model
    end
    
    properties (SetAccess = private)
        i_in %Indices corresponding to input data xi
        i_out %Indices corresponding to output data xi_d
        nbData %Number of data in each demonstration
        nbVar %Dimension of [xi;xi_d]
        nbSamples %Number of demonstration
        order = 1;
    end
    
    properties
        K %Number of Gaussians
        Mu %A 2D matrix representing center of Gaussians
        Sigma %A 3D matrix representing the covariance of Gaussian functions
        %delta is used to define the D domain. For a point xi \in Omega_i
        %it belongs to D domain if p(xi) < delta(i)
        delta
        R = 0.6; %An scalar weighting the error in estimating the direction of xi_d [default= 0.6]
        Q = 0.4; %An scalar weighting the error in estimating the magnitude of xi_d [default= 0.4]
        e_max = 0.15; %Maximum allowed error in estimation [default= 0.15]
        %A scalar defining the cost in having a gap between datapoints.
        %Higher Gap_value, less gap between datapoints. The quality of
        %results strongly depend on this parameter. To have a good model,
        %Gap_value should be defined such that the algorithms would be able
        %to find out relevant corresponding datapoints, while keeping the
        %gap between datapoints as little as possible.[default= 0.2] 
        Gap_value = 0.2; 
        %Tolerance that is used in the covariance matrix to avoid singularity,
        %we use the mean value of xi and xi_dot, to adjust automatically
        %BM.tol [default= 10^-6]
        tol = 10^-6;
        x0 %The avarage of all demonstration's starting point.
        %For simplicity we position the target on the origin through a
        %transformation. dx_shift keeps the value of this transformation.
        dx_shift
        % The original datapoints provided by user. These data are
        % used in BM.Reset function, and also can be further used for
        % evaluation purposes.
        Original_Data=struct('Time',[],'Data',[],'index',[]);
    end
    
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
    methods
%-------------------------------------------------------------------------%        
        function bm = BM(Data,index,varargin)
% bm = BM(Data,index) : Constructs a class of BM using default values
% bm = BM(Data,index, R, Q, e_max, Gap_value) : Constructs a class of BM
%           using the specified values for R, Q, etc. Put [] for variables
%           that you want to use their default values. For example: 
%           bm = BM(Data,index, [], [], e_max) 
%
% Input variables ---------------------------------------------------------
%   o Data:      A 2-D matrix containing all demonstration data points.
%                The first Row of Data corresponds to the time. Other rows
%                contains values for demonstration datapoints in task
%                space (i.e. xi). Time is only used to compute the first
%                derivative of xi. Each column of Data stands for a
%                datapoint. All demonstrations are put next to each other
%                along the second dimension. For example, if we have 3
%                demonstrations D1, D2, and D3, with their time stamps as
%                T1, T2, T3 respectively, then the matrix Data is:
%                      Data = [[T1;D1] [T2;D2] [T3;D3]];
%   o index:     A vector defining the initial index of each demonstration.
%                Following the previous example, index = [i1 i2 i3]
%                indicates 
%                that columns i1:i2-1 belongs to the first demonstration,
%                i2:i3-1 -> 2nd demonstration, and i3:end -> 3rd demonstration. 
%                Obviously, we always have i1=1;
% 	o R:         An scalar weighting the error in estimating the direction
%             	 of xi_d [default=0.6] 
% 	o Q:         An scalar weighting the error in estimating the magnitude
%                of xi_d [default= 0.4] 
% 	o e_max:     Maximum allowed error in estimation. Lower e_max results
%           	 in having a more accurate model, but at the cost of having
%           	 higher number of Gaussian [default= 0.15]. 
% 	o Gap_value: A scalar defining the cost in having a gap between datapoints.
%             	 Higher Gap_value, less gap between datapoints. The quality
%             	 of results strongly depend on this parameter. To have a
%             	 good model, Gap_value should be defined such that the
%             	 algorithms would be able to find out relevant
%             	 corresponding datapoints, while keeping the gap between
%             	 datapoints as little as possible.[default= 0.2] 
%   o order:     Order of the model. Here, we only consider first and second
%                order models.
%
% Output variables is an object with the class BM, with the following
% parameters --------------------------------------------------------------
%   o i_in:          Indices corresponding to input data xi
%   o i_out:         Indices corresponding to output data xi_d
%   o K:             Number of Gaussians
%   o nbData:        Number of data in each demonstration
%   o nbVar:         Dimension of [xi;xi_d]
%   o nbSamples:     Number of demonstration
%   o tol:           Tolerance that is used in the covariance matrix to
%                    avoid singularity, we use the mean value of xi and
%                    xi_dot, to adjust automatically BM.tol 
%   o x0:            The avarage of all demonstration's starting point.
%   o dx_shift:      For simplicity we position the target on the origin
%                    through a transformation. dx_shift keeps the value of
%                    this transformation.
%   o Original_Data: The original datapoints provided by user. These data
%                    are used in BM.Reset function, and also can be
%                    further used for evaluation purposes. 
%   o Data:          A 3-D [nbVar nbData nbSamples] matrix containing all
%                    demonstration data points. BM uses a sample alignment
%                    function to align demonstrations and equalize their
%                    number of datapoints. Demonstrations are put along the
%                    third dimension.

            bm.T=Data(1,:);
            bm.Data=Data(2:end,:);
            bm.Original_Data.Time=bm.T;
            bm.Original_Data.Data=bm.Data;
            bm.Original_Data.index=index;
            for i=1:length(varargin)
                if ~isempty(varargin{i})
                    switch i
                        case 1
                            bm.R=varargin{i};
                        case 2
                            bm.Q=varargin{i};
                        case 3
                            bm.e_max=varargin{i};
                        case 4
                            bm.Gap_value=varargin{i};
                        case 5
                            bm.order=varargin{i};
                    end
                end
            end
            
            if bm.order==1 %first order model
                bm.nbVar=2*size(bm.Data,1); %nbVar = dimension of [xi;xi_d]
            elseif bm.order==2 %second order model
                bm.nbVar=4*size(bm.Data,1); %nbVar = dimension of [xi;xi_d;xi_d;xi_dd]
            end
            bm.i_in=1:bm.nbVar/2; %indices corresponding to input data xi
            bm.i_out=bm.i_in+bm.nbVar/2; %indices corresponding to output data xi_d
            bm.nbSamples=size(index,2); %number of demonstrations
            
            %Tolerance that is used in the covariance matrix to avoid
            %singularity, we use the mean value of xi and xi_dot, to adjust
            %automatically bm.tol.
            if bm.order==1
                bm.tol=min(abs(mean(bm.Mat_Vec_Norm(bm.Data(bm.i_in,:)))))*10^-6;
            else
                bm.tol=min(abs(mean(bm.Mat_Vec_Norm(bm.Data(1:bm.nbVar/4,:)))))*10^-6;
            end
            
            %Equalizing the number of datapoints for each demonstraions,
            %and compute the first derivative.
            bm=bm.preprocess_data(index);
        end % BM
        
%-------------------------------------------------------------------------%        
        function bm = Reset(bm,varargin)
% bm = Reset() : Reload the original data provided by the user. This
%           command is equal to bm = BM(Data, index)
% bm = Reset(R, Q, e_max, Gap_value) : Reload the original data provided by
%           the user, and also change the default value for R, Q, e_max,
%           Gap_value to the one given by the user. Put [] for variables
%           that you want to use their default values. For example: 
%           bm = Reset([], [], e_max). This command is equal to
%           bm = BM(Data, index, R, Q, e_max, Gap_value)  
%
% Optional input variables ------------------------------------------------
% 	o R:         An scalar weighting the error in estimating the direction
%             	 of xi_d [default=0.6] 
% 	o Q:         An scalar weighting the error in estimating the magnitude
%                of xi_d [default= 0.4] 
% 	o e_max:     Maximum allowed error in estimation. Lower e_max results
%           	 in having a more accurate model, but at the cost of having
%           	 higher number of Gaussian [default= 0.15]. 
% 	o Gap_value: A scalar defining the cost in having a gap between datapoints.
%             	 Higher Gap_value, less gap between datapoints. The quality
%             	 of results strongly depend on this parameter. To have a
%             	 good model, Gap_value should be defined such that the
%             	 algorithms would be able to find out relevant
%             	 corresponding datapoints, while keeping the gap between
%             	 datapoints as little as possible.[default= 0.2]
%
% Output variable ---------------------------------------------------------
%   o bm:     An object with the class of BM

            bm.T=bm.Original_Data.Time;
            bm.Data=bm.Original_Data.Data;
            for i=1:length(varargin)
                if ~isempty(varargin{i})
                    switch i
                        case 1
                            bm.R=varargin{i};
                        case 2
                            bm.Q=varargin{i};
                        case 3
                            bm.e_max=varargin{i};
                        case 4
                            bm.Gap_value=varargin{i};
                    end
                end
            end
            
            %Equalizing the number of datapoints for each demonstraions,
            %and compute the first derivative.
            bm=bm.preprocess_data(bm.Original_Data.index);
            bm.IsInitialized=false; %change the status of the model to Uninitilized.
        end % Reset
        
%-------------------------------------------------------------------------%        
        function bm = Learning(bm,varargin)
% bm = Learning() : Learning the motion from demonstrations.
% bm = Learning(R, Q, e_max, Gap_value) : Learning the motion from
%                demonstrations, using the specified value for R, Q, e_max,
%                Gap_value. Put [] for variables that you want to use their
%                default values. For example: bm = Learning(Data,index, [], [],e_max).
%
% Optional input variables ------------------------------------------------
% 	o R:         An scalar weighting the error in estimating the direction
%             	 of xi_d [default=0.6] 
% 	o Q:         An scalar weighting the error in estimating the magnitude
%                of xi_d [default= 0.4] 
% 	o e_max:     Maximum allowed error in estimation. Lower e_max results
%           	 in having a more accurate model, but at the cost of having
%           	 higher number of Gaussian [default= 0.15]. 
% 	o Gap_value: A scalar defining the cost in having a gap between datapoints.
%             	 Higher Gap_value, less gap between datapoints. The quality
%             	 of results strongly depend on this parameter. To have a
%             	 good model, Gap_value should be defined such that the
%             	 algorithms would be able to find out relevant
%             	 corresponding datapoints, while keeping the gap between
%             	 datapoints as little as possible.[default= 0.2]
%
% Output variables is an object with the class BM, with the following
% learned parameters ------------------------------------------------------
% 	o Mu:    A [nbVar K] matrix representing center of Gaussians
% 	o Sigma: A [nbVar nbVar K] matrix representing the covariance of Gaussians
% 	o delta: A [1 K] vector that is used to define the D domain. For a
%            point xi \in Omega_i it belongs to D domain if p(xi) < delta(i)

            for i=1:length(varargin)
                if ~isempty(varargin{i})
                    switch i
                        case 1
                            bm.R=varargin{i};
                        case 2
                            bm.Q=varargin{i};
                        case 3
                            bm.e_max=varargin{i};
                        case 4
                            bm.Gap_value=varargin{i};
                            bm=bm.Reset(); %only change in the gap_value requires to restore the original datapoints
                    end
                end
            end
            bm=bm.Initializing(); %Initializing step
            [bm n_iter]=bm.Iteration(); %Iterating step
            bm=bm.Rem_Singular_Gaussian(); %Removing singular Gaussian. This function
            %       is not a part of BM; however, it improves the obtained performance.
            bm=bm.find_delta(); %computing the value of delta which will be 
            %       used to define D domain
            disp(' ')
            disp('---------------------------------------------------------------------------')
            fprintf('Number of successful iterations: %d\n', n_iter(2))
            fprintf('Total number of iterations: %d\n', n_iter(1))
            disp('---------------------------------------------------------------------------')
            disp(' ')
        end %learning
        
        function bm = Learning_Straight(bm,varargin)
% bm = Learning() : Learning the motion from demonstrations.
% bm = Learning(R, Q, e_max, Gap_value) : Learning the motion from
%                demonstrations, using the specified value for R, Q, e_max,
%                Gap_value. Put [] for variables that you want to use their
%                default values. For example: bm = Learning(Data,index, [], [],e_max).
%
% Optional input variables ------------------------------------------------
% 	o R:         An scalar weighting the error in estimating the direction
%             	 of xi_d [default=0.6] 
% 	o Q:         An scalar weighting the error in estimating the magnitude
%                of xi_d [default= 0.4] 
% 	o e_max:     Maximum allowed error in estimation. Lower e_max results
%           	 in having a more accurate model, but at the cost of having
%           	 higher number of Gaussian [default= 0.15]. 
% 	o Gap_value: A scalar defining the cost in having a gap between datapoints.
%             	 Higher Gap_value, less gap between datapoints. The quality
%             	 of results strongly depend on this parameter. To have a
%             	 good model, Gap_value should be defined such that the
%             	 algorithms would be able to find out relevant
%             	 corresponding datapoints, while keeping the gap between
%             	 datapoints as little as possible.[default= 0.2]
%
% Output variables is an object with the class BM, with the following
% learned parameters ------------------------------------------------------
% 	o Mu:    A [nbVar K] matrix representing center of Gaussians
% 	o Sigma: A [nbVar nbVar K] matrix representing the covariance of Gaussians
% 	o delta: A [1 K] vector that is used to define the D domain. For a
%            point xi \in Omega_i it belongs to D domain if p(xi) < delta(i)

            for i=1:length(varargin)
                if ~isempty(varargin{i})
                    switch i
                        case 1
                            bm.R=varargin{i};
                        case 2
                            bm.Q=varargin{i};
                        case 3
                            bm.e_max=varargin{i};
                        case 4
                            bm.Gap_value=varargin{i};
                            bm=bm.Reset(); %only change in the gap_value requires to restore the original datapoints
                    end
                end
            end
            bm=bm.Initializing(); %Initializing step
            [bm n_iter]=bm.Iteration_Direct();
            bm=bm.Rem_Singular_Gaussian(); %Removing singular Gaussian. This function
            %       is not a part of BM; however, it improves the obtained performance.
            bm=bm.find_delta(); %computing the value of delta which will be 
            %       used to define D domain
            disp(' ')
            disp('---------------------------------------------------------------------------')
            fprintf('Number of successful iterations: %d\n', n_iter(2))
            fprintf('Total number of iterations: %d\n', n_iter(1))
            disp('---------------------------------------------------------------------------')
            disp(' ')
        end %learning_straight
        
%-------------------------------------------------------------------------%        
        function bm = MergeModel(bm,bm_m)
            % bm = MergeModel(bm,bm_1,bm_2) : Learning the motion from demonstrations.
            bm.K=bm.K+bm_m.K;
            bm.delta=[bm.delta bm_m.delta];
            bm.V=[bm.V bm_m.V];
            bm.Mu=[bm.Mu bm_m.Mu];
            bm.Sigma=zeros(4,4,bm.K);
            bm.Sigma(:,:,1:bm.K)=bm.Sigma;
            bm.Sigma(:,:,bm.K+1:bm.K)=bm_m.Sigma;
            bm.iK_model=[bm.iK_model bm.K];
            
        end %MergeModel
        
%-------------------------------------------------------------------------%        
        function err=Total_Error(bm)
% e = Total_Error() : computing the root mean square error in estimating
%           xi_d in each subdomain Omega_i.
%
% Output variable ---------------------------------------------------------
% 	o e:    A [1 K] vector representing rms error in all subdomains of D

            err=zeros(1,bm.K);
            for i=1:bm.K
                d = reshape(bm.Data(:,bm.b_ind(i):bm.b_ind(i+1)-1,:),bm.nbVar,[]);
                xd=bm.GMR(d(bm.i_in,:),'training');
                err(i)=bm.estimation_error(d(bm.i_out,:),xd);
            end
        end %Total_Error
        
%-------------------------------------------------------------------------%        
        function plotModel(bm)
% plotModel() : Plots mean and covariance of the learned model's Gaussian
%           function. In the resultant graph, means are represented as a
%           cross, and covariances are shown in ellipses.

            n_in=bm.nbVar/2;
            if n_in==2
                figure('name','GMM/GMR','position',[653 45 560 420])
                bm.plotGMM([1 2], [0 .8 0], 1);
                plot(bm.Data(1,:),bm.Data(2,:),'r.')
                xlabel('$\xi_1$','interpreter','latex','fontsize',15);
                ylabel('$\xi_2$','interpreter','latex','fontsize',15);
                grid on;box on
                axis tight;ax=get(gca);
                axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                    ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                axis equal;
                
            elseif n_in==3
                figure('name','GMM/GMR','position',[77 45 560 420])
                bm.plotGMM([1 2], [0 .8 0], 1);
                plot(bm.Data(1,:),bm.Data(2,:),'r.')
                xlabel('$\xi_1$','interpreter','latex','fontsize',15);
                ylabel('$\xi_2$','interpreter','latex','fontsize',15);
                grid on;box on
                axis tight;ax=get(gca);
                axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                    ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                axis equal
                
                figure('name','GMM/GMR','position',[653 45 560 420])
                bm.plotGMM([1 3], [0 .8 0], 1);
                plot(bm.Data(1,:),bm.Data(3,:),'r.')
                xlabel('$\xi_1$','interpreter','latex','fontsize',15);
                ylabel('$\xi_3$','interpreter','latex','fontsize',15);
                grid on;box on
                axis tight;ax=get(gca);
                axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                    ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                axis equal
                
            elseif n_in>3
                figure('name','GMM/GMR','position',[1070  146  513  807])
                for i=2:n_in
                    subplot(n_in-1,1,i-1)
                    hold on
                    bm.plotGMM([1 i], [0 .8 0], 1);
                    plot(bm.Data(1,:),bm.Data(i,:),'r.')
                    axis tight;
                    ylabel(['$\xi_' num2str(i) '$'],'interpreter','latex','fontsize',15);
                    grid on;box on
                    axis tight;ax=get(gca);
                    axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                        ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                    if i==n_in
                        xlabel(['$\xi_' num2str(1) '$'],'interpreter','latex','fontsize',15);
                    end
                    axis equal
                end
            end
        end %plotModel
        
%-------------------------------------------------------------------------%        
        function plotPhaseplot(bm)
% plotPhaseplot() : Representing the phase plots of the learned model's
%           Gaussian function. In the resultant graph, means are
%           represented as a cross, and covariances are shown in ellipses.
            
            figure('name','GMM/GMR','position',[1070  146  513  807])
            for i=bm.i_in
                subplot(bm.nbVar/2,1,i)
                hold on
                bm.plotGMM([i i+bm.nbVar/2], [0 .8 0], 1);
                plot(bm.Data(i,:),bm.Data(i+bm.nbVar/2,:),'r.')
                axis tight;
                xlabel(['$\xi_' num2str(i) '$'],'interpreter','latex','fontsize',15);
                ylabel(['$\xi_' num2str(i+bm.nbVar/2) '$'],'interpreter','latex','fontsize',15);
                grid on;box on
                axis tight;ax=get(gca);
                axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                    ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                axis equal
            end

        end %plotPhaseplot
        
%-------------------------------------------------------------------------%        
        function [xd, domain, Sigma_xd] = GMR(bm,x,varargin)
% [xd, domain, Sigma_xd] = GMR(x) : This function performs Gaussian Mixture
%           Regression (GMR), using the parameters of a Gaussian Mixture
%           Model (GMM). Given partial input data, the algorithm computes
%           the expected distribution for the resulting dimensions. By
%           providing temporal values as inputs, it thus outputs a smooth
%           generalized version of the data encoded in GMM, and associated
%           constraints expressed by covariance matrices. 
%
% Input variable ----------------------------------------------------------
%   o x:       [nbVar/2 N] matrix representing N datapoints of nbVar/2 dimensions.
% Output variables --------------------------------------------------------
%   o y:       [nbVar/2 N] matrix representing the retrieved N datapoints of
%              nbVar/2 dimensions, i.e. expected means.
%   o domain:  [K 1] vector representing The region that the output is computed there.
%   o Sigma_y: [nbVar/2 nbVar/2 N] matrix representing the N expected covariance
%              matrices retrieved.

            nb_x = size(x,2);
            if isempty(varargin)
                job='reproduction';
            else
                job=varargin{1};
            end
            
            [domain ind_neigh Pxi] = bm.find_domain(x); %finding the domain that each detapoint belongs
            beta = Pxi./repmat(sum(Pxi,2)+realmin,1,2);
            
            if strcmpi('reproduction',job) && bm.order==1
                %Here the points outside the D domain are found. Then their
                %domain is changed into 1 (i.e. the stable Gaussian, which
                %its center is on the target, will be used to compute their
                %outputs).
                i=find(sum(Pxi,2)'<bm.delta(ind_neigh(:,1))  & (sum(Pxi,2)'~=0 | ind_neigh(:,1)'==1)); %the second condition just solve a tiny bug for very close to singular covariance function
                if ~isempty(i)
                    ind_neigh(i,:)=repmat([1 1],length(i),1);
                    beta(i,:)=repmat([0.5 0.5],length(i),1);
                    domain(i)=1;
                end
            end
            
            
            if nb_x < bm.K %faster for the lower number of datapoints
                for i=1:nb_x
                    % Compute expected means xd, given input x
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    for j=1:2
                        xd_tmp(:,j) = bm.Mu(bm.i_out,ind_neigh(i,j)) + ...
                            bm.Sigma(bm.i_out,bm.i_in,ind_neigh(i,j))...
                            /bm.Sigma(bm.i_in,bm.i_in,ind_neigh(i,j)) * ...
                            (x(:,i)-bm.Mu(bm.i_in,ind_neigh(i,j)));
                    end
                    xd(:,i) = sum(repmat(beta(i,:),[bm.nbVar/2 1]) .* xd_tmp,2);
                    % Compute expected covariance matrices Sigma_xd, given input x
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    if nargout==3
                        Sigma_xd_tmp=[];
                        for j=1:2
                            Sigma_xd_tmp(:,:,j) = bm.Sigma(bm.i_out,bm.i_out,ind_neigh(i,j)) -...
                                (bm.Sigma(bm.i_out,bm.i_in,ind_neigh(i,j))/...
                                 bm.Sigma(bm.i_in,bm.i_in,ind_neigh(i,j)))*...
                                 bm.Sigma(bm.i_in,bm.i_out,ind_neigh(i,j));
                        end
                        Sigma_xd(:,:,i) = sum(repmat(reshape(beta(i,:).^2,[1 1 2]), [bm.nbVar/2 bm.nbVar/2 1]) .* Sigma_xd_tmp,3);
                    end
                end
            else %faster for higher number of datapoints
                for i=1:bm.K
                    Slope=bm.Sigma(bm.i_out,bm.i_in,i)...
                            /bm.Sigma(bm.i_in,bm.i_in,i);
                    ind=find(ind_neigh==i);
                    ind_x=ind;
                    ind_x(ind>nb_x)=ind_x(ind>nb_x)-nb_x;
                    xd_tmp(:,ind) = repmat(bm.Mu(bm.i_out,i),1,length(ind)) + ...
                        Slope * (x(:,ind_x)-repmat(bm.Mu(bm.i_in,i),1,length(ind)));    
                    if nargout==3
                        Sigma_xd_tmp(:,:,ind) = repmat(bm.Sigma(bm.i_out,bm.i_out,i) -...
                            Slope * bm.Sigma(bm.i_in,bm.i_out,i),[1 1 length(ind)]);
                    end
                end
                xd_tmp2 = repmat(reshape(beta,[1 size(beta)]),[bm.nbVar/2 1 1]) .* reshape(xd_tmp,[size(xd_tmp,1) nb_x 2]);
                xd = sum(xd_tmp2,3);
                if nargout==3
                    beta_tmp = reshape(beta,[1 1 size(beta)]);
                    Sigma_xd_tmp2 = repmat(beta_tmp.*beta_tmp, [bm.nbVar/2 bm.nbVar/2 1 1]) ...
                                   .* reshape(Sigma_xd_tmp,bm.nbVar/2,bm.nbVar/2,nb_x,2);
                    Sigma_xd = sum(Sigma_xd_tmp2,4);
                end
            end
            
            %Solving a tiny bug for very close to singular covariance function
            ind=find(all(Pxi'==0) & all(x~=0) & ind_neigh(:,2)'~=1);
            if ~isempty(ind)
                xd(:,ind)=bm.Mu(bm.i_out,ind_neigh(ind,2));
            end
            
            if strcmpi('reproduction',job) && bm.order==1 && norm(xd(:,end))==0 && domain(:,end)~=1
                i=nb_x;
                % Compute expected means xd, given input x
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                for j=1:2
                    xd_tmp(:,j) = bm.Mu(bm.i_out,ind_neigh(i,j)) + ...
                        bm.Sigma(bm.i_out,bm.i_in,ind_neigh(i,j))...
                        /bm.Sigma(bm.i_in,bm.i_in,ind_neigh(i,j)) * ...
                        (x(:,i)-bm.Mu(bm.i_in,ind_neigh(i,j)));
                end
                xd(:,i) = sum(repmat(beta(i,:),[bm.nbVar/2 1]) .* xd_tmp,2);
                % Compute expected covariance matrices Sigma_xd, given input x
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if nargout==3
                    Sigma_xd_tmp=[];
                    for j=1:2
                        Sigma_xd_tmp(:,:,j) = bm.Sigma(bm.i_out,bm.i_out,ind_neigh(i,j)) -...
                            (bm.Sigma(bm.i_out,bm.i_in,ind_neigh(i,j))/...
                            bm.Sigma(bm.i_in,bm.i_in,ind_neigh(i,j)))*...
                            bm.Sigma(bm.i_in,bm.i_out,ind_neigh(i,j));
                    end
                    Sigma_xd(:,:,i) = sum(repmat(reshape(beta(i,:).^2,[1 1 2]), [bm.nbVar/2 bm.nbVar/2 1]) .* Sigma_xd_tmp,3);
                end
            end
        end %GMR
        
%-------------------------------------------------------------------------%        
        function plotD(bm,varargin)
% plotD() : Plots D domain of the learned model.
% plotD(quality) : Plots D domain of the learned model with the speicifed
%           quality. The possible values for quality are:
%           quality = {'low', 'medium', or 'high'} [default='low']
%This function can be used for models with any dimension, however, it
%may become very slow in higher dimensions. The model best works for 2D and
%3D models.
%
% Optional input variables ------------------------------------------------
%   o quality:     An string defining the quality of the plot. It can have
%                  one of the following value: 'low', 'medium', or 'high'
%                  [default='low']
            
            n_in=bm.nbVar/2;
            if isempty(varargin)
                quality='low';
            else
                quality=varargin{1};
            end
            
            %defining the number of points per dimension
            if strcmpi(quality,'high')
                n=(n_in-1)*round((600*600)^(1/n_in));
            elseif strcmpi(quality,'medium')
                n=(n_in-1)*round((400*400)^(1/n_in));
            else
                n=(n_in-1)*round((30*30)^(1/n_in));
            end
            
            %finding the range of axis
            D=min(bm.Data(bm.i_in,:),[],2);
            D(:,2)=max(bm.Data(bm.i_in,:),[],2);
            for i=bm.i_in
                D(i,:)=[D(i,1)-(D(i,2)-D(i,1))/8 D(i,2)+(D(i,2)-D(i,1))/8];
            end

            %computing the mesh points along each axis
            for i=bm.i_in
                ax(i,:)=linspace(D(i,1),D(i,2),n);
            end
            
            %meshing the input domain
            if n_in==2
                [x1 x2]=meshgrid(ax(1,:),ax(2,:));
                if n_in==2
                    x=[x1(:) x2(:)]';
                end
            else
                str1='[';str2='ndgrid(';
                for i=bm.i_in
                    str1=[str1 'x' num2str(i) ' '];
                    str2=[str2 'ax(' num2str(i) ',:),'];
                end
                str1(end)=[];str2(end)=[];
                str1=[str1 ']'];
                str2=[str2 ');'];
                eval([str1 '=' str2])
                for i=bm.i_in
                    x(i,:)=eval(['x' num2str(i) '(:)'])';
                end
            end
            
            
            z=zeros(1,n^n_in);
            [domain ind_neigh Pxi] = bm.find_domain(x); %finding the domain that each detapoint belongs
            
            %correcting neighbor indices for regions that can only have one
            %Gaussian (for first and last Gaussian)
            Pxi(ind_neigh(:,1)==ind_neigh(:,2),2)=0;
            
            z(sum(Pxi,2)'>=bm.delta(ind_neigh(:,1)))=1; %finding points that belong to D
            z=reshape(z,n*ones(1,n_in));
            
            %plotting domains
            fig=figure('name','Stability Domain','position',[74   553   560   420]);
            for i=2:n_in
                subplot(n_in-1,1,i-1)
                hold on
                z_plot=z;
                if n_in > 2
                    ind=bm.i_in;
                    if i==2
                        ind([1 2])=[];
                    else
                        ind([2 i])=[];
                    end
                    for j=length(ind):-1:1
                        z_plot=squeeze(sum(z_plot,ind(j)));
                    end
                    z_plot(z_plot>1)=1;
                    if i~=2
                        z_plot=z_plot';
                    end
                end
                
                [C h]=contourf(ax(1,:),ax(i,:),z_plot,[1 1]);
                delete(h);figure(fig)
                counter=1;
                while counter<size(C,2)
                    i_s=counter+1;
                    i_e=(C(2,counter)+i_s-1);
                    for j=1:2
                        C_interp(j,:)=smooth(C(j,i_s:i_e),15);
                    end
                    patch(C_interp(1,:),C_interp(2,:), [0.6 1 0.6], 'LineStyle', 'none')
                    clear C_interp
                    counter=i_e+1;
                end
                bm.plotGMM([1 i], [0 .8 0], 1);
                ylabel(['$\xi_' num2str(i) '$'],'interpreter','latex','fontsize',15);
                axis equal;box on
                if i==n_in
                    xlabel('$\xi_1$','interpreter','latex','fontsize',15);
                end
            end
        end %plotD
        
%-------------------------------------------------------------------------%        
        function plotStreamLines(bm,varargin)
% plotStreamLines() : Plots stream lines of the learned model.
% plotStreamLines(quality) : Plots stream lines of the learned model with
%           the speicifed quality. The possible values for quality are:
%           quality = {'low', 'medium', or 'high'} [default='low']
% plotStreamLines(quality,true) : Plots stream lines of the learned model
%           with the speicifed quality, and plotting D domain behind it.
%           Put [], if you don't want to specify quality [default=false]
%This function can be only used for 2D models.
%
% Optional input variable -------------------------------------------------
%   o quality: An string defining the quality of the plot. It can have one
%              of the following value: 'low', 'medium', or 'high' [default='low']
%   o b_plotD: A boolean that control plotting of D domain. It plots the
%              model's D domain when b_plotD=true [default=false]
            
            if bm.nbVar/2~=2
                disp('This function can only be used for 2D models!')
                return
            end
            dim=[1 2];
            
            quality='low';b_plotD=false;
            for i=1:length(varargin)
                if ~isempty(varargin{i})
                    switch i
                        case 1
                            quality=varargin{1};
                        case 2
                            b_plotD=varargin{2};
                    end
                end
            end
            
            if strcmpi(quality,'high')
                nx=600;
                ny=600;
            elseif strcmpi(quality,'medium')
                nx=400;
                ny=400;
            else
                nx=200;
                ny=200;
            end
            
            if b_plotD
                bm.plotD(quality);
            else
                %finding the range of axis
                figure('position',[74   553   560   420]);hold on;
                bm.plotGMM(dim, [0 .8 0], 1);
            end
            axis tight
            ax=get(gca);
            axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/8 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/8 ...
                ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/8 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/8]);
            axis equal
            ax=get(gca);
            ax.YLim(2)=300;
            ax_x=linspace(ax.XLim(1),ax.XLim(2),nx); %computing the mesh points along each axis
            ax_y=linspace(ax.YLim(1),ax.YLim(2),ny); %computing the mesh points along each axis
            [x_tmp y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
            x=[x_tmp(:) y_tmp(:)]';
            z=zeros(1,nx*ny);
            
            xd = bm.GMR(x); %compute outputs
            streamslice(x_tmp,y_tmp,reshape(xd(1,:),ny,nx),reshape(xd(2,:),ny,nx),1,'method','cubic')
            axis([ax.XLim ax.YLim]);box on
        end %plotStreamLines
        
%-------------------------------------------------------------------------%        
        function plotGMM(bm, dim, color, display_mode)
% plotGMM(dim, color, display_mode) : This function plots a representation
%           of the components (means and covariance amtrices) of a Gaussian
%           Mixture Model (GMM) or a Gaussian Mixture Regression (GMR).
%
% Inputs variables --------------------------------------------------------
%   o dim:          A 1 x 2 vector indicating the plotting dimension.
%   o color:        Color used for the representation
%   o display_mode: Display mode (1 is used for a GMM, 2 is used for a GMR
%                   with a 2D representation.
%
%  Copyright (c) 2006 Sylvain Calinon, LASA Lab, EPFL, CH-1015 Lausanne,
%                    Switzerland, http://lasa.epfl.ch
            
            lightcolor = color + [0.6,0.6,0.6];
            lightcolor(lightcolor>1.0) = 1.0;
            %lightcolor=[1 0.9 0.9];
            if display_mode==1
                nbDrawingSeg = 40;
                t = linspace(-pi, pi, nbDrawingSeg)';
                for j=1:bm.K
                    stdev = sqrtm(3.0.*bm.Sigma(dim,dim,j));
                    X = [cos(t) sin(t)] * real(stdev) + repmat(bm.Mu(dim,j)',nbDrawingSeg,1);
                    patch(X(:,1), X(:,2), lightcolor, 'lineWidth', 1.5, 'EdgeColor', color); %linewidth=2
                    hold on;
                    plot(X(:,1), X(:,2),'k')
                    plot(bm.Mu(dim(1),:), bm.Mu(dim(2),:), 'kx', 'lineWidth', 2);
                end
            elseif display_mode==2
                nbDrawingSeg = 40;
                lightcolor=[0.7 0.7 0];
                t = linspace(-pi, pi, nbDrawingSeg)';
                for j=1:bm.K
                    stdev = sqrtm(3.0.*bm.Sigma(dim,dim,j)); %1.0->3.0
                    X = [cos(t) sin(t)] * real(stdev) + repmat(bm.Mu(dim,j)',nbDrawingSeg,1);
                    patch(X(:,1), X(:,2), lightcolor, 'LineStyle', 'none');
                end
                hold on
                plot(bm.Mu(dim(1),:), bm.Mu(dim(2),:), '-', 'lineWidth', 3, 'color', color);
            end
        end %plotGMM
        
%-------------------------------------------------------------------------%        
        function plotHyperplanes(bm,varargin)
% plotHyperplanes() : Plots hyperplanes of the learned model.
% plotD(quality) : Plots hyperplanes of the learned model with the speicifed
%           quality. The possible values for quality are:
%           quality = {'low', 'medium', or 'high'} [default='low']
%
%This function can be used for 2-D models. plotHyperplanes load plotD
%function and superimposes the hyperplanes on the plotted D domain.
%
% Optional input variable -------------------------------------------------
%   o quality:     An string defining the quality of the plot. It can have
%                  one of the following value: 'low', 'medium', or 'high'
%                  [default='low']

            if bm.nbVar/2~=2
                disp('This function can only be used for 2D models!')
                return
            end
            
            if isempty(varargin)
                bm.plotD();
            else
                bm.plotD(varargin{1});
            end
            
            quiver(bm.Mu(1,:),bm.Mu(2,:),bm.V(1,:),bm.V(2,:))
            nx=100;
            for i=1:bm.K
                % finding one of the end points of the hyperplane
                m=[-bm.V(2,i);bm.V(1,i)];
                range=[0 nx];u_old=0;u_new=-1;j=1;
                while norm(u_old-u_new)>bm.tol && j<100
                    u_old=range(1);
                    u=linspace(range(1),range(2),nx);
                    x=m*u+repmat(bm.Mu(bm.i_in,i),1,nx);
                    [domain ind_neigh Pxi] = bm.find_domain(x);
                    Pxi(ind_neigh(:,1)==ind_neigh(:,2),2)=0;
                    ind=find(domain==i & (sum(Pxi,2)>=bm.delta(ind_neigh(:,1))'), 1, 'last' );
                    u_new=u(ind);
                    if ind==nx
                        range=[range(2)+1 range(2)^2];
                    elseif isempty(ind) || ind==1
                        range(2)=u(2);
                        u_new=u(2);
                    else
                        range=u(ind:ind+1);
                    end
                    j=j+1;
                end
                xh=m*u(ind)+bm.Mu(bm.i_in,i);
                
                % finding the other end point of the hyperplane
                range=[0 nx];u_old=0;u_new=-1;j=1;
                while norm(u_old-u_new)>bm.tol && j<100
                    u_old=range(1);
                    u=linspace(range(1),range(2),nx);
                    x=-m*u+repmat(bm.Mu(bm.i_in,i),1,nx);
                    [domain ind_neigh Pxi] = bm.find_domain(x);
                    Pxi(ind_neigh(:,1)==ind_neigh(:,2),2)=0;
                    ind=find(domain==i & sum(Pxi,2)>=bm.delta(ind_neigh(:,1))', 1, 'last' );
                    u_new=u(ind);
                    if ind==nx
                        range=[range(2)+1 range(2)^2];
                    elseif isempty(ind) || ind==1
                        range(2)=u(2);
                        u_new=range(2);
                    else
                        range=u(ind:ind+1);
                    end
                    j=j+1;
                end
                xh=[xh -m*u(ind)+bm.Mu(bm.i_in,i)];
                plot(xh(1,:),xh(2,:),'k','linewidth',2)
            end
            axis auto
        end %plotHyperplanes
        
%-------------------------------------------------------------------------%        
        function [x xd t]=Simulation(bm,x0,varargin)
% [x xd t]=Simulation(x0) : Simulates the motion starting from the point x0
% [x xd t]=Simulation(x, b_graph, dT, i_max, tol) : Simulates the motion
%           starting from the point x0, and with the specified options. Put
%           [] for the parameters that you want to use their default values.
%           For example:
%                  Simulation(x0,[],[],100);
% In this example only the value of i_max is changed, and the rest remains
% as the default values.            
%
% This function simulate tasks that were learnt by GMM with the dynamics
% specified as: xd=f(x)
%
% Inputs variables --------------------------------------------------------
%   o x:       [nbVar/2 1] Column vector representing the starting point.
%   o b_graph: It is a boolean variable, and it shows the graphical
%              representation of the trajectory if its value is true [default=true]
%   o dT:      Time interval that should be used for the integration [default=0.02]
%   o i_max:   The maximum step of the trajectory. If the trajectory
%              doesn't reach to the target within less than i_max, then the
%              simulation will be stopped [default=1000]
%   o tol:     An scalar value that defines the minimum distance between
%              two iterative step. If the distance between two different
%              steps become less than tol in each direction, then the
%              condition is satisfied and the simulation will be stopped
%              [default=0.001].
% Output variables --------------------------------------------------------
%   o x:       [nbVar/2 N] matrix containing the position of points in the
%              simulated trajectory.
%   o xd:      [nbVar/2 N] matrix containing the velocity of points in the
%              simulated trajectory.
%   o t:       [1 N] vector containing the time values of points in the
%              simulated trajectory.
            
            % Defining inputs
            b_graph=true;dT=0.02;i_max=1000;tol=10^-3;
            for i=1:length(varargin)
                if ~isempty(varargin{i})
                    switch i
                        case 1
                            b_graph=varargin{i};
                        case 2
                            dT=varargin{i};
                        case 3
                            i_max=varargin{i};
                        case 4
                            tol=varargin{i};
                    end
                end
            end
            
            nbSPoint=size(x0,2); %number of starting points
           
            %initialization
            for i=1:nbSPoint
                x(:,1,i)=x0(:,i);
            end
            xd=zeros(size(x)); %Initial value of the velocity (i.e. at starting point)
            t=0; %starting time
            x_f=zeros(size(bm.i_in))'; %defining the target position (which is by default at the origin)
            
            %initializing figure
            if b_graph && bm.nbVar/2==2
                figure('name','2D Simulation of the task','position',[653 550 560 420])
                bm.plotGMM([1 2], [0 .8 0], 1);
                axis tight;ax=get(gca);
                axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                    ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                axis equal;grid on;box on;hold on
                plot(0,0,'k*','EraseMode','none','markersize',10,'linewidth',1.5);
                for j=1:nbSPoint
                    plot(x(1,1,j),x(2,1,j),'ok','markersize',2,'linewidth',7.5)
                    p2(j)= plot(x(1,1,j),x(2,1,j),'EraseMode','none');
                end
                xlabel('$\xi_1$','interpreter','latex','fontsize',15);
                ylabel('$\xi_2$','interpreter','latex','fontsize',15);
            elseif b_graph && bm.nbVar/2==3
                figure('name','3D Simulation of the task','position',[653 550 560 420])
                plot3(0,0,0,'k*','EraseMode','none','markersize',10,'linewidth',1.5);
                hold on;grid on
                for j=1:nbSPoint
                    plot3(x(1,1,j),x(2,1,j),x(3,1,j),'ok','markersize',2,'linewidth',7.5)
                    p2(j)= plot3(x(1,1,j),x(2,1,j),x(3,1,j),'EraseMode','none');
                end
                xlabel('$\xi_1$','interpreter','latex','fontsize',15);
                ylabel('$\xi_2$','interpreter','latex','fontsize',15);
                zlabel('$\xi_3$','interpreter','latex','fontsize',15);
            elseif b_graph
                figure('name','Joint Angle Simulation of the task','position',[542   146   513   807])
                for i=2:bm.nbVar/2
                    subplot(bm.nbVar/2-1,1,i-1)
                    bm.plotGMM([1 i], [0 .8 0], 1);
                    axis tight;ax=get(gca);
                    axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                        ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                    axis equal;grid on;box on;hold on
                    plot(0,0,'k*','EraseMode','none','markersize',10,'linewidth',1.5);
                    for j=1:nbSPoint
                        plot(x(1,1,j),x(i,1,j),'ok','markersize',2,'linewidth',7.5)
                        p2(i,j)= plot(x(1,1,j),x(i,1,j),'EraseMode','none');
                    end
                    ylabel(['$\xi_' num2str(i) '$'],'interpreter','latex','fontsize',15);
                    if i==bm.nbVar/2
                        xlabel(['$\xi_' num2str(1) '$'],'interpreter','latex','fontsize',15);
                    end
                    %axis([min([x(1,1)-.1 0]) max([x(1,1)+.1 0]) min([x(i,1)-.1 0]) max([x(i,1)+.1 0])])
                end
            end
            
            i=1;
            check=true;
            while check==true
                %Finding xd using GMR.
                xd(:,i,:)=reshape(bm.GMR(squeeze(x(:,i,:))),[bm.nbVar/2 1 nbSPoint]);
                x(:,i+1,:)=x(:,i,:)+xd(:,i,:)*dT;
                t(i+1)=t(i)+dT;
                
                if b_graph && bm.nbVar/2==2
                    for j=1:nbSPoint
                        set(p2(j),'XData',x(1,1:i+1,j),'YData',x(2,1:i+1,j))
                    end
                    ax=get(gca);
                    if (x(1,i+1)>ax.XLim(2) || x(1,i+1)<ax.XLim(1)) || (x(2,i+1)>ax.YLim(2) || x(2,i+1)<ax.YLim(1))
                        axis tight
                        ax=get(gca);
                        axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                            ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                    end
                    drawnow
                elseif b_graph && bm.nbVar/2==3
                    for j=1:nbSPoint
                        set(p2(j),'XData',x(1,1:i+1,j),'YData',x(2,1:i+1,j),'ZData',x(3,1:i+1,j))
                    end
                    ax=get(gca);
                    if (x(1,i+1)>ax.XLim(2) || x(1,i+1)<ax.XLim(1)) || (x(2,i+1)>ax.YLim(2) || x(2,i+1)<ax.YLim(1)) || (x(3,i+1)>ax.YLim(2) || x(3,i+1)<ax.YLim(1))
                        axis tight
                        ax=get(gca);
                        axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                            ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10 ...
                            ax.ZLim(1)-(ax.ZLim(2)-ax.ZLim(1))/10 ax.ZLim(2)+(ax.ZLim(2)-ax.ZLim(1))/10]);
                    end
                    drawnow
                elseif b_graph
                    for j=2:bm.nbVar/2
                        subplot(bm.nbVar/2-1,1,j-1)
                        for k=1:nbSPoint
                            set(p2(j,k),'XData',x(1,1:i+1,k),'YData',x(j,1:i+1,k))
                        end
                        ax=get(gca);
                        if (x(1,i+1)>ax.XLim(2) || x(1,i+1)<ax.XLim(1)) || (x(j,i+1)>ax.YLim(2) || x(j,i+1)<ax.YLim(1))
                            axis tight
                            ax=get(gca);
                            axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                                ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                        end
                        drawnow
                    end
                end
                try
                    if (all(all(all(abs(xd(:,end-3:end,:))<tol))) && all(bm.Mat_Vec_Norm(x(:,i,:))<100)) || i>i_max-2 %Checking if the conditions are satisfied.
                        i=i+1;
                        xd(:,i,:)=reshape(bm.GMR(squeeze(x(:,i,:))),[bm.nbVar/2 1 nbSPoint]);
                        disp(['Number of Iterations: ' sprintf('%1.0f',i)])
                        tmp='';
                        for j=1:bm.nbVar/2
                            tmp=[tmp ' %1.4f ;'];
                        end
                        tmp=tmp(2:end-2);
                        fprintf('Final Time: %1.2f (sec)\n',t(1,end,1))
                        fprintf(['Final Point: [' tmp ']\n'],squeeze(x(:,end,:)))
                        fprintf(['Target Position: [' tmp ']\n'],zeros(1,bm.nbVar/2))
                        disp('## #####################################################')
                        disp(' ')
                        disp(' ')
                        disp(' ')
                        break
                    end
                end
                i=i+1;
            end

            if b_graph && bm.nbVar/2==2
                ax=get(gca);
                if (any(x(1,:)>ax.XLim(2)) || any(x(1,:)<ax.XLim(1))) || (any(x(2,:)>ax.YLim(2)) || any(x(2,:)<ax.YLim(1)))
                    axis tight
                    ax=get(gca);
                    axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                        ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                end
            elseif b_graph && bm.nbVar/2==3
                ax=get(gca);
                if (any(x(1,:)>ax.XLim(2)) || any(x(1,:)<ax.XLim(1))) || (any(x(2,:)>ax.YLim(2)) || any(x(2,:)<ax.YLim(1))) || (any(x(3,:)>ax.ZLim(2)) || any(x(3,:)<ax.ZLim(1)))
                    axis tight
                    ax=get(gca);
                    axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                        ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10 ...
                        ax.ZLim(1)-(ax.ZLim(2)-ax.ZLim(1))/10 ax.ZLim(2)+(ax.ZLim(2)-ax.ZLim(1))/10]);
                    
                end
            elseif b_graph
                for j=2:bm.nbVar/2
                    subplot(bm.nbVar/2-1,1,j-1)
                    ax=get(gca);
                    if (x(1,end)>ax.XLim(2) || x(1,end)<ax.XLim(1)) || (x(j,end)>ax.YLim(2) || x(j,end)<ax.YLim(1))
                        axis tight
                        ax=get(gca);
                        axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                            ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                    end
                end
            end
        end %Simulation
        
        %-------------------------------------------------------------------------%        
        function [x xd t]=InteractiveSimulation(bm,varargin)
            for i=2:bm.nbVar/2
                if bm.nbVar/2==2
                    bm.plotD();
                else
                    figure('name','Intractive Simulation')
                    bm.plotGMM([1 i],[0 0.8 0],1);
                    plot(bm.Data(1,:),bm.Data(i,:),'r.')
                end
                set(gcf,'name','Intractive Simulation')
                axis tight;
                xlabel(['$\xi_' num2str(1) '$'],'interpreter','latex','fontsize',15);
                ylabel(['$\xi_' num2str(i) '$'],'interpreter','latex','fontsize',15);
                grid on;box on
                axis tight;ax=get(gca);
                axis([ax.XLim(1)-(ax.XLim(2)-ax.XLim(1))/10 ax.XLim(2)+(ax.XLim(2)-ax.XLim(1))/10 ...
                    ax.YLim(1)-(ax.YLim(2)-ax.YLim(1))/10 ax.YLim(2)+(ax.YLim(2)-ax.YLim(1))/10]);
                axis equal
                if i>2
                    ax=get(gca);
                    plot([x0(1) x0(1)],ax.YLim,'k','linewidth',2)
                end
                [x_tmp x0(i) but]=ginput(1);
                if but~=1
                    close(gcf)
                    x=[];xd=[];t=[];
                    return
                end
                if i==2
                    x0(1)=x_tmp;
                end
                close(gcf)
            end
            [x xd t]=bm.Simulation(x0(:));
        end
        
    end % methods
    
%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
    methods (Static)    
        function bm = upgradeBM(bm_old)
            bm=BM([0 1;0 1],1);
            tmp = ?BM;
            for i=1:length(tmp.Properties)
                name_field=tmp.Properties{i}.Name;
                eval(['bm.' name_field '=' 'bm_old.' name_field ';']);
            end
        end %upgradeBM
    end %method Static

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
    methods (Static , Access = private)
%-------------------------------------------------------------------------%        
        function v=Compute_Norm(xd,Sigma)
% v=Compute_Norm(xd,Sigma) : computes the norm of hyperplanes Phi_i for the
%           given Gaussin. In fact, it finds out all the eigen vector of
%           Sigma, and then select the one which its direction is more
%           along the given xd.
%
% Input variables ---------------------------------------------------------
%   o xd:    A [nbVar/2 1] column vector indicating the direction of motion.
%            It is usually Mu(i_out,i), where i is the index of the desired
%            Gaussins
%   o Sigma: A [nbVar/2 nbVar/2] matrix representing the covariance
%            function of the Gaussian function only "on th input space".
%            Thus in fact, it is Sigma(i_out,i_out,i)
%
% Output variable ---------------------------------------------------------
%   o v:    A [nbVar/2 1] column vector defining the norm of the hyperplane

            [v_tmp tmp]=eig(Sigma); %#ok<NASGU>
            ang=v_tmp'*xd;
            [i_v i_v]=max(abs(ang));
            v=sign(ang(i_v))*v_tmp(:,i_v); %we correct the direction of the norm to be in the direction of motion by multiplying it by the sign of dot product
        end %Compute_Norm
        
        function norm_X=Mat_Vec_Norm(X)
            % norm_X=Mat_Vec_Norm(X) : computes the norm of a set of datapoints. Its
            %           main advantage over the MATLAB's norm function is its matrix
            %           computation which is faster.
            %
            % Input variable ----------------------------------------------------------
            %   o X:      A [d n] matrix representing n datapoints of d dimensions.
            %
            % Output variable ---------------------------------------------------------
            %   o norm_X: A [1 n] vector of the norm of n given datapoints
            norm_X=sqrt(sum(X.*X,1));
        end %Mat_Vec_Norm
        
%-------------------------------------------------------------------------%        
        function prob = gaussPDF(Data, Mu, Sigma)
% prob = gaussPDF(Data, Mu, Sigma) : This function computes the Probability
%           Density Function (PDF) of a multivariate Gaussian represented
%           by means and covariance matrix.
%
% Input variables ---------------------------------------------------------
%   o Data:  A [nbVar/2 n] matrix representing n datapoints of nbVar/2 dimensions.
%   o Mu:    A [nbVar/2 1] matrix representing the centers of the given
%            Gaussian function.
%   o Sigma: A [nbVar/2 nbVar/2] matrix representing the covariance
%            matrices of the given Gaussian function
% Output variable ---------------------------------------------------------
%   o prob:  [1 n]vector representing the probabilities for the n datapoints.
%
%   Copyright (c) 2006 Sylvain Calinon, LASA Lab, EPFL, CH-1015 Lausanne,
%                  Switzerland, http://lasa.epfl.ch
            
            [nbVar,nbData] = size(Data);
            Data = Data' - repmat(Mu',nbData,1);
            prob = sum((Data/Sigma).*Data, 2); %by Mohammad
            prob = exp(-0.5*prob) / sqrt((2*pi)^nbVar * (abs(det(Sigma))+realmin));
        end %gaussPDF        
    end %method static private

%-------------------------------------------------------------------------%
%-------------------------------------------------------------------------%
    methods (Access = private)
%-------------------------------------------------------------------------%        
        function err=estimation_error(bm,xd,xd_hat)
% err=estimation_error(bm,xd,xd_hat) : computes the mean square error of
%           the estimations xd_hat wrt the ground truth xd. The function
%           uses the BM parameters R and Q to weight the cost of having
%           error in direction and magnitude.
%
% Input variables ---------------------------------------------------------
%   o xd:     A [nbVar/2 n] matrix representing the real value of n
%             datapoints of nbVar/2 dimensions. 
%   o xd_hat: A [nbVar/2 1] matrix representing the estimation values of xd
% Output variable ---------------------------------------------------------
%   o err:    An scalar indicating the error in estimating xd

            xd_norm=bm.Mat_Vec_Norm(xd);
            i=find(xd_norm==0);
            xd_norm(i)=[];xd(:,i)=[];xd_hat(:,i)=[];
            e_xd_Norm=bm.Mat_Vec_Norm(xd-xd_hat)./xd_norm; %the error term in the magnitude of velocity
            e_Dir=acos(dot(xd,xd_hat)./(xd_norm.*bm.Mat_Vec_Norm(xd_hat)+realmin)); %the error term in the direction of velocity
            err=mean(sqrt(bm.R*e_Dir.^2+bm.Q*e_xd_Norm.^2)); %total error
        end %estimation_error
        
%-------------------------------------------------------------------------%        
        function st=StabilityCheck(bm,data,i_d)
% st=StabilityCheck(data,i_d) : check the stability condition for the given
%           data over the region i_d.
%
% Input variables ---------------------------------------------------------
%   o data: A [nbVar n] matrix representing n datapoints of nbVar
%           dimensions.
%   o i_d:  A scalar defining the index of the region Omega that data
%           should be evaluated there. Note that, data should lies between
%           two hyperplanes Phi_{i_d} and Phi_{i_d+1}
% Output variable ---------------------------------------------------------
%   o st:   A boolean indicating the result of stability analysis. It is
%           true if the given data passes the stability conditions.

            st=true;
            s=sum(((data(bm.i_in,:)-repmat(bm.Mu(bm.i_in,i_d+1),1,size(data,2)))'/bm.Sigma(bm.i_in,bm.i_in,i_d+1)...
                   -(data(bm.i_in,:)-repmat(bm.Mu(bm.i_in,i_d),1,size(data,2)))'/bm.Sigma(bm.i_in,bm.i_in,i_d))'.*data(bm.i_out,:));
            if any(s<=0)
                st=false;
            end

            %A more clear version
%             st=true;
%             for i=1:size(data,2)
%                 s=(data(bm.i_in,i)-bm.Mu(bm.i_in,i_d+1))'/bm.Sigma(bm.i_in,bm.i_in,i_d+1)*data(bm.i_out,i)...
%                     -(data(bm.i_in,i)-bm.Mu(bm.i_in,i_d))'/bm.Sigma(bm.i_in,bm.i_in,i_d)*data(bm.i_out,i);
%                 if s<=0
%                     st=false;
%                     return;
%                 end
%             end
        end %StabilityCheck
        
%-------------------------------------------------------------------------%        
        function p_fit=Bezier_Fit(bm,Mu0,Mu1,t)
% p_fit = Bezier_Fit(Mu0,Mu1,t) : Given, mean of two Gaussians, it fits a
%           smooth bezier function between them, and then retrieve the
%           point(s) at the specified t. Note that t is a real value between
%           [0 1]. t=0 and t=1 corresponds to Mu0 and Mu1 respectively.
%
% Input variables ---------------------------------------------------------
%   o Mu0: A [nbVar 1] vector defining the starting point of Bezier function
%   o Mu1: A [nbVar 1] vector defining the final point of Bezier function
%   o t:   A [1 n] vector defining the indices of retrieval points. all
%          values of t should lies between [0 1]
% Output variable ---------------------------------------------------------
%   o p_fit: A [nbVar n] matrix containing values of n interpolated datapoints

            P1=Mu1(bm.i_in)-norm(Mu1(bm.i_in)-Mu0(bm.i_in))/2*Mu1(bm.i_out)/norm(Mu1(bm.i_out)); %finding an intermediate point
            P=Mu0(bm.i_in)*(1-t).^2+2*P1*((1-t).*t)+Mu1(bm.i_in)*t.^2; %computing points using quadratic bezier function
            Pd_dir=-2*Mu0(bm.i_in)*(1-t)+2*P1*(1-2*t)+2*Mu1(bm.i_in)*t; %computing direction of velocity at each point
            Pd_dir=Pd_dir./repmat(bm.Mat_Vec_Norm(Pd_dir)+realmin,length(bm.i_in),1); %normalizing velocity values
            Pd_mag=(norm(Mu1(bm.i_out))-norm(Mu0(bm.i_out)))*t+norm(Mu0(bm.i_out))*ones(size(t)); %computing the magnitude of velocity using a linear interpolation
            Pd=repmat(Pd_mag,length(bm.i_in),1).*Pd_dir;
            p_fit=[P;Pd];
        end %Bezier
        
%-------------------------------------------------------------------------%        
        function [domain ind_neigh Pxi_reduced] = find_domain(bm, x)
% [domain ind_neigh Pxi_reduced] = find_domain(x) : finding out the
%           corresponding domain of the given datapoints.
%
% Input variable ----------------------------------------------------------
%   o x:   A [nbVar/2 n] matrix representing n datapoints of nbVar/2
%           dimension in the task space.
% Output variable ---------------------------------------------------------
%   o domain:      A [n 1] vector indicating the corresponding domain of
%                  each datapoint. 
%   o ind_neigh:   A [n 2] matrix defining the indices of the two Gaussians
%                  that should be used to compute xd. By construction
%                  domain = ind_neigh(1,:)
%   o Pxi_reduced: The computed probability of the given datapoints with the
%                  Gaussians defined by ind_neigh

            nb_x=size(x,2);
            % first we compute probability of x for all Gaussians
            for i=1:bm.K
                Pxi(:,i) = bm.gaussPDF(x, bm.Mu(bm.i_in,i), bm.Sigma(bm.i_in,bm.i_in,i));
            end
            
            %For each datapoint, finding the Gaussian that has the highest
            %possibiliy. We use this Gaussian as a starting point to find
            %the domain.
            [i_hmax i_hmax]=max(Pxi,[],2);
           
            
            %%checking if the point is in the left or right handside of the hyperplane
            domain_check=sum(bm.V(:,i_hmax).*(x-bm.Mu(bm.i_in,i_hmax)));
            
            %For points very close to the hyperplane, we assume they lies
            %on the hyperplane. This assumption is not necessary, but
            %improves the algorithm's performance.
            domain_check(abs(domain_check)<bm.tol)=0;
            ind_neigh=zeros(nb_x,2);
            if isempty(bm.iK_model)
                bm.iK_model=bm.K;
            end
            %finding points that lie on the left hand side of hyperplanes
            i=find(domain_check<=0);
            if ~isempty(i)
                for j=1:length(bm.iK_model)
                    i_e=find(i_hmax(i)==bm.iK_model(j));
                    if ~isempty(i_e)
                        ind_neigh(i(i_e),:)=[i_hmax(i(i_e)) i_hmax(i(i_e))]; %defining the active Gaussians
                        i(i_e)=[];
                    end
                end
                
                ind_neigh(i,:)=[i_hmax(i) i_hmax(i)+1]; %defining the active Gaussians
                
                %now we check if points lie on the right hand side of the
                %other hyperplane (if domain computed properly, points
                %should lie between two hyperplanes). For those points that
                %this criterion is not satisfied, we continue going
                %backward to find the correct domain.
                domain_check_n=sum(bm.V(:,ind_neigh(i,2)).*(x(:,i)-bm.Mu(bm.i_in,ind_neigh(i,2))));
                %domain_check_n=sum(bm.V(:,min([ind_neigh(i,2) bm.K*ones(size(i))'],[],2)).*(x(:,i)-bm.Mu(bm.i_in,min([ind_neigh(i,2) bm.K*ones(size(i))'],[],2))));
                i2=find(sign(domain_check(i))==sign(domain_check_n) & domain_check_n~=domain_check(i));
                while ~isempty(i2)
                    ind_neigh(i(i2),:)=ind_neigh(i(i2),:)+1;
                    domain_check(i(i2))=domain_check_n(i2);
                    i=i(i2);
                    for j=1:length(bm.iK_model)
                        i_e=find(ind_neigh(i,1)==bm.iK_model(j) & ind_neigh(i,2)>bm.iK_model(j));
                        if ~isempty(i_e)
                            ind_neigh(i(i_e),2)=ind_neigh(i(i_e),1); %defining the active Gaussians
                            i(i_e)=[];
                        end
                    end
                    domain_check_n=sum(bm.V(:,ind_neigh(i,2)).*(x(:,i)-bm.Mu(bm.i_in,ind_neigh(i,2))));
                    i2=find(sign(domain_check(i))==sign(domain_check_n) & domain_check_n~=domain_check(i));
                end
            end
            
            %finding points that lie on the right hand side of hyperplanes
            i=find(domain_check>0);
            if ~isempty(i)
                ind_neigh(i,:)=[i_hmax(i)-1 i_hmax(i)]; %defining the active Gaussians
                
                %now we check if points lie on the left hand side of the
                %other hyperplane (if domain computed properly, points
                %should lie between two hyperplanes). For those points that
                %this criterion is not satisfied, we continue going
                %forward to find the correct domain.
                domain_check_n=sum(bm.V(:,max([ind_neigh(i,1) ones(size(i))'],[],2)).*(x(:,i)-bm.Mu(bm.i_in,max([ind_neigh(i,1) ones(size(i))'],[],2))));
                i2=find(sign(domain_check(i))==sign(domain_check_n) & domain_check_n~=domain_check(i));
                while ~isempty(i2)
                    ind_neigh(i(i2),:)=ind_neigh(i(i2),:)-1;
                    domain_check(i(i2))=domain_check_n(i2);
                    i=i(i2);
                    domain_check_n=sum(bm.V(:,max([ind_neigh(i,1) ones(size(i))'],[],2)).*(x(:,i)-bm.Mu(bm.i_in,max([ind_neigh(i,1) ones(size(i))'],[],2))));
                    i2=find(sign(domain_check(i))==sign(domain_check_n) & domain_check_n~=domain_check(i));
                end
            end
            
            %correcting invalid domains. Note that for some points, there
            %is only one active Gaussian (applicable only for the first and
            %last Gaussians).
            ind_neigh(ind_neigh<1)=1;
            ind_neigh(ind_neigh>bm.K)=bm.K;
            
            domain=ind_neigh(:,1); %defining domain

            %Trimming Pxi matrix. Now we only keeps the cells corresponding
            %to the active Gaussians. The more clear meaning of this line
            %can be understood from the 3 commented lines after it.
            Pxi_reduced=Pxi(nb_x*(ind_neigh-1)+repmat([1:nb_x]',1,2));
%             for i=1:nb_x
%                 Pxi_reduced(i,:)=Pxi(i,ind_neigh(i,:));
%             end
        end %find domain
        
%-------------------------------------------------------------------------%        
        function bm=preprocess_data(bm,index)
% bm = preprocess_data(index) : Computes the first derivative of the
%           demonstration datapoints (saved in BM.Data). It also computes
%           x0, and dx_shift. This function is called when one creates an
%           object of class BM.
%
% Input variable ----------------------------------------------------------
%   o index:     A vector defining the initial index of each demonstration.
%                Following the previous example, index = [i1 i2 i3]
%                indicates 
%                that columns i1:i2-1 belongs to the first demonstration,
%                i2:i3-1 -> 2nd demonstration, and i3:end -> 3rd demonstration. 
%                Obviously, we always have i1=1;
% Output variable ---------------------------------------------------------
%   o bm:        An object with the class of BM

            index_tmp=zeros(1,bm.nbSamples); %after smoothing, demonstration's index may changed
            index=[index(:);size(bm.Data,2)+1]; %for simplicity we add this value to index, at the end we will remove it
            Pos=[];Time=[];x0_tmp=[];dx_shift=[];
            s_v=25; %smoothing value. This parameter is not necessary for BM, but enhances the performance.
            for i=1:bm.nbSamples
                index_tmp(i)=size(Pos,2)+1; %saving the new index
                %Reading data from different demonstrations
                T=bm.T(index(i):index(i+1)-1); %reading time
                X=bm.Data(1:end,index(i):index(i+1)-1); %reading the data value
                
                if bm.order==1
                    for j=bm.i_in %smoothing
                        X(j,:)=smooth(X(j,:),s_v);
                    end
                else
                    for j=1:bm.nbVar/4 %smoothing
                        X(j,:)=smooth(X(j,:),s_v);
                    end
                end
                
                dx_shift=[dx_shift X(:,end)]; %saving the starting point of each dem.
                
                if bm.order==2
                    Xd=diff(X,1,2)./repmat(diff(T),bm.nbVar/4,1); %computing the derivative of data
                    X=[X;[Xd zeros(bm.nbVar/4,1)]];
                else
                    Xd=diff(X,1,2)./repmat(diff(T),bm.nbVar/2,1); %computing the derivative of data
                end
                
                %trimming each demonstration such that it has one zero velocity at the
                %starting and final point
                tmp=find(bm.Mat_Vec_Norm(Xd)>bm.tol);
                i_min=min(tmp);
                i_max=max(tmp)+1;
                T=T(i_min:i_max)-T(i_min);
                X=X(:,i_min:i_max);
                
                for j=bm.i_in %shifting the target to the origin
                    X(j,:)=X(j,:)-X(j,end);
                end
                
                Pos=[Pos X];
                x0_tmp=[x0_tmp X(:,1)]; %starting point of each demonstration
                Time=[Time T];
            end
            
            index=[index_tmp size(Pos,2)+1]; %new index
            
            % Time Scaling demonstrations. This part is not necessary for the
            % algorithm, but it enhances the performance siginificantly.
            tf=Time(index(2:end)-1); %reading the final time of each dem.
            tf_mean=mean(tf);
            for i=1:bm.nbSamples %scaling all trajectories in time
                Time(index(i):index(i+1)-1)=Time(index(i):index(i+1)-1)/tf(i)*tf_mean;
            end
            
            %now we sample all trajectories in the same time index
            avg_length=round(mean(diff(index))); %computing the average length of dem.
            t_sample=linspace(0,tf_mean,avg_length); %computing the sampling time
            Data=zeros(bm.nbVar,avg_length,bm.nbSamples); %the final refined output data
            for i=1:bm.nbSamples
                for j=1:bm.nbVar/2
                    Data(j,:,i)=spline(Time(index(i):index(i+1)-1),Pos(j,index(i):index(i+1)-1),t_sample);
                end
            end
            Time=repmat(t_sample,[bm.nbVar/2,1,bm.nbSamples]);
            
            Data(bm.i_out,1:end-1,:)=diff(Data(bm.i_in,:,:),1,2)./diff(Time,1,2); %computing the first derivative of data
            bm.x0=mean(x0_tmp,2); %mean starting point of each demonstration
            bm.dx_shift=mean(dx_shift,2); %mean final point of each demonstraion (before shifting to origin)
            bm.Data=Data(:,end:-1:1,:); %Data after trimming, smoothing, computing its first derivative, and reversing
            bm.T=Time(:,end:-1:1,:);
            bm.nbData=size(Data,2);
            bm.tol = min([abs(mean(bm.Mat_Vec_Norm(bm.Data(bm.i_in,:)))) abs(mean(bm.Mat_Vec_Norm(bm.Data(bm.i_out,:))))])*10^-6;
            bm.IsInitialized=false;
        end %preprocess_data
        
%-------------------------------------------------------------------------%
        function bm=Constructing_last_Gaussian(bm)
% bm = Constructing_last_Gaussian() : This function construct a Gaussian on
%           the target (it always has the index of one in Mu and Sigma)
%           such that it becomes globally stable (2nd condition of
%           stability). This function is called in the first line of
%           Iteration learning step.
%
% Output variable ---------------------------------------------------------
%   o bm:           An object with the class of BM

            % Constructing the first Gaussian (which its center is on the target)
            r=norm(sqrt(var(bm.Data(bm.i_in,:),0,2))/5); % all data inside a sphere of radius r is used for the last Gaussian
            
            %finding data within the radius r
            ind=zeros(1,bm.nbSamples);
            for i=1:bm.nbSamples
                ind(i)=find(bm.Mat_Vec_Norm(bm.Data(bm.i_in,:,i))<r, 1, 'last' ); %indices of the points that will be used for the last gaussian
            end
            ind=1:max([min(ind) 10]);
            Data_1=reshape(bm.Data(:,ind,:),bm.nbVar,[]); % this variable is used to keep the data of the last gaussian
            
            %Create the Gaussian. We makes the data symmetric w.r.t. the
            %origin. Thus, mean of the Gaussian always is on the target (i.e. origin)
            Sigma_1=cov([Data_1 -Data_1]')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
            
            %If the resultant Gaussian is not stable, we remove the outer
            %datapoints up to the point that it becomes stable.
            while ~all(eig(Sigma_1(bm.i_out,bm.i_in)/Sigma_1(bm.i_in,bm.i_in))<0) && length(ind)>1
                ind(end)=[];
                Data_1=reshape(bm.Data(:,ind,:),bm.nbVar,[]); % this variable is used to keep the data of the last gaussian
                Sigma_1=cov([Data_1 -Data_1]')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
            end
            
            bm.Mu(:,1)=zeros(bm.nbVar,1);
            bm.Sigma(:,:,1)=Sigma_1;
            mu=mean(Data_1,2);
            bm.V(:,1)=bm.Compute_Norm(mu(bm.i_out),bm.Sigma(bm.i_in,bm.i_in,1)); %we use the velocity of the second Gaussian to help us find the correct eigenvector
            bm.Mu(:,ind(2:end))=[];bm.Sigma(:,:,ind(2:end))=[];bm.V(:,ind(2:end),:)=[];bm.b_ind(ind(2:end))=[];
            bm.K=size(bm.Mu,2);
            %correcting domains
            bm.i_Omega(:,ind)=1;
            bm.i_Omega(:,ind(end)+1:end)=bm.i_Omega(:,ind(end)+1:end)-(max(ind)-1);
        end
        
%-------------------------------------------------------------------------%        
        function [bm merge_status]=Merge_Gaussians(bm,ind)
% [bm merge_status]=Merge_Gaussians(ind) : This function merges two
%           Gaussian [ind ind+1], and check stability and accuracy
%           conditions. If both conditions are satisfied, then the merged
%           model is sent as a output with the flag merge_status=true.
%           Otherwise, the model keep them unmerged and a flag
%           merge_status=false is generated. Note that the input 'ind' is
%           an integer less than BM.K.
%
% Input variable ----------------------------------------------------------
%   o ind:          A integer defining the index of Gaussian to be merged.
%                   The algorithm merges Gaussians with indices of [ind ind+1].
%                   Obviously, one should consider ind < BM.K
% Output variables --------------------------------------------------------
%   o bm:           An object with the class of BM
%   o merge_status: A flag indicating the result of merging. It is true if
%                   the merged merged model satisfies both stability and
%                   accuracy conditions. Otherwise, it false messeage is
%                   sent.
            
            %Backing up the unmerged model
            Mu_bk=bm.Mu;
            Sigma_bk=bm.Sigma;
            V_bk=bm.V;
            i_Omega_bk=bm.i_Omega;
            
            %removing one of two Gaussians
            bm.Mu(:,ind)=[];bm.Sigma(:,:,ind)=[];bm.V(:,ind)=[];
            bm.K=bm.K-1;
            
            %selecting data belongs to Gaussians with index ind and ind+1
            d = reshape(bm.Data(:,bm.b_ind(ind):bm.b_ind(ind+2)-1,:),bm.nbVar,[]);
            
            if ind==1
                %For the Gaussian on the target we need to makes data
                %symmetric and check one more stability condition
                d_sym=[d -d];
                bm.Mu(:,1)=zeros(bm.nbVar,1);
                bm.Sigma(:,:,1)=cov(d_sym')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                bm.V(:,1)=bm.Compute_Norm(mean(d(bm.i_out,:),2),bm.Sigma(bm.i_in,bm.i_in,1)); %finding the norm of hyperplane
                if ~all(eig(bm.Sigma(bm.i_out,bm.i_in,1)/bm.Sigma(bm.i_in,bm.i_in,1))<0)
                    merge_status=false;
                    bm.Mu=Mu_bk;
                    bm.Sigma=Sigma_bk;
                    bm.V=V_bk;
                    bm.i_Omega=i_Omega_bk;
                    bm.K=bm.K+1;
                    return
                end
            else
                %For other Gaussians, we simply generate the merged
                %Gaussian from the data
                bm.Mu(:,ind)=mean(d,2);
                bm.Sigma(:,:,ind)=cov(d')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                bm.V(:,ind)=bm.Compute_Norm(bm.Mu(bm.i_out,ind),bm.Sigma(bm.i_in,bm.i_in,ind)); %finding the norm of hyperplane
            end
            
            [xd domain]=bm.GMR(d(bm.i_in,:),'training'); %compute estimations for the selected data
            bm.i_Omega(bm.i_Omega>ind)=bm.i_Omega(bm.i_Omega>ind)-1; %correcting the domain of other points
            %putting the computed domain of the selected data into i_Omega
            bm.i_Omega(:,bm.b_ind(ind):bm.b_ind(ind+2)-1)=reshape(domain,[],size(bm.i_Omega,1))';
            
            %Checking stability and accuracy conditions
            merge_status=true;
            for i=[min([ind bm.K-1]) max([ind-1 1])]
                d_check=bm.Data(:,bm.i_Omega'==i);
                if ~isempty(d_check)
                    xd=bm.GMR(d_check(bm.i_in,:),'training'); %,i*ones(size(d_check,2),1)
                    if (bm.estimation_error(d_check(bm.i_out,:),xd)>bm.e_max) || ~bm.StabilityCheck(d_check,i)
                        merge_status=false;
                        bm.Mu=Mu_bk;
                        bm.Sigma=Sigma_bk;
                        bm.V=V_bk;
                        bm.i_Omega=i_Omega_bk;
                        bm.K=bm.K+1;
                        break;
                    end
                end
            end
            if merge_status
                bm.b_ind(ind+1)=[];
                fprintf('Gaussians G%d and G%d merged into G%d -> Total # of Gaussian = %d\n',[ind ind+1 ind bm.K])
            end
        end %merge Gaussians
        
%-------------------------------------------------------------------------%        
        function [bm indices]=Align_Demonstrations(bm)
% [bm indices]=Align_Demonstrations() : Given demonstrations from the user
%           usually have different lengths. Even if they have the same
%           size, the points with the same index doesn't usually
%           corresponds to the same part of the motion. To ensure stability
%           at initialization, we need to have trajectories with the same
%           size. Besides the point with the same index should almost
%           contains the same information about the motion. we use sample
%           alignment to have satisfy these two requirements. This function
%           is called in the first line of initializing learning step.
%
% Output variables --------------------------------------------------------
%   o bm:      An object with the class of BM
%   o indices: A [nbSamples n] matrix indicating the indices of the Data
%              matrix that are selected after sample alignment.
            
            % Finding a reference trajectory. This procedure is not
            % necessary for the algorithm, but it enhances the performance
            % siginificantly.
            ref_finder=zeros(bm.nbSamples);
            for i=1:bm.nbSamples
                for j=i+1:bm.nbSamples
                    ref_finder(i,j)=mean(bm.Mat_Vec_Norm(bm.Data(:,:,i)-bm.Data(:,:,j)));
                end
            end
            ref_finder=ref_finder+ref_finder';
            [i_ref i_ref]=sort(sum(ref_finder),'ascend');
            indices=[];
            
            % Sample alignment
            for i=1:bm.nbSamples-1
                if i==1
                    X=bm.Data(:,:,i_ref(i));
                else
                    X=Y(:,J);
                end
                Y=bm.Data(:,:,i_ref(i+1));
                [I, J] = samplealign([1:size(X,2);X]',[1:size(Y,2);Y]','Band',50,'weights',[0 ones(size(bm.i_in)) 0.02*ones(size(bm.i_out))],'Gap',bm.Gap_value);%,'DistanceValue',syn_signal);
                
                if i==1
                    Data_aligned=reshape(X(:,I),[bm.nbVar,length(I),1]);
                    Time_aligned=bm.T(:,I,i_ref(i));
                    indices=I';
                else
                    Data_aligned=Data_aligned(:,I,:);
                    Time_aligned=Time_aligned(:,I,:);
                    indices=indices(:,I);
                end
                Data_aligned(:,:,end+1)=Y(:,J); %#ok<AGROW>
                Time_aligned(:,:,end+1)=bm.T(:,J,i_ref(i)); %#ok<AGROW>
                indices(end+1,:)=J;
            end
            bm.nbData=size(Data_aligned,2);
            bm.Data=Data_aligned;
            bm.T=Time_aligned;
            bm.IsInitialized=true;
            bm.x0=mean(bm.Data(bm.i_in,end,:),3); %updating the average initial point
        end %Align_Demonstrations
        
%-------------------------------------------------------------------------%        
        function bm=Initializing(bm)
% bm = Initializing() : This function is the first step of BM learning
%           procedure. At first, it aligns demonstration by using a sample
%           alignment procedure. Then it construct one Gaussian per
%           corresponding data points of all demonstrations. For example,
%           given 3 demonstrations with the length K, it creates K
%           Gaussians, each of which generated from three datapoints.
%
% Output variable ---------------------------------------------------------
%   o bm:   An object with the class of BM
            
            if ~bm.IsInitialized
                % The following procedure is not necessary for datapoints
                % that were initializied before.
                bm=bm.Align_Demonstrations();
                Data=zeros(bm.nbVar,bm.nbData,bm.nbSamples);
                bm.V=zeros(bm.nbVar/2,bm.nbData); %This will be the matrix of all normal vector v, which are norms of hyperplanes
                for i=bm.nbData:-1:2
                    X=squeeze(bm.Data(:,i,:)); %taking corresponding data of all dem.
                    mu=mean(X,2); %computing mean
                    v=bm.Compute_Norm(mu(bm.i_out),cov(X(bm.i_in,:)')); %norm of hyper plane Phi_i
                    bm.V(:,i)=v;
                    %Now we can generate a hyper-plane where v is its norm and mu is one point
                    %on it. The equation of this hyper-plane is Phi: v(x-mu)=0.

                    %now for each demonstration, we find its intersection with the hyperplane
                    for j=1:bm.nbSamples
                        % now we find a point x' such that it lies on the hyperplane Phi for
                        % this we have two options:
                        % 1) construct a line that paths through the point X(i_in,j) with
                        % the slope of X(i_out,j), and then find out where it crosses the
                        % hyper plane, which is:
                        u=(v'*(mu(bm.i_in)-X(bm.i_in,j)))/(v'*X(bm.i_out,j)+realmin);
                        X_path=X(bm.i_in,j)+u*X(bm.i_out,j);

                        % 2) project the point onto the hyperplane.
                        X_proj=X(bm.i_in,j)-v'*(X(bm.i_in,j)-mu(bm.i_in))*v;

                        if u >= -bm.tol || i==bm.nbData % the point is in the left hand side of (or on) the hyperplane. The case i==nbData is here to avoid error
                            i_neigh=i-1;
                        else
                            i_neigh=i+1;
                        end

                        % now we choose the best solution between them
                        if norm(X_path-X(bm.i_in,j)) + norm(X_path-bm.Data(bm.i_in,i_neigh,j)) < norm(X_proj-X(bm.i_in,j)) + norm(X_proj-bm.Data(bm.i_in,i_neigh,j))
                            Data(bm.i_in,i,j)=X_path;
                        else
                            Data(bm.i_in,i,j)=X_proj;
                        end

                        % Since the position of the new point (projected or ...) is
                        % different from the previous point, we need to correct the time
                        % index based on this new position.
                        if i<bm.nbData
                            bm.T(:,i,j)=max([10^-6 min([2*diff(bm.T(1,i:-1:i-1,j),1,2) norm(Data(bm.i_in,i,j)-Data(bm.i_in,i+1,j))./norm(bm.Data(bm.i_out,i,j)+realmin)])])+bm.T(:,i+1,j);
                        end
                    end
                end
                %Now we compute again the velocity based on the new obtained data
                Data(bm.i_out,2:end,:)=diff(Data(bm.i_in,:,:),1,2)./(diff(bm.T,1,2)+realmin);
                bm.Data=Data;
                
                %  Constructing Gussians per pair of datapoint
                bm.Mu=sum(bm.Data(:,1:end,:),3)/bm.nbSamples;
                bm.K=bm.nbData; %setting the number of Gaussians to the number of Datapoints
                bm.Sigma=zeros(bm.nbVar,bm.nbVar,bm.nbData); %computing covariance of Gaussians
                for i=1:bm.nbData
                    bm.Sigma(:,:,i)=cov(squeeze(bm.Data(:,i,:))')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                end
                
                % Checking stability of initialization, and construct it such
                % that it always satisfies stability conditions
                i=2;
                while i<bm.nbData
                    d_check=squeeze(bm.Data(:,i,:));
                    if ~bm.StabilityCheck(d_check,i);
                        %This checks if velocities of all points have the same direction.
                        %If it is not true, then we remove the point
                        for j=2:size(d_check,2)
                            ch_sign(j-1)=dot(d_check(bm.i_out,1),d_check(bm.i_out,j))<0;
                        end
                        if any(ch_sign)
                            bm.Mu(:,i)=[];
                            bm.Sigma(:,:,i)=[];
                            bm.V(:,i)=[];
                            bm.Data(:,i,:)=[];
                            bm.T(:,i,:)=[];
                            bm.nbData=bm.nbData-1;
                            bm.K=bm.K-1;
                            i=i-1;
                        else
                            %now we check if the two hyperplanes intersect each other
                            %inside the D domain or not. If yes, then we remove the point.
                            for j=1:size(d_check,2)
                                ch_sign2(j)=dot(bm.V(:,i+1),d_check(bm.i_in,j)-bm.Mu(bm.i_in,i+1))<0;
                            end
                            if any(ch_sign2)
                                bm.Mu(:,i+1)=[];
                                bm.Sigma(:,:,i+1)=[];
                                bm.V(:,i+1)=[];
                                bm.Data(:,i+1,:)=[];
                                bm.T(:,i+1,:)=[];
                                bm.nbData=bm.nbData-1;
                                bm.K=bm.K-1;
                                i=i-1;
                            else
                                %If we reach to this point, it means that data points are good, but we need
                                %to have some points in between to satisfy stability conditions. To do so,
                                %we use a bezier function to find an appropriate datapoint between a two
                                %given datapoints such that it satisfies stability conditions.
                                bm.Mu(:,i+2:end+1)=bm.Mu(:,i+1:end);
                                bm.Sigma(:,:,i+2:end+1)=bm.Sigma(:,:,i+1:end);
                                for t=2:20
                                    for j=1:bm.nbSamples
                                        new_Data(:,j)=bm.Bezier_Fit([bm.Data(bm.i_in,i+1,j);bm.Mu(bm.i_out,i+1)],[bm.Data(bm.i_in,i,j);bm.Mu(bm.i_out,i)],1-1/t);
                                    end
                                    bm.Mu(:,i+1)=mean(new_Data,2);
                                    bm.Sigma(:,:,i+1)=cov(new_Data')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                                    if bm.StabilityCheck(d_check,i)
                                        bm.V(:,i+2:end+1)=bm.V(:,i+1:end);
                                        bm.Data(:,i+2:end+1,:)=bm.Data(:,i+1:end,:);
                                        bm.T(:,i+2:end+1,:)=bm.T(:,i+1:end,:);
                                        bm.Data(:,i+1,:)=new_Data;
                                        bm.T(:,i+1,:)=diff(bm.T(:,i:i+1,:),1,2)*(1-1/t)+bm.T(:,i,:);
                                        bm.V(:,i+1)=bm.Compute_Norm(bm.Mu(bm.i_out,i+1),bm.Sigma(bm.i_in,bm.i_in,i+1)); %norm of hyper plane Phi_i
                                        bm.nbData=bm.nbData+1;
                                        bm.K=bm.K+1;
                                        break;
                                    end
                                end
                                if t==20 && ~bm.StabilityCheck(d_check,i)
                                    bm.Mu(:,i+1:i+2)=[];
                                    bm.Sigma(:,:,i+1:i+2)=[];
                                    bm.V(:,i+1)=[];
                                    bm.Data(:,i+1,:)=[];
                                    bm.T(:,i+1,:)=[];
                                    bm.nbData=bm.nbData-1;
                                    bm.K=bm.K-1;
                                    i=i-1;
                                end
                            end
                        end
                    end
                    i=i+1;
                end
            else
                bm.V=zeros(bm.nbVar/2,bm.nbData); %This will be the matrix of all normal vector v, which are norms of hyperplanes
                for i=bm.nbData:-1:2
                    X=squeeze(bm.Data(:,i,:)); %taking corresponding data of all dem.
                    mu=mean(X,2); %computing mean
                    bm.V(:,i)=bm.Compute_Norm(mu(bm.i_out),cov(X(bm.i_in,:)')); %norm of hyper plane Phi_i
                end
                %  Constructing Gussians per pair of datapoint
                bm.Mu=sum(bm.Data(:,1:end,:),3)/bm.nbSamples;
                bm.Sigma=zeros(bm.nbVar,bm.nbVar,bm.nbData); %computing covariance of Gaussians
                for i=1:bm.nbData
                    bm.Sigma(:,:,i)=cov(squeeze(bm.Data(:,i,:))')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                end
                bm.K=bm.nbData;
            end
            
            bm.b_ind=1:bm.K+1; %b_ind is a variable like b_ind that is used to keep the indices of data corresponding to each Gaussian
            bm.i_Omega=repmat(1:bm.K,bm.nbSamples,1); %This defines the region that each data points belong to
        end %Initializing
        
%-------------------------------------------------------------------------%
        function [bm n_iter]=Iteration(bm)
% [bm n_iter] = Iteration() : This function is the second step in BM
%           learning. It starts merging Gaussians iteratively until it
%           converges to a local minimum number. At each iteration, one
%           picks at random a pair of adjacent Gaussians and merges them
%           into a new Gaussian, by computing the new means and covariance
%           on the union of data points associated to each of the two
%           Gaussians. If the region defined by this pair satisfies the
%           stability and accuracy conditions, then the new model is now
%           composed of $K-1$ Gaussians. The procedure terminates  when it
%           is no longer possible to merge any pair of Gaussians without
%           depreciating the accuracy or becoming unstable. The model
%           converges within the maximum nbData*(nbData-1)/2.            
%
% Output variables --------------------------------------------------------
%   o bm:     An object with the class of BM
%   o n_iter: A [1 2] vector indicating the number of iterations. n_iter(1)
%             specifies the total number of iterations, while n_iter(2)
%             represents the number of successful iterations.

            bm=bm.Constructing_last_Gaussian();
            % Iterative procedure to merge Gaussian
            neighbor=[-1 1]; %each Gaussian has two neigbours
            i=0; %going into the while loop
            n_iter=[0 0]; %counting the number of iteration.
            while i~=bm.K
                ind_rand=randperm(bm.K); %computing a set of random indices
                for i=1:bm.K
                    n_iter(1)=n_iter(1)+1;
                    if ind_rand(i)==1
                        l_j=1; %there is only one neighbor
                        neigh=1;
                    elseif ind_rand(i)==bm.K
                        l_j=1; %there is only one neighbor
                        neigh=-1;
                    else
                        l_j=2;
                        neigh=neighbor(randperm(2)); %randomizing the order of neighbors
                    end
                    
                    for j=1:l_j
                        if neigh(j)==-1
                            ind=ind_rand(i)-1; %we just seve the index with the lower value, and as a rule, just define the neighbor to be the one with i_n=i+1
                        else
                            ind=ind_rand(i);
                        end
                        [bm merge_status]=bm.Merge_Gaussians(ind);
                        if merge_status %if merging the selected random Gaussians is successful, then its value becomes true
                            n_iter(2)=n_iter(2)+1;
                            break;
                        end
                    end
                    if merge_status
                        break;
                    end
                end
            end
        end %Iteration
        
        function [bm n_iter]=Iteration_Direct(bm)
% [bm n_iter] = Iteration() : This function is the second step in BM
%           learning. It starts merging Gaussians iteratively until it
%           converges to a local minimum number. At each iteration, one
%           picks at random a pair of adjacent Gaussians and merges them
%           into a new Gaussian, by computing the new means and covariance
%           on the union of data points associated to each of the two
%           Gaussians. If the region defined by this pair satisfies the
%           stability and accuracy conditions, then the new model is now
%           composed of $K-1$ Gaussians. The procedure terminates  when it
%           is no longer possible to merge any pair of Gaussians without
%           depreciating the accuracy or becoming unstable. The model
%           converges within the maximum nbData*(nbData-1)/2.            
%
% Output variables --------------------------------------------------------
%   o bm:     An object with the class of BM
%   o n_iter: A [1 2] vector indicating the number of iterations. n_iter(1)
%             specifies the total number of iterations, while n_iter(2)
%             represents the number of successful iterations.

            bm=bm.Constructing_last_Gaussian();
            i=1;
            n_iter=[0 0]; %counting the number of iteration.
            while i<bm.K
                n_iter=n_iter+1; %counting the number of iteration.
                [bm merge_status]=bm.Merge_Gaussians(i);
                if ~merge_status %if merging the selected random Gaussians failed
                    i=i+1;
                    n_iter(2)=n_iter(2)-1;
                end
            end
        end %Iteration Direct
        
%-------------------------------------------------------------------------%        
        function bm=Rem_Singular_Gaussian(bm)
% bm = Rem_Singular_Gaussian() : Removing singular Gaussians. This function
%           is not part of BM; however it improves the performance. By
%           singular Gaussians, we mean the Gaussians that have lower rank
%           than the state space.
%
% Output variable ---------------------------------------------------------
%   o bm:     An object with the class of BM

            pSigma_ind=find(diff(bm.b_ind)==1); %finding singular Gaussians
            while ~isempty(pSigma_ind)
                i=pSigma_ind(1);
                Mu_bk=bm.Mu;Sigma_bk=bm.Sigma;V_bk=bm.V; %backing up stuffs
                bm.K=bm.K-1; %decreasing the number of Gaussians by one, since we are sure that we will merge it.
                
                %Each singular Gaussian has two neighbors (except for first
                %and last Gaussians). We try to merge the singular
                %Gaussians with its both neighbors, and then we will choose
                %the best case.
                if i-1>0
                    bm.Mu(:,i)=[];bm.Sigma(:,:,i)=[];bm.V(:,i)=[];
                    d = reshape(bm.Data(:,bm.b_ind(i-1):bm.b_ind(i),:),bm.nbVar,[]);
                    if i-1==1
                        d_sym=[d -d];
                        bm.Mu(:,1)=zeros(bm.nbVar,1);
                        bm.Sigma(:,:,1)=cov(d_sym')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                        bm.V(:,1)=bm.Compute_Norm(mean(d(bm.i_out,:),2),bm.Sigma(bm.i_in,bm.i_in,1)); %finding the norm of hyperplane
                        if ~all(eig(bm.Sigma(bm.i_out,bm.i_in,1)/bm.Sigma(bm.i_in,bm.i_in,1))<0)
                            e1=+10^10; %considering a big value
                        else
                            [xd domain]=bm.GMR(d(bm.i_in,:),'training');
                            e1=bm.estimation_error(d(bm.i_out,:),xd);
                        end
                    else
                        bm.Mu(:,i-1)=mean(d,2);
                        bm.Sigma(:,:,i-1)=cov(d')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                        bm.V(:,i-1)=bm.Compute_Norm(bm.Mu(bm.i_out,i-1),bm.Sigma(bm.i_in,bm.i_in,i-1)); %finding the norm of hyperplane
                        [xd domain]=bm.GMR(d(bm.i_in,:),'training');
                        e1=bm.estimation_error(d(bm.i_out,:),xd);
                    end
                else
                    e1=+10^10; %considering a big value
                end
                
                if length(bm.b_ind)>=i+2
                    d = reshape(bm.Data(:,bm.b_ind(i):bm.b_ind(i+2)-1,:),bm.nbVar,[]);
                    bm.Mu=Mu_bk;bm.Sigma=Sigma_bk;bm.V=V_bk;
                    bm.Mu(:,i)=[];bm.Sigma(:,:,i)=[];bm.V(:,i)=[];
                    bm.Mu(:,i)=mean(d,2);
                    bm.Sigma(:,:,i)=cov(d')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                    bm.V(:,i)=bm.Compute_Norm(bm.Mu(bm.i_out,i),bm.Sigma(bm.i_in,bm.i_in,i)); %finding the norm of hyperplane
                    [xd domain2]=bm.GMR(d(bm.i_in,:),'training');
                    e2=bm.estimation_error(d(bm.i_out,:),xd);
                else
                    e2=+10^10; %considering a big value
                end
                
                %choosing the best option which causes less error
                if e1<e2 || (e1==e2 && length(bm.b_ind(i-1):bm.b_ind(i))<size(d,2)) %the second condition give priority to small gaussian in case the error are equal
                    d = reshape(bm.Data(:,bm.b_ind(i-1):bm.b_ind(i),:),bm.nbVar,[]);
                    bm.Mu=Mu_bk;bm.Sigma=Sigma_bk;bm.V=V_bk;
                    bm.Mu(:,i)=[];bm.Sigma(:,:,i)=[];bm.V(:,i)=[];
                    bm.Mu(:,i-1)=mean(d,2);
                    bm.Sigma(:,:,i-1)=cov(d')+diag([ones(1,bm.nbVar/2) zeros(1,bm.nbVar/2)]*bm.tol);
                    bm.V(:,i-1)=bm.Compute_Norm(bm.Mu(bm.i_out,i-1),bm.Sigma(bm.i_in,bm.i_in,i-1)); %finding the norm of hyperplane
                    bm.i_Omega(bm.i_Omega>i-1)=bm.i_Omega(bm.i_Omega>i-1)-1;
                    bm.i_Omega(:,bm.b_ind(i-1):bm.b_ind(i+1)-1)=reshape(domain,[],size(bm.i_Omega,1))';
                    bm.b_ind(i)=[];
                    if e1>bm.e_max
                        fprintf('Warning: Minimum Allowed Accuracy changed locally to %2.2f to avoid bad scaled Gaussian!\n',e1)
                    end
                    fprintf('Gaussians G%d and G%d merged into G%d -> Total # of Gaussian = %d\n',[i-1 i i-1 bm.K])
                else
                    bm.i_Omega(bm.i_Omega>i)=bm.i_Omega(bm.i_Omega>i)-1;
                    bm.i_Omega(:,bm.b_ind(i):bm.b_ind(i+2)-1)=reshape(domain2,[],size(bm.i_Omega,1))';
                    bm.b_ind(i+1)=[];
                    if e2>bm.e_max
                        fprintf('Warning: Minimum Allowed Accuracy changed locally to %2.2f to avoid bad scaled Gaussian!\n',e1)
                    end
                    fprintf('Gaussians G%d and G%d merged into G%d -> Total # of Gaussian = %d\n',[i i+1 i bm.K])
                end
                pSigma_ind=find(diff(bm.b_ind)==1);
            end
        end %Rem_Singular_Gaussian
        
%-------------------------------------------------------------------------%        
        function bm=find_delta(bm)
% bm = find_delta(bm) : This functions computes the threshold value delta
%           for each subdomain. delta is used to define the D domain. For a
%           point xi \in Omega_i, it belongs to D domain if p(xi) < delta(i)
%
% Output variable ---------------------------------------------------------
%   o bm:     An object with the class of BM
            
            for i=1:bm.K
                d_check=bm.Data(:,bm.i_Omega'==i); %finding all points belong to Omega_i
                if i==bm.K %these points only belong to one Gaussian
                    ind_neigh=i;
                else
                    ind_neigh=[i i+1];
                end
                for j=1:length(ind_neigh) %computing their probability
                    Pxi(:,j) = bm.gaussPDF(d_check(bm.i_in,:), bm.Mu(bm.i_in,ind_neigh(j)), bm.Sigma(bm.i_in,bm.i_in,ind_neigh(j)));
                end
                %We define delta such that it contains all datapoints with
                %the safety margins of '1.5^(bm.nbVar/2-1)'. We define this
                %safety margin flexible such that it a bit adapts itself
                %w.r.t. the dimensionality of datapoints.
                bm.delta(i)=min(sum(Pxi,2)/1.5^(bm.nbVar/2-1));
                clear Pxi
            end
            bm.delta(bm.delta==0)=realmin; %Correcting invalid delta values
        end %find_delta
    end %%methods = private
end % classdef