%%------------------------Compute Synthetic Data------------------------%%
% Compute Synthetic Data and Parameters of a Gaussian located on the target
%
% Variables ---------------------------------------------------------------

%   o Data:          Original training set
%   o in:            Input Dimensions
%   o out:           Output Dimensions
%   o Mu:            Zero vector representing center of the last Gaussians
%   o Sigma:         Matrix representing the covariance of the last Gaussians
function [Mu Sigma] = ComputeSyntheticData(Data, in, out)
            
            nbVar = size(Data, 1);
            tol = 10^-7;
            
            % all data inside a sphere of radius r is used for the last Gaussian
            r=norm(std(Data(in, :), 0, 2) / 5); 
            
            %finding data within the radius r
            ind = find(Mat_Vec_Norm(Data(in,:))<r);%, 1, 'last' ); %indices of the points that will be used for the last gaussian
        
            
            %ind=1:max([min(ind) 10]); 
            
            % this variable is used to keep the data of the last gaussian
            Data_1 = Data(:,ind);
            
            %Create the Gaussian. We makes the data symmetric w.r.t. the
            %origin. Thus, mean of the Gaussian always is on the target (i.e. origin)
            Sigma_1 = cov([Data_1 -Data_1]') + diag([ones(1, nbVar)] * tol);
            
            %If the resultant Gaussian is not stable, we remove the outer
            %datapoints up to the point that it becomes stable.
            while ~all(eig(Sigma_1(out, in) / Sigma_1(in, in)) < 0) && length(ind) > 1
                ind(end) = [];
                Data_1 = Data(:, ind); % this variable is used to keep the data of the last gaussian
                Sigma_1=cov([Data_1 -Data_1]') + diag([ones(1,nbVar/2) zeros(1,nbVar/2)]* tol);
            end
            
            Mu = zeros(nbVar,1);
            Sigma = Sigma_1;
%%----------------------------------------------------------------------%%%         
end
%%----------------------------------------------------------------------%%%
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