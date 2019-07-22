% Learning Nonlinear Dynamics with GMM: Learning a point to point motion from a set of demonstrations.
% This files includes the following variables:
%
% Variables ---------------------------------------------------------------
%   o folder:        Folder containing files with training data, e.g.
%                    folder = 'data/'
%   o file:          Name prexif of the files containing trainig data (e.g.,
%                    if a file with demonstration has a name 'demo1.csv',
%                    then file ='demo');
%   o nbFiles:       Number of files
%   o in:            Input Dimensions
%   o out:           Output Dimensions
%   o ts:            Time step
%   o Mu:            Matrix representing center of Gaussians ()
%   o Sigma:         Matrix representing the covariance of Gaussians
%   o Priors:        Array representing trained priors of Gaussians
%   o nbStates:      Number of Gaussians in GMM representation
%   o tol:           Tolerance is used to avoid singularity in Sigma [default=10^-6]
%   o x_tar:         Variable that stores an original attractor's position,
%                    before translating it to the origin
%   o structGMM:     Structure containig a trained GMM model
%
% Copyright(c) 2009 Elena Gribovskaya, LASA Lab, EPFL, Lausanne,
%               Switzerland, http://lasa.epfl.ch/elena
%
% The program is free for non-commercial academic use. The software must
% not be modified or distributed without prior permission of the authors.

function structGMM = TrainGMMDynamics(folder, file, nbFiles, in, out, ts)

load([folder file]);

smooth_flag = true;
smoothing_factor = 30;

nbData = 100;
nbStates = 5;
nbSteps = 100;
nbDim = size(Data, 1);
time_span = 100;
tol = 10^-1;
nbFiles = 5;
%%--for testing purposes---------------------------------------------------

ts = 0.01;

if(smooth_flag)
    for (i = 1: nbFiles)
        for(j = 1 : size(in, 2))
            Data(j, (i-1) * nbData + 1 : i * nbData)  = smooth(Data(j, (i-1) * nbData + 1 : i * nbData), smoothing_factor);
        end;
    end;
end;

Data = [Data; zeros(size(out, 2), size(Data, 2))];
for (i = 1 : nbFiles)
    
    Data(out, (i-1) * nbData + 1 : i * nbData) = ...
        [Data(in, (i-1) * nbData + 2) - Data(in, (i-1) * nbData + 1) ...
        Data(in, (i-1) * nbData + 2 : i * nbData) - Data(in, (i-1) * nbData + 1 : i * nbData - 1)] ./ ts;
end;
DataSize = nbData * ones(1, nbFiles);
%%------------------------------------------------------------------------
% %% %%%%%%%%Prepare Data%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% uncomment and modify according to the format of input data---------------
normData = [];
x_tar = 0;
for (i = 1 : nbFiles)
    
    if (Data(:, i * nbData) ~= zeros(size(Data, 1), 1))
        x_tar = Data(:, i * nbData) + x_tar;
    
        x = Data(:, (i-1) * nbData + 1 : i * nbData) - repmat(Data(:, i * nbData), 1, nbData);
   
        normData = [normData x(:, 1:nbData)];

    end;
end;

x_tar = x_tar ./ nbFiles;

disp 'COMPUTING SYNTHETIC DATA AT THE ORIGIN...';

%%--------------Add Synthetic Data, restimate last gaussian---------%%%
[Mu_last Sigma_last] = ComputeSyntheticData(Data, in, out);
%%------------------------------------------------------------------%%%
%%----for testing purposes---------------------------------------------%%%%
%szData = 100 * ones(1, nbFiles);
%%-------------------------------------------------------------------------
stability = false;
 
disp 'TRAINING THE GMM MODEL....';
%%------------------Training GMM model---------------------------------%%%%
while ~stability && nbStates < 15
    
    %%-----------------Initialize EM-----------------------------------%%%%

    [Priors, Mu, Sigma] = EM_init_regularMeshAmplitude(Data, DataSize, nbStates);
    [Priors, Mu, Sigma, nbStep, Pix] = EM(Data, Priors, Mu, Sigma);
    

    %%----Substitute last component with Synthetic Gaussians----------%%%%%
    Mu(:, end) = Mu_last;
    Sigma(:, :, end) = Sigma_last;
   
    %% --------------Rough stability verification---------------------%%%%%%
    %%--------Integrate forward the GMM estimation starting from training--%
    %%--------data and verify whether converge----------------------%%%%%%%
    for (i = 1 : nbFiles)
        
        [T,Y] = ode15s(@GMR_ode,[0 time_span], Data(in, sum(DataSize(1, 1:i-1)) + 1) , ...
            [], Priors, Mu, Sigma, in, out);    
        
         stability = norm(Y(end, :)) < tol;
         
         if (~stability)
             break;
         end;
    end;
     
    if (~stability)
         disp (['nbStates=' int2str(nbStates) '; a stable model is not found, continue...']);
         nbStates = nbStates + 1;
    end;
  
    %%---------------------------------------------------------------%%%%%%
    %%
     
end;    

%%---------if a stable solution is found-----------------------------%%%%%%
if (stability)
    
    disp(['Stable solution is found; nbStates =' int2str(nbStates) '; the model is stored in the file']);
    disp 'STORING AND VISUALIZING...';
%%--- uncomment together with data preparation section---------------------  
%    structGMM.tar = x_tar;
    structGMM.Mu  = Mu;
    structGMM.Sigma = Sigma;
    structGMM.Priors = Priors;
    structGMM.nbStates = nbStates;
    structGMM.Data = Data;
    structGMM.x_tar = x_tar;
    structGMM.DataSize = DataSize;
    save ([folder file '_learnedDynamics.mat'], 'structGMM');

%% %%%%%%%%%%%%%%%Plot GMM%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%-----------------Generate sample trajectory-------------------%%%%%%%
    v  = [];
    x = [];
    v(:, 1) = zeros(size(out, 2), 1); 
    x(:, 1) = Data(in, 1);
    
    for (i = 2 : nbData)
        v(:, i) = GMR(Priors, Mu, Sigma, x(:, i-1), in, out); 
        x(:, i) = x(:, i-1) + v(:, i) * ts;
    end;
    %%-------------------------------------------------------------%%%%%%%%
    figure;
    k = 1;
    for (i = 1 : size(in, 2)-1)
        for (j = i+1 : size(in, 2))
            
            subplot(2, 7, k); hold on; grid on;
            plotGMM(Mu([i,j], :), Sigma([i,j], [i,j],:), [0 .8 0], 1);
            for (i1 = 1: nbFiles)
              plot(Data(i, sum(DataSize(1, 1:i1-1)) + 1 : sum(DataSize(1, 1:i1))), Data(j, sum(DataSize(1, 1:i1-1)) + 1 : sum(DataSize(1, 1:i1))));
            end;
            plot(x(i, :), x(j, :), 'LineWidth', 1.5, 'markersize', 18, 'color', [0 0 0]);
            xlabel(['$x_' num2str(i) '$'],'interpreter','latex','fontsize',18); 
            ylabel(['$x_' num2str(j) '$'],'interpreter','latex','fontsize',18);

            k = k + 1;
        end;
    end;
    
    for (i = 1 : size(in, 2))
     
            
            subplot(2, 7, k); hold on; grid on;
            plotGMM(Mu([i, i + size(in, 2)], :), Sigma([i,i + size(in, 2)], [i,i + size(in, 2)],:), [0 .8 0], 1);
            for (i1 = 1: nbFiles)
              plot(Data(i, sum(DataSize(1, 1:i1-1)) + 1 : sum(DataSize(1, 1:i1))), Data(i + size(in, 2), sum(DataSize(1, 1:i1-1)) + 1 : sum(DataSize(1, 1:i1))));
            end;
            plot(x(i, :), v(i, :), 'LineWidth', 1.5, 'markersize', 18, 'color', [0 0 0]);
            xlabel(['$x_' num2str(i) '$'],'interpreter','latex','fontsize',18); 
            ylabel(['$\dot{x}_' num2str(i) '$'],'interpreter','latex','fontsize',18);

            k = k + 1;
       
    end;
    
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
else
    disp 'Stable solution is not found. Check training data'
end;
