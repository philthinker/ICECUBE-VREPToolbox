% Motion Planning in the task space
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

function ManipulationPlanning()
close all

if isempty(regexp(path,['plotting' pathsep]))
 addpath('plotting'); 
end

folder      = 'data/i-cub/'
posFile     = 'Data_end';
orientFile  = 'Data_AxisAngle';

options.retrain = false;

nbData = 100;

nbFiles = 5;
ts = 0.01;

%%%%%------------------------Training-------------------------------%%%%%%
disp 'Checking whether the trained models exist....'
disp ''
if ~(exist([folder posFile '_learnedDynamics.mat'],'file')) 
    disp 'A trained model for position control exists';
end;
if (options.retrain)
    disp ' '
    disp 'the model will be retrained according to chosen options'
end;

if ~(exist([folder posFile '_learnedDynamics.mat'],'file')) || options.retrain
    disp 'TRAINING POSITION CONTROLLER'
    disp ' ' 
    structGMMPos = TrainGMMDynamics(folder, posFile, nbFiles, [1:3], [4:6], ts);

else
    
    load([folder posFile '_learnedDynamics.mat'])
    structGMMPos = structGMM;

end;

if ~(exist([folder orientFile '_learnedDynamics.mat'],'file')) 
    
    disp 'A trained model for orientation control exists';
end;
if (options.retrain)
    disp ' '
    disp 'the model will be retrained according to chosen options'
end;

if ~(exist([folder orientFile '_learnedDynamics.mat'],'file')) || options.retrain
    disp 'TRAINING ORIENTATION CONTROLLER'
    disp ' '
    structGMMOrient = TrainGMMDynamics(folder, orientFile, nbFiles, [1:4], [5:8], ts);
    
else
    load([folder orientFile '_learnedDynamics.mat']);
    structGMMOrient = structGMM;
end;

    

%%-----------------Reproduction-------------------------------------%%%%%%%
    disp 'Reproduction is started from one of training data....'
    v  = [];
    x = [];
    
    v(:, 1)     =  zeros(3, 1); 
    x(:, 1)     =  structGMMPos.Data(1:3, 1);
    
    o           = [];
    v_o         = [];
    
   
    v_o(:, 1)   = zeros(4, 1);
    o(:, 1)     = structGMMOrient.Data(1:4, 1);
    
    h_fig       = figure;
    hold on;
    grid on;
    
    disp 'target is highlighted with a cube'
    
    DrawCube([0 0 0], [1; 0; 0], [0; 0; 1], h_fig);
    %%--------------------Motion Generation------------------------%%%%%%%%
    for (i = 2 : nbData)
        
      
            
            v(:, i) = GMR(structGMMPos.Priors, structGMMPos.Mu,...
                structGMMPos.Sigma, x(:, i-1), [1:3], [4:6]);
            
     
        
       
            
            v_o(:, i) = GMR(structGMMOrient.Priors, structGMMOrient.Mu,...
                structGMMOrient.Sigma, [o(:, i-1)], [1:4], [5:8]);
            
      
      
             x(:, i) = x(:, i-1) + v(:, i) * ts;
        
             o(:, i) = o(:, i-1) + v_o(:, i) * ts;
        
             plotRotation(o(:, i), x(:, i), h_fig, structGMMOrient.x_tar);
        
    end;
    
    %%----------------Plot Position Planning-----------------------%%%%%%%%
    disp 'reproduction is finished; plotting trajectories'
    figure;
    k = 1;
    for (i = 1 : 2)
        for (j = i+1 : 3)
            
            subplot(3, 4, k); hold on; grid on;
            plotGMM(structGMMPos.Mu([i,j], :), structGMMPos.Sigma([i,j], [i,j],:), [0 .8 0], 1);
            for (i1 = 1: nbFiles)
              plot(structGMMPos.Data(i, sum(structGMMPos.DataSize(1, 1:i1-1)) + 1 : sum(structGMMPos.DataSize(1, 1:i1))), ...
                  structGMMPos.Data(j, sum(structGMMPos.DataSize(1, 1:i1-1)) + 1 : sum(structGMMPos.DataSize(1, 1:i1))));
            end;
            plot(x(i, :), x(j, :), 'LineWidth', 1.5, 'markersize', 18, 'color', [0 0 0]);
            xlabel(['$x_' num2str(i) '$'],'interpreter','latex','fontsize',18); 
            ylabel(['$x_' num2str(j) '$'],'interpreter','latex','fontsize',18);

            k = k + 1;
        end;
    end;
    
    for (i = 1 : 3)
     
            
            subplot(3, 4, k); hold on; grid on;
            plotGMM(structGMMPos.Mu([i, i + 3], :), structGMMPos.Sigma([i,i + 3], [i,i + 3],:), [0 .8 0], 1);
            for (i1 = 1: nbFiles)
              plot(structGMMPos.Data(i, sum(structGMMPos.DataSize(1, 1:i1-1)) + 1 : sum(structGMMPos.DataSize(1, 1:i1))), ...
                  structGMMPos.Data(i + 3, sum(structGMMPos.DataSize(1, 1:i1-1)) + 1 : sum(structGMMPos.DataSize(1, 1:i1))));
            end;
            plot(x(i, :), v(i, :), 'LineWidth', 1.5, 'markersize', 18, 'color', [0 0 0]);
            xlabel(['$x_' num2str(i) '$'],'interpreter','latex','fontsize',18); 
            ylabel(['$\dot{x}_' num2str(i) '$'],'interpreter','latex','fontsize',18);

            k = k + 1;
       
    end;
    %%%---------------------Plot Orientation Control-----------------%%%%%%
    figure;
    k = 1;
    for (i = 1 : 3)
        for (j = i+1 : 4)
            
            subplot(3, 4, k); hold on; grid on;
            plotGMM(structGMMOrient.Mu([i,j], :), structGMMOrient.Sigma([i,j], [i,j],:), [0 .8 0], 1);
            for (i1 = 1: nbFiles)
              plot(structGMMOrient.Data(i, sum(structGMMOrient.DataSize(1, 1:i1-1)) + 1 : sum(structGMMOrient.DataSize(1, 1:i1))), ...
                  structGMMOrient.Data(j, sum(structGMMOrient.DataSize(1, 1:i1-1)) + 1 : sum(structGMMOrient.DataSize(1, 1:i1))));
            end;
            plot(o(i, :), o(j, :), 'LineWidth', 1.5, 'markersize', 18, 'color', [0 0 0]);
            xlabel(['$o_' num2str(i) '$'],'interpreter','latex','fontsize',18); 
            ylabel(['$o_' num2str(j) '$'],'interpreter','latex','fontsize',18);

            k = k + 1;
        end;
    end;
    
    for (i = 1 : 4)
     
            
            subplot(3, 4, k); hold on; grid on;
            plotGMM(structGMMOrient.Mu([i, i + 4], :), structGMMOrient.Sigma([i,i + 4], [i,i + 4],:), [0 .8 0], 1);
            for (i1 = 1: nbFiles)
              plot(structGMMOrient.Data(i, sum(structGMMOrient.DataSize(1, 1:i1-1)) + 1 : sum(structGMMOrient.DataSize(1, 1:i1))), ...
                  structGMMOrient.Data(i + 4, sum(structGMMOrient.DataSize(1, 1:i1-1)) + 1 : sum(structGMMOrient.DataSize(1, 1:i1))));
            end;
            plot(o(i, :), v_o(i, :), 'LineWidth', 1.5, 'markersize', 18, 'color', [0 0 0]);
            xlabel(['$o_' num2str(i) '$'],'interpreter','latex','fontsize',18); 
            ylabel(['$\dot{o}_' num2str(i) '$'],'interpreter','latex','fontsize',18);

            k = k + 1;
       
    end;
    
    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

