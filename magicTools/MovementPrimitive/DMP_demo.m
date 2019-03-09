%   DMP_demo
%   Haopeng Hu
%   2019.02.26

%   You can just follow the blog:
%   https://studywolf.wordpress.com/2013/11/16/dynamic-movement-primitives-part-1-the-basics/
%   It is helpful, but the 'tau' in the blog is inverse of the tau here.

%% Just run a DMP

% % Initialize a DMP
% N = 10;
% dmp = DMP(25,5,8,N);
% 
% % Initialize the Gaussian kernels (Not recommended)
% % h = 100*ones(N,1);      % variances
% % c = linspace(0.1,1,N)'; % centers
% % dmp = dmp.initGaussian(h,c);
% 
% % Set the goal and origin
% dmp = dmp.setGoalOrigin(1,0);
% % Run it
% tau = 1;   % Duration
% [y,dy,ddy,x,dx,f] = dmp.run(tau);
% % Show the plots
% dmp.plot(x,y,dy,ddy,tau);
% dmp.plotGaussian(x,f,tau);

%% Run a DMP with manual weights

% % Initialize the necessary parameters
% tau = 1; N = 10; c = linspace(tau/N,tau,N)'; h = 10*N./c;
% % Initialize the DMP
% dmp = DMP(25,5,10,N);
% dmp = dmp.initGaussian(h,c);
% dmp = dmp.setGoalOrigin(10,0);
% % Assign the weights
% w = 100 * randn(size(h));
% dmp = dmp.assignWeights(w);
% % Run it
% [y,dy,ddy,x,dx,f] = dmp.run(tau);
% % Show the plots
% dmp.plot(x,y,dy,ddy,tau);
% dmp.plotGaussian(x,f,tau);

%% Run 3 DMP to formulate a 3D trajectory with manual weigths

% % Initialize the necessary parameters
% tau = 1; N = 100; c = linspace(tau/N,tau,N)'; h = 10*N./c; w = 200 * randn(size(h));
% % Initialize the DMPs
% dmps = cell(3,1);
% for i = 1:3
%     dmps{i} = DMP(25,5,10,N);
%     dmps{i} = dmps{i}.initGaussian(h,c);
%     dmps{i} = dmps{i}.setGoalOrigin(10,0);
%     dmps{i} = dmps{i}.assignWeights(w);
% end
% % Run them
% trajx = dmps{1}.run(tau);
% trajy = dmps{2}.run(tau);
% trajz = dmps{3}.run(tau);
% 
% % Show the plots
% dmps{1}.plot3d([trajx,trajy,trajz]);

%% Learn a 1D trajectory

% % Initialize a minimal jerk trajectory
% load('Data\SimpleMinJerk.mat')
dmp = DMP(25,5,5,50);  % alpha, beta, alphax, N
% Learn a trajectory
dmp = dmp.BatchLearnLWR(T(:,1),T(:,2),T(:,3));
% Run it
tau = 1;
[y,dy,ddy,x,dx,f] = dmp.run(tau);
% % Show the plots
% dmp.plot(x,y,dy,ddy,tau);
% dmp.plotGaussian(x,f,tau);
% Comparison
dmp.plotComparison(y,dy,ddy,T(:,1),T(:,2),T(:,3),tau);