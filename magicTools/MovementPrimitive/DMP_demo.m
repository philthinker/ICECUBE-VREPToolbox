%   DMP_demo
%   Haopeng Hu
%   2019.02.26

%   You can just follow the blog:
%   https://studywolf.wordpress.com/2013/11/16/dynamic-movement-primitives-part-1-the-basics/
%   It is pretty helpful, but the 'tau' in the blog is inverse of the tau
%   here, which is the duration here.

%% Just run a DMP

% % Initialize a DMP
% N = 10;
% dmp = DMP(25,5,10,N);
% % Initialize the Gaussian kernels
% h = 100*ones(N,1);      % variances
% c = linspace(0.1,1,N)'; % centers
% dmp = dmp.initGaussian(h,c);
% % Set the goal and origin
% dmp = dmp.setGoalOrigin(1,0);
% % Run it
% tau = 1;   % Duration
% [y,dy,ddy,x,dx,f] = dmp.run(tau);
% % Show the plots
% dmp.plot(x,y,dy,ddy,tau);
% dmp.plotGaussian(x,f,tau);

%% Run a DMP with manual weights

% Initialize the necessary parameters
tau = 1; N = 10; c = linspace(tau/N,tau,N)'; h = 10*N./c;
% Initialize the DMP
dmp = DMP(25,5,10,N);
dmp = dmp.initGaussian(h,c);
dmp = dmp.setGoalOrigin(10,0);
% Assign the weights
w = 100 * randn(size(h));
dmp = dmp.assignWeights(w);
% Run it
[y,dy,ddy,x,dx,f] = dmp.run(tau);
% Show the plots
dmp.plot(x,y,dy,ddy,tau);
dmp.plotGaussian(x,f,tau);
