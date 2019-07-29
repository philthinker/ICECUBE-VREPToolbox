% SchaalDMPDemo
% Haopeng Hu
% 2019.03.23

%% Run an empty DMP

% % alphax>0, D>0, K>0, K<D^2/4
% dmp = SchaalDMP(6,600,50,40,0.005);
% tau = 1;
% [Y,x,fx] = dmp.run(3,9,tau);
% dmp.plot(x,Y,tau);
% dmp.plotGaussian(x,fx,tau);

%% Learn a minimal jerk trajectory

% % load('Data\SimpleMinJerk.mat');
% dmp = SchaalDMP(6,600,50,40,0.005);
% dmp = dmp.LWR(T,1);
% [Y,x,fx] = dmp.run(T(1,1),T(end,1),1);
% dmp.plot(x,Y,1);
% dmp.plotGaussian(x,fx,1);
% dmp.plotCompare(Y,T,1);

%% Run a learned DMP

% % We use the DMP learned in the last section
% tau = 2;
% [Y,x,fx] = dmp.run(1,5,tau);
% dmp.plot(x,Y,tau);
% dmp.plotGaussian(x,fx,tau);
