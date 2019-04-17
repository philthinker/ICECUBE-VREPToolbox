% DMP3dDemo
% Haopeng Hu
% 2019.03.27

% A demo for @DMPGroup

%% Traj generation

% % You can also load 'Data\SimpleMinJerk3.mat'
% anchors = [0,0,0; 1,1,1; 2,0.5,2; 2,2,3];
% velocities = [0,0,0; 1,0,1; 0,0,1; 0,0,0];
% M = 1000; K = size(anchors,1)-1;
% T = zeros(M*K,3); dT = zeros(M*K,3); ddT = zeros(M*K,3);
% for i = 2:K+1
%     [T((i-2)*M+1:(i-1)*M,:),dT((i-2)*M+1:(i-1)*M,:),ddT((i-2)*M+1:(i-1)*M,:)] = ...
%         jtraj(anchors(i-1,:),anchors(i,:),M,velocities(i-1,:),velocities(i,:));
% end

%% Initialize and run empty DMPs

% dt = 0.001;
% dmp = DMPGroup(IjspeertDMP(6,25,5,40,dt),3,dt);
% [Y,x,fx] = dmp.run(0,1,1);
% tau = 1;
% dmp.plot(x,Y(:,1:3),tau);
% dmp.plotGaussian(x,fx(:,1),tau);

%% Learn a 3DoF trajectory

% % Never forget to run the last section before.
% target = zeros(size(T,1),size(T,2)*3);  % [t1,dt1,ddt1,t2,dt2,ddt2,...]
% tau = 1;
% for i = 1:3
%     target(:,(i-1)*3+1) = T(:,i);
%     target(:,(i-1)*3+2) = dT(:,i);
%     target(:,(i-1)*3+3) = ddT(:,i);
% end
% dmp = dmp.learn(target,tau);

%% Run the learned 3DoF DMP

% % Never forget to run the last section before.
% y0 = [0,0,0]; g = [2,2,3]; tau = 1;
% [Y,x,fx] = dmp.run(y0,g,tau);
% dmp.plot(x, Y(:,1:3),tau);
% dmp.plot3([Y(:,1),Y(:,4),Y(:,7)]);
