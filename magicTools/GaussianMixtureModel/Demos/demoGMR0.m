%demoGMR0 A simple demo demonstrating the use of GMM and GMR
%
% Haopeng Hu
% 2019.07.30
% All rights reserved
%

load('Data/LetterC.mat');

%% Data initialization

nSamples = 10;
dt = 1e-3;
nData = size(demos{1}.pos,2);   % Only position considered
dData = size(demos{1}.pos,1);
Data = zeros(dData,nSamples*nData);
Demos = cell(1,nSamples);
for i = 1:nSamples
    Data(:,(i-1)*nData+1:i*nData) = demos{i}.pos;   % Not used in Parameter estimation but in Figure
    Demos{i} = [(1:nData)'.*dt,(demos{i}.pos)'];    % Directly used in Parameter estimation, the 1st column is time
end

%% GMM initialization

gmm = GMMZero(7,dData+1,dt);

%% Parameters estimation

gmm = gmm.initGMMKMeans(Demos);
gmm = gmm.learnGMM(Demos);

%% Gaussian Mixture Regression

[expData, expSigma] = gmm.GMR((1:nData)'.*gmm.dt);

%% Figure

figure('position',[10,10,1300,500]); 
%Plot GMM
subplot(1,2,1); hold on; axis off; title('GMM');
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.5 .5 .5]);
gmm.plotGMM2GMR(gmm.Mu(:,2:gmm.nVar), gmm.Sigma(2:gmm.nVar,2:gmm.nVar,:), [.8 0 0], .5);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
%Plot GMR
subplot(1,2,2); hold on; axis off; title('GMR');
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.5 .5 .5]);
gmm.plotGMM2GMR(expData, expSigma, [0 .8 0], .03);
plot(expData(:,1),expData(:,2),'-','linewidth',2,'color',[0 .4 0]);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
