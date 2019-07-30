%demoGMM0 A simple GMM demo demonstrating the use of GMMZero
% 
% Haopeng Hu
% 2019.07.30
% All rights reserved
%

load('Data/LetterC.mat');

%% Data initialization

nSamples = 6;
nData = size(demos{1}.pos,2);   % Only position considered
dData = size(demos{1}.pos,1);
Data = zeros(dData,nSamples*nData);
Demos = cell(1,nSamples);
for i = 1:nSamples
    Data(:,(i-1)*nData+1:i*nData) = demos{i}.pos;   % Not used in Parameter estimation but in Figure
    Demos{i} = (demos{i}.pos)';                     % Directly used in Parameter estimation
end

%% GMM initialization

gmm = GMMZero(5,dData);

%% Parameters estimation

gmm = gmm.initGMMKMeans(Demos);
gmm = gmm.learnGMM(Demos);

%% Figure

figure('position',[10,10,700,500]); hold on; axis off;
plot(Data(1,:),Data(2,:),'.','markersize',8,'color',[.5 .5 .5]);
gmm.plotGMM2SC( [.8 0 0],.5);
axis equal; set(gca,'Xtick',[]); set(gca,'Ytick',[]);
