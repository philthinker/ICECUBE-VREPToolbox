function demo1
%
% Demonstration of a method based on Gaussian Mixture Regression (GMR) and
% mass-spring-damper system to learn and reproduce the dynamics of a set 
% of motions. 
% This source code is the implementation of the algorithms described in 
% Section 5.1, p.130 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% The program first encapsulates the motion dynamics in a Gaussian Mixture
% Model (GMM) estimating locally the joint distribution P(x,dx) of
% position 'x' and velocity 'dx' variables. 
% Gaussian Mixture Regression (GMR) is then used to retrieve at each 
% iteration a desired position 'X' and desired velocity 'DX' by knowing 
% the current position and velocity of the system. A mass-spring damper sytem 
% (impedance controller) of the form 'ddx*M+(dx-DX)*B+(x-X)*K=0' is then used 
% to reach for the desired position and desired velocity, which allows to 
% folllow the learned dynamics while coming back to desired trajecotry in 
% case of perturbations.
%
% This source code is given for free! However, I would be grateful if you refer 
% to the book (or corresponding article) in any academic publication that uses 
% this code or part of it. Here are the corresponding BibTex references: 
%
% @book{Calinon09book,
%   author="S. Calinon",
%   title="Robot Programming by Demonstration: A Probabilistic Approach",
%   publisher="EPFL/CRC Press",
%   year="2009",
%   note="EPFL Press ISBN 978-2-940222-31-5, CRC Press ISBN 978-1-4398-0867-2"
% }
% 
% @inproceedings{Calinon09ICAR,
% 	author = "S. Calinon and P. Evrard and E. Gribovskaya and A. Billard and A. Kheddar",
%	title = "Learning collaborative manipulation tasks by demonstration using a haptic interface",
%	booktitle = "Proc. Intl Conf. on Advanced Robotics ({ICAR})",
%	year = "2009",
%	month="June",
%	location="Munich, Germany"
% }

%% Definition of the number of components used in GMM.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbStates = 7; %Number of states in the GMM
nbSamples = 3; %Number of demonstrations
nbData = 100; %Number of points in each demonstration
nbDataRepr = 200; %Number of points retrieved for the reproduction
kV = 1.0; %Gain for the velocity target
kP = 0.16; %Gain for the position target

%% Load a dataset consisting of 3 demonstrations of trajectories of
%% 100 points each, where each point has 4 dimensions (position and 
%% velocity in 2D space.
%% 'Data' is thus of size 4x300. Data(1:2,:) represents the positions 
%% and Data(3:4,:) represents the velocity components.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/data.mat'); %load 'Data'

%% Training of the joint distribution P(x,dx) in a Gaussian Mixture 
%% Model (GMM) through Expectation-Maximization (EM) algorithm.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Estimation of GMM encoding P(x,dx)...');
%The GMM is initialized with k-means clustering
[Priors, Mu, Sigma] = EM_init_kmeans(Data, nbStates);
%EM is then refining the model
[Priors, Mu, Sigma] = EM(Data, Priors, Mu, Sigma);

%% Robust reproduction of the motion by combining a controller following
%% the desired dynamics with an attractor to the demonstrated trajectory.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Robust reproduction of the motion through GMR...');
currPos = [-0.04;-0.08]; %Current position (initialization)
currVel = [0; 0]; %Current velocity (initialization)
currAcc = [0; 0]; %Current acceleration (initialization)
%Reproduction loop
for i=1:nbDataRepr
  %Keep trace of the motion
  reprData(:,i) = currPos;
  %Compute the influence of each Gaussian
  for j=1:nbStates
    h(j) = gaussPDF([currPos;currVel], Mu(1:4,j), Sigma(1:4,1:4,j));
  end
  h = h./sum(h);
  %Compute the desired position and desired velocity through GMR
  targetPos=[0;0]; targetVel=[0;0];
  for j=1:nbStates
    targetPos = targetPos + h(j) .* (Mu(1:2,j) + ...
      Sigma(1:2,3:4,j)*inv(Sigma(3:4,3:4,j)) * (currVel-Mu(3:4,j)));
    targetVel = targetVel + h(j) .* (Mu(3:4,j) + ...
      Sigma(3:4,1:2,j)*inv(Sigma(1:2,1:2,j)) * (currPos-Mu(1:2,j)));
  end
  %Acceleration defined by mass-spring-damper system (impedance controller)
  currAcc = (targetVel-currVel).*kV + (targetPos-currPos).*kP;
  %Update velocity
  currVel = currVel + currAcc;
  %Update position
  currPos = currPos + currVel;
end

%% Plot of the results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; box on; hold on; 
%Plot of the GMM position encoding results
plotGMM(Mu(1:2,:), Sigma(1:2,1:2,:), [.2 .8 .2], 1);
%Plot of the position data
plot(Data(1,:), Data(2,:),'x','lineWidth',1,'color',[0 0 0]);
%Plot of the retrieved data
plot(reprData(1,:), reprData(2,:),'-','lineWidth',2,'color',[.8 .2 .2]);
axis equal;
xlabel('x_1'); ylabel('x_2');

pause;
close all;
