function demo1
%
% Demonstration of the use of Gaussian Mixture Regression (GMR) and 
% inverse kinematics to reproduce a task by considering constraints both 
% in joint space and in task space (by considering positions only). 
% This source code is the implementation of the algorithms described in 
% Section 4.1.2, p.104 of the book "Robot Programming by Demonstration: A 
% Probabilistic Approach". 
%
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% The programs shows a robotic arm composed of 2 links and moving in 
% 2D space. Several demonstrations of a skill are provided, by 
% starting from different initial positions. The skill consists of moving 
% each joint sequentially and then writing the alphabet letter 'N' at a 
% specific position in the 2D space. 
% Constraints in joint space and in task space are represented
% through Gaussian Mixture Models (GMMs) and Gaussian Mixture Regression 
% (GMR). By using an inverse kinematics process based on a pseudo-inverse 
% Jacobian, the constraints in task space are then projected in joint 
% space. By considering the projected constraints with the ones originally 
% encoded in joint space, an optimal controller is found for the 
% reproduction of the task. We see through this example that the system is
% able to generalize the learned skill to new robotic arms (different links
% lengths) and to new initial positions of the robot.
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
% @inproceedings{Calinon08IROS,
%   author="S. Calinon and A. Billard",
%   title="A Probabilistic Programming by Demonstration Framework Handling Constraints in Joint Space and Task Space",
%   booktitle="Proc. {IEEE/RSJ} Intl Conf. on Intelligent Robots and Systems ({IROS})",
%   year = "2008",
%   month="September",
%   location="Nice, France"
%   pages="367--372"
% }

%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 1000; %Length of each trajectory
nbSamples = 3; %Number of demonstrations
nbStatesX = 5; %Number of states in the GMM for the constraints in task space
nbStatesT = 5; %Number of states in the GMM for the constraints in joint space
linksLength_demo = [6; 6]; %Length of the two arm's links used for demonstrations
linksLength_repro = [5; 7]; %Length of the two arm's links used for reproduction
pos_demo = [1 4; 3 1; 3 3]'; %Initial positions of the arm for the demonstrations
pos_repro = [3 2]'; %Initial positions of the arm for the reproduction

%% Compute the Jacobian matrix and its inverse
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Analytic computation of the Jacobian by using symbolic equations
syms L1 L2 %Arm links length 
syms t t1 t2 %Joint angles
syms posx posy %Initial position
syms x %Position of the end effector
syms J JX %Jacobian matrix and its inverse
syms R T %Rotation matrices and position vectors
%Forward kinematics
t = [t1;t2]; %Vector of joint angles
T(1:2,1) = [posx;posy]; %Position of the first link
R(1:2,1:2,1) = [cos(t1) -sin(t1); sin(t1) cos(t1)]; %Orientation of the first link
T(1:2,2) = T(1:2,1) + R(1:2,1:2,1) * [L1;0]; %Position of the second link
R(1:2,1:2,2) = [cos(t2) -sin(t2); sin(t2) cos(t2)]; %Orientation of the second link
x = T(1:2,2) + R(1:2,1:2,2) * [L2;0]; %Position of the end effector
%Computation of the Jacobian matrix
for i=1:2
  for j=1:2
    J(i,j) = diff(x(i),t(j));
  end
end
%Inversion of the Jacobian matrix
if size(J,1)==size(J,2)
  JX = inv(J);
else
  JX = pinv(J);
end

%% Convert the symbolic expressions into m files (results are faster than 
%% when using the 'subs' command from Matlab) 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create the 'subs_J' function
fid = fopen('subs_JX.m', 'wt');
fprintf(fid,'function JX = subs_JX(t1,t2,L1,L2)\n\n');
for i=1:2
  for j=1:2
    fprintf(fid,'%s',['JX(' num2str(i) ',' num2str(j) ') = ']);
    fprintf(fid,'%s',char(vectorize(JX(i,j))));
    fprintf(fid,';\n');
  end
  fprintf(fid,'\n');
end
fclose(fid);
%Create the 'subs_x' function
fid = fopen('subs_x.m', 'wt');
fprintf(fid,'function x = subs_x(t1,t2,L1,L2,posx,posy)\n\n');
for i=1:2
  fprintf(fid,'%s',['x(' num2str(i) ') = ']);
  fprintf(fid,'%s',char(vectorize(x(i))));
  fprintf(fid,';\n');
end
fclose(fid);

% Load data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('data/data1.mat'); %Load DataT (in joint space) and DataX (in task space)
nbVarX = size(DataX,1); 
nbVarT = size(DataT,1);

%% Training of GMM by EM algorithm, initialized by k-means clustering
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[PriorsT, MuT, SigmaT] = EM_init_regularTiming(DataT, nbStatesT);
[PriorsX, MuX, SigmaX] = EM_init_regularTiming(DataX, nbStatesX);
[PriorsT, MuT, SigmaT] = EM(DataT, PriorsT, MuT, SigmaT);
[PriorsX, MuX, SigmaX] = EM(DataX, PriorsX, MuX, SigmaX);

%% Retrieval of generalized trajecotries through GMR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
expT(1,:) = linspace(min(DataT(1,:)),max(DataT(1,:)),nbData);
expX(1,:) = linspace(min(DataX(1,:)),max(DataX(1,:)),nbData);
[expT(2:nbVarT,:), expSigmaT] = GMR(PriorsT,MuT,SigmaT,expT(1,:),[1],[2:nbVarT]);
[expX(2:nbVarX,:), expSigmaX] = GMR(PriorsX,MuX,SigmaX,expX(1,:),[1],[2:nbVarX]);

%% Iterative IK using Jacobian
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Replacement of the symbolic euqations by real values
L1 = linksLength_repro(1); L2 = linksLength_repro(2);
posx = pos_repro(1,1); posy = pos_repro(2,1);

% BEGIN LOOP for the method used for reproduction
% N=1 corresponds to a controller using constraints in joint space only
% N=2 corresponds to a controller using constraints in task space only
% N=3 corresponds to a controller using both constraints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:3

%Initialization of Theta
Theta(:,1) = expT(2:3,1);
%Initialization of X through direct kinematics
t1 = Theta(1,1); t2 = Theta(2,1);
X(1:2,1) = subs_x(t1,t2,L1,L2,posx,posy);

%Iteration steps
for i=1:nbData-1 
  %Compute delta-values
  DT = expT(2:3,i+1) - Theta(:,i);
  DX = expX(2:3,i+1) - X(:,i);
  
  %GMR represntation of DT
  Mu_T = DT;
  Sigma_T = expSigmaT(:,:,i+1);
  
  %GMR representation of DX in the joint space by projecting the
  %Gaussian components through the linear transformation defined by JX
  t1 = Theta(1,i); t2 = Theta(2,i);
  [Mu_X,Sigma_X] = projectGMM(DX, expSigmaX(:,:,i+1), subs_JX(t1,t2,L1,L2));
  
  %Multiplication of the Gaussian distributions to find an appropriate
  %controller for the task satisfying constraints both on DT and on DX
  [Mu_tot,Sigma_tot] = gaussProduct(Mu_T, Sigma_T, Mu_X, Sigma_X);

  %Update of Theta by considering the different methods for reproduction 
  if n==1
    Theta(:,i+1) = Theta(:,i) + Mu_T;
  elseif n==2
    Theta(:,i+1) = Theta(:,i) + Mu_X;
  elseif n==3
    Theta(:,i+1) = Theta(:,i) + Mu_tot;
  end
  
  %Update of X through direct kinematics
  t1 = Theta(1,i+1); t2 = Theta(2,i+1);
  X(:,i+1) = subs_x(t1,t2,L1,L2,posx,posy);
end

if n==1
  %Reproduction by using only constraints on Theta (Mu_T)
  X1 = X; Theta1 = Theta;
elseif n==2
  %Reproduction by using only constraints on X (Mu_X)
  X2 = X; Theta2 = Theta;
elseif n==3
  %Reproduction by combining both constraints (Mu_tot)
  X3 = X; Theta3 = Theta;
end

% END LOOP for the method used for reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


%% Pre-computation of the axis limits for the graphs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tminy = min([Theta1 Theta2 Theta3]');
Tmaxy = max([Theta1 Theta2 Theta3]');
Tcy = (Tminy+Tmaxy)./2;
Tdy = max(Tmaxy-Tminy)./2 + 0.2;
Xminy = min([X1 X2 X3]');
Xmaxy = max([X1 X2 X3]');
Xcy = (Xminy+Xmaxy)./2;
Xdy = max(Xmaxy-Xminy)./2 + 1;

%% Plot of the demonstrations and reproduction in the 2D space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 600 300],'name','Demonstrations and reproduction');
%Plot demonstrations
subplot(1,2,1); hold on; title('Demonstrations');
plotArm(DataT(2:3,end),linksLength_demo,pos_demo(:,end));
for n=1:nbSamples
  plot(DataX(2,(n-1)*nbData+1:n*nbData),DataX(3,(n-1)*nbData+1:n*nbData),'-','lineWidth',2,'color',[.4 .4 .4]);
  plot(DataX(2,(n-1)*nbData+1),DataX(3,(n-1)*nbData+1),'.','markerSize',15,'color',[.4 .4 .4]);
end
xlabel('x_1'); ylabel('x_2');
axis([-2 12 -2 12]); box on; axis square;
text(-.5,-.8,'L1=6, L2=6','fontSize',14);
%Plot reproduction
subplot(1,2,2); hold on; title('Reproduction');
plotArm(Theta3(:,end),linksLength_repro,pos_repro);
for n=1:nbSamples
  plot(DataX(2,(n-1)*nbData+1:n*nbData),DataX(3,(n-1)*nbData+1:n*nbData),'-','lineWidth',2,'color',[.8 .8 .8]);
  plot(DataX(2,(n-1)*nbData+1),DataX(3,(n-1)*nbData+1),'.','markerSize',15,'color',[.8 .8 .8]);
end
plot(X3(1,:),X3(2,:),'-','lineWidth',2,'color',[.4 .4 .4]);
plot(X3(1,1),X3(2,1),'.','markerSize',15,'color',[.4 .4 .4]);
xlabel('x_1'); ylabel('x_2');
axis([-2 12 -2 12]); box on; axis square;
text(-.5,-.8,'L1=5, L2=7','fontSize',14);

%% Plot GMM encoding in joint space and task space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20 20 640 480],'name','GMM encoding');
%GMM encoding in joint space
for i=1:2
  subplot(2,2,(i-1)*2+1); hold on; box on;
  plotGMM(MuT([1,i+1],:), SigmaT([1 i+1],[1 i+1],:), [.8 0 0], 1);
  for n=1:nbSamples
    plot(DataT(1,(n-1)*nbData+1:n*nbData),DataT(i+1,(n-1)*nbData+1:n*nbData),'-','color',[.4 .4 .4]);
  end
  axis([1 nbData Tcy(i)-Tdy Tcy(i)+Tdy]);
  xlabel('t'); ylabel(['\theta_{' num2str(i) '}']);
end
%GMM encoding in task space
for i=1:2
  subplot(2,2,(i-1)*2+2); hold on; box on;
  plotGMM(MuX([1,i+1],:), SigmaX([1 i+1],[1 i+1],:), [0 .8 0], 1);
  for n=1:nbSamples
    plot(DataX(1,(n-1)*nbData+1:n*nbData),DataX(i+1,(n-1)*nbData+1:n*nbData),'-','color',[.4 .4 .4]);
  end
  axis([1 nbData Xcy(i)-Xdy Xcy(i)+Xdy]);
  xlabel('t'); ylabel(['x_{' num2str(i) '}']);
end
subplot(2,2,1); title('Joint space');
subplot(2,2,2); title('Task space');

%% Plot of the GMR representation of the constraints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[30 30 640 480],'name','GMR constraints');
%Constraints in joint space
for i=1:2
  subplot(2,2,(i-1)*2+1); hold on; box on;
  plotGMM(expT([1,i+1],:), expSigmaT(i,i,:), [.8 0 0], 3);
  axis([1 nbData Tcy(i)-Tdy Tcy(i)+Tdy]);
  xlabel('t'); ylabel(['\theta_{' num2str(i) '}']);
end
%Constraints in task space
for i=1:2
  subplot(2,2,(i-1)*2+2); hold on; box on;
  plotGMM(expX([1,i+1],:), expSigmaX(i,i,:), [0 .8 0], 3);
  axis([1 nbData Xcy(i)-Xdy Xcy(i)+Xdy]);
  xlabel('t'); ylabel(['x_{' num2str(i) '}']);
end
subplot(2,2,1); title('Joint space');
subplot(2,2,2); title('Task space');

%% Plot of the reproductions in joint space and task space
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[40 40 640 480],'name','Reproduction');
%Reproduction in joint space
for i=1:2
  subplot(2,2,(i-1)*2+1); hold on; box on;
  plot(Theta3(i,:),'-','lineWidth',2,'color',[0 0 0]);
  plot(Theta1(i,:),'--','lineWidth',2,'color',[.6 .6 .6]);
  plot(Theta2(i,:),':','lineWidth',2,'color',[.6 .6 .6]);
  axis([1 nbData Tcy(i)-Tdy Tcy(i)+Tdy]);
  xlabel('t'); ylabel(['\theta_{' num2str(i) '}']);
end
%Reproduction task space
for i=1:2
  subplot(2,2,(i-1)*2+2); hold on; box on;
  plot(X3(i,:),'-','lineWidth',2,'color',[0 0 0]);
  plot(X1(i,:),'--','lineWidth',2,'color',[.6 .6 .6]);
  plot(X2(i,:),':','lineWidth',2,'color',[.6 .6 .6]);
  axis([1 nbData Xcy(i)-Xdy Xcy(i)+Xdy]);
  xlabel('t'); ylabel(['x_{' num2str(i) '}']);
end
subplot(2,2,1); title('Joint space');
subplot(2,2,2); title('Task space');

pause;
close all;



