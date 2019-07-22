function demo_BM
% Binary Merging (BM): Learning a point to point motion from a set of
% demonstrations. Type: doc BM for more information.
%
% Copyright(c) 2009 S. Mohammad Khansari Zadeh, LASA Lab, EPFL, Lausanne,
%               Switzerland, http://lasa.epfl.ch/khansari
%
% The program is free for non-commercial academic use. The software must
% not be modified or distributed without prior permission of the authors.

e_max=0.15; %Maximum allowed error in estimation
gap_value=0.5; %A scalar defining the cost in having a gap between datapoints
R=0.6; %Weighting for the error in estimating the direction of xi_d
Q=1-R; %Weighting for the error in estimating the magnitude of xi_d
order=1;

disp('Which demonstration do you want to see?')
disp(' 1) Straight Line (2-D example)')
disp(' 2) One Arc (2-D example)')
disp(' 3) Three Arc (2-D example)')
disp(' 4) Z-Shape (2-D example)')
disp(' 5) Cup Task (3-D example)')
disp(' 6) Saucer Task (3-D example)')
disp(' 7) Katana joint angle demonstration (4-D example - dimension was reduced by PCA)')
disp(' 8) Katana joint angle demonstration (6-D example)')
disp(' 9) Loop''s motion (4-D example: 2nd order example)')
disp('10) Demonstrate your own 1st order motion by mouse! (2-D example)')
disp('11) Demonstrate your own 2nd order motion by mouse! (4-D example)')
c=input(' ');

load demo_Data
switch c
    case 1
        Data=Data_Line;index=index_Line;
    case 2
        Data=Data_Arc;index=index_Arc;
    case 3
        Data=Data_3Arc;index=index_3Arc;
    case 4
        Data=Data_ZShape;index=index_ZShape;
        e_max=0.2;gap_value=0.25;
        fprintf('e_max was set to %1.2f\n',e_max)
        fprintf('gap_value was set to %1.2f\n',gap_value)        
        disp(' ')
    case 5
        Data=Data_Cup;index=index_Cup;
        e_max=0.05;
        fprintf('e_max was set to %1.2f\n',e_max)
        disp(' ')
    case 6
        Data=Data_Saucer;index=index_Saucer;
        e_max=0.2;gap_value=0.4;
        fprintf('gap_value was set to %1.2f\n',gap_value)        
        disp(' ')
    case 7
        index=index_J_PCA;Data=Data_J_PCA;
        gap_value=3;
        fprintf('gap_value was set to %1.2f\n',gap_value)
        disp(' ')
    case 8
        Data=Data_J6;index=index_J6;
        e_max=0.03;gap_value=0.4;
        fprintf('e_max was set to %1.2f\n',e_max)
        fprintf('gap_value was set to %1.2f\n',gap_value)
        disp(' ')
    case 9
        Data=Data_loop;index=index_loop;
        order=2;e_max=0.4;gap_value=0.3;
        fprintf('e_max was set to %1.2f\n',e_max)
        fprintf('gap_value was set to %1.2f\n',gap_value)
    case 10
        nbSample=3; %number of demonstrations
        nbData=150; %number of datapoints in each demonstration
        Data=grabDataFromCursor(nbSample,nbData); %grabbing data from mouse
        close(gcf)
        index=1;
        for i=1:nbSample-1 %index defines the starting point of each demonstration in the variable Data
            index=[index nbData*i+1];
        end
        Data(2:3,:)=Data(2:3,:)*2000; %correcting the order of data
        Data(1,:)=Data(1,:)/10; %the first row in data is time which is necessary to compute velocity
        save lastmodel Data index R Q e_max gap_value order
    case 11
        nbSample=5; %number of demonstrations
        nbData=200; %number of datapoints in each demonstration
        Data=grabDataFromCursor(nbSample,nbData); %grabbing data from mouse
        close(gcf)
        index=1;
        for i=1:nbSample-1 %index defines the starting point of each demonstration in the variable Data
            index=[index nbData*i+1];
        end
        Data(2:3,:)=Data(2:3,:)*2000; %correcting the order of data
        Data(1,:)=Data(1,:)/10; %the first row in data is time which is necessary to compute velocity
        order=2;e_max=0.3;gap_value=0.3;
        save lastmodel Data index R Q e_max gap_value order
    otherwise
        disp('Wrong Input! Returned to the MATLAB Command')
        return
end

bm=BM(Data,index,R,Q,e_max,gap_value,order); %constructing a BM class
bm=bm.Learning(); %learning the model
save lastmodel bm
bm.Simulation(bm.x0,true,0.025,[],[],2500,0.001); %simulating the motion
% bm=bm.Learning_Straight();bm.Simulation(bm.x0);
% return
%plotting result
bm.plotModel();
if bm.nbVar/2==2
    bm.plotHyperplanes('high') %can only be used for 2D model
end