function [  ] = plotUR5Joints( jointTraj,T )
%plotUR5Joints Plot the joint degrees of UR5
% jointTraj: N x 6, joint trajectory
% T: the terminal time ( default: 1 )

if nargin < 2
    T = 1;
end

figure 
joint1 = subplot(2,3,1);
joint2 = subplot(2,3,2);
joint3 = subplot(2,3,3);
joint4 = subplot(2,3,4);
joint5 = subplot(2,3,5);
joint6 = subplot(2,3,6);
joints = [joint1,joint2,joint3,joint4,joint5,joint6];

t = linspace(0,T,size(jointTraj,1));

for i = 1:6
    plot(joints(i),t,jointTraj(:,i));
    title(joints(i),strcat('Joint',int2str(i)));
    ylabel(joints(i),'rad');
    grid on
end

end

