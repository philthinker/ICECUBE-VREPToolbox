function [  ] = saveJointTraj( jointTraj,fileName )
%saveJointTraj Save the joints into a txt file
% jointTraj: N x 6, joint angles
% fileName: String,the file name

f = fopen(fileName,'w');
N = size(jointTraj,1);
for i=1:N
    fprintf(f,'%f,%f,%f,%f,%f,%f\r\n',jointTraj(i,:));
end
fclose(f);

end

