function [  ] = export_joints( joints,fname )
%EXPORT_JOINTS exports the joints into a txt file
% joints: N x 6, joint angles
% fname: String,the file name

f = fopen(fname,'w');
N = size(joints,1);
for i=1:N
    fprintf(f,'%f,%f,%f,%f,%f,%f\r\n',joints(i,:));
end
fclose(f);

end

