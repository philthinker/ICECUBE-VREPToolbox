function [ quatSXYZ ] = pry2quat( pryRad )
%pry2quat Transform pry(ZYX) angles to quaternion 
% pryRad: N x [pitch, roll, yaw] angles (ZYX) 
% quatSXYZ: N x [s,x,y,z] quaternion

N = size(pryRad,1);
quatSXYZ = zeros(N,4);

for i = 1:N
    R1 = axang2rotm([0,0,1,pryRad(i,1)]);
    R2 = axang2rotm([0,1,0,pryRad(i,2)]);
    R3 = axang2rotm([1,0,0,pryRad(i,3)]);
    R = R3*R2*R1;
    quatSXYZ(i,:) = rotm2quat(R);
end

end

