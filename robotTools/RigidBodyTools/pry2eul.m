function [ eulRad ] = pry2eul( pryRad )
%pry2eul Transform pry(ZYX) angles to ZYX eul angles
% pryRad: N x [pitch, roll, yaw] angles (ZYX) 
% eulRad: N x [eulz, euly, eulx] angles (ZYX)

N = size(pryRad,1);
eulRad = zeros(N,3);
for i = 1:N
    R1 = axang2rotm([0,0,1,pryRad(i,1)]);
    R2 = axang2rotm([0,1,0,pryRad(i,2)]);
    R3 = axang2rotm([1,0,0,pryRad(i,3)]);
    R = R3*R2*R1;
    eulRad(i,:) = rotm2eul(R);
end

end

