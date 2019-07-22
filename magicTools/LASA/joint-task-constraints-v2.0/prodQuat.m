function q = prodQuat(q1,q2)
%
% Product of two quaternions.
% Author:	Sylvain Calinon, 2009
%			http://programming-by-demonstration.org
%
% Given two unit quaternions
% Q1=(w1, x1, y1, z1);
% Q2=(w2, x2, y2, z2);
% 
% A combined rotation of unit two quaternions is achieved by
% Q1 * Q2 =( w1.w2 - v1.v2, w1.v2 + w2.v1 + v1*v2)
% where 
% v1= (x1, y1, z1)
% v2 = (x2, y2, z2)
% and both . and * are the standard vector dot and cross product.
% 
% However an optimization can be made by rearranging the terms to produce
% w=w1w2 - x1x2 - y1y2 - z1z2
% x = w1x2 + x1w2 + y1z2 - z1y2
% y = w1y2 + y1w2 + z1x2 - x1z2
% z = w1z2 + z1w2 + x1y2 - y1x2 

q(1,1) = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
q(2,1) = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
q(3,1) = q1(1)*q2(3) + q1(3)*q2(1) + q1(4)*q2(2) - q1(2)*q2(4);
q(4,1) = q1(1)*q2(4) + q1(4)*q2(1) + q1(2)*q2(3) - q1(3)*q2(2);
