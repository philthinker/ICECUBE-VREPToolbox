function JX = subs_JX(t1,t2,L1,L2)

JX(1,1) = -cos(t2)./(L1.*sin(t1 - t2));
JX(1,2) = -sin(t2)./(L1.*sin(t1 - t2));

JX(2,1) = cos(t1)./(L2.*sin(t1 - t2));
JX(2,2) = sin(t1)./(L2.*sin(t1 - t2));

