function x = subs_x(t1,t2,L1,L2,posx,posy)

x(1) = posx + L1.*cos(t1) + L2.*cos(t2);
x(2) = posy + L1.*sin(t1) + L2.*sin(t2);
