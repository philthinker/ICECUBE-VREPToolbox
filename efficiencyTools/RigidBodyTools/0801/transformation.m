function [r1,t1]=transformation(p1,p2,p3)
r1=zeros(3,3);

t1=p1;
r1(:,2)=(p2-p1)/norm(p2-p1);%y÷·
r1(:,1)=(p3-p1)/norm(p3-p1);%x÷·
r1(:,3)=cross(r1(:,1),r1(:,2));




