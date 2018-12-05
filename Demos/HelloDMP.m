% HelloDMP

% Haopeng Hu
% 2018.12.05

% Just a demo for the Dynamic Movement Primitive

dmp = DynamicMovementPrimitive(5,1,2,GaussianBasis(1,0.5));

%% Plot show

% [t,z] = ode45(@(t,z)CanonicalSystem(t,z,dmp),[0,1],2);
[t,zx] = ode45(@(t,zx)TransformedSystem(t,zx,dmp),[0,10],[1;0.5;0]);
fz = dmp.transformationFunc(zx(:,1));

% plot(t,zx(:,1));
hold on
plot(t,zx(:,2));

[fz, psiz] = dmp.transformationSignal(zx(:,1));
plot(t,fz);
plot(t,psiz);