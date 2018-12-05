function [ dot_zx ] = TransformedSystem( t,zx, dmp)
%TransformedSystem Dynamics of the transformed system
%   dot_x1 = tau*x2
%   dot_x2 = tau*alpha_x(beta_x(goal - x1)-x2)+tau*A*f(z)

dot_zx = [-1*dmp.tau*dmp.alpha_z*zx(1);
          dmp.tau*zx(3);
          dmp.tau*dmp.alpha_x*(dmp.beta_x*(dmp.goal - zx(2))-zx(3))+dmp.tau*dmp.A*dmp.transformationFunc(zx(1))];

end

