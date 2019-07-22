function [x,y,z] = ReconstructFrame(s,p)
    
    x = s;
    if (~exist('p','var'))
        z2 = 0.5;
        z3 = 0.5;
        y3 = 0.5;
        z1 = (-x(2,1) * z2 - x(3,1) * z3) / (x(1, 1));
        z = [z1; z2; z3]; 
    else
        z = p;
    end;
    
    
    z = z ./ dot(z, z)^0.5;
    
    y = cross(z,x);

    
    y = y ./ dot(y, y)^0.5;
    
    
      
    