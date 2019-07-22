function res = ComputeVel(v_x, s, s_prec, phi, phi_prec)
   
    phi = phi/2;
    phi_prec = phi_prec/2;
    
    q_prec = quater();
    q = quater();
    
    q_prec.o = [cos(phi_prec); sin(phi_prec).* s_prec];
    q.o = [cos(phi); sin(phi).* s];
    
    %instantaneous rotation in global referential
    q_m = q * q_prec';
    %instantaneous rotation in (t-1) referential
    %q_m = q_prec' * q;
    q_m_o = q_m.o;
    
    if (q_m_o(1,1) > 0.99999999999)
        v_phi = 0;
    else
        v_phi = 2 * acos(q_m_o(1,1));
    end;
    
    if (v_phi == 0)
       v_s = [0; 0; 0];
    else
        v_s  = q_m_o(2:4,1);
        v_s = v_s ./ norm(v_s);
    end;
 
   
    angle_vel = v_phi .* v_s;
    
    
    velocity_lin = v_x;
    
    res = [velocity_lin; 100*angle_vel];
    res = [velocity_lin; angle_vel];