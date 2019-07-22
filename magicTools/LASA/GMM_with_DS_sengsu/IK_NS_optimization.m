

function [v_joints_curr, joints_curr] = IK_NS_optimization(JOINTS, v_cart, s_curr, s_prev, phi_curr, phi_prev, ts)
    
    v_full = ComputeVel(v_cart * ts, s_curr, s_prev, phi_curr, phi_prev);
    v_angular = v_full(4:6, 1);
    
    JAC = Tsubs_J(JOINTS);
    
    [V,D] = eig(JAC(4:6, :)' * JAC(4:6, :));
    
    ind = find(abs(D) > 10^(-6));
    
    E_r = V(:,ceil(ind(1)/6):end);
    E_n = V(:, 1:(ceil(ind(1)/6)-1));
    V_r = D(ceil(ind(1)/6):end, ceil(ind(1)/6):end);
    V_n = D( 1:(ceil(ind(1)/6)-1), 1:(ceil(ind(1)/6)-1));
    
    [V1 D1] = eig(E_n' * JAC(1:3, :)' * JAC(1:3, :) * E_n);
    ind = find(abs(D1) > 10^(-6));
    E_r1 = V1(:,ceil(ind(1)/6):end);
    E_n1 = V1(:, 1:(ceil(ind(1)/6)-1));
    V_r1 = D1(ceil(ind(1)/6):end, ceil(ind(1)/6):end);
    V_n1 = D1( 1:(ceil(ind(1)/6)-1), 1:(ceil(ind(1)/6)-1));
    
    v_joints_curr = E_r * pinv(V_r) * E_r' * JAC(4:6, :)'* v_angular + ...
        E_n * E_r1 * pinv(V_r1) * E_r1' * E_n' * JAC(1:3, :)' *  v_cart * avg_time_step;

   
    joints_curr = JOINTS(:, i) + th(:, i+1) * ts; 
