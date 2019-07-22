function plotRotation(s, pos, h_fig, s_tar)


    x = quater();
    x.o = [1;1;0;0];
    y = quater();
    y.o = [1;0;1;0];
    z= quater();
    z.o = [1;0;0;1];
    Q = quater();
    Q.o = [cos(s(4, 1) / 2); sin(s(4, 1) / 2) .* s(1:3,1) ./ norm(s(1:3,1))];
    
    x_t = Q * x * Q';
    y_t = Q * y * Q';
    z_t = Q * z * Q';

    v_x = x_t.o;
    v_y = y_t.o;
    v_z = z_t.o;
    

    figure(h_fig);
    quiver3(pos(1,1), pos(2,1), pos(3, 1), v_x(2,1), v_x(3,1), v_x(4, 1), 0.01, 'r');
    hold on;
    quiver3(pos(1,1), pos(2,1), pos(3, 1), v_y(2,1), v_y(3,1), v_y(4, 1), 0.01, 'g');
    hold on;
    quiver3(pos(1,1), pos(2,1), pos(3, 1), v_z(2,1), v_z(3,1), v_z(4, 1), 0.01, 'b');