function DrawCube(pos,o,p, h_fig)

       init                  = dual_quater();
    
        init                  = DualVectorQuater(init, [0; 0; 0]);

       phi = 0;
        d   = [0; 0; 0];

       s   = [0; 0; 1];
       
      
        q_R_tar = ReconstructQuaternion(o, p);
        
        
         q_v = q_R_tar.o;
    
          phi = 2 * acos(q_v(1,1));
          if (norm(q_v(2:4,1)) ~= 0)
            s = q_v(2:4,1) ./ dot(q_v(2:4,1), q_v(2:4,1)) ^0.5;  
            b   = tan(phi / 2) .* s;
            l = cross(Pole(d, b), s);
          else
              s = [0;0;1];
              l = [0;0;0];
          end;

    
  
        
        %initial dual screw for Right Arm
        state_R_tar   = dual_quater();
        state_R_tar   = DualQuater(state_R_tar, s, l);
      
        
 
      
        tar_angle_R             = [phi ScrewTran(d, s)];


            axisX = DualVectorQuater(init, [1; 0; 0]);
            axisY = DualVectorQuater(init, [0; 1; 0]);
            axisZ = DualVectorQuater(init, [0; 0; 1]);
            origin= DualVectorQuater(init, [0; 0; 0]);
           
            origin= TransformOrientation(origin, state_R_tar, tar_angle_R);
            axisX = TransformOrientation(axisX, state_R_tar, tar_angle_R) - origin;
            axisY = TransformOrientation(axisY, state_R_tar, tar_angle_R) - origin;
            axisZ = TransformOrientation(axisZ, state_R_tar, tar_angle_R) - origin;
            
            
            axisX_pos = axisX.p;
            axisY_pos = axisY.p;
            axisZ_pos = axisZ.p;
            origin_pos= origin.p;
            
            origin_v  = origin_pos.o;
            axisX_v   = axisX_pos.o;
            axisY_v   = axisY_pos.o;
            axisZ_v   = axisZ_pos.o;
        
            
            d = 0.05;
            x = [];
            
        x(:, 1) = d/2 * axisX_v(2:4, 1) + d / 2 * axisZ_v(2:4, 1) + d / 2 * axisY_v(2:4, 1);
        x(:, 2) =  d/2 * axisX_v(2:4, 1) -d / 2 * axisZ_v(2:4, 1) + d / 2 * axisY_v(2:4, 1);
        x(:, 3) =  d/2 * axisX_v(2:4, 1) -d / 2 * axisZ_v(2:4, 1) - d / 2 * axisY_v(2:4, 1);
        x(:, 4) =  d/2 * axisX_v(2:4, 1) + d / 2 * axisZ_v(2:4, 1) - d / 2 * axisY_v(2:4, 1);



        x(:, 5) = x(:, 4) - d * axisX_v(2:4, 1);
        x(:, 6) = x(:, 5) + d * axisY_v(2:4, 1);
        x(:, 7) = x(:, 6) - d * axisZ_v(2:4, 1);
        x(:, 8) = x(:, 3) - d * axisX_v(2:4, 1);

        origin_v = repmat(origin_v(2:4,1)',8,1) + repmat(pos, 8, 1);

        vert = x' + origin_v;



        fac  = [1 2 3 4; 1 4 5 6; 1 2 7 6; 6 5 8 7; 7 2 3 8; 3 4 5 8];
        tcolor = [1 0 1;1 0 1; 1 0 1; 1 0 1; 1 0 1; 1 0 1];
        figure(h_fig);
         p = patch('Faces',fac,'Vertices',vert,'FaceVertexCData',tcolor,...
        'FaceColor','flat','FaceLighting', 'gouraud');
        
     quiver3(origin_v(1, 1), origin_v(1, 2), origin_v(1, 3)+d/2,  ... 
         axisX_v(2, 1), axisX_v(3, 1), axisX_v(4, 1), 0.1, 'Color', 'r', 'LineWidth', 2);

     quiver3(origin_v(1, 1), origin_v(1, 2), origin_v(1, 3)+d/2,  ... 
         axisY_v(2, 1), axisY_v(3, 1), axisY_v(4, 1), 0.1, 'Color', 'g', 'LineWidth', 2);

     quiver3(origin_v(1, 1), origin_v(1, 2), origin_v(1, 3)+d/2,  ... 
         axisZ_v(2, 1), axisZ_v(3, 1), axisZ_v(4, 1), 0.1, 'Color', 'b', 'LineWidth', 2);
             
     axis square
     shading faceted
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    