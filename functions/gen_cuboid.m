function [face, vertex] = gen_cuboid( ...
    origin, bound, rot)

    obs_origin = origin;
    
    obs_bnd = bound; 
    obs_vert = zeros(8, 3);
    rot_bnd = zeros(8, 3);
    
    bnd_arr = [-obs_bnd(1) -obs_bnd(2) -obs_bnd(3);
               obs_bnd(1) -obs_bnd(2) -obs_bnd(3);
               obs_bnd(1) obs_bnd(2) -obs_bnd(3);
               -obs_bnd(1) obs_bnd(2) -obs_bnd(3);
               -obs_bnd(1) -obs_bnd(2) obs_bnd(3);
               obs_bnd(1) -obs_bnd(2) obs_bnd(3);
               obs_bnd(1) obs_bnd(2) obs_bnd(3);
               -obs_bnd(1) obs_bnd(2) obs_bnd(3)];
           
    bnd_arr = bnd_arr .* 0.5;
    
    for i = 1:8
        rot_vect = rot * ... 
            (bnd_arr(i,:))';
        rot_bnd(i,:) = obs_origin + rot_vect';
        obs_vert(i,:) = rot_bnd(i,:);
    end    

    % 6 faces (1234 5678 1265 2376 3487 4158)   
    vertex = obs_vert;
    face= [1 2 3 4 ; ... 
           5 6 7 8 ; ...
           1 2 6 5 ; ...
           2 3 7 6 ; ...
           3 4 8 7 ; ...
           4 1 5 8];
end

