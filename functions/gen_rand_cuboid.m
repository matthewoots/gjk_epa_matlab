function [face, vertex, center] = gen_rand_cuboid( ...
    bnd, max_size)
    % Roll Pitch Yaw
    rand_rot_fact = 2 * rand(1,3) - 1;
    rand_euler = rand_rot_fact * 3.1415;
    % Insert Yaw, Pitch and Roll
    rotmZYX = eul2rotm([rand_euler(3),0,0]);
    
    bnd_diff = (bnd(2,:) - bnd(1,:)) / 2;
    bnd_middle = (bnd(2,:) + bnd(1,:)) / 2;
    % Random point -0.5 to 0.5
    % 1 = origin x, 2 = origin y, 3 = origin z;
    rnd_point = 1 * rand(1,3) - 0.5;

    % 1 = Scale on x, 2 = Scale on y, 3 = Scale on z;
    rnd_scale = rand(1,3);
    % Clamp values between 0.5 and 1
    rnd_scale(rnd_scale<0.5) = 0.5;

    obs_origin = rnd_point .* bnd_diff + bnd_middle;
    obs_bnd = max_size * rnd_scale; 
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
        rot_vect = rotmZYX * ... 
            (bnd_arr(i,:))';
        rot_bnd(i,:) = obs_origin + rot_vect';
        obs_vert(i,:) = rot_bnd(i,:);
    end    

    % 6 faces (1234 5678 1265 2376 3487 4158)   
    vertex = obs_vert;
    face = [1 2 3 4 ; ... 
           5 6 7 8 ; ...
           1 2 6 5 ; ...
           2 3 7 6 ; ...
           3 4 8 7 ; ...
           4 1 5 8];
    center = obs_origin;
end

