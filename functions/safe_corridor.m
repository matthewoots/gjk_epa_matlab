function [face,vertex] = safe_corridor(p_s, p_f, buffer)
    vect = p_f - p_s;
    vect_norm = vect / norm(vect);
    % atan(O=x/A=y)
    yaw = atan(vect_norm(1)/vect_norm(2));
    unrot_vect = eul2rotm([yaw, 0, 0]) * vect_norm';
    pitch = atan(unrot_vect(3)/unrot_vect(2));
    
    % atan(O=z/A=xy)
    rotmZYX = eul2rotm([-yaw, 0, pitch]);
    bound = [buffer norm(vect)+buffer buffer];
    
    origin = (p_f + p_s) / 2;
    
    [face, vertex] = gen_cuboid( ...
        origin, bound, rotmZYX);
end

