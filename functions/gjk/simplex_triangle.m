function [npoints, ndir, result] = simplex_triangle(points)
    
    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    a = points(1,:); b = points(2,:); c = points(3,:);
    ab = b - a;
    ac = c - a;
    ao = -a;
    
    abc = cross(ab,ac);
    
    if same_direction(cross(abc,ac), ao)
        if same_direction(ac, ao)
            npoints = [a ; c];
            ndir = cross(cross(ac,ao),ac);
        else
            [npoints, ndir, result] = ...
                simplex_line([a ; b]);
            return;
        end
    else
        if same_direction(cross(ab,abc), ao)
            [npoints, ndir, result] = ...
                simplex_line([a ; b]);
            return;
        else
            if same_direction(abc, ao)
                npoints = points;
                ndir = abc;
            else
                npoints = [a ; c ; b];
                ndir = -abc;
            end
        end
    end
    
    result = false;
    
end

