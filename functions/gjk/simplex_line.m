function [npoints, ndir, result] = simplex_line(points)

    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    a = points(1,:); b = points(2,:);
    ab = b - a;
    ao = -a;
    
    if same_direction(ab, ao)
        npoints = points;
        ndir = cross(cross(ab,ao),ab);
    else
        npoints = a;
        ndir = ao;
    end
    
    result = false;
end

