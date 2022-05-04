function [npoints, ndir, result] = simplex_tetrahedron(points)
    
    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    a = points(1,:); b = points(2,:); c = points(3,:); d = points(4,:);
    ab = b - a;
    ac = c - a;
    ad = d - a;
    ao = -a;
    
    abc = cross(ab,ac);
    acd = cross(ac,ad);
    adb = cross(ad,ab);
    
    if same_direction(abc, ao)
        [npoints, ndir, result] = ...
            simplex_triangle([a ; b ; c]);
        return;
    end
    if same_direction(acd, ao)
        [npoints, ndir, result] = ...
            simplex_triangle([a ; c ; d]);
        return;
    end
    if same_direction(adb, ao)
        [npoints, ndir, result] = ...
            simplex_triangle([a ; d ; b]);
        return;
    end
    
    npoints = [0 0 0];
    ndir = [0 0 0];
    result = true;
    return   
end

