function [npoints] = simplex_line(v, shape1, shape2)
    %% GJK Winter Dev
    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    %     a = points(1,:); b = points(2,:);
    %     ab = b - a;
    %     ao = -a;
    %     
    %     if same_direction(ab, ao)
    %         npoints = points;
    %         ndir = cross(cross(ab,ao),ab);
    %     else
    %         npoints = a;
    %         ndir = ao;
    %     end
    %     
    %     result = false;

    %% GJK mws262
    % There are some faults with winter.dev conversion hence using GJK
    % Collision Detection from mws262
    % https://github.com/mws262/MATLAB-GJK-Collision-Detection
    npoints(1,:) = support(shape2,shape1,v);
    npoints(2,:) = support(shape2,shape1,-v);
end

