function [sup] = support(vert1, vert2, direction)
    
    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    max_point1 = find_furthest_point(vert1, direction);
    max_point2 = find_furthest_point(vert2, -direction);
    sup = max_point1 - max_point2;
end

