function [max_point] = find_furthest_point(vert_array, direction)
    
    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    arbi_number = -10000;
    max_distance = arbi_number;
    max_point = [arbi_number arbi_number arbi_number];
    
    % vert_array = n x 3
    for i = 1:height(vert_array)
        dist = dot(vert_array(i,:),direction);
        if dist > max_distance
            max_distance = dist;
            max_point = vert_array(i,:);
        end
    end
    if any(max_point(:) == arbi_number)
        error('no furthest point found');
    end
end

