function [result, point] = ray_intersect_quad_face(...
    normal, center, line_dir, line_origin, vertices, line_length)
    
    result = false;
    flt_min = 1e-6;
    query_point = [flt_min, flt_min, flt_min];
    point = [flt_min, flt_min, flt_min];
    %% Find intersection of a ray to a plane
    % https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection
    % assuming vectors are all normalized
    
    denominator = dot(normal, line_dir); 
    % When the denominator is small = line is parallel to plane
    if denominator > flt_min
        center_2_line = center - line_origin; 
        t = dot(center_2_line, normal) / denominator; 

        % The point may be in the quadface but it is after the line
        if (t >= 0 && t < line_length)
            query_point = line_origin + t * line_dir;
        else
            fprintf('t = %f\n', t);
            return;
        end
        fprintf('t = %f, point = %f %f %f\n', ...
            t, query_point(1), query_point(2), query_point(3));
    end 
    %% 1st Check. Find point is within the quad face
    % https://math.stackexchange.com/a/190403

    % First we calculate the edge lengths
    for i=1:4
        % wrap around last index
        if mod(i+1,4) == 0
            wrap_idx = 4;
        else
            wrap_idx = mod(i+1,4);
        end
        
        a(i) = norm(vertices(wrap_idx,:) - vertices(i,:));
        fprintf('a%d (%d,%d) = %f ', i, wrap_idx, i, a(i));
    end
    
    % Next we calculate the lengths of the line segments 
    % (to query point)
    for i=1:4
        b(i) = norm(vertices(i,:) - query_point);
    end
    
    fprintf('\n');
    
    % Calculate the areas using Heron's Formula
    % If the point is inside the rectangle : A = A1+A2+A3+A4
    % If the point is outside the rectangle : A > A1+A2+A3+A4
    A_total = a(1) * a(2);
    A_area = 0;
    for i=1:4
        % wrap around last index
        if mod(i+1,4) == 0
            wrap_idx = 4;
        else
            wrap_idx = mod(i+1,4);
        end
        
        u(i) = (a(i) + b(i) + b(wrap_idx)) / 2;
        A(i) = sqrt( u(i) * (u(i)-a(i)) * ...
            (u(i)-b(i)) * (u(i)-b(wrap_idx)) );
        A_area = A_area + A(i);
    end
    
    if abs(A_area - A_total) < 0.001
        % There is an intersection
        result = true;
        point = query_point;
    else
        % There is no intersection
        % return a bad point
        result = false;
        point = [flt_min, flt_min, flt_min];
    end
    
    fprintf('A_area/A_total %.3f/%.3f\n', A_area, A_total);    
     
    
end

