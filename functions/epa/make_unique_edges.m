function [unique_edge] = make_unique_edges(normals, support, faces)
    %% GJK Winter Dev
    % https://blog.winter.dev/2020/epa-algorithm/
    % From C++ to matlab
    unique_edge = [];
    for i = 1:height(normals)
       % If it is in the same direction
       if dot(normals(i,:), support) > 0
            unique_edge = add_if_unique_edge( ...
                unique_edge, faces, [i 1], [i 2]);
            unique_edge = add_if_unique_edge( ...
                unique_edge, faces, [i 2], [i 3]);
            unique_edge = add_if_unique_edge( ...
                unique_edge, faces, [i 3], [i 1]);
            
            faces(i,:) = faces(end,:); faces(end,:) = [];
            normals(i) = normals(end); normals(end) = [];
            
            i = i - 1;
       end
    end
end

