function [closest_face, min_dist] = get_closest_face(faces, num_faces)
    min_count = 1;
    % Find face that's closest to origin
    min_dist = dot(faces{min_count,1}, ...
                faces{min_count,4});
    while isnan(min_dist)
        min_count = min_count+1;
        min_dist = dot(faces{min_count,1}, ...
            faces{min_count,4});
    end

    closest_face = min_count;
    for i = 1:num_faces
        if i == min_count
           continue; 
        end

        dist = dot(faces{i,1}, faces{i,4});
        if dist < min_dist && ...
                ~any(isnan(faces{i,4}) == 1) && ...
                ~isnan(dist) 
            min_dist = dist;
            closest_face = i;
        end
    end
end

