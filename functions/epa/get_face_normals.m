function [normals, distances, min_triangle] = get_face_normals(polytope, faces)
    %% GJK Winter Dev
    % https://blog.winter.dev/2020/epa-algorithm/
    % From C++ to matlab
    min_triangle = 0;
	min_distance = 1000;

    % for (size_t i = 0; i < faces.size(); i += 3)
    % Each face occupies 3 points
    for i=1:height(faces)
        a = polytope(faces(i,1),:);
        b = polytope(faces(i,2),:);
        c = polytope(faces(i,3),:);
        
        face = cross((b - a),(c - a));
        face_norm = face / norm(face);
        distance = dot(face_norm, a);
        
        if (distance < 0) 
			face_norm = normal * -1;
			distance = distance * -1;
        end
        
        normals(i,:) = face_norm;
        distances(i) = distance;
        
        if distance < min_distance
			min_triangle = i;
			min_distance = distance;
        end
		
    end

end

