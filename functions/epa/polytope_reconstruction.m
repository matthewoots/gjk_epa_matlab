function [epa_faces, num_faces, flag] = ...
    polytope_reconstruction(epa_faces, num_faces, ...
    num_loose_edges, loose_edges, support, limit)
    
    flag = -1;
    
    % Reconstruct polytope with p added
    for i = 1:num_loose_edges
        % assert(num_faces < EPA_MAX_NUM_FACES);
        if num_faces >= limit 
            flag = 2;
            fprintf("[EPA] exit EPA_MAX_NUM_FACES\n");
            break;
        end

        face_vect = cross(loose_edges{i,1} - loose_edges{i,2}, ...
            loose_edges{i,1} - support);

        epa_faces{num_faces+1,1} = loose_edges{i,1};
        epa_faces{num_faces+1,2} = loose_edges{i,2};
        epa_faces{num_faces+1,3} = support;
        epa_faces{num_faces+1,4} = face_vect/norm(face_vect);

        % if any(face_vect == 0) && ...
        %        norm(face_vect) == 0
        %    epa_faces{num_faces+1,4} = face_vect;
        % end

        % Check for wrong normal to maintain CCW winding
        bias = 0.000001; % in case dot result is only slightly < 0 (because origin is on face)

        if dot(epa_faces{num_faces+1,1}, ...
                epa_faces{num_faces+1,4}) + bias < 0

            temp = epa_faces{num_faces+1,1};
            epa_faces{num_faces+1,1} = epa_faces{num_faces+1,2};
            epa_faces{num_faces+1,2} = temp;
            epa_faces{num_faces+1,4} = -epa_faces{num_faces+1,4};
        end

        num_faces = num_faces + 1;
    end % i = 1:num_loose_edges
    
end

