function [epa_faces, num_loose_edges, loose_edges, flag, num_faces] = ...
        check_remove_faces(epa_faces, num_faces, support, limit)
    
    % keep track of edges we need to fix after removing faces
    num_loose_edges = 0;
    loose_edges = {};
    flag = -1;
    
    % Find all triangles that are facing p
    for i = 1:num_faces

        % triangle i faces p, remove it
        if (dot(epa_faces{i,4}, ...
            support-epa_faces{i,1}) > 0) 

            % Add removed triangle's edges to loose edge list.
            % If it's already there, remove it (both triangles it belonged to are gone)
            for j = 1:3 % Three edges per face

                face_a = epa_faces{i,j};
                face_b = epa_faces{i, mod((j+1),3)+1};
                current_edge = {face_a, face_b};
                found_edge = false;

                % Check if current edge is already in list
                for k = 1:num_loose_edges 
                    if isequal(loose_edges{k,2}, current_edge{1}) && ...
                        isequal(loose_edges{k,1}, current_edge{2})

                        % Edge is already in the list, remove it
                        % THIS ASSUMES EDGE CAN ONLY BE SHARED BY 2 TRIANGLES (which should be true)
                        % THIS ALSO ASSUMES SHARED EDGE WILL BE REVERSED IN THE TRIANGLES (which 
                        % should be true provided every triangle is wound CCW)

                        loose_edges{k,1} = loose_edges{num_loose_edges,1}; 
                        % Overwrite current edge
                        loose_edges{k,2} = loose_edges{num_loose_edges,2}; 
                        % with last edge in list
                        num_loose_edges = num_loose_edges - 1;

                        found_edge = true;
                        break; 
                        % exit loop because edge can only be shared once
                    end
                end % endfor loose_edges

                if ~found_edge % add current edge to list
                    % assert(num_loose_edges < EPA_MAX_NUM_LOOSE_EDGES);
                    if num_loose_edges >= limit 
                        flag = 1;
                        fprintf("[EPA] exit EPA_MAX_NUM_LOOSE_EDGES\n");
                        break;
                    end
                    loose_edges{num_loose_edges + 1,1} = ...
                        current_edge{1};
                    loose_edges{num_loose_edges + 1,2} = ...
                        current_edge{2};
                    num_loose_edges = num_loose_edges + 1;
                end
            end % j = 1:3 % Three edges per face

            % Remove triangle i from list
            epa_faces{i,1} = epa_faces{num_faces,1};
            epa_faces{i,2} = epa_faces{num_faces,2};
            epa_faces{i,3} = epa_faces{num_faces,3};
            epa_faces{i,4} = epa_faces{num_faces,4};
            num_faces = num_faces - 1;
            i = i - 1;

        end % (dot(faces{i,4},p-faces{i,1}) > 0) 
    end % i = 1:num_faces
end

