function [edge_idxs_pair] = add_if_unique_edge( ...
        edge_idxs_pair, faces_idxs, a, b)

    if isempty(edge_idxs_pair)
        edge_idxs_pair = [edge_idxs_pair ; ...
            [faces_idxs(a(1), a(2)) faces_idxs(b(1), b(2))]];
        return;
    end
    %  The ‘q’ variable tells you if there is a match somewhere (1) or not (0).
    [q, idx] = ismember(...
        [faces_idxs(b(1), b(2)) faces_idxs(a(1), a(2))], ...
        edge_idxs_pair, 'rows');
    

    % If q ia true means that there is a match
    if q 
		% edges.erase(reverse)
        % Erase the reverse edge pair
        edge_idxs_pair(idx,:) = [];
    else
        edge_idxs_pair = [edge_idxs_pair ; ...
            [faces_idxs(a(1), a(2)) faces_idxs(b(1), b(2))]];
    end
end

