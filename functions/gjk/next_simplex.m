function [npoints, ndir, result] = next_simplex(points)
    % points = n x 3
    switch height(points)
    case 2
        [npoints, ndir, result] = ...
            simplex_line(points);
    case 3
        [npoints, ndir, result] = ...
            simplex_triangle(points);
    case 4
        [npoints, ndir, result] = ...
            simplex_tetrahedron(points);
    otherwise
        error('Unacceptable value');
    end
    
end

