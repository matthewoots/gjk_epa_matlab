clc
clear
close all

addpath('functions');
addpath('functions/gjk');
addpath('functions/epa');

% Boundary XYZ
bnd = [-10 -10 0;
       10 10 10];
max_size = 6; % For scaling properties
num_obs = 2;
faces = cell(1,num_obs); vertexes = cell(1,num_obs);

for i=1:num_obs
    [face, vertex, center] = gen_rand_cuboid( ...
    bnd, max_size);
    faces{i} = face; vertexes{i} = vertex; 
    centers(i,:) = center;
end

%% GJK mws262
% There are some faults with winter.dev conversion hence using GJK
% Collision Detection from mws262
% https://github.com/mws262/MATLAB-GJK-Collision-Detection
% Point 1 and 2 selection (line segment)
gjk_start = tic;

direction = [1 0 0];
[points] = simplex_line(direction,vertexes{2},vertexes{1});

%Point 3 selection (triangle)
[points,flag] = simplex_triangle(points,vertexes{2},vertexes{1});

%Point 4 selection (tetrahedron)
if flag == 1 %Only bother if we could find a viable triangle.
    [points,flag] = simplex_tetrahedron(points,vertexes{2},vertexes{1});
end

% Get the minkowski difference tetrahedron / triangle
md_vert_faces = [];
switch height(points)
    case 3
        md_vert_faces = [1 2 3 nan];
    case 4
        md_vert_faces = [1 2 3 nan;
                         2 3 4 nan;
                         4 1 2 nan;
                         4 1 3 nan];
end

fprintf('Time of GJK %.3f\n', toc(gjk_start));

%% EPA 
% https://github.com/kevinmoran/GJK/blob/master/GJK.h
FLT_MIN = -10000;
epa_col_vect = [FLT_MIN,FLT_MIN,FLT_MIN];

if flag == 1
    epa_start = tic;
    
    EPA_TOLERANCE = 0.0001;
    EPA_MAX_NUM_FACES = 32;
    EPA_MAX_NUM_LOOSE_EDGES = 32;
    EPA_MAX_NUM_ITERATIONS = 32;

    a = points(1,:);
    b = points(2,:);
    c = points(3,:);
    d = points(4,:);
    
    %Init with final simplex from GJK
    epa_faces = {a b c cross(b-a, c-a)/norm(cross(b-a, c-a)); ... %ABC
                 a c d cross(c-a, d-a)/norm(cross(c-a, d-a)); ... %ACD
                 a d b cross(d-a, b-a)/norm(cross(d-a, b-a)); ... %ADB
                 b d c cross(d-b, c-b)/norm(cross(d-b, c-b))};    %BDC
    
    % Start with a tetrahedron
    num_faces = 4;
    exit_flag = -1;
    
    for iterations = 1:EPA_MAX_NUM_ITERATIONS
        
        % Find face and distance that's closest to origin
        [closest_face, min_dist] = get_closest_face( ...
            epa_faces, num_faces);
        
        % Search normal to face that's closest to origin
        search_dir = epa_faces{closest_face,4}; 
        p = support(vertexes{2}, vertexes{1}, search_dir);
        
        if dot(p, search_dir) - min_dist < EPA_TOLERANCE
            % Convergence (new point is not significantly further from origin)
            epa_col_vect = ...
                epa_faces{closest_face, 4} * ...
                dot(p, search_dir); 
            exit_flag = 0;
            fprintf("[EPA] exit CONVERGENCE\n");
            % dot vertex with normal to resolve collision along normal
            break; % This break will end the iterations loop
        end
        
        [epa_faces, num_loose_edges, loose_edges, exit_flag, num_faces] = ...
              check_remove_faces(epa_faces, num_faces, p, EPA_MAX_NUM_LOOSE_EDGES);
          
        [epa_faces, num_faces, exit_flag] = polytope_reconstruction( ...
            epa_faces, num_faces, num_loose_edges, loose_edges, p, EPA_MAX_NUM_FACES);
        
        if exit_flag >= 0
            fprintf("[EPA] exit_flag in iterations %d\n", exit_flag);
        end
    end % iterations = 1:EPA_MAX_NUM_ITERATIONS

    if iterations == EPA_MAX_NUM_ITERATIONS
        fprintf("[EPA] exit EPA_MAX_NUM_ITERATIONS\n");
        % Return most recent closest point
        epa_col_vect = ...
            epa_faces{closest_face,4} * ...
            dot(epa_faces{closest_face,1}, epa_faces{closest_face,4});
    end

    fprintf('Time of EPA %.3f\n', toc(epa_start));
end

%% Plotting
figure(1)
hold on
plot3(centers(:,1), centers(:,2), centers(:,3), ...
    'x','DisplayName','centroids');

for i=1:num_obs
    patch('Faces',faces{i},'Vertices',vertexes{i}, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.3, ...
    'LineStyle','--','DisplayName',"polygon "+num2str(i));
end

% epa plot
if ~any(epa_col_vect(:) == FLT_MIN)
    t_vertexes = translate_polygon(vertexes{1}, epa_col_vect);
    patch('Faces',faces{1},'Vertices',t_vertexes, ...
    'Facecolor',[0.8 0.3 0.4],'FaceAlpha',0.3, ...
    'LineWidth',0.5,'DisplayName',"transformed "+num2str(1));
end

title('Polygon 3D space')
axis([bnd(:,1)', ...
      bnd(:,2)', ...
      bnd(:,3)']);
xlabel('x');
ylabel('y');
zlabel('z');
legend;
view(3);
grid on
hold off

pos1 = get(gcf,'Position'); % get position of Figure(1) 

figure(2)
hold on

plot3(0, 0, 0,'o', ...
        'LineStyle','--','DisplayName',"origin");
plot3(points(:,1), points(:,2), points(:,3),'x', ...
        'LineStyle','--','DisplayName',"simplex vertices");

% epa plot
if ~any(epa_col_vect(:) == FLT_MIN)
    epa_plot = [0,0,0;
            epa_col_vect];
	plot3(epa_plot(:,1), ...
        epa_plot(:,2), ...
        epa_plot(:,3), ...
        'LineStyle','-','DisplayName',"epa closest vector", ...
        'LineWidth',3);
    
    % Sorting epa vertices
    sorted_epa_vertices = [epa_faces{1,1}; ...
                           epa_faces{1,2}; ...
                           epa_faces{1,3}];
    % We will use the rows of the epa_faces to filter the vertices
    % Since we epa_faces are the final product
    for c_idx = 2:height(epa_faces)
        % Each row of the cell has 3 vertices that form up a triangle face
        for row_idx = 1:3
            % Using ismember function to scan through the row
            [check,member_row] = ismember(...
                epa_faces{c_idx, row_idx}, ...
                sorted_epa_vertices, 'rows');
            % If check and not found, we add it into the vertices array
            if ~check
                sorted_epa_vertices( ...
                    height(sorted_epa_vertices)+1, :) = ...
                    epa_faces{c_idx, row_idx};
            end
        end
    end
    
    epa_face_idx = NaN(height(epa_faces),4);
    % We do another pass to put the index represented in sorted_epa_vertices
    % and place it into the epa_face_idx order
    for c_idx = 1:height(epa_faces)
        % Each row of the cell has 3 vertices that form up a triangle face
        for row_idx = 1:3
            % Using ismember function to scan through the row
            [check,member_row] = ismember(...
                epa_faces{c_idx, row_idx}, ...
                sorted_epa_vertices, 'rows');
            epa_face_idx(c_idx, row_idx) = member_row;
        end
    end
    
    patch('Faces',epa_face_idx,'Vertices',sorted_epa_vertices, ...
        'Facecolor',[0.8 0.3 0.4],'FaceAlpha',0.05, ...
        'LineStyle','-','DisplayName',"epa polyhedron");
end

patch('Faces',md_vert_faces,'Vertices',points, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.4, ...
    'LineStyle','--','DisplayName',"gjk simplex");

title('Minkowski Difference')
xlabel('x');
ylabel('y');
zlabel('z');
legend;
view(3);
grid on
hold off

set(gcf,'Position', get(gcf,'Position') + [0,0,150,0]); % When Figure(2) is not the same size as Figure(1)
pos2 = get(gcf,'Position');  % get position of Figure(2) 
set(gcf,'Position', pos2 + [pos1(3),0,0,0]) % Shift position of Figure(2)