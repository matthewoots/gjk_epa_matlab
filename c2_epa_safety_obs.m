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
obs_faces = cell(1,num_obs); 
obs_vertices = cell(1,num_obs);

%% Add obstacles
for i=1:num_obs
    [face, vertex, center] = gen_rand_cuboid( ...
    bnd, max_size);
    obs_faces{i} = face; obs_vertices{i} = vertex; 
    obs_centers(i,:) = center;
end

%% Add random path 
num_line = 3;
buff = 1.5;
faces = cell(1,num_line); vertexes = cell(1,num_line);

point_pairs = cell(1,num_line);

for i=1:num_line+1
    bnd_diff = bnd(2,:) - bnd(1,:);
    
    rnd_point(i,:) = bnd_diff .* rand(1,3) + bnd(1,:);
end

for i=1:num_line
    point_pairs{i} = [rnd_point(i,:);
                      rnd_point(i+1,:)];
end

rots = cell(1,num_line);
for i=1:num_line
    vect = point_pairs{i}(2,:) - point_pairs{i}(1,:);
    vect_norm = vect / norm(vect);
    % atan(O=x/A=y)
    yaw = atan(vect_norm(1)/vect_norm(2));
    unrot_vect = eul2rotm([yaw, 0, 0]) * vect_norm';
    pitch = atan(unrot_vect(3)/unrot_vect(2));
    
    % atan(O=z/A=xy)
    rotmZYX = eul2rotm([-yaw, 0, pitch]);
    bound = [buff norm(vect)+buff buff];
    
    origin = (point_pairs{i}(2,:) + point_pairs{i}(1,:)) / 2;
    
    [face, vertex] = gen_cuboid( ...
        origin, bound, rotmZYX);
    faces{i} = face; vertexes{i} = vertex; 
    
end

%% GJK mws262
% There are some faults with winter.dev conversion hence using GJK
% Collision Detection from mws262
% https://github.com/mws262/MATLAB-GJK-Collision-Detection
% Point 1 and 2 selection (line segment)
col = 0;
col_pair = [];
for i=1:num_line
    
    % for every obs
    for j=1:num_obs
        
        gjk_start = tic;

        direction = [1 0 0];
        [points] = simplex_line(direction,obs_vertices{j},vertexes{i});

        %Point 3 selection (triangle)
        [points,flag] = simplex_triangle(points,obs_vertices{j},vertexes{i});

        %Point 4 selection (tetrahedron)
        if flag == 1 %Only bother if we could find a viable triangle.
            [points,flag] = simplex_tetrahedron(points,obs_vertices{j},vertexes{i});
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
        if flag == 1
            col = col + 1;
            % i = path, j = obs
            col_pair(col,:) = [i j]; 
            col_points{col} = points; 
        end
    end
end

%% EPA 
% https://github.com/kevinmoran/GJK/blob/master/GJK.h
FLT_MIN = -10000;
epa_col_vect = [FLT_MIN,FLT_MIN,FLT_MIN];

if ~isempty(col_pair)
for idx = 1:height(col_pair)
    
    epa_start = tic;
    
    EPA_TOLERANCE = 0.0001;
    EPA_MAX_NUM_FACES = 64;
    EPA_MAX_NUM_LOOSE_EDGES = 32;
    EPA_MAX_NUM_ITERATIONS = 64;
    
    points = col_points{idx};
    path_idx = col_pair(idx,1); obs_idx = col_pair(idx,2);

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
        p = support(obs_vertices{obs_idx}, vertexes{path_idx}, search_dir);
        
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
    
    path_col_pair(idx,:) = [path_idx epa_col_vect];
end
end

%% Plotting
figure(1)
hold on
plot3(obs_centers(:,1), obs_centers(:,2), obs_centers(:,3), ...
    'x','DisplayName','centroids');

for i=1:num_obs
    patch('Faces',obs_faces{i},'Vertices',obs_vertices{i}, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.3, ...
    'LineStyle','--','DisplayName',"obs "+num2str(i));
end

for i=1:num_line
    if isempty(col_pair)
        patch('Faces',faces{i},'Vertices',vertexes{i}, ...
        'Facecolor',[0.8 0.3 1],'FaceAlpha',0.3, ...
        'LineStyle','--','DisplayName',"polygon "+num2str(i));
        continue;
    end
    
    if any(col_pair(:,1) == i)
        patch('Faces',faces{i},'Vertices',vertexes{i}, ...
        'Facecolor',[0.1 0.3 0.3],'FaceAlpha',0.3, ...
        'LineStyle','--','DisplayName',"polygon(col) "+num2str(i));
    
        % epa plot
        if any(path_col_pair(:,1) == i)
            % Using ismember function to scan through the row
            [check,member_row] = ismember(...
                i, path_col_pair(:,1), 'rows');
            
            t_vertexes = translate_polygon(vertexes{i}, ...
                path_col_pair(member_row(1), 2:4));
            
            patch('Faces',faces{i},'Vertices',t_vertexes, ...
            'Facecolor',[0.8 0.3 0.4],'FaceAlpha',0.3, ...
            'LineWidth',0.5,'DisplayName',"transformed "+num2str(i));
        end
    else
        patch('Faces',faces{i},'Vertices',vertexes{i}, ...
        'Facecolor',[0.8 0.3 1],'FaceAlpha',0.3, ...
        'LineStyle','--','DisplayName',"polygon "+num2str(i));
    end
    
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