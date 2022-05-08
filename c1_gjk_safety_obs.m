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
num_line = 2;
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
            col_pair(col,:) = [i j]; 
        end
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