clc
clear
close all

addpath('functions');

% Boundary XYZ
bnd = [-10 -10 0;
       10 10 10];
max_size = 6; % For scaling properties

%% Obstacle creation
num_obs = 2;
faces = cell(1,num_obs); vertexes = cell(1,num_obs);

for i=1:num_obs
    [face, vertex, center] = gen_rand_cuboid( ...
    bnd, max_size);
    faces{i} = face; vertexes{i} = vertex; 
    centers(i,:) = center;
end

%% Ray creation
num_line = 2;

point_pairs = cell(1,num_line);

for i=1:num_line+1
    bnd_diff = bnd(2,:) - bnd(1,:);
    
    rnd_point(i,:) = bnd_diff .* rand(1,3) + bnd(1,:);
end

for i=1:num_line
    point_pairs{i} = [rnd_point(i,:);
                      rnd_point(i+1,:)];
end

%% Ray tracing

ray_start = tic;
intersect_count = 0;
face_count = 0;

for i=1:num_line
    % Get the line direction
    line_dir = point_pairs{i}(2,:) - point_pairs{i}(1,:);
    line_length = norm(line_dir);
    line_dir = line_dir / norm(line_dir);
    line_origin = point_pairs{i}(1,:);
    
    for j=1:num_obs
        for k=1:height(faces{j})
            fprintf('obs %d face %d\n', j, k);
            % i (line), j (obs), k (faces in the obs)
            % Get centroid of the face
            face_center = [0 0 0];
            for f=1:4
                % Each face is a rectangle = 4 vertices
                face_center = face_center + vertexes{j}(faces{j}(k,f),:);
            end
            % Average out the sum of the 4 vertices
            face_center = face_center ./ 4;
            
            ba = vertexes{j}(faces{j}(k,2),:) - vertexes{j}(faces{j}(k,1),:);
            da = vertexes{j}(faces{j}(k,4),:) - vertexes{j}(faces{j}(k,1),:);
            
            % find current winding for vertices and face
            vert_face = [vertexes{j}(faces{j}(k,1),:) ;
                        vertexes{j}(faces{j}(k,2),:) ;
                        vertexes{j}(faces{j}(k,3),:) ;
                        vertexes{j}(faces{j}(k,4),:)];
            
            % Get the normal
            normal = cross(ba, da);
            normal = normal / norm(normal);
            
            % Check that they (line and normal) are facing same direction if not we need to
            % flip the normal
            facing_dir = dot(normal,line_dir);
            % if they are not facing the same direction
            if ~(facing_dir > 0)
                normal = -normal;
            end
            
            [result, point] = ray_intersect_quad_face(...
                normal, face_center, line_dir, line_origin, ...
                vert_face, line_length);
            
            face_count = face_count + 1;
            normals(face_count,:) = normal;
            normal_line(face_count,:) = face_center + normal .* 1;
            face_centers(face_count,:) = face_center;
            
            if result
                intersect_count = intersect_count + 1;
                intersect_array(intersect_count,:) = point;
            end
        end
    end
    fprintf('\n');
    
end

fprintf('Time of ray %.3f\n', toc(ray_start));




%% Plotting
figure(1)
hold on
plot3(centers(:,1), centers(:,2), centers(:,3), ...
    'x','DisplayName','centroids');
plot3(face_centers(:,1), face_centers(:,2), face_centers(:,3), ...
    'x','DisplayName','center of face','MarkerSize',2);

if intersect_count > 0
    for i=1:height(intersect_array)
        plot3(intersect_array(:,1), intersect_array(:,2), intersect_array(:,3), ...
            'o','DisplayName','intersection points','MarkerSize',5);
    end
end

for i=1:height(normal_line)
    face_line = [face_centers(i,:); normal_line(i,:)];
    plot3(face_line(:,1), face_line(:,2), face_line(:,3), ...
        'DisplayName','center of face','MarkerSize',2);
end

for i=1:num_obs
    patch('Faces',faces{i},'Vertices',vertexes{i}, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.3, ...
    'LineStyle','--','DisplayName',"polygon "+num2str(i));
end

for i=1:num_line
    plot3(point_pairs{i}(:,1), point_pairs{i}(:,2), ...
    point_pairs{i}(:,3), '->','DisplayName',"line "+num2str(i));
end

title('Polygon 3D space')
axis([bnd(:,1)', ...
      bnd(:,2)', ...
      bnd(:,3)']);
xlabel('x');
ylabel('y');
zlabel('z');
legend off;
view(3);
grid on
hold off