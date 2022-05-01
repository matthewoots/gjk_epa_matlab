clc
clear
close all

addpath('functions');

% Boundary XYZ
bnd = [-10 -10 0;
       10 10 10];
num_line = 3;
buff = 2;
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


figure(1)
hold on

for i=1:num_line
    plot3(point_pairs{i}(:,1), point_pairs{i}(:,2), ...
    point_pairs{i}(:,3), '->');
end

for i=1:num_line
    patch('Faces',faces{i},'Vertices',vertexes{i}, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.3);
end

axis([bnd(:,1)', ...
      bnd(:,2)', ...
      bnd(:,3)']);
view(3);
xlabel('x');
ylabel('y');
zlabel('z');
grid on