clc
clear
close all

addpath('functions');

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


%% Plotting
figure(1)
hold on

plot3(centers(:,1), centers(:,2), centers(:,3),'x');

for i=1:num_obs
    patch('Faces',faces{i},'Vertices',vertexes{i}, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.3);
end

title('Polygon 3D space')
axis([bnd(:,1)', ...
      bnd(:,2)', ...
      bnd(:,3)']);
xlabel('x');
ylabel('y');
zlabel('z');
view(3);
grid on