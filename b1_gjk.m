clc
clear
close all

addpath('functions');
addpath('functions/gjk');

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

%% GJK
% https://www.youtube.com/watch?v=MDusDn8oTSE
% https://blog.winter.dev/2020/gjk-algorithm/

direction = [0.5 0.5 0];
sup = support(vertexes{2}, vertexes{1}, direction);
simplex_points(1,:) = sup;
direction = -sup;

col = false;
iter = 0;
tic
while (true)
    iter = iter + 1;
    if height(simplex_points) > 4
       error('Unacceptable value');
    end
    
    if iter > height(vertexes{1})*height(vertexes{2})
        col = true;
        fprintf('Exceed iterations, collision found\n');
        break;
    end

    sup = support(vertexes{2}, vertexes{1}, direction);
    if (dot(sup, direction) <= 0)
        fprintf('Support and direction is not in same direction, no collision\n');
        col = false;
        break;
    end
    
    simplex_points(height(simplex_points)+1,:) = sup;
    
    [simplex_points, direction, result] = ...
        next_simplex(simplex_points);
    
    if result
       col = true;
       fprintf('Collision found due to tetrahedron returning %d\n', result);
       break;
    end
    
end
fprintf('Time of GJK %.3f\n', toc);

%% Plotting
figure(1)
hold on

plot3(centers(:,1), centers(:,2), centers(:,3),'x');

for i=1:num_obs
    patch('Faces',faces{i},'Vertices',vertexes{i}, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.3);
end

axis([bnd(:,1)', ...
      bnd(:,2)', ...
      bnd(:,3)']);
xlabel('x');
ylabel('y');
zlabel('z');
view(3);
grid on