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

%% GJK Winter Dev
% https://www.youtube.com/watch?v=MDusDn8oTSE
% https://blog.winter.dev/2020/gjk-algorithm/

% direction = [0.5 0.5 0];
% sup = support(vertexes{2}, vertexes{1}, direction);
% simplex_points(1,:) = sup;
% direction = -sup;
% 
% col = false;
% iter = 0;
tic
% while (true)
%     iter = iter + 1;
%     if height(simplex_points) > 4
%        error('Unacceptable value');
%     end
%     
%     if iter > height(vertexes{1})*height(vertexes{2})
%         col = true;
%         fprintf('Exceed iterations, collision found\n');
%         break;
%     end
% 
%     sup = support(vertexes{2}, vertexes{1}, direction);
%     if (dot(sup, direction) <= 0)
%         fprintf('Support and direction is not in same direction, no collision\n');
%         col = false;
%         break;
%     end
%     
%     simplex_points(height(simplex_points)+1,:) = sup;
%     
%     [simplex_points, direction, result] = ...
%         next_simplex(simplex_points);
%     
%     if result
%        col = true;
%        fprintf('Collision found due to tetrahedron returning %d\n', result);
%        break;
%     end
%     
% end

%% GJK mws262
% There are some faults with winter.dev conversion hence using GJK
% Collision Detection from mws262
% https://github.com/mws262/MATLAB-GJK-Collision-Detection
%Point 1 and 2 selection (line segment)
direction = [1 0 0];
[points] = simplex_line(direction,vertexes{2},vertexes{1});

%Point 3 selection (triangle)
[points,flag] = simplex_triangle(points,vertexes{2},vertexes{1});

%Point 4 selection (tetrahedron)
if flag == 1 %Only bother if we could find a viable triangle.
    [points,flag] = simplex_tetrahedron(points,vertexes{2},vertexes{1});
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
hold off


figure(2)
hold on
for i=1:height(points)
    plot_line(1,:) = points(i,:);
    if mod((i+1),height(points)) == 0
        div = height(points);
    else
        div = mod((i+1),height(points));
    end
    plot_line(2,:) = points(div,:);
    plot3(plot_line(:,1),plot_line(:,2),plot_line(:,3));
end
plot3(0, 0, 0,'o');
plot3(points(:,1), points(:,2), points(:,3),'x');

xlabel('x');
ylabel('y');
zlabel('z');
view(3);
grid on
hold off