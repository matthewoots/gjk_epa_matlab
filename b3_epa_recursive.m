

addpath('functions');
addpath('functions/gjk');
addpath('functions/epa');

max_run = 50;
for run_count = 1:max_run

max_run = 50;
timer_start = tic;
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
%Point 1 and 2 selection (line segment)
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
    EPA_MAX_NUM_FACES = 64;
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
    
    for iterations = 1:EPA_MAX_NUM_ITERATIONS
        % Find face that's closest to origin
        min_dist = dot(epa_faces{1,1}, epa_faces{1,4});
        closest_face = 1;
        for i = 2:num_faces
            dist = dot(epa_faces{i,1}, epa_faces{i,4});
            if dist < min_dist && ...
                    ~any(isnan(epa_faces{i,4}) == 1)
                min_dist = dist;
                closest_face = i;
            end
        end
        
        % Search normal to face that's closest to origin
        search_dir = epa_faces{closest_face,4}; 
        p = support(vertexes{2}, vertexes{1}, search_dir);
        
        if dot(p, search_dir) - min_dist < EPA_TOLERANCE
            % Convergence (new point is not significantly further from origin)
            epa_col_vect = ...
                epa_faces{closest_face, 4} * ...
                dot(p, search_dir); 
            fprintf("[EPA] exit CONVERGENCE\n");
            % dot vertex with normal to resolve collision along normal
            break;
        end
        
        % keep track of edges we need to fix after removing faces
        num_loose_edges = 0;
        loose_edges = {};
        % Find all triangles that are facing p
        for i = 1:num_faces
            
            % triangle i faces p, remove it
            if (dot(epa_faces{i,4}, ...
                p-epa_faces{i,1}) > 0) 
           
                % Add removed triangle's edges to loose edge list.
                % If it's already there, remove it (both triangles it belonged to are gone)
                for j = 1:3 % Three edges per face
                    
                    face_a = epa_faces{i,j};
                    face_b = epa_faces{i, mod((j+1),3)+1};
                    current_edge = {face_a, face_b};
                    found_edge = false;
                    
                    %Check if current edge is already in list
                    for k = 1:num_loose_edges 
                        if isequal(loose_edges{k,2}, current_edge{1}) && ...
                            isequal(loose_edges{k,1}, current_edge{2})
                            
                            % Edge is already in the list, remove it
                            % THIS ASSUMES EDGE CAN ONLY BE SHARED BY 2 TRIANGLES (which should be true)
                            % THIS ALSO ASSUMES SHARED EDGE WILL BE REVERSED IN THE TRIANGLES (which 
                            % should be true provided every triangle is wound CCW)
                            
                            loose_edges{k,1} = loose_edges{num_loose_edges,1}; 
                            % Overwrite current edge
                            loose_edges{k,2} = loose_edges{num_loose_edges,2}; 
                            % with last edge in list
                            num_loose_edges = num_loose_edges - 1;
                            
                            found_edge = true;
                            break; 
                            % exit loop because edge can only be shared once
                        end
                    end % endfor loose_edges
                    
                    if ~found_edge % add current edge to list
                        % assert(num_loose_edges<EPA_MAX_NUM_LOOSE_EDGES);
                        if num_loose_edges >= EPA_MAX_NUM_LOOSE_EDGES 
                            fprintf("[EPA] exit EPA_MAX_NUM_LOOSE_EDGES\n");
                            break;
                        end
                        loose_edges{num_loose_edges + 1,1} = ...
                            current_edge{1};
                        loose_edges{num_loose_edges + 1,2} = ...
                            current_edge{2};
                        num_loose_edges = num_loose_edges + 1;
                    end
                end % j = 1:3 % Three edges per face
                
                % Remove triangle i from list
                epa_faces{i,1} = epa_faces{num_faces,1};
                epa_faces{i,2} = epa_faces{num_faces,2};
                epa_faces{i,3} = epa_faces{num_faces,3};
                epa_faces{i,4} = epa_faces{num_faces,4};
                num_faces = num_faces - 1;
                i = i - 1;
                
            end % (dot(faces{i,4},p-faces{i,1}) > 0) 
        end % i = 1:num_faces
        
        % Reconstruct polytope with p added
        for i = 1:num_loose_edges
            % assert(num_faces<EPA_MAX_NUM_FACES);
            if num_faces >= EPA_MAX_NUM_FACES 
                fprintf("[EPA] exit EPA_MAX_NUM_FACES\n");
                break;
            end
            
            face_vect = cross(loose_edges{i,1} - loose_edges{i,2}, ...
                loose_edges{i,1} - p);
            
            epa_faces{num_faces+1,1} = loose_edges{i,1};
            epa_faces{num_faces+1,2} = loose_edges{i,2};
            epa_faces{num_faces+1,3} = p;
            epa_faces{num_faces+1,4} = face_vect/norm(face_vect);
            
            if any(face_vect == 0) && ...
                    norm(face_vect) == 0
                epa_faces{num_faces+1,4} = face_vect;
            end

            % Check for wrong normal to maintain CCW winding
            bias = 0.000001; % in case dot result is only slightly < 0 (because origin is on face)
            
            if dot(epa_faces{num_faces+1,1}, ...
                    epa_faces{num_faces+1,4}) + bias < 0
                
                temp = epa_faces{num_faces+1,1};
                epa_faces{num_faces+1,1} = epa_faces{num_faces+1,2};
                epa_faces{num_faces+1,2} = temp;
                epa_faces{num_faces+1,4} = -epa_faces{num_faces+1,4};
            end
            
            num_faces = num_faces + 1;
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
        'LineStyle','--','DisplayName',"epa closest vector");
end

patch('Faces',md_vert_faces,'Vertices',points, ...
    'Facecolor',[0.8 0.8 1],'FaceAlpha',0.2, ...
    'LineStyle','--','DisplayName',"gjk simplex");

title('Minkowski Difference')
xlabel('x');
ylabel('y');
zlabel('z');
legend;
view(3);
grid on
hold off

    
%% Set delay to real time 
dt = 0.75;
timer_end = toc(timer_start);
fprintf('run_count %d, time %.2f\n', ...
    run_count, timer_end);  

if timer_end < dt
   pause(dt - timer_end); 
end

if run_count ~= max_run
    clf % Clear figure
    clc
    clear
    close all
end

end