function [npoints, result] = simplex_triangle(points, shape1, shape2)
    %% GJK Winter Dev
    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    %     a = points(1,:); b = points(2,:); c = points(3,:);
    %     ab = b - a;
    %     ac = c - a;
    %     ao = -a;
    %     
    %     abc = cross(ab,ac);
    %     
    %     if same_direction(cross(abc,ac), ao)
    %         if same_direction(ac, ao)
    %             npoints = [a ; c];
    %             ndir = cross(cross(ac,ao),ac);
    %         else
    %             [npoints, ndir, result] = ...
    %                 simplex_line([a ; b]);
    %             return;
    %         end
    %     else
    %         if same_direction(cross(ab,abc), ao)
    %             [npoints, ndir, result] = ...
    %                 simplex_line([a ; b]);
    %             return;
    %         else
    %             if same_direction(abc, ao)
    %                 npoints = points;
    %                 ndir = abc;
    %             else
    %                 npoints = [a ; c ; b];
    %                 ndir = -abc;
    %             end
    %         end
    %     end
    %     
    %     result = false;
    
    %% GJK mws262
    % There are some faults with winter.dev conversion hence using GJK
    % Collision Detection from mws262
    % https://github.com/mws262/MATLAB-GJK-Collision-Detection
    result = 0; %So far, we don't have a successful triangle.

    a = points(1,:);
    b = points(2,:);
    
    %First try:
    ab = b-a;
    ao = -a;
    v = cross(cross(ab,ao),ab); % v is perpendicular to ab pointing in the general direction of the origin.

    c = b;
    b = a;
    a = support(shape2,shape1,v);

    for i = 1:height(shape1)*height(shape2) %iterations to see if we can draw a good triangle.
        %Time to check if we got it:
        ab = b-a;
        ao = -a;
        ac = c-a;

        %Normal to face of triangle
        abc = cross(ab,ac);

        %Perpendicular to AB going away from triangle
        abp = cross(ab,abc);
        %Perpendicular to AC going away from triangle
        acp = cross(abc,ac);

        %First, make sure our triangle "contains" the origin in a 2d projection
        %sense.
        %Is origin above (outside) AB?   
        if dot(abp,ao) > 0
            c = b; %Throw away the furthest point and grab a new one in the right direction
            b = a;
            v = abp; %cross(cross(ab,ao),ab);

            %Is origin above (outside) AC?
        elseif dot(acp, ao) > 0
            b = a;
            v = acp; %cross(cross(ac,ao),ac);

        else
            result = 1;
            break; %We got a good one.
        end
        a = support(shape2,shape1,v);
    end

    npoints(1,:) = a;
    npoints(2,:) = b;
    npoints(3,:) = c;
end

