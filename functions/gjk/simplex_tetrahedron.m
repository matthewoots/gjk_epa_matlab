function [npoints, result] = simplex_tetrahedron(points, shape1, shape2)
    %% GJK Winter Dev
    % https://blog.winter.dev/2020/gjk-algorithm/ 
    % From C++ to matlab
    %     a = points(1,:); b = points(2,:); c = points(3,:); d = points(4,:);
    %     ab = b - a;
    %     ac = c - a;
    %     ad = d - a;
    %     ao = -a;
    %     
    %     abc = cross(ab,ac);
    %     acd = cross(ac,ad);
    %     adb = cross(ad,ab);
    %     
    %     if same_direction(abc, ao)
    %         [npoints, ndir, result] = ...
    %             simplex_triangle([a ; b ; c]);
    %         return;
    %     end
    %     if same_direction(acd, ao)
    %         [npoints, ndir, result] = ...
    %             simplex_triangle([a ; c ; d]);
    %         return;
    %     end
    %     if same_direction(adb, ao)
    %         [npoints, ndir, result] = ...
    %             simplex_triangle([a ; d ; b]);
    %         return;
    %     end
    %     
    %     npoints = [0 0 0];
    %     ndir = [0 0 0];
    %     result = true;
    %     return   

    %% GJK mws262
    % There are some faults with winter.dev conversion hence using GJK
    % Collision Detection from mws262
    % https://github.com/mws262/MATLAB-GJK-Collision-Detection
    result = 0;
    
    a = points(1,:);
    b = points(2,:);
    c = points(3,:);

    ab = b-a;
    ac = c-a;

    %Normal to face of triangle
    abc = cross(ab,ac);
    ao = -a;

    if dot(abc, ao) > 0 %Above
        d = c;
        c = b;
        b = a;

        v = abc;
        a = support(shape2,shape1,v); %Tetrahedron new point

    else %below
        d = b;
        b = a;
        v = -abc;
        a = support(shape2,shape1,v); %Tetrahedron new point
    end

    for i = 1:height(shape1)*height(shape2) %Allowing 10 tries to make a good tetrahedron.
        %Check the tetrahedron:
        ab = b-a;
        ao = -a;
        ac = c-a;
        ad = d-a;

        %We KNOW that the origin is not under the base of the tetrahedron based on
        %the way we picked a. So we need to check faces ABC, ABD, and ACD.

        %Normal to face of triangle
        abc = cross(ab,ac);

        if dot(abc, ao) > 0 %Above triangle ABC
            %No need to change anything, we'll just iterate again with this face as
            %default.
        else
            acd = cross(ac,ad);%Normal to face of triangle

            if dot(acd, ao) > 0 %Above triangle ACD
                %Make this the new base triangle.
                b = c;
                c = d;
                ab = ac;
                ac = ad;            
                abc = acd;     
            elseif dot(acd, ao) < 0
                adb = cross(ad,ab);%Normal to face of triangle

                if dot(adb, ao) > 0 %Above triangle ADB
                    %Make this the new base triangle.
                    c = b;
                    b = d;              
                    ac = ab;
                    ab = ad;
                    abc = adb;           
                else
                    result = 1; 
                    break; %It's inside the tetrahedron.
                end
            end
        end

        %try again:
        if dot(abc, ao) > 0 %Above
            d = c;
            c = b;
            b = a;    
            v = abc;
            a = support(shape2,shape1,v); %Tetrahedron new point
        else %below
            d = b;
            b = a;
            v = -abc;
            a = support(shape2,shape1,v); %Tetrahedron new point
        end
    end
    
    npoints(1,:) = a;
    npoints(2,:) = b;
    npoints(3,:) = c;
    npoints(4,:) = d;
    
end

