function [dir] = same_direction(direction, ao)
    dir = dot(direction, ao) > 0;
end

