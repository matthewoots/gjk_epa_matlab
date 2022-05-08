function [poly] = translate_polygon(...
    polygon, translation)
    for i = 1:height(polygon)
        poly(i,:) = polygon(i,:) + translation;
    end
end

