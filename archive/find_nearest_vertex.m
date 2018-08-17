function [nearest_vert, nearest_vert_idx] = find_nearest_vertex(point, verts)
[min_val, nearest_vert_idx] = min(sum((verts - point).^2, 2));
nearest_vert = verts(nearest_vert_idx, :);
end