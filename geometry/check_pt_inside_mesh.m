function is_inside = check_pt_inside_mesh(point, patch_struct)
%CHECK_PT_INSIDE_MESH Check if a provided point is inside a given CONVEX
%mesh. Does NOT check whether the normals are correctly oriented. They
%should be facing outward. Points on the surface do count as inside.

assert(isrow(point) || iscolumn(point), 'Can only handle a single query point. This must be a row or column.');
assert(length(point) == 3, 'Given point has more or less than 3 elements: %d\n.', length(point));

if iscolumn(point) % Make it tolerate either row or column query point.
    point = point';
end

% Subtract a face vertex from the point, dot with face normal. Positive
% means outside, negative means inside.
is_inside = ~any(dot(patch_struct.face_normals, point - patch_struct.vertices(patch_struct.faces(:,1),:),2) > 0);

end

