function vert_norm = get_vertex_normal(vertex_idx, faces, vertices, normals)
connected_faces = find(any(faces == vertex_idx, 2));
vertex_val = vertices(vertex_idx, :);
angle_sum = 0;
normals_sum = 0;
for i = 1:length(connected_faces)
[v1, v2, v3] = get_verts_from_face_idx(connected_faces(i), faces, vertices);
face_normal = normals(connected_faces(i),:);

vert_diffs = [v1; v2; v3] - vertex_val;
nonzero_idx = find(max(abs(vert_diffs),[],2) ~= 0);
vert_cross = cross(v1, v2) + cross(v2, v3) + cross(v3, v1); % Only one should be nonzero.
    
vert_cross_n = norm(vert_cross)/(norm(vert_diffs(nonzero_idx(1), :))*norm(vert_diffs(nonzero_idx(2), :)));
% Values can get the tiniest bit over 1. This leads to imaginary asin.
if vert_cross_n > 1
    vert_cross_n = 1;
elseif vert_cross_n < -1
    vert_cross_n = -1;
end

angle = asin(vert_cross_n);
assert(angle >= 0);

normals_sum = normals_sum + face_normal * angle;
angle_sum = angle_sum + angle;

end

vert_norm = normals_sum/angle_sum;
vert_norm = vert_norm/norm(vert_norm);

end