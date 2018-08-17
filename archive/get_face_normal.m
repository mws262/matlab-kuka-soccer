function normvec = get_face_normal(face_idx, faces, verts)
[v1, v2, v3] = get_verts_from_face_idx(face_idx, faces, verts)
normvec = cross(v1 - v2, v3 - v2); % Clockwise wound?
end