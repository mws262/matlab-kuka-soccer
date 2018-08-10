function [v1, v2, v3] = get_verts_from_face_idx(face_idx, faces, verts)
fvert = verts(faces(face_idx,:),:);
v1 = fvert(1,:);
v2 = fvert(2,:);
v3 = fvert(3,:);
end

