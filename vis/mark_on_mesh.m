function mark_on_mesh(face_idxs, patch_to_mark)
%MARK_ON_MESH Mark selected face in red. Mark adjacent ones in yellow for
%visibility.
assert(~isempty(patch_to_mark.FaceVertexCData), 'Patch does not have FaceVertexCData assigned. Assign this before calling mark_on_mesh, AND set FaceColor to flat or something.');
for k = 1:length(face_idxs)
    face_idx = face_idxs(k);
    adj_verts = patch_to_mark.Faces(face_idx,:);
    patch_to_mark.FaceVertexCData(face_idx,:) = [1,0,0];
    for i = 1:3
        vert = adj_verts(i);
        adj_faces = find(any(patch_to_mark.Faces == vert,2));
        for j = 1:length(adj_faces)
            if ~any(face_idxs == adj_faces(j)) % Not the original patch.
                patch_to_mark.FaceVertexCData(adj_faces(j),:) = [1,0.8,0.2];
            end
        end
    end
end

