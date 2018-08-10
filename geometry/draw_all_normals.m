function [normals, centers, quiverobj] = draw_all_normals(scaling, faces, vertices)

normals = zeros(size(faces));
centers = zeros(size(faces));
for i = 1:size(faces,1)
    normals(i,:) = get_face_normal(i, faces, vertices);
    [v1,v2,v3] = get_verts_from_face_idx(i, faces, vertices);
    centers(i,:) = (v1 + v2 + v3)/3;
end
quiverobj = quiver(0,0,0,0);
quiverobj.AutoScale = 'off';
quiverobj.XData = centers(:,1)';
quiverobj.YData = centers(:,2)';
quiverobj.ZData = centers(:,3)';
quiverobj.UData = normals(:,1)' * scaling;
quiverobj.VData = normals(:,2)' * scaling;
quiverobj.WData = normals(:,3)' * scaling;
end

