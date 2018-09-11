function quiverobj = draw_all_normals(scaling, mesh_data)
centers = (mesh_data.vertices(mesh_data.faces(:,1),:) + mesh_data.vertices(mesh_data.faces(:,2),:) + mesh_data.vertices(mesh_data.faces(:,3),:))/3;
quiverobj = quiver(0,0,0,0);
quiverobj.AutoScale = 'off';
quiverobj.XData = centers(:,1)';
quiverobj.YData = centers(:,2)';
quiverobj.ZData = centers(:,3)';
quiverobj.UData = mesh_data.face_normals(:,1)' * scaling;
quiverobj.VData = mesh_data.face_normals(:,2)' * scaling;
quiverobj.WData = mesh_data.face_normals(:,3)' * scaling;
end

