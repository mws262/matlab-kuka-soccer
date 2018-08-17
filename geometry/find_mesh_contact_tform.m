function [rotation, current_pt, current_normal] = find_mesh_contact_tform(point_near_mesh, up_vector, approach_ang, faces, vertices, face_normals, vertex_normals)
% Projects given point to mesh. Finds transformation from some plane
% defined by up_vector to plane defined by nearest normal and the approach
% angle.

[ distance, current_pt, current_normal ] = point2trimesh_with_normals( point_near_mesh, faces, vertices, face_normals, vertex_normals );
% if distance > 1e-3
%     warning('Given initial surface point in velocity integration is further away from the mesh than the threshold. This may be totally ok, but it could be a sign that geometry elsewhere is fubar.');
% end
rotation = get_rotation_from_vecs(up_vector, -current_normal);
rotation = axang2rotm([-current_normal, approach_ang])*rotation;
end