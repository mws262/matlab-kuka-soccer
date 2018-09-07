function [rotation, current_pt, current_normal] = find_mesh_contact_tform(point_near_mesh, up_vector, approach_ang, mesh_data)
% Projects given point to mesh. Finds transformation from some plane
% defined by up_vector to plane defined by nearest normal and the approach
% angle.

validateattributes(point_near_mesh, {'numeric'}, {'2d', 'numel', 3, 'real'});
validateattributes(up_vector, {'numeric'}, {'2d', 'numel', 3, 'real'});
validateattributes(approach_ang, {'numeric'}, {'scalar', 'real'});
validateattributes(mesh_data, {'struct'}, {});

[ ~, current_pt, current_normal ] = point2trimesh_with_normals( point_near_mesh, mesh_data );

rotation = get_rotation_from_vecs(up_vector, -current_normal);
rotation = axang2rotm([-current_normal, approach_ang])*rotation;
end