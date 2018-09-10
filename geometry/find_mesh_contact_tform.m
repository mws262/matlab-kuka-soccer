function [total_tform, untformed_surface_pt, untformed_surface_normal] = find_mesh_contact_tform(point_near_mesh, destination_normal, ... 
    destination_location, normal_twist_angle, mesh_data)

% Projects given point to mesh. Finds transformation from some plane
% defined by up_vector to plane defined by nearest normal and the approach
% angle.
% up_vector -- we want the surface normal to point AGAINST this. E.g.
% up_vector would be ball normal. We want to match surface normal AGAINST
% this.
% current_pt and current_normal are before any transformation. Just the
% projected stuff.

validateattributes(point_near_mesh, {'numeric'}, {'vector', 'numel', 3, 'real'});
validateattributes(destination_normal, {'numeric'}, {'vector', 'numel', 3, 'real'});
validateattributes(destination_location, {'numeric'}, {'vector', 'numel', 3, 'real'});
validateattributes(normal_twist_angle, {'numeric'}, {'scalar', 'real'});
validateattributes(mesh_data, {'struct'}, {});

% Projection to untransformed mesh.
[ ~, untformed_surface_pt, untformed_surface_normal ] = point2trimesh_with_normals( point_near_mesh, mesh_data );

surface_pt_to_origin = trvec2tform(-untformed_surface_pt); % Translate the original surface point to the origin.
twist_about_untformed_normal = rotm2tform(axang2rotm([untformed_surface_normal, normal_twist_angle])); % Rotate about the normal vector.
untformed_surface_normal_to_target = rotm2tform(get_rotation_from_vecs(untformed_surface_normal, -destination_normal)); % Rotation taking the original normal vector to the opposite of the target vector.
origin_to_destination = trvec2tform(destination_location); % Translate a point at the origin to the speficied destination.

total_tform = origin_to_destination*untformed_surface_normal_to_target*twist_about_untformed_normal*surface_pt_to_origin;
end