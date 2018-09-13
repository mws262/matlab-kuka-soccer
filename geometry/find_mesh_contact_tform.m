function [complete_transform, untransformed_surface_point, untransformed_surface_normal] = find_mesh_contact_tform(mesh_data, point_near_mesh, ... 
    destination_normal, twist_angle_about_normal, destination_location)
% FIND_MESH_CONTACT_TFORM Finds a 4x4 transform to put some face of a
% triangle mesh tangent to some place.
%
% [total_tform, untformed_surface_pt, untformed_surface_normal] = FIND_MESH_CONTACT_TFORM(point_near_mesh, destination_normal, ... 
%    twist_angle_about_normal, mesh_data, destination_location)
%
% [total_tform, untformed_surface_pt, untformed_surface_normal] = FIND_MESH_CONTACT_TFORM(point_near_mesh, destination_normal, ... 
%    twist_angle_about_normal, mesh_data)
%
%   Inputs:
%       `mesh_data` -- Structure with faces, vertices, face_normals, and
%       vertex_normals data.
%       `point_near_mesh` -- Point in untransformed mesh coordinates. This
%       gets projected to the surface of the mesh to find a contact
%       location.
%       `destination_normal` -- Transform will make the projected surface
%       point be OPPOSITE this normal. For example, we give an outward
%       surface normal vector for the ball, and this will match the normal
%       against the ball.
%       `twist_angle_about_normal` -- Rotates the mesh about the selected
%       point's normal by this angle (in radians). Does this transform
%       before aligning the normals.
%       `destination_location` -- (OPTIONAL) The projected surface point will be
%       translated to coincide with this point. If not specified, then this
%       additional translation will not occur.
%
%   Outputs:
%       `complete_transform` -- 4x4 rotation/translation transformation
%       which is the product of all sub-transformations needed to align the
%       mesh's normal as specified.
%       `untransformed_surface_point' -- The point on the untransformed
%       mesh that we projected from `point_near_mesh`. This is a
%       sometimes-useful sub-result from POINT2TRIMESH_WITH_NORMALS, to
%       avoid duplicating computations.
%       `untransformed_surface_normal' -- The normal vector on the 
%       untransformed mesh that we projected from `point_near_mesh`. This 
%       is a sometimes-useful sub-result from POINT2TRIMESH_WITH_NORMALS,
%       to avoid duplicating computations.
%
%   See also POINT2TRIMESH_WITH_NORMALS, VALIDATE_MESH_STRUCT, TEST_FIND_MESH_CONTACT_TFORM.

validateattributes(point_near_mesh, {'numeric'}, {'vector', 'numel', 3, 'real'});
validateattributes(destination_normal, {'numeric'}, {'vector', 'numel', 3, 'real'});
validateattributes(twist_angle_about_normal, {'numeric'}, {'scalar', 'real'});
validateattributes(mesh_data, {'struct'}, {});

% Projection to untransformed mesh.
[ ~, untransformed_surface_point, untransformed_surface_normal ] = point2trimesh_with_normals( point_near_mesh, mesh_data );

surface_pt_to_origin = trvec2tform(-untransformed_surface_point); % Translate the original surface point to the origin.
twist_about_untformed_normal = rotm2tform(axang2rotm([untransformed_surface_normal, twist_angle_about_normal])); % Rotate about the normal vector.
untformed_surface_normal_to_target = rotm2tform(get_rotation_from_vecs(untransformed_surface_normal, -destination_normal)); % Rotation taking the original normal vector to the opposite of the target vector.

if nargin == 5
    validateattributes(destination_location, {'numeric'}, {'vector', 'numel', 3, 'real'});
    origin_to_destination = trvec2tform(destination_location); % Translate a point at the origin to the speficied destination.
else
    origin_to_destination = trvec2tform(untransformed_surface_point); % Translate directly back if no destination is specified.
end

% Compose the final transform:
complete_transform = origin_to_destination*untformed_surface_normal_to_target*twist_about_untformed_normal*surface_pt_to_origin;
end