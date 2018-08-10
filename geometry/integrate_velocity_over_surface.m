function [result_path_pts, result_path_normals] = integrate_velocity_over_surface(tspan, vspan, initial_position, up_vector, patch_struct)
% Note: all resulting points are relative to untransformed (i.e. the input) version of the
% mesh.

% Unpack mesh info.
faces = patch_struct.faces;
vertices = patch_struct.vertices;
face_normals = patch_struct.face_normals;
vertex_normals = patch_struct.vertex_normals;

% Figure out initial position stuff.
[ distance, current_pt, current_normal ] = point2trimesh_with_normals( initial_position, faces, vertices, face_normals, vertex_normals );
if distance > 1e-6
    warning('Given initial surface point in velocity integration is further away from the mesh than the threshold. This may be totally ok, but it could be a sign that geometry elsewhere is fubar.');
end
tform = get_rotation_from_vecs(up_vector, current_normal); % TODO: THIS IS JUST A SIMPLE CASE. GENERALIZE.

% Prepare output value arrays.
total_steps = length(tspan) - 1;
result_path_pts = zeros(total_steps,3);
result_path_normals = zeros(total_steps,3);

% Integration loop.
for i = 1:total_steps
    dt = tspan(i + 1) - tspan(i);
    vel = vspan(i,:);
    
    pt_star = current_pt + (tform*vel')'*dt; % Take a step with the current transform.
    
    [ ~, surface_point, new_normal_vec ] = point2trimesh_with_normals( pt_star, faces, vertices, face_normals, vertex_normals );
    current_pt = surface_point; % The next surface point will actually be this projection.
    
    tform = get_rotation_from_vecs(current_normal, new_normal_vec)*tform; % Also advancing the transform based on the evolution of the normals.
    current_normal = new_normal_vec;
    
    result_path_pts(i,:) = current_pt;
    result_path_normals(i,:) = current_normal;
    
end

end