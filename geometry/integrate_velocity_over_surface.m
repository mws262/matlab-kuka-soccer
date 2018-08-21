function [result_path_pts, result_path_normals, rotations, failure_flag] = integrate_velocity_over_surface(tspan, vspan, initial_position, up_vectors, approach_ang, patch_struct, varargin)
% Note: all resulting points are relative to untransformed (i.e. the input) version of the
% mesh.
% varargin can include a convex patch which represents banned areas.
check_for_banned = false;
if ~isempty(varargin)
    check_for_banned = true;
    banned_regions = varargin{1};
end

failure_flag = false; % No failure unless otherwise detected.

% Unpack mesh info.
faces = patch_struct.faces;
vertices = patch_struct.vertices;
face_normals = patch_struct.face_normals;
vertex_normals = patch_struct.vertex_normals;

% Figure out initial position stuff.
[rotation, current_pt, current_normal] = find_mesh_contact_tform(initial_position, up_vectors(1,:), approach_ang, faces, vertices, face_normals, vertex_normals);
% init_normal = current_normal;
% Prepare output value arrays.
total_steps = length(tspan);
result_path_pts = zeros(total_steps,3);
result_path_normals = zeros(total_steps,3);
rotations = zeros(3,3,total_steps);

% Add initial points.
result_path_pts(1,:) = current_pt;
result_path_normals(1,:) = current_normal;
rotations(:,:,1) = rotation;

% Integration loop.
for i = 1:total_steps - 1
    % Replaced euler below with RK4. Hanging onto this fragment until we've
    % tested it more.
%     dt = tspan(i + 1) - tspan(i);
%     vel = -vspan(i,:);
%     
%     pt_star = current_pt + (rotation*vel')'*dt; % Take a step with the current transform.
%     
%     [ ~, surface_point, new_normal_vec ] = point2trimesh_with_normals( pt_star, faces, vertices, face_normals, vertex_normals );
%     current_pt = surface_point; % The next surface point will actually be this projection.
%     
%     % Check if the integration has entered a banned area if a banned_region
%     % was given in the arguments.
%     if check_for_banned
%        if check_pt_inside_mesh(current_pt, banned_regions)
%           failure_flag = true;
% %           warning('Wow, we integrated along the surface right into a banned area! Cool.');
%           return;
%        end
%     end
%     
%     
%  
%     rotation = get_rotation_from_vecs(current_normal, new_normal_vec) * rotation; % Also advancing the transform based on the evolution of the normals.
% %get_rotation_from_vecs(up_vectors(i - 1,:), up_vectors(i, :)) *
% 
%     current_normal = new_normal_vec;
%     
%     rotation = modified_gram_schmidt(rotation); % Fix the rotation matrix.
%     
%     result_path_pts(i,:) = current_pt;
%     result_path_normals(i,:) = current_normal;
%     rotations(:,:,i) = rotation;
%     
    %%%%%%
        dt = tspan(i + 1) - tspan(i);
        % Using current velocity.
        vel_k1 = -vspan(i,:);
        k_1 = (rotation*vel_k1')';
        
        % Half-step velocity evaluated with initial velocity.
        pt_star_k2 = current_pt + 0.5 * k_1 * dt;
        [ ~, ~, new_normal_vec_k2 ] = point2trimesh_with_normals( pt_star_k2, faces, vertices, face_normals, vertex_normals );
        rotation_k2 = get_rotation_from_vecs(current_normal, new_normal_vec_k2) * rotation;
        vel_k2 = -(vspan(i,:) + vspan(i + 1,:))/2; % Just assume average.
        k_2 = (rotation_k2*vel_k2')';
        
        % Half-step velocity evaluated with intermediate velocity.
        pt_star_k3 = current_pt + 0.5 * k_2 * dt;
        [ ~, ~, new_normal_vec_k3 ] = point2trimesh_with_normals( pt_star_k3, faces, vertices, face_normals, vertex_normals );
        rotation_k3 = get_rotation_from_vecs(current_normal, new_normal_vec_k3) * rotation;
        vel_k3 = vel_k2;
        k_3 = (rotation_k3*vel_k3')';
        
        % Whole step velocity evaluated with second intermediate velocity.
        pt_star_k4 = current_pt + k_3 * dt;
        [ ~, ~, new_normal_vec_k4 ] = point2trimesh_with_normals( pt_star_k4, faces, vertices, face_normals, vertex_normals );
        rotation_k4 = get_rotation_from_vecs(current_normal, new_normal_vec_k4) * rotation;
        vel_k4 = -vspan(i + 1,:);
        k_4 = (rotation_k4*vel_k4')';
    
        % Weight them and actually step forward.
        new_pt_star = current_pt + (1/6) * (k_1+2*k_2+2*k_3+k_4) * dt;
        [ ~, surface_point, new_normal_vec ] = point2trimesh_with_normals( new_pt_star, faces, vertices, face_normals, vertex_normals );
        % Check if the integration has entered a banned area if a banned_region
        % was given in the arguments.
        if check_for_banned
            if check_pt_inside_mesh(surface_point, banned_regions)
                failure_flag = true;
                return;
            end
        end
        
        rotation = get_rotation_from_vecs(current_normal, new_normal_vec) * rotation;

        current_pt = surface_point;
        current_normal = new_normal_vec;
        rotation = modified_gram_schmidt(rotation); % Fix the rotation matrix.
        
        result_path_pts(i + 1,:) = current_pt;
        result_path_normals(i + 1,:) = current_normal;
        rotations(:,:,i + 1) = rotation;
end
end
