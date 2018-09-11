function [result_path_pts, result_path_normals, rotations, failure_flag] = integrate_velocity_over_surface(user_problem, user_options)
%problem.time_vector, vspan, initial_position, up_vectors, approach_ang, patch_struct, varargin)

%% Problem structure.
default_problem.time_vector = [];
default_problem.velocity_vector = [];
default_probelm.initial_surface_point = [];
default_problem.up_vectors = [];
default_problem.approach_ang = [];
default_problem.mesh_data = '';

%% Options structure.
default_options.check_banned_regions = false;
default_options.banned_region_mesh = '';

%% Return template problem / options.
if nargin == 0
    result_path_pts = default_problem;
    return;
elseif nargin == 1 % If single argument given, return either options or probelm structure.
    switch user_problem
        case 'options'
            result_path_pts = default_options;
        case 'problem'
            result_path_pts = default_problem;
        otherwise
            warning('Unrecognized single variable argument given: %s. Returning problem structure instead.', usr_problem);
            result_path_pts = default_problem;
    end
    return;
end

%% Merge user and default options.
problem = mergeOptions(default_problem, user_problem, 'Problem struct');
options = mergeOptions(default_options, user_options, 'Options struct');

% Correct transposes, if needed.
if isrow(problem.time_vector)
    problem.time_vector = problem.time_vector';
end

if size(problem.velocity_vector, 1) ~= length(problem.time_vector)
   if size(problem.velocity_vector, 2) ~= length(problem.time_vector)
       error('Input velocities do not match the dimension of the time span.');
   else
      problem.velocity_vector = problem.velocity_vector'; 
   end    
end

% Make sure inputs are valid.
validateattributes(problem.time_vector, {'single', 'double'}, {'real', 'vector', 'increasing', 'nonempty'});
validateattributes(problem.velocity_vector, {'single', 'double'}, {'real', '2d'});

validateattributes(options.check_banned_regions, {'logical'}, {'scalar'})
validate_mesh_struct(problem.mesh_data);

if options.check_banned_regions
    validate_mesh_struct(options.banned_region_mesh);
end

failure_flag = false; % No failure unless otherwise detected.

% Figure out initial position stuff.
[rotation, current_pt, current_normal] = find_mesh_contact_tform(initial_position, up_vectors(1,:), approach_ang, mesh_data);

% Prepare output value arrays.
total_steps = length(problem.time_vector);
result_path_pts = zeros(total_steps,3);
result_path_normals = zeros(total_steps,3);
rotations = zeros(3,3,total_steps);

% Add initial points.
% result_path_pts(1,:) = current_pt;
% result_path_normals(1,:) = current_normal;
% rotations(:,:,1) = rotation;

% Integration loop.
for i = 1:total_steps - 1
    dt = problem.time_vector(i + 1) - problem.time_vector(i);
    % Using current velocity.
    vel_k1 = -vspan(i,:);
    k_1 = (rotation*vel_k1')';
    
    % Half-step velocity evaluated with initial velocity.
    pt_star_k2 = current_pt + 0.5 * k_1 * dt;
    [ ~, ~, new_normal_vec_k2 ] = point2trimesh_with_normals( pt_star_k2, mesh_data );
    rotation_k2 = get_rotation_from_vecs(current_normal, new_normal_vec_k2) * rotation;
    vel_k2 = -(vspan(i,:) + vspan(i + 1,:))/2; % Just assume average.
    k_2 = (rotation_k2*vel_k2')';
    
    % Half-step velocity evaluated with intermediate velocity.
    pt_star_k3 = current_pt + 0.5 * k_2 * dt;
    [ ~, ~, new_normal_vec_k3 ] = point2trimesh_with_normals( pt_star_k3, mesh_data );
    rotation_k3 = get_rotation_from_vecs(current_normal, new_normal_vec_k3) * rotation;
    vel_k3 = vel_k2;
    k_3 = (rotation_k3*vel_k3')';
    
    % Whole step velocity evaluated with second intermediate velocity.
    pt_star_k4 = current_pt + k_3 * dt;
    [ ~, ~, new_normal_vec_k4 ] = point2trimesh_with_normals( pt_star_k4, mesh_data );
    rotation_k4 = get_rotation_from_vecs(current_normal, new_normal_vec_k4) * rotation;
    vel_k4 = -vspan(i + 1,:);
    k_4 = (rotation_k4*vel_k4')';
    
    % Weight them and actually step forward.
    new_pt_star = current_pt + (1/6) * (k_1+2*k_2+2*k_3+k_4) * dt;
    [ ~, surface_point, new_normal_vec ] = point2trimesh_with_normals( new_pt_star, mesh_data );
    % Check if the integration has entered a banned area if a banned_region
    % was given in the arguments.
    if options.check_banned_regions
        if check_pt_inside_mesh(surface_point, options.banned_region_mesh)
            failure_flag = true;
            return;
        end
    end
    
    rotation = get_rotation_from_vecs(current_normal, new_normal_vec) * rotation;
    
    current_pt = surface_point;
    current_normal = new_normal_vec;
    rotation = modified_gram_schmidt(rotation); % Fix the rotation matrix.
    
    result_path_pts(i,:) = current_pt;
    result_path_normals(i,:) = current_normal;
    rotations(:,:,i) = rotation;
end
% TODO FIX THIS PROPERLY. Just replicating the next to last value so the
% dimensions are nicer.
result_path_pts(end,:) = result_path_pts(end - 1,:);
result_path_normals(end,:) = result_path_normals(end - 1, :);
rotations(:,:,end) = rotations(:,:,end - 1);

end
