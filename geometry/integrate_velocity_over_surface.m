function output = integrate_velocity_over_surface(user_problem, user_options)

%% Problem structure.
default_problem.time_vector = [];
default_problem.velocity_vector = [];
default_problem.initial_surface_point = [];
default_problem.normals_to_match = [];
default_problem.orientations_about_normal = [];
default_problem.mesh_data = '';

%% Options structure.
default_options.check_banned_regions = false;
default_options.banned_region_mesh = '';
default_options.use_face_normals = false;

%% Return template problem / options.
if nargin == 0
    output.mesh_surface_path = default_problem;
    return;
elseif nargin == 1 % If single argument given, return either options or probelm structure.
    switch user_problem
        case 'options'
            output = default_options;
        case 'problem'
            output = default_problem;
        otherwise
            warning('Unrecognized single variable argument given: %s. Returning problem structure instead.', user_problem);
            output.mesh_surface_path = default_problem;
    end
    return;
end

%% Merge user and default options and validate.
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
validateattributes(problem.initial_surface_point, {'single', 'double'}, {'real', 'vector', 'numel', 3});
validateattributes(options.check_banned_regions, {'logical'}, {'scalar'})
validate_mesh_struct(problem.mesh_data);

if options.check_banned_regions
    validate_mesh_struct(options.banned_region_mesh);
end

output.failure_flag = false; % No failure unless otherwise detected.

%% Setup the problem
[tform, current_pt, current_normal] = find_mesh_contact_tform(problem.mesh_data, problem.initial_surface_point, ...
    problem.normals_to_match(1,:), problem.orientations_about_normal);

rotation = tform2rotm(tform)';

% Prepare output value arrays.
total_steps = length(problem.time_vector);
output.mesh_surface_path = zeros(total_steps,3);
output.mesh_surface_normals = zeros(total_steps,3);
output.mesh_rotations = zeros(3,3,total_steps);

% Add initial points.
output.mesh_surface_path(1,:) = current_pt;
output.mesh_surface_normals(1,:) = current_normal;
output.mesh_rotations(:,:,1) = rotation;

% Integration loop.
for i = 1:total_steps - 1
    dt = problem.time_vector(i + 1) - problem.time_vector(i);
    % Using current velocity.
    vel_k1 = -problem.velocity_vector(i,:);
    k_1 = (rotation*vel_k1')';
    
    % Half-step velocity evaluated with initial velocity.
    pt_star_k2 = current_pt + 0.5 * k_1 * dt;
    [ ~, ~, new_normal_vec_k2 ] = point2trimesh_with_normals( pt_star_k2, problem.mesh_data, options.use_face_normals);
    rotation_k2 = get_rotation_from_vecs(current_normal, new_normal_vec_k2) * rotation;
    vel_k2 = -(problem.velocity_vector(i,:) + problem.velocity_vector(i + 1,:))/2; % Just assume average.
    k_2 = (rotation_k2*vel_k2')';
    
    % Half-step velocity evaluated with intermediate velocity.
    pt_star_k3 = current_pt + 0.5 * k_2 * dt;
    [ ~, ~, new_normal_vec_k3 ] = point2trimesh_with_normals( pt_star_k3, problem.mesh_data, options.use_face_normals );
    rotation_k3 = get_rotation_from_vecs(current_normal, new_normal_vec_k3) * rotation;
    vel_k3 = vel_k2;
    k_3 = (rotation_k3*vel_k3')';
    
    % Whole step velocity evaluated with second intermediate velocity.
    pt_star_k4 = current_pt + k_3 * dt;
    [ ~, ~, new_normal_vec_k4 ] = point2trimesh_with_normals( pt_star_k4, problem.mesh_data, options.use_face_normals );
    rotation_k4 = get_rotation_from_vecs(current_normal, new_normal_vec_k4) * rotation;
    vel_k4 = -problem.velocity_vector(i + 1,:);
    k_4 = (rotation_k4*vel_k4')';
    
    % Weight them and actually step forward.
    new_pt_star = current_pt + (1/6) * (k_1+2*k_2+2*k_3+k_4) * dt;
    [ ~, surface_point, new_normal_vec ] = point2trimesh_with_normals( new_pt_star, problem.mesh_data, options.use_face_normals );
    % Check if the integration has entered a banned area if a banned_region
    % was given in the arguments.
    if options.check_banned_regions
        if check_pt_inside_mesh(surface_point, options.banned_region_mesh)
            output.failure_flag = true;
            return;
        end
    end
    
    rotation = get_rotation_from_vecs(current_normal, new_normal_vec) * rotation;
    
    current_pt = surface_point;
    current_normal = new_normal_vec;
    rotation = modified_gram_schmidt(rotation); % Fix the rotation matrix.
    
    output.mesh_surface_path(i+1,:) = current_pt;
    output.mesh_surface_normals(i+1,:) = current_normal;
    output.mesh_rotations(:,:,i+1) = rotation;
end
% TODO FIX THIS PROPERLY. Just replicating the next to last value so the
% dimensions are nicer.
% output.mesh_surface_path(end,:) = output.mesh_surface_path(end - 1,:);
% output.mesh_surface_normals(end,:) = output.mesh_surface_normals(end - 1, :);
% output.mesh_rotations(:,:,end) = output.mesh_rotations(:,:,end - 1);

end
