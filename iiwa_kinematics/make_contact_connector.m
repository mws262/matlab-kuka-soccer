function [optim_solution, joint_plan] = make_contact_connector(user_problem, user_options)
% Connector is found from one contact region to another, from *prevous to
% *current. Call with no arguments or single argument 'problem' to get a
% template problem structure. Call with 'options' to get the template
% argument structure. Makes two different kinds of waypoints.
% Departure/approach waypoints are not optimized (yet). They match the
% contact orientation that is about to happen / just happened. Also
% locations are picked to match tangential velocity. They other kind of
% waypoint has bounded, but arbitrary location and orientation in space.
% These are optimized.

%% Problem structure.
default_problem.robot = '';
default_problem.ball_position_pp = '';
default_problem.joint_plan_previous = '';
default_problem.joint_plan_current = '';
default_problem.dummy_link_motion_previous = '';
default_problem.dummy_link_motion_current = '';

default_problem.ball_radius = 0.1;
default_problem.ik_link = 'iiwa_link_6';
default_problem.number_of_waypoints = 1; % Number of waypoints to optimize. The approach/departure waypoints are not counted in this parameter.
default_problem.optim_initial_guess = zeros(5 * default_problem.number_of_waypoints, 1); % Size depends on the number of waypoints.

%% Options structure.
default_options.departure.velocity_normal = 0.5; % Normal component of velocity during the approach/departure segments. Tangent velocity is fully-determined.
default_options.departure.duration_fraction = 1/8; % Fraction of the total connector time spent on approach/departure segments.

default_options.approach.velocity_normal = 0.5;
default_options.approach.duration_fraction = 1/8;

default_options.collision_ball_radius = 0.115 + 0.02; % During connector, collisions are checked by assuming foot is a sphere with this radius. Hand-tuned to fit around the whole foot area nicely.

default_options.verbose = true;

% Saving/loading/what to evaluate
default_options.just_evaluate_guess = false; % Just evaluate the provided CMAES guess, rather than re-running the whole optimization. If also loading a file, then just evaluating will just return the loaded results.
default_options.save_to_file = true; % Save the results to file?
default_options.load_from_file = false; % Load previously-saved results?
default_options.save_directory = '../data/optim_results/';
default_options.save_file = 'noncontact_kinematics_result.mat';

% Optimization bounds.
default_options.min_perpendicular_shift = -0.1; % Outward perpendicular shift from nominal linear connector.
default_options.max_perpendicular_shift = 0.4; 
default_options.min_upward_shift = 0.0; % Upward perpendicular shift from nominal linear connector. 
default_options.max_upward_shift = 0.4;
default_options.min_orientation_angle = -pi; % Orientation bounds. Probably a terrible idea to limit these since they have little physical significance right now. May want 0 to 2*pi instead or something but not a limited range.
default_options.max_orientation_angle = pi;

% CMAES options, all others are default.
default_options.cmaes_opts = cmaes;

% Stopping params + important stuff
default_options.sigma_multiplier = 0.2; % Set CMAES initial covariance to be some fraction of upper/lower bound range. CMAES recommends 0.2 to 0.5.
default_options.cmaes_opts.StopFitness = 5e-8;
default_options.cmaes_opts.MaxFunEvals = 1000;
default_options.cmaes_opts.PopSize = 8;

% Relatively stupid settings.
default_options.cmaes_opts.LogPlot = 'off';
default_options.cmaes_opts.LogModulo = 0;
default_options.cmaes_opts.ReadSignals = 'off';
default_options.cmaes_opts.SaveVariables = 'off';
default_options.cmaes_opts.DispModulo = 25;


%% Return default problem or option structs if no or one arguments are given.
if nargin == 0
    optim_solution = default_problem;
    return;
elseif nargin == 1 % If single argument given, return either options or probelm structure.
   switch user_problem
       case 'options'
           optim_solution = default_options;
       case 'problem'
           optim_solution = default_problem;
       otherwise
           warning('Unrecognized single variable argument given: %s. Returning problem structure instead.', usr_problem);
           optim_solution = default_problem;
   end
   return;
end

% Merge user's problem and options with the defaults. If the user has any
% optional parameters missing, these are filled in from the default.
problem = mergeOptions(default_problem, user_problem, 'Problem struct');
options = mergeOptions(default_options, user_options, 'Options struct');

options.cmaes_opts.LBounds = zeros(problem.number_of_waypoints * 5,1);
options.cmaes_opts.LBounds(1:problem.number_of_waypoints) = options.min_perpendicular_shift;
options.cmaes_opts.LBounds(problem.number_of_waypoints + 1: 2*problem.number_of_waypoints) = options.min_upward_shift;
options.cmaes_opts.LBounds(2*problem.number_of_waypoints + 1:end) = options.min_orientation_angle;

options.cmaes_opts.UBounds = zeros(problem.number_of_waypoints * 5,1);
options.cmaes_opts.UBounds(1:problem.number_of_waypoints) = options.max_perpendicular_shift;
options.cmaes_opts.UBounds(problem.number_of_waypoints + 1: 2*problem.number_of_waypoints) = options.max_upward_shift;
options.cmaes_opts.UBounds(2*problem.number_of_waypoints + 1:end) = options.max_orientation_angle;

if options.verbose
   disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
   disp('ENTERING NON-CONTACT KINEMATICS OPTIMIZATION');
   disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
end

%% If we're just loading from file, then do it now.
if options.load_from_file
    if options.verbose
       disp('About to try to load a previous contact-connector solution');
    end
    if isfile([options.save_directory, options.save_file])
        loaded_dat = load([options.save_directory, options.save_file], 'optim_solution', 'joint_plan');
        optim_solution = loaded_dat.optim_solution;
        joint_plan = loaded_dat.joint_plan;
        if options.verbose
           disp('Load success.'); 
        end
        if options.just_evaluate_guess
            if options.verbose
                disp('Already loaded, and just_evaluate_guess is true. Nothing to do. Returning old solution.');
            end
            return;
        else
            if options.verbose
                disp('Loaded solution, but just_evaluate_guess is false. Going to rerun the optimization using the old solution as a guess.');
            end
            problem.optim_initial_guess = optim_solution; % Load the solution, but just use it as a guess to re-run.
        end
    else
        warning('Load from file is true, but the save file was not found. Re-running as if load were not true.');
    end
end

%% Validate - mandatory fields must be changed from '' to something else.
% TODO: actually make sure that the 'something else' is the right thing.
if ischar(problem.robot) % These absolutely need to be overridden.
    error('Problem structure does not contain robot data.');
end
if ischar(problem.ball_position_pp)
    error('Problem structure does not contain ball position piecewise polynomial data.');
end
if ischar(problem.joint_plan_previous) || ischar(problem.joint_plan_current)
    error('Problem structure does not have one or more of the boundary joint plans assigned.');
end
if ischar(problem.dummy_link_motion_previous) || ischar(problem.dummy_link_motion_current)
   error('Problem structure does not have boundary condition dummy link motion assigned.'); 
end

%% Unpack
% Start conditions determined by the first set of dummy link motions and
% joint plan.
contact_point_start = problem.dummy_link_motion_previous.world_contact_position(end,:);
contact_point_velocity_start = problem.dummy_link_motion_previous.world_surface_velocity_relative(end,:);
joint_config_start = problem.joint_plan_previous.angles{end};
start_t = problem.joint_plan_previous.breaks(end);
ball_com_start = ppval(problem.ball_position_pp, start_t)';

% End conditions determined by the second set of dummy link motions and
% joint plan.
contact_point_end = problem.dummy_link_motion_current.world_contact_position(1,:);
contact_point_velocity_end = problem.dummy_link_motion_current.world_surface_velocity_relative(1,:);
joint_config_end = problem.joint_plan_current.angles{1};
end_t = problem.joint_plan_current.breaks(1);

if end_t <= start_t
    if end_t == 0 % If we're looping back around to the beginning, this is ok.
        if options.verbose
           disp('Looks like this contact-connector needs to wrap around back to the beginning. Overriding the end time which was 0.'); 
        end
        end_t = problem.ball_position_pp.breaks(end);
    else
        error('End time must be greater than the start time.');
    end
end
ball_com_end = ppval(problem.ball_position_pp, end_t)';

ball_com_start(3) = problem.ball_radius;
ball_com_end(3) = problem.ball_radius; % MAKE SURE that we haven't subtracted out the radius of the ball.


%% Approach/departure waypoints are close to the original start/end, but kicked out a little bit. They try to match velocities.
contact_t_diff = end_t - start_t;
departure_waypoint_t_offset = contact_t_diff * options.departure.duration_fraction;
approach_waypoint_t_offset = contact_t_diff * options.approach.duration_fraction;

% Normals of the contact points on both ends of the contactless section.
start_normal = contact_point_start - ball_com_start;
start_normal = start_normal / norm(start_normal);

end_normal = contact_point_end - ball_com_end;
end_normal = end_normal / norm(end_normal);

departure_waypoint_t = start_t + departure_waypoint_t_offset;
approach_waypoint_t = end_t - approach_waypoint_t_offset;

% Translations off of the original/end configs.
departure_waypoint_displacement = contact_point_velocity_start * departure_waypoint_t_offset + ...
    start_normal * options.departure.velocity_normal * departure_waypoint_t_offset;

approach_waypoint_displacement =  -contact_point_velocity_end * approach_waypoint_t_offset + ...
    end_normal * options.approach.velocity_normal * approach_waypoint_t_offset;

departure_waypoint_tform = trvec2tform(departure_waypoint_displacement) * getIIWATForm(problem.robot, joint_config_start, problem.ik_link);
approach_waypoint_tform = trvec2tform(approach_waypoint_displacement) * getIIWATForm(problem.robot, joint_config_end, problem.ik_link);

[departure_waypoint_angles, ~] = single_ik_call(problem.robot, departure_waypoint_tform, joint_config_start, problem.ik_link);
[approach_waypoint_angles, ~] = single_ik_call(problem.robot, approach_waypoint_tform, joint_config_end, problem.ik_link);

departure_tform_actual = getIIWATForm(problem.robot, departure_waypoint_angles, problem.ik_link);
approach_tform_actual = getIIWATForm(problem.robot, approach_waypoint_angles, problem.ik_link);

depart_approach_tan = tform2trvec(approach_tform_actual) - tform2trvec(departure_tform_actual);
depart_approach_tan = depart_approach_tan/norm(depart_approach_tan);
depart_approach_perp = cross(depart_approach_tan, [0,0,1]);

% Check if either of this is already in penetration.
[ground_penetration_st, ball_penetration_st] = get_penetrations(departure_tform_actual, ball_com_start);
[ground_penetration_e, ball_penetration_e] = get_penetrations(approach_tform_actual, ball_com_end);

if ground_penetration_st > 0 || ball_penetration_st > 0
    warning('Departure waypoint configuration is already in penetration.');
end
if ground_penetration_e > 0 || ball_penetration_e > 0
    warning('Approach waypoint configuration is already in penetration.');
end

%% Any midway waypoints.
num_waypoint = problem.number_of_waypoints;
waypoint_t = linspace(departure_waypoint_t, approach_waypoint_t, num_waypoint + 2)'; % Waypoint times are evenly spaced between departure and approach times.
waypoint_t = waypoint_t(2:end-1); % Don't want to replicate departure_waypoint_t or approach_waypoint_t.

% Nominal trajectories (before perpendicular shifts) are linear between
% departure/approach.
waypoint_ball_pos = (ball_com_end - ball_com_start) .* repmat(waypoint_t - start_t, [1,3]) + ball_com_start; % Assuming constant velocity during this section for the ball.
waypoint_foot_nominal = (tform2trvec(approach_tform_actual) - tform2trvec(departure_tform_actual)) .* repmat(waypoint_t - departure_waypoint_t, [1,3]) + tform2trvec(departure_tform_actual);
waypoint_trans_nominal = trvec2tform(waypoint_foot_nominal);

%% Run the optimization.
if ~options.just_evaluate_guess
    if options.verbose
        disp('About to run optimization. Initial guess:');
        disp(problem.optim_initial_guess);
    end
    [optim_solution, cost] = cmaes( ...
        @waypoint_cost_wrapper, ...
        problem.optim_initial_guess, ...
        options.sigma_multiplier * (options.cmaes_opts.UBounds - options.cmaes_opts.LBounds), ...
        options.cmaes_opts);
    if options.verbose
        fprintf('Optimization completed with a cost of: %f\n', cost);
        disp('Final solution:');
        disp(optim_solution);
    end
else
    if options.verbose 
        disp('Just evaluating the given guess.');
    end
    optim_solution = problem.optim_initial_guess;
end

[~, waypoint_jnt_sol] = waypoint_cost_wrapper(optim_solution); % Get the joint solutions of the winning parameters.

%% Put all the waypoint joint angles/breaks together.
joint_plan.angles = [{departure_waypoint_angles}, waypoint_jnt_sol, {approach_waypoint_angles}];
joint_plan.breaks = [departure_waypoint_t; waypoint_t; approach_waypoint_t];

if options.save_to_file && ~(options.just_evaluate_guess && options.load_from_file) % If we loaded and just evaluated, then don't save that.
    if options.verbose
       disp('Saving this contact-connector to file. Location: ');
       disp([options.save_directory, options.save_file]);
    end
    save([options.save_directory, options.save_file], 'optim_solution', 'joint_plan');
end

%% Just wraps the normal cost function so it has the single input that CMAES likes.
    function [cost, jnts] = waypoint_cost_wrapper(X)
        perp_comp = X(1:num_waypoint); % First n elements are the spatial offsets perpendicular to the direct line between departure/approach.
        up_comp = X(num_waypoint + 1:2*num_waypoint); % Second n elements are the spatial offsets upward (i.e. [0 0 1]) to the direct line between departure/approach waypoints.
        angles = reshape(X(2*num_waypoint + 1 : end), [], 3); % Next 3*n elements are orientation parameters for the waypoints.
        [jnts, cost] = waypoint_cost(perp_comp, up_comp, angles);
    end

%% Cost function to optimize location of waypoints in cartesian space and the orientation of the foot at those waypoints.
% This includes 2 spatial components, perpendicular to a straight line
% between the departure and approach waypoints. 3 parameters are for
% the orientation of the foot link. Cost tries to minimize total
% distance traveled in joint space. Large penalties for penetrating the
% ground or the ball.
    function [waypt_joints, cost] = waypoint_cost(perp_components, up_components, angles)
        cost = 0; % Construct the cost as a sum.
        
        %% Construct spatial transforms for the waypoints.
        rot_tforms = rotm2tform(angle2dcm(angles(:,1), angles(:,2), angles(:,3))); % Rotation of foot picked by optimizer. I use euler angles, but not in any particular way. I don't care what the angles themselves really are. TODO: make sure that we aren't having gimbal lock issues.
        waypoint_skew = trvec2tform(depart_approach_perp .* repmat(perp_components, [1,3]) + [0 0 1] .* repmat(up_components, [1,3])); % Perpendicular-to-endpoint-path shift.
        waypoint_result_tform = zeros(4,4,num_waypoint);
        
        for i = 1:num_waypoint
            waypoint_result_tform(:,:,i) = waypoint_skew(:,:,i)*waypoint_trans_nominal(:,:,i)*rot_tforms(:,:,i);
        end
        
        %% Do IK to achieve the waypoints.
        % Note that it's ok if the IK doesn't succeed, as long as it
        % produces a good 'best_available' solution.
        waypt_joints = problem.robot.make_multi_point_trajectory(waypoint_result_tform, departure_waypoint_angles, problem.ik_link, false);
        
        %% Cost is the sum of squares of the distances in joint space traveled between all waypoints and goals.
        cost = cost + sum(([waypt_joints{1}.JointPosition] - [departure_waypoint_angles.JointPosition]).^2 + ...
            ([waypt_joints{end}.JointPosition] - [approach_waypoint_angles.JointPosition]).^2); % For at the boundaries between the approach/departure and the ones we just found.
        
        for i = 2:length(waypt_joints)
            cost = cost + sum(([waypt_joints{i}.JointPosition] - [waypt_joints{i - 1}.JointPosition]).^2); % For between the interior waypoints.
        end
        
        %% Extra cost penalties for penetrating the ground or ball.
        % These should always be much greater than any cost associated with
        % distance in joint space.
        for i =1:length(waypt_joints)
            result_tform = problem.robot.getIIWATForm(waypt_joints{i}, problem.ik_link); % Check where the IK ACTUALLY got us to (not the target, potentially).
            [ground_penetration, ball_penetration] = get_penetrations(result_tform, waypoint_ball_pos(i,:));
            if ground_penetration > 0
                % Ground collision!
                cost = cost + 100*ground_penetration / num_waypoint; % Divided by number of waypoints so the cost isn't too affected by adding additional waypoints.
            elseif ball_penetration > 0
                % Ball collision!
                cost = cost + 50*ball_penetration / num_waypoint;
            end
        end
    end

% Positive means penetration, negative is margin. Foot link is just a
% sphere with radius defined in the options.
    function [ground_penetration, ball_penetration] = get_penetrations(foot_tform, ball_position)
        collision_ball_center = foot_tform * [0;0.02;0;1]; % 0.02 hand-tuned collision sphere offset to cover entire foot.
        collision_ball_center = collision_ball_center(1:3);
        ground_penetration = -collision_ball_center(3) + options.collision_ball_radius;
        ball_penetration = options.collision_ball_radius + problem.ball_radius - norm(collision_ball_center - ball_position);
    end
end
