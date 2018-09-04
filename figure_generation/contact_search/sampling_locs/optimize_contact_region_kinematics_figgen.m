function [optim_solution, joint_plan, dummy_link_motion] = ...
    optimize_contact_region_kinematics(user_problem, user_options)

%% Problem structure.
default_problem.robot = '';
default_problem.ball_position_pp = '';
default_problem.ball_radius = 0.1;
default_problem.optim_initial_guess = zeros(4,1); % projection_height, projection_angle, placement_ang, arc_angle

%% Options structure.
default_options.min_approach_angle = 0; % Approach angle of the foot link towards the ball around the contact normal.
default_options.max_approach_angle = 2*pi;
default_options.min_arc_angle = 0; % Contact point on arc on side of the ball. 0 is at horizontal equator. pi/2 is at the north pole.
default_options.max_arc_angle = pi/2 - 0.03;
default_options.min_sample_angle = 0; % Sampling angle in cylindrical coordinates around the dummy link.
default_options.max_sample_angle = 2*pi;
default_options.min_sample_height_offset = -0.05; % Offsets for sampling in cylindrical coordinates around dummy link. 0 is OK, but may want to extend if regions around the caps of the cylinder are being ignored.
default_options.max_sample_height_offset = 0.05;

default_options.debug_plot = false; % Display sampling-on-foot debug plot.
default_options.debug_plot_id = 103;

default_options.num_spline_eval_points = 50; % Number of points along the ball-position-spline to evaluate.
default_options.num_optimization_ik_waypoints = 10; % Number of waypoints used during optimization.
default_options.num_results_waypoints = 25; % Number of waypoints used when just evaluating the results / for return values.

default_options.model_detail = 'high'; % high, mid, or low. How detailed of a dummy foot model to use. This is not just visual. This model's surface is used for integration.
default_options.verbose = true; % Display warnings, status updates, etc. Does not affect CMAES options.

% Saving/loading/what to evaluate
default_options.just_evaluate_guess = false; % Just evaluate the provided CMAES guess, rather than re-running the whole optimization. If also loading a file, then just evaluating will just return the loaded results.
default_options.save_to_file = true; % Save the results to file?
default_options.load_from_file = false; % Load previously-saved results?
default_options.save_directory = './';
default_options.save_file = 'contact_kinematics_result.mat';

% CMAES options, all others are default.
default_options.sigma_multiplier = 0.2; % Set CMAES initial covariance to be some fraction of upper/lower bound range. CMAES recommends 0.2 to 0.5.
default_options.cmaes_opts = cmaes;
default_options.cmaes_opts.StopFitness = 5e-8;
default_options.cmaes_opts.MaxFunEvals = 150;
default_options.cmaes_opts.PopSize = 8;
default_options.cmaes_opts.LogPlot = 'off';
default_options.cmaes_opts.LogModulo = 0;
default_options.cmaes_opts.ReadSignals = 'off';
default_options.cmaes_opts.SaveVariables = 'off';
default_options.cmaes_opts.DispModulo = 25;

% Return template problem structure with no arguments.
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

%% Merge user-provided and default options.
default_problem.joint_initial_guess = user_problem.robot.home_config; % Use home configuration as the default joint configuration guess for IK.

problem = mergeOptions(default_problem, user_problem, 'Problem struct');
options = mergeOptions(default_options, user_options, 'Options struct');

if options.verbose
   disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
   disp('ENTERING CONTACT REGION KINEMATICS OPTIMIZATION');
   disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
end

%% If we're just loading from file, then do it now.
if options.load_from_file
    if options.verbose
        disp('Attempting to load contact segment solution from file.');
    end
    if isfile([options.save_directory, options.save_file])
        loaded_dat = load([options.save_directory, options.save_file], 'optim_solution', 'joint_plan', 'dummy_link_motion');
        optim_solution = loaded_dat.optim_solution;
        joint_plan = loaded_dat.joint_plan;
        dummy_link_motion = loaded_dat.dummy_link_motion;
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
%% Validate
if ischar(problem.robot) % These absolutely need to be overridden.
    error('Problem structure does not contain robot data.');
end
if ischar(problem.ball_position_pp)
    error('Problem structure does not contain ball position piecewise polynomial data.');
end

%% Calculate stuff based on the position spline.
[tspan, ball_pos, ball_vel, ball_accel, ball_omega, ball_quat, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(problem.ball_position_pp, problem.ball_radius, options.num_spline_eval_points);

%% Import data for the dummy planning foot and the banned regions on it.
if options.verbose
    fprintf('%s detail dummy mesh loaded.\n', options.model_detail);
end
        geo_data = load('../../../data/iiwa_merged_end_effector.mat');
        dummy_foot_dat = geo_data.merged_iiwa(2);


        banned_region_dat = geo_data.banned_regions;

% Bounds: cylinder Z, cylinder angle, approach angle, arc angle.
% Get bounding cylinder data (any mesh resolution should be ok.
[cylinder_radius, cylinder_center_offset_x, cylinder_center_offset_y, cylinder_center_offset_z, cylinder_height] = ...
    get_bounding_cylinder_params(dummy_foot_dat);

options.cmaes_opts.LBounds = [options.min_sample_height_offset
    options.min_sample_angle
    options.min_approach_angle
    options.min_arc_angle];

options.cmaes_opts.UBounds = [options.max_sample_height_offset + cylinder_height
    options.max_sample_angle
    options.max_approach_angle
    options.max_arc_angle];


% Not the main scene, just shows where samples are being pulled from on the
% dummy foot.
if options.debug_plot
    sampling_figure = figure(options.debug_plot_id);
    sampling_figure.Color = [1,1,1];
    ax = axes;
    ax.Visible = 'off';
    sampling_figure.Position = [0,0,800,800];
    hold on;
    sampling_foot_patch = patch('Faces', dummy_foot_dat.faces, 'Vertices', dummy_foot_dat.vertices, 'FaceNormals', dummy_foot_dat.face_normals, ...
        'VertexNormals', dummy_foot_dat.vertex_normals);
    sampling_foot_patch.EdgeColor = 'none';
    sampling_foot_patch.FaceColor = [0.7, 0.7, 0.9];
    sampling_foot_patch.FaceAlpha = 0.9;
    sampling_foot_patch.BackFaceLighting = 'lit';
    
    banned_region_patch = patch('Faces', banned_region_dat.faces, 'Vertices', banned_region_dat.vertices, 'FaceNormals', banned_region_dat.face_normals, ...
        'VertexNormals', banned_region_dat.vertex_normals);
    banned_region_patch.EdgeColor = 'none';
    banned_region_patch.FaceAlpha = 0.3;
    banned_region_patch.FaceColor = 'r';
    banned_region_patch.BackFaceLighting = 'lit';
    campos([    1.3716    0.8149    1.5562]);
end

%% Make cylinder around the dummy link and pick permissible areas to sample from.

if options.debug_plot
    if options.verbose
        disp('Setting up contact sampling plot');
    end
%     [y_cyl, z_cyl, x_cyl] = cylinder(cylinder_radius, 20);
%     cyl_patch = patch(surf2patch(x_cyl, y_cyl, z_cyl, 'triangles'))
%     cyl_patch.Vertices(:,1) = cyl_patch.Vertices(:,1) * (cylinder_height + options.max_sample_height_offset) + cylinder_center_offset_x;
%     cyl_patch.Vertices(:,2) = cyl_patch.Vertices(:,2) + cylinder_center_offset_y;
%     cyl_patch.Vertices(:,3) = cyl_patch.Vertices(:,3) + cylinder_center_offset_z;
%     cyl_patch.FaceAlpha = 0.1;
%     plot3([cylinder_center_offset_x - options.min_sample_height_offset, ...
%         cylinder_center_offset_x + cylinder_height + options.max_sample_height_offset], ...
%         cylinder_radius * [cos(options.min_sample_angle), cos(options.min_sample_angle)] + cylinder_center_offset_y, ...
%         cylinder_radius * [sin(options.min_sample_angle), sin(options.min_sample_angle)] + cylinder_center_offset_z, ...
%         '-g', 'LineWidth', 2);
%     
%     plot3([cylinder_center_offset_x - options.min_sample_height_offset, ...
%         cylinder_center_offset_x + cylinder_height + options.max_sample_height_offset], ...
%         [cylinder_radius * cos(options.max_sample_angle), cylinder_radius * cos(options.max_sample_angle)] + cylinder_center_offset_y, ...
%         [cylinder_radius * sin(options.max_sample_angle), cylinder_radius * sin(options.max_sample_angle)] + cylinder_center_offset_z, ...
%         '-r', 'LineWidth', 2);
    
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     view(3);
    daspect([1,1,1]);
    
    camlight HEADLIGHT;
    cmap_hot = hot(13); % Colormap to show how good each point is. Hotter == better.
    hold on;
    
    ball_fig = figure;
    ax = axes;
    bpatch = make_ball(0.1);
    bpatch.FaceColor = [0.7, 0.7, 0.7];
    ball_fig.Position = [0,0,800, 800];
    view(3);
    axis equal;
    ball_fig.Color = [1 1 1];
    ax.Visible = 'off';
    camlight headlight;
    hold on;
    t = linspace(0, pi/2, 100)';
    arc = 0.1001 * [cos(t), zeros(size(t)), sin(t)] + [0, 0, 0.1];
    plot3(arc(:,1), arc(:,2), arc(:,3), 'g', 'LineWidth', 1);
    campos([ 1.0841   -0.9389    1.0673]);
    
    framerate = 5;
    vid_writer = VideoWriter(['../../../data/videos/raw/', 'sampling_ball_arm6.avi']); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
    
end

%% Set up optimization
done = false;
if ~options.just_evaluate_guess
    if options.verbose
        disp('About to run optimization. Initial guess:');
        disp(problem.optim_initial_guess);
    end
    [optim_solution, ~] = cmaes( ...
        @cost_fun_wrap, ...
        problem.optim_initial_guess, ...
        options.sigma_multiplier *(options.cmaes_opts.UBounds - options.cmaes_opts.LBounds), ...
        options.cmaes_opts)
    
    disp('o');
else
    if options.verbose
        disp('Just evaluating the given guess.');
    end
    optim_solution = problem.optim_initial_guess;
end
done = true;

%% Evaluate the optimization winner.
[~, joint_angles_optim, breaks_optim, ~, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span, surface_vel_span] = ...
    cost_fun(optim_solution(1), optim_solution(2), optim_solution(3), optim_solution(4), min(options.num_results_waypoints, options.num_spline_eval_points), false); % Don't kill IK even if bad solution occurs.

dummy_link_motion.surface_contact_position = result_path_pts;
dummy_link_motion.surface_contact_rotation = path_rot_integrated;
dummy_link_motion.surface_contact_normal = up_vector_span;
dummy_link_motion.world_contact_position = world_contact_desired_span;
dummy_link_motion.world_surface_velocity_relative = surface_vel_span;
dummy_link_motion.data_tspan = tspan;
dummy_link_motion.ball_pos = ball_pos;
dummy_link_motion.ball_vel = ball_vel;
dummy_link_motion.ball_accel = ball_accel;
dummy_link_motion.ball_omega = ball_omega;
dummy_link_motion.ball_quat = ball_quat;

joint_plan.breaks = breaks_optim;
joint_plan.angles = joint_angles_optim;

vid_writer.close();

if options.save_to_file && ~(options.just_evaluate_guess && options.load_from_file) % If we loaded and just evaluated, then don't save that.
    if options.verbose
        disp('Saving this contact region to file. Location: ');
        disp([options.save_directory, options.save_file]);
    end
    save(['./', options.save_file], 'optim_solution', 'joint_plan', 'dummy_link_motion');
end

% Wrap the cost function in 'SISO' form for CMAES to use.
    function err = cost_fun_wrap(X)
        err = cost_fun(X(1,:), X(2,:), X(3,:),X(4,:), options.num_optimization_ik_waypoints, true);
    end


%     function point_to_project = sample_on_capsule(projection_height, projection_angle)
%             
%         if projection_height 
%            point_to_project_to_surface = [projection_height + cylinder_center_offset_x, ...
%             cylinder_radius * cos(projection_angle) + cylinder_center_offset_y, ...
%             cylinder_radius * sin(projection_angle) + cylinder_center_offset_z];
%     end

% Cost function with full result outputs.
    function [error, jnt_angles_optim, breaks_optim, solinfo_optim, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span, surface_vel_span] = ...
            cost_fun(projection_height, projection_angle, placement_ang, arc_angle, num_ik_waypoints, kill_on_bad_sol)
        
        point_to_project_to_surface = [projection_height + cylinder_center_offset_x, ...
            cylinder_radius * cos(projection_angle) + cylinder_center_offset_y, ...
            cylinder_radius * sin(projection_angle) + cylinder_center_offset_z];
        
%         if projection_height < blah
%             height_deficit = blah - projection_height;
%             baseline_position = [cylinder_center_offset_x; cylinder_center_offset_y; cylinder_center_offset_z];
%             sphere_pos_unrotated = [-cylinder_radius*sin(height_deficit/cylinder_radius);
%                 cylinder_radius*cos(height_deficit/cylinder_radius);
%                 0];
%             
%             sphere_rot = [1, 0, 0; 0, cos(projection_angle), -sin(projection_angle); 0, sin(projection_angle), cos(projection_angle)];
%             
%             point_to_project_to_surface = sphere_rot * sphere_pos_unrotated + baseline_position;
%         end
%         
        % Evaluate terms which depend on the chosen arc angle.
        world_contact_desired_span = world_contact_loc_desired_fcn(arc_angle * ones(size(tspan)));
        contact_desired_rel_com_span = contact_loc_desired_rel_com_fcn(arc_angle * ones(size(tspan)));
        
        up_vector_span = contact_desired_rel_com_span/problem.ball_radius;
        surface_vel_span = cross(ball_omega, contact_desired_rel_com_span, 2); % Do we want a diff(contact_loc) term?
        
        % Attempt the kinematics.
        [jnt_angles_optim, breaks_optim, solinfo_optim, proj_start_pt, result_path_pts, path_rot_integrated, fail_flag] = attempt_kinematics_over_surface( ...
            point_to_project_to_surface, ...
            placement_ang, ...
            tspan, ...
            surface_vel_span, ...
            up_vector_span, ...
            world_contact_desired_span, ...
            dummy_foot_dat, ...
            num_ik_waypoints, ...
            problem.robot, ...
            problem.joint_initial_guess, ...
            kill_on_bad_sol, ...
            banned_region_dat);
        % Catch velocity integration over surface.
        if fail_flag
            error = 100 * (length(tspan) - find(all(result_path_pts == 0,2),1,'first') - 1)/length(tspan) + 20; % Make it slightly less bad if the integration of the path makes it most of the way.
            if options.verbose
                fprintf('Unsuccessful integration of velocity over surface for a cost of: %f\n', error);
            end
        else
            solres = [solinfo_optim{:}];
            error = sum([solres.PoseErrorNorm]);%.*(length(solres):-1:1));
            error = error/num_ik_waypoints;
            if options.verbose
               fprintf('Successful integration for a cost of: %f.\n', error); 
            end
        end

        if options.debug_plot
            figure(sampling_figure); % Bring to top.
            %             quiver(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3),
            
            [rotation, current_pt, current_normal] = find_mesh_contact_tform(proj_start_pt, up_vector_span(1,:), placement_ang, dummy_foot_dat.faces, dummy_foot_dat.vertices, dummy_foot_dat.face_normals, dummy_foot_dat.vertex_normals);
            
            if done
                vel_dir = -0.2*(rotation*surface_vel_span(1,:)');
                quiver3(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3), vel_dir(1) , vel_dir(2), vel_dir(3), 'Color', [0 1 0] );
                plot3(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3), '.', 'MarkerSize', 50, 'Color', [0 1 0]);
                
            else
                plot3(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3), '.', 'MarkerSize', 25, 'Color', cmap_hot(ceil(-log10(error/1000)),:));
                
                vel_dir = -0.2*(rotation*surface_vel_span(1,:)');
                quiver3(proj_start_pt(1), proj_start_pt(2), proj_start_pt(3), vel_dir(1) , vel_dir(2), vel_dir(3), 'Color',cmap_hot(ceil(-log10(error/1000)),:) );
            end
            
            
            campos([    1.3716    0.8149    1.5562]);
            arm_fr = getframe(sampling_figure);
            campos([   -1.5483   -0.7688   -0.3433]);
            arm_fr2 = getframe(sampling_figure);
            
            figure(ball_fig);
            plot3(cos(arc_angle)*0.1001, 0, sin(arc_angle) * 0.1001+0.1, '.', 'MarkerSize', 25, 'Color', cmap_hot(ceil(-log10(error/1000)),:));
            if done
                plot3(cos(arc_angle)*0.1001, 0, sin(arc_angle) * 0.1001+0.1, '.', 'MarkerSize', 25, 'Color', [0 1 0]);
                
            end
            drawnow;
            
            ball_fr = getframe(ball_fig);
            arm_fr.cdata = [arm_fr.cdata, arm_fr2.cdata ball_fr.cdata];
            writeVideo(vid_writer, arm_fr);
            
        end
    end

% Returns bounding cylinder information. Assumes that the dummy link is
% aligned with the x axis.
    function [cylinder_radius, cylinder_center_offset_x, cylinder_center_offset_y, cylinder_center_offset_z, cylinder_height] = get_bounding_cylinder_params(model_data)
        maxx = max(model_data.vertices(:,1),[],1);
        minx = min(model_data.vertices(:,1),[],1);
        maxy = max(model_data.vertices(:,2),[],1);
        miny = min(model_data.vertices(:,2),[],1);
        maxz = max(model_data.vertices(:,3),[],1);
        minz = min(model_data.vertices(:,3),[],1);
        
        % Longitudinal axis aligned with x on the dummy link.
        cylinder_radius = max(maxy - miny, maxz - minz)/2;
        cylinder_center_offset_y = (miny + maxy)/2;
        cylinder_center_offset_z = (minz + maxz)/2;
        cylinder_center_offset_x = minx;
        cylinder_height = maxx - minx;   
    end

end
