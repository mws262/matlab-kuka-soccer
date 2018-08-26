function test_mesh_on_ball_multiseg_optim
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;
addpath ../path_optim/;
addpath ../dynamics/;
addpath ../iiwa_kinematics/;
addpath ../util/;

%% Import data for the dummy planning foot and the banned regions on it.
dummy_foot_dat = get_mesh_data('dummy_manipulator_mid_res');

%% Set up world scene.
% World scene.
scene_fig = make_visualizer_scene();

% Ball.
hold on;
ball_radius = 0.1;
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

% Whole robot.
iiwa = IIWAImporter(scene_fig);

% % Dummy floating foot (i.e. 'what should be happening').
% dummy_foot_patch = patch('Faces', dummy_foot_dat.faces, 'Vertices', dummy_foot_dat.vertices, 'FaceNormals', dummy_foot_dat.face_normals, ...
%     'VertexNormals', dummy_foot_dat.vertex_normals);
% dummy_foot_patch.EdgeColor = 'none';
% dummy_foot_patch.FaceColor = 'flat';
% dummy_foot_patch.FaceVertexCData = repmat([0 0 1], [size(dummy_foot_patch.Faces,1),1]);
% dummy_foot_patch.FaceAlpha = 0.3;
% dummy_foot_patch.BackFaceLighting = 'lit';
% dummy_foot_tform = hgtransform;
% dummy_foot_patch.Parent = dummy_foot_tform;
% dummy_foot_patch.Visible = 'off';
% campos([1.6970 1.5293 0.9466]);

%% Make goal path on the ground.
num_evaluation_pts = 150;
pos_pp = get_path_pp('triangle', []);
zero_accel_tol = 1e-4;

[accel_zero_break_start, accel_zero_break_end, contact_polys, shifted_position_pp] = find_zero_accel_breaks_from_pos_pp(pos_pp, 3, zero_accel_tol);

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ~, ~] = evaluate_spline(shifted_position_pp, ball_radius, num_evaluation_pts);

draw_path_and_accel(posspan, accelspan, 1); % Draw out the path and acceleration arrows.

zero_starts = ppval(pos_pp, accel_zero_break_start)';
zero_ends = ppval(pos_pp, accel_zero_break_end)';

% Make some interpolators for these values.
posspan_interpolator = general_interpolator(3);
posspan_interpolator.add_to_end(tspan, posspan);

velspan_interpolator = general_interpolator(3);
velspan_interpolator.add_to_end(tspan, velspan);

omegaspan_interpolator = general_interpolator(3);
omegaspan_interpolator.add_to_end(tspan, velspan);

quatspan_interpolator = general_interpolator(4);
quatspan_interpolator.add_to_end(tspan, quatspan);

% Plot the beginnings and ends of the contact sections.
plot(zero_starts(:,1), zero_starts(:,2), '.r',  'MarkerSize', 20);
plot(zero_ends(:,1), zero_ends(:,2), '.g',  'MarkerSize', 20);

drawnow;


%% Split into a bunch of smaller problems for each contacting section and each connecting section.
% Initial guess. Each column is for one contact region.
contact_guesses = zeros(4,3);
connector_guesses = zeros(5,3);

contact_guesses(:,1) = [
    0.1403
    2.6181
    5.4607
    1.4374];
    
contact_guesses(:,2) = [
    0.1498
    2.2048
    4.7332
    1.5408];
    
contact_guesses(:,3) = [
    0.2197
    3.3002
    6.2832
    1.2950];

connector_guesses(:,1) = [
   -0.0641
    0.0806
   -3.1416
    1.2198
   -2.6212];

connector_guesses(:,2) = [
    0.0293
         0
    2.1244
    0.4303
   -2.8944];

connector_guesses(:,3) = [
    0.3476
    0.0046
   -3.1416
    1.3787
    3.1416];

%% Set up contact kinematics problem.
contact_kinematics_problem = optimize_contact_region_kinematics('problem');
contact_kinematics_problem.robot = iiwa;
contact_kinematics_options = optimize_contact_region_kinematics('options');
contact_kinematics_options.just_evaluate_guess = true;
contact_kinematics_options.load_from_file = true;


%% Set up non-contact kinematics problem.
noncontact_kinematics_problem = make_contact_connector('problem');
noncontact_kinematics_problem.robot = iiwa;
noncontact_kinematics_problem.ball_position_pp = shifted_position_pp;
noncontact_kinematics_options = make_contact_connector('options');
noncontact_kinematics_options.just_evaluate_guess = true;
noncontact_kinematics_options.load_from_file = true;    

%% Make interpolators for holding data we need for plotting later.
surface_path_pts_interp = general_interpolator(3);
surface_path_rot_interp = general_interpolator([3,3]);
world_contact_pt_interp = general_interpolator(3);
up_vector_interp = general_interpolator(3);
surface_vel_interp = general_interpolator(3);

joint_contact_plans = {};
dummy_contact_motion = {};
all_jnt_angles = {};
all_jnt_breaks = [];

joint_guess = iiwa.home_config;

%% Figure out kinematics for all sections.
for i = 1:length(contact_polys)
    
    %% Change settings specific to this contact region and run optimization.
    contact_kinematics_problem.ball_position_pp = contact_polys(i);
    contact_kinematics_problem.joint_initial_guess = joint_guess;
    contact_kinematics_problem.optim_initial_guess = contact_guesses(:,i);
    contact_kinematics_options.save_file = ['contact_kinematics_result', num2str(i), '.mat'];
    
    [solution, joint_plan_current, dummy_link_motion_current] = ...
        optimize_contact_region_kinematics(contact_kinematics_problem, contact_kinematics_options);
    joint_guess = joint_plan_current.angles{end}; % Use end config as guess for the next optimization.
    
    joint_contact_plans{end + 1} = joint_plan_current; % Hang on to joint and dummy motion for just contact regions also.
    dummy_contact_motion{end + 1} = dummy_link_motion_current;
    
    if i == 1
        joint_plan_current.angles = joint_plan_current.angles(2:end); % HACK because of stupid sign change in acceleration of piecewise polynomial. Just throwing this point away. Not terrible, but not a great way to fix.
        joint_plan_current.breaks = joint_plan_current.breaks(2:end);
    end
    
    
    %% Add quantities to interpolators for animation later.
    section_tspan = linspace(contact_polys(i).breaks(1), contact_polys(i).breaks(end), contact_kinematics_options.num_spline_eval_points);
   
    surface_path_pts_interp.add_to_end_at_time(section_tspan, dummy_link_motion_current.surface_contact_position);
    surface_path_rot_interp.add_to_end_at_time(section_tspan, permute(dummy_link_motion_current.surface_contact_rotation,[3,1,2]));
    world_contact_pt_interp.add_to_end_at_time(section_tspan, dummy_link_motion_current.world_contact_position);
    up_vector_interp.add_to_end_at_time(section_tspan, dummy_link_motion_current.surface_contact_normal);
    surface_vel_interp.add_to_end_at_time(section_tspan, dummy_link_motion_current.world_surface_velocity_relative); % This is rel COM.
    
    %% Find a connector between contact regions. Contact regions will be totally determined before this is run (hence, i > 1).
    if i > 1
        noncontact_kinematics_problem.dummy_link_motion_previous = dummy_contact_motion{i - 1};
        noncontact_kinematics_problem.dummy_link_motion_current = dummy_contact_motion{i};
        noncontact_kinematics_problem.joint_plan_previous = joint_contact_plans{i - 1};
        noncontact_kinematics_problem.joint_plan_current = joint_contact_plans{i};
        
        noncontact_kinematics_problem.optim_initial_guess = connector_guesses(:,i-1);
        noncontact_kinematics_options.save_file = ['noncontact_kinematics_result', num2str(i - 1), '.mat'];
        
        [optim_solution, joint_plan] = make_contact_connector(noncontact_kinematics_problem, noncontact_kinematics_options);
        all_jnt_angles = [all_jnt_angles, joint_plan.angles];
        all_jnt_breaks = [all_jnt_breaks; joint_plan.breaks];
    end
    
    % Add the contact region.
    all_jnt_angles = [all_jnt_angles, joint_plan_current.angles];
    all_jnt_breaks = [all_jnt_breaks; joint_plan_current.breaks];

end

%% Add transition back to the beginning
noncontact_kinematics_problem.dummy_link_motion_previous = dummy_contact_motion{end};
noncontact_kinematics_problem.dummy_link_motion_current = dummy_contact_motion{1};
noncontact_kinematics_problem.joint_plan_previous = joint_contact_plans{end};
noncontact_kinematics_problem.joint_plan_current = joint_contact_plans{1};
noncontact_kinematics_problem.optim_initial_guess = connector_guesses(:,end);
noncontact_kinematics_options.save_file = ['noncontact_kinematics_result', num2str(length(contact_polys)), '.mat'];

[optim_solution, joint_plan] = make_contact_connector(noncontact_kinematics_problem, noncontact_kinematics_options);

all_jnt_angles = [all_jnt_angles, joint_plan.angles];
all_jnt_breaks = [all_jnt_breaks; joint_plan.breaks];


% Convert surface velocity to relative to world not ball COM.
surface_vel_rel_world = interp1(tspan, velspan,surface_vel_interp.tspan_complete) + surface_vel_interp.dspan_complete;

% 0's for no contact, 1's for contact, made to span ball timings. Not the
% exact times of leaving/entering contact.
contact_status = zeros(size(tspan));
for i = 1:length(contact_polys)
contact_status = contact_status | (contact_polys(i).breaks(1) <= tspan & contact_polys(i).breaks(end) >= tspan);
end

%% Save to file.
save_plan_to_file('example_dat1', all_jnt_angles, all_jnt_breaks, ...
    tspan, posspan + [0,0,ball_radius], velspan, accelspan, omegaspan, quatspan, ...
    world_contact_pt_interp.tspan_complete, world_contact_pt_interp.dspan_complete, surface_vel_rel_world, contact_status);

%% Play the winning solution.
campos([1.6970    1.5293    0.9466]);
tfactor = 1;
tic;
curr_time = 0;

% Video recording if desired
write_to_vid = false;
if write_to_vid
    framerate = 60;
    vid_writer = VideoWriter('triangle.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end

% For figuring out the size of the collision ball around the foot.
% [col_ball, ~] = make_ball(0.115);
% col_ball.Vertices(:,3) = col_ball.Vertices(:,3) - ball_radius;
% col_ball.FaceColor = [1,0,0];
% col_tform = hgtransform;
% col_ball.Parent = col_tform;

while ishandle(scene_fig)
    
    quat_eval = interp1(tspan, quatspan, curr_time);
    pos_eval = interp1(tspan, posspan, curr_time);
    ball_patch.Vertices = quatrotate(quat_eval, ball_verts_untransformed) + repmat(pos_eval  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    % Interpolate quantities in time.
    upvec_eval = up_vector_interp.get_at_time(curr_time);
    path_rot_eval = surface_path_rot_interp.get_at_time(curr_time);
    world_contact_pt_eval = world_contact_pt_interp.get_at_time(curr_time);
    surface_pts_eval = surface_path_pts_interp.get_at_time(curr_time);
    
    %interp1(section_tspan(1:end-1), dummy_link_motion.surface_contact_position, tspan(i));
    %     first_idx = find(section_tspan <= tspan(i), 1, 'last');
    %     last_idx = find(section_tspan >= tspan(i), 1, 'first');
    %     path_rot_eval = (dummy_link_motion.surface_contact_rotation(:,:,last_idx) - dummy_link_motion.surface_contact_rotation(:,:,first_idx)) * (tspan(i) - section_tspan(first_idx)) + dummy_link_motion.surface_contact_rotation(:,:,first_idx);
    %     path_rot_interp = interp1(eval_pt_tspan, dummy_link_motion.surface_contact_rotation, tspan(i));
    
    % TODO fix upvec rotation.
    %        if ~all(isnan(surface_pts_eval))
    %            surface_to_origin_translation = trvec2tform(-surface_pts_eval);
    %            world_origin_to_goal_pt_translation = trvec2tform(world_contact_pt_eval);
    %
    %            surf_vel_untransformed_to_planar_vel = rotm2tform(squeeze(path_rot_eval)');
    %            ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(up_vector_interp.get_at_time(0), upvec_eval)); % upvec_rot(:,:,i));%
    %            dummy_foot_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    %        end

    
    joint_pos_struct = interpolate_traj(iiwa, all_jnt_breaks, all_jnt_angles, curr_time);
    display_at_pose(iiwa, joint_pos_struct);
    
    % For figuring out the size of the collision ball around the foot.
%     curr_tform = iiwa.getIIWATForm(joint_pos_struct, 'iiwa_link_6');
%     curr_p = tform2trvec(curr_tform);
%     col_tform.Matrix = curr_tform*trvec2tform([0,0.02,0]);
    
    drawnow;

    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
        curr_time = 1./framerate * tfactor + curr_time;      
    else
       curr_time = toc * tfactor;
    end
       curr_time = mod(curr_time, tspan(end)); 
end

if write_to_vid
    close(vid_writer);
    % Convert from avi to mp4.
    !ffmpeg -i make_8_vid.avi make_8_vid.mp4
end