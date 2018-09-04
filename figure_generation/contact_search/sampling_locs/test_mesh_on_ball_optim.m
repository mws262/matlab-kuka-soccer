function test_mesh_on_ball_optim
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;
addpath ../path_optim/;
addpath ../iiwa_kinematics/;
addpath ../dynamics/;
addpath ../util/;

%% Import data for the dummy planning foot and the banned regions on it.
        geo_data = load('../../../data/iiwa_merged_end_effector.mat');
        dummy_foot_dat = geo_data.merged_iiwa(2);
%% Set up world scene.

% World scene.
scene_fig = make_visualizer_scene();

% Ball.
hold on;
ball_radius = 0.1;
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

% Whole robot.
iiwa = IIWAImporter(scene_fig);

% Dummy floating foot (i.e. 'what should be happening').
dummy_foot_patch = patch('Faces', dummy_foot_dat.faces, 'Vertices', dummy_foot_dat.vertices, 'FaceNormals', dummy_foot_dat.face_normals, ...
    'VertexNormals', dummy_foot_dat.vertex_normals);
dummy_foot_patch.EdgeColor = 'none';
dummy_foot_patch.FaceColor = 'flat';
dummy_foot_patch.FaceVertexCData = repmat([0 0 1], [size(dummy_foot_patch.Faces,1),1]);
dummy_foot_patch.FaceAlpha = 0.3;
dummy_foot_patch.BackFaceLighting = 'lit';
dummy_foot_tform = hgtransform;
dummy_foot_patch.Parent = dummy_foot_tform;
dummy_foot_patch.Visible = 'off';
campos([1.6970 1.5293 0.9466]);

%% Make goal path on the ground.
total_time = 5;
num_evaluation_pts = 50;
pos_pp = get_path_pp('small_arc_knot', total_time);

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(pos_pp, ball_radius, num_evaluation_pts);

draw_path_and_accel(posspan, accelspan, 3); % Draw out the path and acceleration arrows.

%% Set up optimization

% Initial guess.
guess = [    0.1422
    1.8375
    3.3521
    0.8112];
guess = [    2.9296 % Bad guess to demo.
    1.6533
    2.3734
    0.2952];

contact_kinematics_problem = optimize_contact_region_kinematics('problem');
contact_kinematics_problem.robot = iiwa;
contact_kinematics_options = optimize_contact_region_kinematics('options');
contact_kinematics_options.model_detail = 'high';
contact_kinematics_options.just_evaluate_guess = true;

contact_kinematics_options.load_from_file = false;;
contact_kinematics_problem.ball_position_pp = pos_pp;
contact_kinematics_problem.joint_initial_guess = iiwa.home_config;
contact_kinematics_problem.optim_initial_guess = guess;
contact_kinematics_options.save_file = ['contact_kinematics_result', '_tmp1', '.mat'];
contact_kinematics_options.num_spline_eval_points = 200;
contact_kinematics_options.num_results_waypoints = 50;
contact_kinematics_options.num_optimization_ik_waypoints = 10;
contact_kinematics_options.cmaes_opts.StopFitness = 5e-9
contact_kinematics_options.debug_plot = true;


[solution, joint_plan_current, plan_data] = optimize_contact_region_kinematics_figgen(contact_kinematics_problem, contact_kinematics_options);

% % Convert surface velocity to relative to world not ball COM.
% surface_vel_rel_world = interp1(tspan, velspan,surface_vel_interp.tspan_complete) + surface_vel_interp.dspan_complete;

% 0's for no contact, 1's for contact, made to span ball timings. Not the
% exact times of leaving/entering contact.
contact_status = ones(size(plan_data.data_tspan));

%% Save to file.
save_plan_to_file(['small_arc', '_dat'], joint_plan_current.angles, joint_plan_current.breaks, ...
    plan_data.data_tspan, plan_data.ball_pos + [0,0,ball_radius], plan_data.ball_vel, plan_data.ball_accel, plan_data.ball_omega, plan_data.ball_quat, ...
    plan_data.data_tspan, plan_data.world_contact_position, plan_data.ball_vel + plan_data.world_surface_velocity_relative, contact_status);

figure(scene_fig);
hold on;
% path_pts_robot_plot = plot3(path_pts_robot(1,:),path_pts_robot(2,:),path_pts_robot(3,:),'b','LineWidth',8);

%% Play the winning solution.
write_to_vid = true;
if write_to_vid
    vid_writer = VideoWriter('../../../data/videos/raw/little_segment_optim_result_bad.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = 60;
    vid_writer.Quality = 100;
    framerate = 60;
    open(vid_writer);
end

curr_time = 0;
laps = 1;
tfactor = 1;
prev_time = 0;
campos([ 1.1527    1.2286    0.6807]);
while curr_time < plan_data.data_tspan(end)
    
    quat = interp1(plan_data.data_tspan, plan_data.ball_quat, curr_time);
    pos = interp1(plan_data.data_tspan, plan_data.ball_pos, curr_time);
    ball_patch.Vertices = quatrotate(quat, ball_verts_untransformed) + repmat(pos + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    % Just for manipulating the blue ghost version.
%     surface_to_origin_translation = trvec2tform(-plan_data.surface_contact_position(i,:));
%     world_origin_to_goal_pt_translation = trvec2tform(plan_data.world_contact_position(i,:));
%     
%     surf_vel_untransformed_to_planar_vel = rotm2tform(plan_data.surface_contact_rotation(:,:,i)');
%     ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(plan_data.surface_contact_normal(1,:), plan_data.surface_contact_normal(i,:))); % upvec_rot(:,:,i));%
%     dummy_foot_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
%     
    joint_pos_struct = interpolate_traj(iiwa, joint_plan_current.breaks, joint_plan_current.angles, curr_time);
    display_at_pose(iiwa, joint_pos_struct);
    
%     robot_tform_path_pts = getTransform(iiwa.robot, joint_pos_struct, 'iiwa_link_6','iiwa_link_0')*path_pts_robot;
%     path_pts_robot_plot.XData = robot_tform_path_pts(1,:);
%     path_pts_robot_plot.YData = robot_tform_path_pts(2,:);
%     path_pts_robot_plot.ZData = robot_tform_path_pts(3,:);
    
    drawnow;
    prev_time = curr_time;
    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
        curr_time = 1./framerate * tfactor + curr_time;      
    else
       curr_time = toc * tfactor;
    end
       curr_time = mod(curr_time, tspan(end));  % If we want repeat
       if prev_time > curr_time
           laps = laps - 1;
           if laps == 0
               break;
           end
       end
end


end
