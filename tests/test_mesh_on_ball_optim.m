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
guess = [0.1404
    0.2772
    5.8308
    1.0442];

contact_kinematics_problem = optimize_contact_region_kinematics('problem');
contact_kinematics_problem.robot = iiwa;
contact_kinematics_options = optimize_contact_region_kinematics('options');
contact_kinematics_options.model_detail = 'high';
contact_kinematics_options.just_evaluate_guess = true;
contact_kinematics_options.load_from_file = false;
contact_kinematics_problem.ball_position_pp = pos_pp;
contact_kinematics_problem.joint_initial_guess = iiwa.home_config;
contact_kinematics_problem.optim_initial_guess = guess;
contact_kinematics_options.save_file = ['contact_kinematics_result', '_tmp1', '.mat'];
contact_kinematics_options.num_spline_eval_points = 200;
contact_kinematics_options.num_results_waypoints = 50;
contact_kinematics_options.num_optimization_ik_waypoints = 10;
contact_kinematics_options.cmaes_opts.StopFitness = 5e-9
contact_kinematics_options.debug_plot = false;


[solution, joint_plan_current, plan_data] = optimize_contact_region_kinematics(contact_kinematics_problem, contact_kinematics_options);

% % Convert surface velocity to relative to world not ball COM.
% surface_vel_rel_world = interp1(tspan, velspan,surface_vel_interp.tspan_complete) + surface_vel_interp.dspan_complete;

% 0's for no contact, 1's for contact, made to span ball timings. Not the
% exact times of leaving/entering contact.
contact_status = ones(size(plan_data.data_tspan));

%% Save to file.
save_plan_to_file(['small_arc', '_dat'], joint_plan_current.angles, joint_plan_current.breaks, ...
    plan_data.data_tspan, plan_data.ball_pos + [0,0,ball_radius], plan_data.ball_vel, plan_data.ball_accel, plan_data.ball_omega, plan_data.ball_quat, ...
    plan_data.data_tspan, plan_data.world_contact_position, plan_data.ball_vel + plan_data.world_surface_velocity_relative, contact_status);


%% Some sanity checking.

% Check length of path as defined on the foot and on the ball.

% Transform velocity at surface to a plane. Check length of the trace.
planar_vel = zeros(size(plan_data.world_surface_velocity_relative));
for i = 1:size(plan_data.world_surface_velocity_relative,1)
    planar_vel(i,:) = get_rotation_from_vecs(plan_data.surface_contact_normal(i,:), [0 0 1])*plan_data.world_surface_velocity_relative(i,:)';
end

% Estimate length of path on the ball.
planar_ball_path = cumtrapz(plan_data.data_tspan, planar_vel);
planar_ball_trace_length = sum(sqrt(sum(diff(planar_ball_path).*diff(planar_ball_path),2)));
% Estimate length of path on the foot.
foot_surface_trace_length = sum(sqrt(sum(diff(plan_data.surface_contact_position).*diff(plan_data.surface_contact_position),2)));
assert(max(foot_surface_trace_length - planar_ball_trace_length) < 1e-3);

joints_over_time = zeros(length(joint_plan_current.angles),7);
for i = 1:length(joint_plan_current.angles)
    joints_over_time(i,:) = [joint_plan_current.angles{i}.JointPosition];
end

pp_joints = spline(joint_plan_current.breaks, joints_over_time');
pp_joints_vel = fnder(pp_joints,1);
est_joint_vel = ppval(pp_joints_vel, joint_plan_current.breaks)';%diff(filt_joints)./diff(breaks)

est_joint_times = joint_plan_current.breaks(1:end);
est_link_vel = zeros(6,size(est_joint_vel,1));
est_link_vel_jac = zeros(6,size(est_joint_vel,1));
est_joint_vel_jac = zeros(size(est_joint_vel,1),7);

ball_pt_vel = interp1(plan_data.data_tspan, plan_data.world_surface_velocity_relative + plan_data.ball_vel, est_joint_times);
for i = 1:size(est_joint_vel,1)
    jac = geometricJacobian(iiwa.robot, joint_plan_current.angles{i}, 'iiwa_link_6');
    est_link_vel(:,i) = jac*est_joint_vel(i,:)';
    est_joint_vel_jac(i,:) = (pinv(jac(4:end,:))*ball_pt_vel(i,:)')';
    est_link_vel_jac(:,i) = jac*est_joint_vel_jac(i,:)';
end

total_dummy_to_robot = [0    1.0000         0         0
         0         0   -1.0000         0
   -1.0000         0         0    0.2155
         0         0         0    1.0000];
     
dummy_to_robot_rotation = tform2rotm(total_dummy_to_robot);
dummy_to_robot_translation = tform2trvec(total_dummy_to_robot);
% Transform path points on dummy to link's so that
% getTransform(...)*path_pts_robot matches the path in the real world.

robot_to_dummy = rotm2tform(dummy_to_robot_rotation')*trvec2tform(-dummy_to_robot_translation);
path_pts_robot = robot_to_dummy*[plan_data.surface_contact_position';ones(1,size(plan_data.surface_contact_position,1))];

arm_pt_vel = cross(est_link_vel_jac(1:3,:)', interp1(plan_data.data_tspan, path_pts_robot(1:3,:)', est_joint_times)) + est_link_vel_jac(4:end,:)'
figure;

save('test_traj.mat', 'joints_over_time', 'ball_pt_vel', 'est_joint_times', 'iiwa');
for i = 1:3
    subplot(3,1,i);
    hold on;
    plot(joint_plan_current.breaks,   ball_pt_vel(:,i) - arm_pt_vel(:,i), 'r.');
%     plot(joint_plan_current.breaks, ball_pt_vel(:,i), 'b.');
end

norm_vel_error = sqrt(sum((arm_pt_vel - ball_pt_vel).^2, 2));

figure(scene_fig);
hold on;
path_pts_robot_plot = plot3(path_pts_robot(1,:),path_pts_robot(2,:),path_pts_robot(3,:),'b','LineWidth',8);

%% Play the winning solution.
campos([1.6970    1.5293    0.9466]);
for i = 1:length(plan_data.data_tspan) - 1
    
    ball_patch.Vertices = quatrotate(plan_data.ball_quat(i,:), ball_verts_untransformed) + repmat(plan_data.ball_pos(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    % Just for manipulating the blue ghost version.
    surface_to_origin_translation = trvec2tform(-plan_data.surface_contact_position(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(plan_data.world_contact_position(i,:));
    
    surf_vel_untransformed_to_planar_vel = rotm2tform(plan_data.surface_contact_rotation(:,:,i)');
    ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(plan_data.surface_contact_normal(1,:), plan_data.surface_contact_normal(i,:))); % upvec_rot(:,:,i));%
    dummy_foot_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    joint_pos_struct = interpolate_traj(iiwa, joint_plan_current.breaks, joint_plan_current.angles, plan_data.data_tspan(i));
    display_at_pose(iiwa, joint_pos_struct);
    
    robot_tform_path_pts = getTransform(iiwa.robot, joint_pos_struct, 'iiwa_link_6','iiwa_link_0')*path_pts_robot;
    path_pts_robot_plot.XData = robot_tform_path_pts(1,:);
    path_pts_robot_plot.YData = robot_tform_path_pts(2,:);
    path_pts_robot_plot.ZData = robot_tform_path_pts(3,:);
    
    drawnow;
    pause(0.001);
end


end
