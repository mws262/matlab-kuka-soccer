function test_mesh_on_ball_optim
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;
addpath ../path_optim/;

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

plot_flag = false;
just_evaluate_guess = false;

% Initial guess.
guess = [    0.1268
    0.3809
    6.1150
    1.2588];

[solution, jnt_angles_optim, breaks_optim, result_path_pts, path_rot_integrated, world_contact_desired_span, up_vector_span] = ...
    optimize_contact_region_kinematics(iiwa, guess, pos_pp, num_evaluation_pts, ball_radius, plot_flag, just_evaluate_guess);

%% Play the winning solution.
campos([1.6970    1.5293    0.9466]);
for i = 1:length(tspan) - 1
    
    ball_patch.Vertices = quatrotate(quatspan(i,:), ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    % Just for manipulating the blue ghost version.
    surface_to_origin_translation = trvec2tform(-result_path_pts(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(world_contact_desired_span(i,:));
    
    surf_vel_untransformed_to_planar_vel = rotm2tform(path_rot_integrated(:,:,i)');
    ball_contact_shift_tform = rotm2tform(get_rotation_from_vecs(up_vector_span(1,:), up_vector_span(i,:))); % upvec_rot(:,:,i));%
    dummy_foot_tform.Matrix = world_origin_to_goal_pt_translation * ball_contact_shift_tform * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    joint_pos_struct = interpolate_traj(iiwa, breaks_optim, jnt_angles_optim, tspan(i));
    display_at_pose(iiwa, joint_pos_struct);
    
    drawnow;
    pause(0.001);
end


end
