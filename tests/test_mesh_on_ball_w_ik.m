% Extends test_mesh_on_ball_contact and test_ik_goal to have the link roll
% over the ball to make it follow a path. IK tries to track this as best as
% possible. Everything has to be selected carefully for IK to succeed.
close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;
addpath ../iiwa_kinematics/;
addpath ../dynamics/;

%% Ball and scene
ball_radius = 0.1;
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(ball_radius);

%% Import data for the dummy planning foot and the banned regions on it.
dummy_foot_dat = get_mesh_data('dummy_manipulator_high_res');
banned_region_dat = get_mesh_data('manipulator_banned1');
picked_pts = load('../data/picked_faces_single_end.mat', 'selections'); % For targeting specific previously selected faces if desired.

% Settings for the dummy foot patch.
hold on;
dummy_foot_patch = patch('Faces', dummy_foot_dat.faces, 'Vertices', dummy_foot_dat.vertices, 'FaceNormals', dummy_foot_dat.face_normals, ...
    'VertexNormals', dummy_foot_dat.vertex_normals);
dummy_foot_patch.EdgeColor = 'none';
dummy_foot_patch.FaceColor = 'flat';
dummy_foot_patch.FaceVertexCData = repmat([0 0 1], [size(dummy_foot_patch.Faces,1),1]);
dummy_foot_patch.FaceAlpha = 0.3;
dummy_foot_patch.BackFaceLighting = 'lit';
dummy_foot_tform = hgtransform;
dummy_foot_patch.Parent = dummy_foot_tform;

%% Whole robot import.
iiwa = IIWAImporter(scene_fig);
link_name = 'iiwa_link_6'; % Tip of ee. iiwa_link_ee is just the last full link.
home = iiwa.home_config;
guess = home;

%% Load path spline.
total_time = 5;
num_pts = 100; % Points to evaluate at along the path.
pos_pp = get_path_pp('small_arc', total_time);

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(pos_pp, ball_radius, num_pts);

draw_path_and_accel(posspan, accelspan, 3); % Draw out the path and acceleration arrows.

% States, etc
approach_angle = pi- 0.5;
arc_angle = pi/2 - 0.1  + zeros(size(tspan));
contact_loc_over_time = world_contact_loc_desired_fcn(arc_angle);
contact_loc_rel_com = contact_loc_desired_rel_com_fcn(arc_angle);
up_vector = contact_loc_rel_com/ball_radius;
surface_vel = cross(omegaspan, contact_loc_rel_com, 2);

picked_idx = 1;
initial_surface_point = picked_pts.selections.points(picked_idx,:); % Grab a surface point from our hand-selected bunch.
% mark_on_mesh(picked_pts.selections.faces(picked_idx), iiwa_patch); % Mark it on the mesh.

[jnt_angles, breaks, solinfo, proj_start_pt, result_path_pts, path_rot_integrated, fail_flag] = attempt_kinematics_over_surface( ...
    initial_surface_point, ... % Point on (or near) surface of dummy link.
    approach_angle, ... % Angle to place the foot down at.
    tspan, ... % Time interval to plan over.
    surface_vel, ... % Velocity to integrate over the surface.
    up_vector, ... % Normal unit vector to the ball.
    contact_loc_over_time, ... % Contact point with the ball, in world coords.
    dummy_foot_dat, ... % Geometry data for the disembodied foot.
    25, ... % Number of points to run IK at along the path.
    iiwa, ... % Robot object for running IK.
    home, ... % Initial guess pose structure for arm.
    false, ... % Do we stop looking if we fail to successfully integrate over the surface of the foot?
    banned_region_dat); % 

if fail_flag
    disp('Integration over the surface resulted in rolling over bad parts of the arm. It terminated.');
else
    solres = [solinfo{:}];
    err = sum([solres.PoseErrorNorm])/length(solres)
    fprintf('Average pose error: %f\n', err);
    any(~cellfun(@(stat)(strcmp(stat.Status, 'success')), solinfo)) % Were any stages not successful?
end

% Video recording if desired
write_to_vid = false;
if write_to_vid
    framerate = 60;
    vid_writer = VideoWriter('rolly_link_vid2.avi'); % Convert for Slack with 'ffmpeg -i infile.avi youroutput.mp4'
    vid_writer.FrameRate = framerate;
    vid_writer.Quality = 100;
    open(vid_writer);
end
campos([1.6970    1.5293    0.9466]);
for i = 1:length(tspan) - 1
    ball_patch.Vertices = quatrotate(quatspan(i,:), ball_verts_untransformed) + repmat(posspan(i,:)  + [0, 0, ball_radius], [size(ball_verts_untransformed,1),1]);
    
    % Just for manipulating the blue ghost version.
    surface_to_origin_translation = trvec2tform(-result_path_pts(i,:));
    world_origin_to_goal_pt_translation = trvec2tform(contact_loc_over_time(i,:));
    surf_vel_untransformed_to_planar_vel = rotm2tform(path_rot_integrated(:,:,i)');
    dummy_foot_tform.Matrix = world_origin_to_goal_pt_translation * surf_vel_untransformed_to_planar_vel * surface_to_origin_translation;
    
    
    joint_pos_struct = interpolate_traj(iiwa, breaks, jnt_angles, tspan(i));
    display_at_pose(iiwa, joint_pos_struct);
    
    %     camvec = (angle2dcm(0,0,i/200, 'xyz')*(lin_vel(i,:)*1.2 + [0.0 0 .25])')';
    %     camtarget(pos(i,:));
    %     campos(pos(i,:) + camvec);
    drawnow;
    %pause(0.001);
    if write_to_vid
        writeVideo(vid_writer, getframe(scene_fig));
    end
end

if write_to_vid
    close(vid_writer);
    % Convert from avi to mp4.
    !ffmpeg -i make_8_vid.avi make_8_vid.mp4
end