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
num_pts = 200; % Points to evaluate at along the path.
pos_pp = get_path_pp('small_arc', total_time);

[tspan, posspan, velspan, accelspan, omegaspan, quatspan, ...
    world_contact_loc_desired_fcn, contact_loc_desired_rel_com_fcn] = evaluate_spline(pos_pp, ball_radius, num_pts);

draw_path_and_accel(posspan, accelspan, 3); % Draw out the path and acceleration arrows.

% States, etc
approach_angle = pi - 0.4;
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
    50, ... % Number of points to run IK at along the path.
    iiwa, ... % Robot object for running IK.
    home, ... % Initial guess pose structure for arm.
    false, ... % Do we stop looking if we fail to successfully integrate over the surface of the foot?
    banned_region_dat); % 

if fail_flag
    disp('Integration over the surface resulted in rolling over bad parts of the arm. It terminated.');
else
    solres = [solinfo{:}];
    err = sum([solres.PoseErrorNorm])/length(solres);
    fprintf('Average pose error: %f\n', err);
    any(~cellfun(@(stat)(strcmp(stat.Status, 'success')), solinfo)) % Were any stages not successful?
end


foot_path_trace = plot3(result_path_pts(:,1), result_path_pts(:,2), result_path_pts(:,3), 'r', 'LineWidth', 5);
foot_path_trace.Parent = dummy_foot_tform;
foot_path_trace2 = plot3(result_path_pts(:,1), result_path_pts(:,2), result_path_pts(:,3), 'g', 'LineWidth', 10);
test_tform = hgtransform;
foot_path_trace2.Parent = test_tform;

%% Some sanity checking.

% Check length of path as defined on the foot and on the ball.

% Transform velocity at surface to a plane. Check length of the trace.
planar_vel = zeros(size(surface_vel));
for i = 1:size(surface_vel,1)
    planar_vel(i,:) = get_rotation_from_vecs(up_vector(i,:), [0 0 1])*surface_vel(i,:)';
end

% Estimate length of path on the ball.
planar_ball_path = cumtrapz(tspan, planar_vel);
planar_ball_trace_length = sum(sqrt(sum(diff(planar_ball_path).*diff(planar_ball_path),2)));
% Estimate length of path on the foot.
foot_surface_trace_length = sum(sqrt(sum(diff(result_path_pts).*diff(result_path_pts),2)));
assert(max(foot_surface_trace_length - planar_ball_trace_length) < 1e-3);

joints_over_time = zeros(length(jnt_angles),7);
for i = 1:length(jnt_angles)
    joints_over_time(i,:) = [jnt_angles{i}.JointPosition];
end

[b,a] = butter(2,0.1);
filt_joints = filtfilt(b,a,joints_over_time);

pp_joints = pchip(breaks, joints_over_time')
pp_joints_vel = fnder(pp_joints,1);
est_joint_vel = ppval(pp_joints_vel, breaks)';%diff(filt_joints)./diff(breaks)

est_joint_times = breaks(1:end);
est_link_vel = zeros(6,size(est_joint_vel,1));
for i = 1:size(est_joint_vel,1)
    est_link_vel(:,i) = geometricJacobian(iiwa.robot, jnt_angles{i}, link_name)*est_joint_vel(i,:)'
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
path_pts_robot = robot_to_dummy*[result_path_pts';ones(1,size(result_path_pts,1))];


% arm_pt_vel = cross(est_link_vel(1:3,:)', interp1(tspan, path_pts_robot(1:3,:)', est_joint_times)) + est_link_vel(4:end,:)'
% ball_pt_vel = interp1(tspan, surface_vel + velspan, est_joint_times);
% figure;
% hold on;
% plot(arm_pt_vel);
% plot(ball_pt_vel);
% hold off;


path_pts_robot_plot = plot3(path_pts_robot(1,:),path_pts_robot(2,:),path_pts_robot(3,:),'b','LineWidth',8);


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
    
    
    robot_tform_path_pts = getTransform(iiwa.robot, joint_pos_struct, 'iiwa_link_6','iiwa_link_0')*path_pts_robot;
    path_pts_robot_plot.XData = robot_tform_path_pts(1,:);
    path_pts_robot_plot.YData = robot_tform_path_pts(2,:);
    path_pts_robot_plot.ZData = robot_tform_path_pts(3,:);
%     test_tform.Matrix = link_tform;
    
    display_at_pose(iiwa, joint_pos_struct);

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