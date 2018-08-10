close all; clear all;
addpath ..;
addpath ../vis;
addpath ../geometry
load('../data/picked_faces.mat');
scene_fig = make_visualizer_scene();
iiwa = IIWAImporter(scene_fig);
hold on;
% for i = 2:size(connecting_pps,1)
%     eval_pp = ppval(connecting_pps(i), linspace(accel_zero_break_start(i), accel_zero_break_end(i), 25))';
%     connecting_spline_plot = plot3(eval_pp(:,1), eval_pp(:,2), eval_pp(:,3), 'LineWidth', 1.5); % Draw line projection downwards
%     connecting_spline_plot.Color = [0.8, 0.8, 1, 0.9];
%     connecting_spline_plot_shadows = plot3(eval_pp(:,1), eval_pp(:,2), 0.001*ones(size(eval_pp,1),1), 'LineWidth', 1); % Draw line projection downwards
%     connecting_spline_plot_shadows.Color = [0.1, 0.1, 0.1, 0.1];
%     break;  
% end
tar_face = selected_faces(5);
lpatch = iiwa.link_patches{2};
cdata = zeros(size(lpatch.Faces));
cdata(:,1) = 1;
cdata(tar_face,:) = [0 1 0];
lpatch.FaceVertexCData = cdata;
lpatch.FaceColor = 'flat';
lpatch.CDataMapping = 'direct';
verts = lpatch.Vertices;
faces = lpatch.Faces;

link_name = 'iiwa_link_7'; % Tip of ee. iiwa_link_ee is just the last full link.
home = iiwa.home_config;
guess = home;
tform_init = getIIWATForm(iiwa, home, link_name);
%%%%%%

%link_origin = [0, 1, 5.2610]; % End effector position in baseline pose, as shown in the stls when loaded.
[v1, v2, v3] = get_verts_from_face_idx(tar_face, faces, verts);
link_surface_normal = double(get_face_normal(tar_face, faces, verts));
link_surface_normal = link_surface_normal/norm(link_surface_normal);

scrub_angle = 0;
goal_normal = [0 1 0];
goal_pt = [0.0, 0.0, 0.9];
surface_pt = double([v1(1), v1(2), v1(3)]);%double((v1 + v2 + v3)/3);
link_surface_normal = double([link_surface_normal(1), link_surface_normal(2), link_surface_normal(3)]);
net_tform = transform_link_to_touch_surface_pt(surface_pt, link_surface_normal, goal_pt, goal_normal, scrub_angle);

plot3(goal_pt(1), goal_pt(2), goal_pt(3), '.b', 'MarkerSize', 20);
%%%%%%

tform2 = trvec2tform([-0.4,-0.4,0.5]);
tform3 = trvec2tform([2,-1,1]);

duration = 6;
numpts = 2;
breaks = linspace(0, duration, numpts);
% 
% start_rot = get_rotation_from_vecs([0,0,1], eval_pp(2,:) - eval_pp(1,:))
% tform_list(:,:,1) = trvec2tform(eval_pp(1,:));
% tform_list(1:3,1:3,1) = start_rot;
% 
% tform_list(:,:,2) = trvec2tform(eval_pp(floor(size(eval_pp,1)/2),:))
% 
% end_rot = get_rotation_from_vecs([0,0,1], eval_pp(end,:) - eval_pp(end - 1,:))
tform_list(:,:,1) = tform_init;
weird_tform = [tform_init(1:3,1:3)', [-tform_init(1:3,4)];[0 0 0 1]];

weird_tform2 = [tform_init(1:3,1:3), [tform_init(1:3,4)];[0 0 0 1]];
tform_list(:,:,2) = net_tform*tform_init;
% trvec2tform(eval_pp(end,:));
% tform_list(1:3,1:3,3) = end_rot;

[jnt_angles, solinfo] = make_multi_point_trajectory(iiwa, tform_list, guess, link_name);
any(~cellfun(@(stat)(strcmp(stat.Status, 'success')), solinfo)) % Were any stages not successful?

curr_time = 0;
prev_time = 0;

tic;
while curr_time < duration
    
    joint_pos_struct = interpolate_traj(iiwa, breaks, jnt_angles, curr_time);
    display_at_pose(iiwa, joint_pos_struct);
    prev_time = curr_time;
    curr_time = toc;
    drawnow;
end