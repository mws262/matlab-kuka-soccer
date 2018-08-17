% Test getting a point on the foot to a point in space with a specific
% orientation.

close all; clear all;
addpath ..;
addpath ../vis;
addpath ../geometry;
loaded_faces = load('../data/picked_faces.mat');

scene_fig = make_visualizer_scene();
iiwa = IIWAImporter(scene_fig);
hold on;

% Goal face on robot
tar_face = 2900;

foot_patch = iiwa.link_patches{2}; % Grab the foot link.
cdata = zeros(size(foot_patch.Faces));
cdata(:,1) = 1;
foot_patch.FaceVertexCData = cdata;
foot_patch.FaceColor = 'flat';
foot_patch.CDataMapping = 'direct';
mark_on_mesh(tar_face, foot_patch);

verts = foot_patch.Vertices;
faces = foot_patch.Faces;

link_name = 'iiwa_link_6'; % Tip of ee.
home = iiwa.home_config;
guess = home;
tform_init = getIIWATForm(iiwa, home, link_name);
[v1, v2, v3] = get_verts_from_face_idx(tar_face, faces, verts);
[vertex_normals, face_normals] = get_all_normals(faces, verts);

%% Goal parameters
scrub_angle = 0; % Rotation about contact normal.
goal_normal = [0 0 1]; % Match link face normal to this.
goal_pt = [0.0, 0.0, 0.9]; % Location to bring the target face to.
surface_pt = double([v1(1), v1(2), v1(3)]); % Point on surface of the foot.

link_surface_normal = face_normals(tar_face, :);
goal_tform = transform_link_to_touch_surface_pt(surface_pt, link_surface_normal, goal_pt, goal_normal, scrub_angle);
plot3(goal_pt(1), goal_pt(2), goal_pt(3), '.b', 'MarkerSize', 20);

duration = 4;
numpts = 2;
breaks = linspace(0, duration, numpts);
tform_list(:,:,1) = tform_init;
tform_list(:,:,2) = goal_tform*tform_init;

[jnt_angles, solinfo] = make_multi_point_trajectory(iiwa, tform_list, guess, link_name, false);
any(~cellfun(@(stat)(strcmp(stat.Status, 'success')), solinfo)) % Were any stages not successful?

%% Animation loop
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