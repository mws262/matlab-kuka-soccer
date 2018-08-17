% Test getting a link to a face point matching its normal. NOT a test of
% IK, just teleports the link there. See test_ik_goal for the extension of
% this.

close all; clear all;
addpath ../;
addpath ../vis;
addpath ../geometry;

scene_fig = make_visualizer_scene();
iiwa = IIWAImporter(scene_fig);

lpatch = iiwa.link_patches{2};
verts = lpatch.Vertices;
faces = lpatch.Faces;
tar_face = 4417;
cdata = zeros(size(lpatch.Faces));
cdata(:,1) = 1;
cdata(tar_face,:) = [0 1 0];
lpatch.FaceVertexCData = cdata;
lpatch.FaceColor = 'flat';
lpatch.CDataMapping = 'direct';


ee_reference = [0, 10, 1.2610]; % End effector position in baseline pose, as shown in the stls when loaded.
[v1, v2, v3] = get_verts_from_face_idx(tar_face, faces, verts);
[vertex_normals, face_normals] = get_all_normals(faces, verts);
surf_normal = double(face_normals(tar_face,:));

rel_target = double(v1);
target_vec = [0 0.8 0.8];
target_normal = [0,-1,0];
target_normal = target_normal/norm(target_normal);

hold on;
plot3(rel_target(1), rel_target(2), rel_target(3), '.g', 'MarkerSize', 20);
original_link = patch('Faces', faces, 'Vertices', verts);
plot3(target_vec(1), target_vec(2), target_vec(3), '.b', 'MarkerSize', 20);

for i = 0:0.05:2*pi
    net_transform = transform_link_to_touch_surface_pt(rel_target, surf_normal, target_vec, target_normal, i);
    iiwa.hgtransforms{2}.Matrix = net_transform;   
    drawnow;
    pause(0.005);
end