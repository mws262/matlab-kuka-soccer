% Test the mesh marking. This should mark a specified index on the mesh
% with red and the surrounding ones with yellow for visibility.

close all; clear all;
addpath ../;
addpath ../vis/;
addpath ../geometry/;
addpath ../derived_autogen/;
addpath ../data/;

% Ball and scene
radius = 0.1; %BALL radius
ball_velocity = [0, -1, 0];
scene_fig = make_visualizer_scene();
[ball_patch, ball_verts_untransformed] = make_ball(radius);

% Manipulator mesh
detail_level = 1;
geo_data = load('../data/iiwa_merged_end_effector.mat');
picked_pts = load('../data/picked_faces_top.mat', 'selections');
faces = geo_data.merged_iiwa(detail_level).faces;
vertices = geo_data.merged_iiwa(detail_level).vertices;
face_normals = geo_data.merged_iiwa(detail_level).face_normals;
vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
iiwa_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
iiwa_patch.EdgeColor = [0.1, 0.1, 0.1];
iiwa_patch.FaceColor = [1 0 0];
iiwa_patch.FaceAlpha = 0.85;
iiwa_tform = hgtransform;
iiwa_patch.Parent = iiwa_tform;
iiwa_patch.FaceColor = 'flat'
iiwa_patch.FaceVertexCData = repmat([0.3,1,0.5], [size(iiwa_patch.Faces,1),1]);


face_idx = 1100:1:1150;

mark_on_mesh(face_idx, iiwa_patch);