clear all; close all;
addpath ../geometry;
addpath ../;

% Unreduced load of mesh.
[merged_faces_full, merged_vertices_full, merged_face_normals_full] = stlread('iiwa_merged_end_effector_simplified.stl');
merged_vertices_full = merged_vertices_full/100; % cm -> meters
merged_vertex_normals_full = STLVertexNormals(merged_faces_full, merged_vertices_full, merged_face_normals_full);

% Plot
fig = figure;
iiwa_patch = patch('Faces', merged_faces_full, 'Vertices', merged_vertices_full, 'FaceNormals', merged_face_normals_full, 'VertexNormals', merged_vertex_normals_full);
iiwa_patch.EdgeColor = 'none';
iiwa_patch.FaceColor = 'r';
iiwa_patch.FaceNormalsMode = 'auto';

camlight HEADLIGHT;
view(3);
daspect([1,1,1]);

% Reduce patch. Mid-res
reducepatch(iiwa_patch,0.5);
drawnow;
merged_faces_mid = iiwa_patch.Faces;
merged_vertices_mid = iiwa_patch.Vertices;
merged_face_normals_mid = -iiwa_patch.FaceNormals;
merged_vertex_normals_mid = STLVertexNormals(merged_faces_mid, merged_vertices_mid, merged_face_normals_mid);


% Reduce patch. Low-res
reducepatch(iiwa_patch,0.2); % MULTIPLICATIVE WITH PREVIOUS REDUCTION.
drawnow;
merged_faces_low = iiwa_patch.Faces;
merged_vertices_low = iiwa_patch.Vertices;
merged_face_normals_low = -iiwa_patch.FaceNormals;
merged_vertex_normals_low = STLVertexNormals(merged_faces_low, merged_vertices_low, merged_face_normals_low);

merged_iiwa = struct('faces', {merged_faces_full, merged_faces_mid, merged_faces_low}, ...
    'vertices', {merged_vertices_full, merged_vertices_mid, merged_vertices_low}, ...
    'face_normals', {merged_face_normals_full, merged_face_normals_mid, merged_face_normals_low}, ...
    'vertex_normals', {merged_vertex_normals_full, merged_vertex_normals_mid, merged_vertex_normals_low});

save('iiwa_merged_end_effector.mat', 'merged_iiwa');

hold on;
vert_n = quiver(0,0);
vert_n.AutoScale = 'off';
vert_n.XData = merged_vertices_full(merged_faces_full(:,1),1);
vert_n.YData = merged_vertices_full(merged_faces_full(:,1),2);
vert_n.ZData = merged_vertices_full(merged_faces_full(:,1),3);
vert_n.UData = merged_face_normals_full(:,1)*0.02;
vert_n.VData = merged_face_normals_full(:,2)*0.02;
vert_n.WData = merged_face_normals_full(:,3)*0.02;

