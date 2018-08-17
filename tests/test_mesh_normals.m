% Visualize the face and vertex normals. Make sure they all point outward
% as expected.
close all; clear all;
addpath ..;
addpath ../vis;
addpath ../geometry;
addpath ../data/;

mesh_data = get_mesh_data('dummy_manipulator_low_res');

fig = figure;
lpatch = patch('Faces', mesh_data.faces, 'Vertices', mesh_data.vertices, 'FaceNormals', mesh_data.face_normals, 'VertexNormals', mesh_data.vertex_normals);
lpatch.FaceAlpha = 0.8;
lpatch.LineStyle = '-';
lpatch.EdgeAlpha = 1;
lpatch.EdgeColor = [0 0 0];
lpatch.FaceColor = 'r';

camlight HEADLIGHT;
daspect([1,1,1]);
view(3);
hold on;

% Plot vertex normals.
quiver3(mesh_data.vertices(:,1), mesh_data.vertices(:,2), mesh_data.vertices(:,3), ...
    mesh_data.vertex_normals(:,1), mesh_data.vertex_normals(:,2), mesh_data.vertex_normals(:,3));

% Plot face normals.
centroids = get_face_centroids(mesh_data.faces, mesh_data.vertices);
quiver3(centroids(:,1), centroids(:,2), centroids(:,3), ...
    mesh_data.face_normals(:,1), mesh_data.face_normals(:,2), mesh_data.face_normals(:,3));

hold off;