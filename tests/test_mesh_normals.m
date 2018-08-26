% Visualize the face and vertex normals. Make sure they all point outward
% as expected.
close all; clear all;
addpath ..;
addpath ../vis;
addpath ../geometry;
addpath ../data/;

mesh_data = get_mesh_data('dummy_manipulator_high_res');

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



v = mesh_data.vertices;
[xg, yg] = meshgrid(linspace(min(v(:,1)),max(v(:,1)),10), linspace(min(v(:,2)),max(v(:,2)),10));%, linspace(min(v(:,3)),max(v(:,3)),20));
test_pts = [xg(:), yg(:), yg(:)*0 + 0.2];
test_pts = [test_pts; xg(:), yg(:)*0 + 0.2, yg(:)];
test_pts = [test_pts; xg(:) * 0 + 0.2, xg(:), yg(:)];
test_pts = [test_pts; -test_pts];

% test_pts = zeros(1000,3);
% test_pts(:,1) = linspace(-.1,.15,1000);
% test_pts(:,2) = -0.1;
% test_pts(:,3) = 0.2;
surf_pts = zeros(size(test_pts));
norm_vec = zeros(size(test_pts));

for i = 1:size(test_pts,1)
   [ distance, surf_pts(i,:), norm_vec(i,:), face_idx ] = point2trimesh_with_normals( test_pts(i,:), mesh_data.faces, v, mesh_data.face_normals, mesh_data.vertex_normals ); 
   
end

quiver3(surf_pts(:,1), surf_pts(:,2), surf_pts(:,3), norm_vec(:,1), norm_vec(:,2), norm_vec(:,3), 'y');

% [ distance, surface_point, normal_vec, face_idx ] = point2trimesh_with_normals( point, faces, vertices, face_normals, vertex_normals );
% 
% distance, surface_point, normal_vec, face
