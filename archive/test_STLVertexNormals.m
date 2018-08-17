% Make sure that STLVertexNormals (used by SampleShapeData) is calculating
% normals in the correct directions.
close all; clear all;
addpath ../geometry;
[pstruct_1, pstruct_2] = SampleShapeData();

% pstruct_1.vertices = pstruct_1.vertices*0.05;
p = patch('Vertices', pstruct_1.vertices, 'Faces', pstruct_1.faces);
p.FaceColor = [1,0,0];
p.FaceAlpha = 0.7;
hold on;
quiver3(pstruct_1.vertices(:,1),pstruct_1.vertices(:,2),pstruct_1.vertices(:,3),pstruct_1.vertex_normals(:,1),pstruct_1.vertex_normals(:,2),pstruct_1.vertex_normals(:,3));
centroids = zeros(size(pstruct_1.faces));
for i = 1:size(centroids,1)
    [v1,v2,v3] = get_verts_from_face_idx(i, pstruct_1.faces, pstruct_1.vertices);
   centroids(i,:) = (v1 + v2 + v3)/3;
end
quiver3(centroids(:,1),centroids(:,2),centroids(:,3),pstruct_1.face_normals(:,1),pstruct_1.face_normals(:,2),pstruct_1.face_normals(:,3));

camlight HEADLIGHT;
daspect([1,1,1]);
hold off;