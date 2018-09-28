
mdata = get_mesh_data('twentyhedron');

close all;
figure;
ptch = patch('Vertices', mdata.vertices, 'Faces', mdata.faces);
view(3);
axis equal;

ptch.FaceColor = 'r';

all_edges = [mdata.faces(:,1:2); mdata.faces(:,2:3)];
unique_edges = unique(sort(all_edges,2),'rows');

hold on;
[X,Y,Z] = sphere(50);
surf_pts = zeros(numel(X),3);
normal_vecs = zeros(numel(X),3);
for i = 1:numel(X)
    [ distance, surface_point, normal_vec, face_idx ] = point2trimesh_with_normals(0.18*[X(i),Y(i),Z(i)], mdata);
    surf_pts(i,:) = surface_point;
    normal_vecs(i,:) = normal_vec;
end

quiv = quiver3(surf_pts(:,1), surf_pts(:,2), surf_pts(:,3), 0.02*normal_vecs(:,1), 0.02*normal_vecs(:,2), 0.02*normal_vecs(:,3));
quiv.AutoScale = 'off';



% edge_v1 = mdata.vertices(unique_edges(:,1),:);
% edge_vn1 = mdata.vertex_normals(unique_edges(:,1),:);
% edge_v2 = mdata.vertices(unique_edges(:,2),:);
% edge_vn2 = mdata.vertex_normals(unique_edges(:,2),:);
% 
% edge_vecs = edge_v2 - edge_v1;
% edge_norms = edge_vn2 - edge_vn1;
% edge_lengths = sqrt(sum(edge_vecs.*edge_vecs,2));
% 
% curvatures = dot(edge_norms, edge_vecs,2)./edge_lengths.^2;
% radii = curvatures.^-1;



