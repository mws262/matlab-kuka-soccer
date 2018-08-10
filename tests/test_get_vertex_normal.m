close all; clear all;
addpath ..;
addpath ../vis;
addpath ../geometry
fig = figure;
SampleShapeData;


lpatch = patch('Faces', F1, 'Vertices', V1);
% iiwa = IIWAImporter(fig);
% lpatch = iiwa.link_patches{2};
% 
% fig2 = figure;
% fig2.Position = [3000, 1000, 1000, 1000];
% ax = axes;
% lpatch.Parent = ax;
lpatch.FaceAlpha = 0.8;
lpatch.LineStyle = '-';
lpatch.EdgeAlpha = 1;
lpatch.EdgeColor = [0 0 0];
lpatch.FaceColor = 'r';
% usrcallback = @(obj,hit,faceIndex)(disp(faceIndex));
% lpatch.ButtonDownFcn = @(obj,hit)patchfacefcn(obj,hit,usrcallback,'face');
% close(fig);


verts = lpatch.Vertices;
faces = lpatch.Faces;
normals = lpatch.FaceNormals;

camlight HEADLIGHT;
hold on;

% vert_vals = verts(vert_idx, :);
% plot3(vert_vals(1), vert_vals(2), vert_vals(3), '.r', 'MarkerSize', 15);

normals = zeros(size(faces));
for i = 1:size(faces,1)
    normals(i,:) = get_face_normal(i, faces, verts);
    normals(i,:) = normals(i,:)/norm(normals(i,:));
end

for i = 1:1:size(verts,1)
    vert_vals = verts(i, :);
vert_norm = get_vertex_normal(i, faces, verts, normals);
quiv = quiver(0,0);
quiv.XData = vert_vals(1);
quiv.YData = vert_vals(2);
quiv.ZData = vert_vals(3);
quiv.UData = vert_norm(1)*0.5;
quiv.VData = vert_norm(2)*0.5;
quiv.WData = vert_norm(3)*0.5;
end

drawnow;daspect([1,1,1]);view(3);

hold off;