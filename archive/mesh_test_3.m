function mesh_test_3
close all;
clear all;
addpath ./geometry;

fig = figure;
iiwa = IIWAImporter(fig);
lpatch = iiwa.link_patches{6};
% verts = double(lpatch.Vertices);
% lpatchhull = convhull(verts(:,1), verts(:,2), verts(:,3), 'simplify', true);
% lpatch.Faces = lpatchhull;
% reducepatch(lpatch,0.1);
fig2 = figure;
%fig2.Position = [3000, 1000, 1000, 1000];
ax = axes;
lpatch.Parent = ax;
lpatch.FaceAlpha = 1;
lpatch.LineStyle = '-';
lpatch.EdgeAlpha = 1;
lpatch.EdgeColor = [0 0 0];
close(fig);
% [sphere_x,sphere_y,sphere_z] = sphere(12);
% ball_patch = patch(surf2patch(sphere_x, sphere_y, sphere_z, 'triangles'))
% ball_patch.FaceColor = 'b'
% ball_patch.FaceAlpha = 0.3

view(3);
daspect([1,1,1])
xlabel('x');
ylabel('y');
zlabel('z');

faces = lpatch.Faces;
verts = lpatch.Vertices;
hold on;

load('dattest.mat', 'v_surfx_eval', 'v_surfy_eval')
dt = 0.001;
current_face = ceil(size(faces,1)- 100);
start_face_verts = verts(faces(current_face,:), :);
current_pt = mean(start_face_verts,1); % sstart at a consistent point on a mesh.
current_normal = get_face_normal(current_face, faces, verts);
plot3(current_pt(1), current_pt(2), current_pt(3), '.g', 'MarkerSize', 20);

k = 10;
V = verts;
[IDX, C] = kmeans(verts, k);%, 'Distance', 'cosine');
CData = zeros(size(verts));
for h=1:k
    colour_hsv = [360/k*h/360,1,1];
    colour_rgb = hsv2rgb(colour_hsv);
    [rows,cols] = find(IDX==h);
    %plot3(V(rows,1),V(rows,2),V(rows,3), '.', 'Color',colour_rgb, 'MarkerSize', 10);
    CData(rows,:) = repmat(colour_rgb, [length(rows),1]);
end
lpatch.FaceColor = 'interp';
lpatch.FaceVertexCData = CData;

camlight HEADLIGHT;

maxx = min(verts(:,1),[],1);
minx = max(verts(:,1),[],1);
maxy = min(verts(:,2),[],1);
miny = max(verts(:,2),[],1);
maxz = min(verts(:,3),[],1);
minz = max(verts(:,3),[],1);

cyl_rad = max(maxx - minx, maxy - miny)/2;
cyl_off_x = (minx + maxx)/2;
cyl_off_y = (miny + maxy)/2;
cyl_off_z = minz;
cyl_height = maxz - minz;

[x_cyl, y_cyl, z_cyl] = cylinder(cyl_rad, 20);
cyl_patch = patch(surf2patch(x_cyl, y_cyl, z_cyl, 'triangles'))
cyl_patch.Vertices(:,3) = cyl_patch.Vertices(:,3)*cyl_height + cyl_off_z;
cyl_patch.Vertices(:,1) = cyl_patch.Vertices(:,1) + cyl_off_x;
cyl_patch.Vertices(:,2) = cyl_patch.Vertices(:,2) + cyl_off_y;
cyl_patch.FaceAlpha = 0.3;

num_rots = 20;
cyl_th = 0:0.4:2*pi*num_rots;
cyl_pts_x = cyl_rad*cos(cyl_th) + cyl_off_x;
cyl_pts_y = cyl_rad*sin(cyl_th) + cyl_off_y;
cyl_pts_z = linspace(cyl_off_z, cyl_height + cyl_off_z, length(cyl_th));

plot3(cyl_pts_x, cyl_pts_y, cyl_pts_z, '.b', 'MarkerSize', 15);

% tform = get_rotation_from_vecs([0 0 1], current_normal);
[ distances, surface_points ] = point2trimesh('Faces', faces, 'Vertices', verts, 'QueryPoints', [cyl_pts_x; cyl_pts_y; cyl_pts_z]', 'Algorithm', 'vectorized');
plot3(surface_points(:,1), surface_points(:,2), surface_points(:,3), '.g', 'MarkerSize', 20);
% for n = 1:length(cyl_th)
%     pt_star = [cyl_pts_x(n), cyl_pts_y(n), cyl_pts_z(n)];
%     [current_pt, new_face, new_normal_vec] = find_nearest_face(pt_star, faces, verts);
%     if ~isnan(current_pt)
%     plot3(current_pt(1), current_pt(2), current_pt(3), '.g', 'MarkerSize', 20);
%     end
%     disp('hi');
%     drawnow;
% end

    function [projected_pt, nearest_face_idx, normvec] = find_nearest_face(point, faces, verts)
        [nearest_vert, nearest_vert_idx] = find_nearest_vertex(point, verts); % Find the nearest vertex.
        [match_rows, match_cols] = ind2sub(size(faces), find(faces == nearest_vert_idx)); % Find all faces which share this vertex.
        %           plot3(nearest_vert(:,1), nearest_vert(:,2), nearest_vert(:,3), '.r', 'MarkerSize', 20);
        %         tri_corners = plot3(0,0,0, '.g', 'MarkerSize', 15);
        %         activept = plot3(point(1), point(2), point(3), '.r', 'MarkerSize', 15);
        for i = 1:length(match_rows)
            pt1 = verts(faces(match_rows(i),1),:);
            pt2 = verts(faces(match_rows(i),2),:);
            pt3 = verts(faces(match_rows(i),3),:);
            
            normvec = get_face_normal(match_rows(i), faces, verts);
            normvec = normvec./norm(normvec);
            
            p21 = pt2 - pt1;
            p31 = pt3 - pt1;
            p32 = pt3 - pt2;
            % Dot with perpendicular edge vec decides whether the point is
            % within the plane of this face.
            edge_perp21 = cross(p21, normvec);
            edge_perp32 = cross(p32, normvec);
            edge_perp13 = cross(-p31, normvec);
                 
            inside_edge_p21 = dot(edge_perp21, point - pt1) >= 0; % This should catch edges and faces.
            inside_edge_p32 = dot(edge_perp32, point - pt2) >= 0;
            inside_edge_p13 = dot(edge_perp13, point - pt1) >= 0;
            tri_corners.XData = [pt1(1); pt2(1); pt3(1)];
            tri_corners.YData = [pt1(2); pt2(2); pt3(2)];
            tri_corners.ZData = [pt1(3); pt2(3); pt3(3)];
            
            if inside_edge_p21 && inside_edge_p32 && inside_edge_p13
                nearest_face_idx = match_rows(i);
                defect = dot(point - nearest_vert, normvec);
                projected_pt = point - defect*normvec; % Subtract out the out of plane component.
                return;
            end    
        end
        
        % May want to handle cases where the 
        
%         verts_connected_to_nearest = unique(faces(match_rows,:));
%         verts_connected_to_nearest = verts_connected_to_nearest(verts_connected_to_nearest ~= nearest_vert_idx);
%         
        
            projected_pt = NaN;
            nearest_face_idx = NaN; % Above a vertex or edge. TODO handle these cases if needed.
        % Case in which the vertex is the closest thing.
    end
end