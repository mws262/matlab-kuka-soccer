% Make sure that check_pt_inside_mesh works.
% Loads the dummy manipulator mesh and the banned region mesh. Samples
% points along the surface. The ones not inside the banned region should be
% green. The ones outside should be red.

clear all; close all;
addpath ../geometry/;
addpath ../data/;

arm_dat = get_mesh_data('dummy_manipulator_high_res');
banned_region_dat = get_mesh_data('manipulator_banned1');


arm_patch = patch('Faces', arm_dat.faces, 'Vertices', arm_dat.vertices);
arm_patch.FaceColor = [1,0,1];
arm_patch.FaceAlpha = 0.9;

hold on;
banned_region = patch('Faces', banned_region_dat.faces, 'Vertices', banned_region_dat.vertices);
banned_region.FaceAlpha = 0.4;

camlight HEADLIGHT;
view(3);
daspect([1,1,1]);
campos([1.0329    1.1155    1.3412]);

% Sample points on the arm.
points_to_project = arm_dat.vertices(1:100:end, :);

for i = 1:size(points_to_project, 1)
    % Project to surface.
    [ distance, surface_point, normal_vec, face_idx ] = point2trimesh_with_normals( points_to_project(i,:), arm_dat.faces, arm_dat.vertices, arm_dat.face_normals, arm_dat.vertex_normals );
    
    % Check if it's in the banned region and plot accordingly.
    if check_pt_inside_mesh(surface_point, banned_region_dat)
        plot3(surface_point(1), surface_point(2), surface_point(3), '.r', 'MarkerSize', 30);
    else
        plot3(surface_point(1), surface_point(2), surface_point(3), '.g', 'MarkerSize', 30);
    end

end

hold off;