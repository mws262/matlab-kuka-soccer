% Integrate a velocity over the surface of the arm link. Do it in all
% model resolutions to make sure things look roughly the same.
clear all; close all;
addpath ../vis;
addpath ../geometry;
addpath ../;

% Point to project to the surface of the mesh.
projection_point = [1,1,1];
% Velocity:
tspan = linspace(0, 3, 100)';
vspan = 0.2*[0.2*sin(tspan/100), -0.8 + cos(tspan) * 0, 0*sin(tspan)];

geo_data = load('../data/iiwa_merged_end_effector.mat');
for detail_level = 1:3 % Try all mesh detail levels
    faces = geo_data.merged_iiwa(detail_level).faces;
    vertices = geo_data.merged_iiwa(detail_level).vertices;
    face_normals = geo_data.merged_iiwa(detail_level).face_normals;
    vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
    
    %% New figure for each.
    fig = figure(detail_level);
    iiwa_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
    iiwa_patch.EdgeColor = [0.1, 0.1, 0.1];
    iiwa_patch.FaceAlpha = 0.85;
    iiwa_patch.FaceColor = 'flat';
    iiwa_patch.FaceVertexCData = repmat([0 0 1], [size(iiwa_patch.Faces,1),1]);
    
    camlight HEADLIGHT;
    view(3);
    daspect([1,1,1]);
    hold on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold on;
    
    %% Project desired point to surface.
    [ distance, initial_position, normal_vec, initial_face ] =...
        point2trimesh_with_normals( projection_point, faces, vertices, ...
        face_normals, vertex_normals );
    % Mark this point.
    mark_on_mesh(initial_face, iiwa_patch);
    
    tic
    [result_path_pts, result_path_normals] = integrate_velocity_over_surface(tspan, vspan, initial_position, [0 0 1], -pi/4, geo_data.merged_iiwa(detail_level));
    toc
    
    plot3(result_path_pts(:,1), result_path_pts(:,2), result_path_pts(:,3), '.r', 'MarkerSize', 20);
    quiver3(result_path_pts(:,1), result_path_pts(:,2), result_path_pts(:,3), result_path_normals(:,1), result_path_normals(:,2), result_path_normals(:,3))
    drawnow;
    hold off;
end