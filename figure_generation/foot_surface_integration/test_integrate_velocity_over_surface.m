% Integrate a velocity over the surface of the arm link. Do it in all
% model resolutions to make sure things look roughly the same.
clear all; close all;
addpath ../vis;
addpath ../geometry;
addpath ../;


% Point to project to the surface of the mesh.
projection_point = [0,-0.1,0];
% Velocity:
veldata = load('surface_vel_dat.mat');
tspan = veldata.ts;
vspan = [veldata.velx(:,1), veldata.vely(:,1), zeros(size(veldata.velx(:,1)))];
% tspan = linspace(0, 3, 100)';
% vspan = 0.2*[0.2*sin(tspan/100), -0.8 + cos(tspan) * 0, 0*sin(tspan)];

geo_data = load('../../data/iiwa_merged_end_effector.mat');
for detail_level = 1:3 % Try all mesh detail levels
    faces = geo_data.merged_iiwa(detail_level).faces;
    vertices = geo_data.merged_iiwa(detail_level).vertices;
    face_normals = geo_data.merged_iiwa(detail_level).face_normals;
    vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
    
    %% New figure for each.
    fig = figure(detail_level);
    fig.Color = [1,1,1];
    fig.Position = [0,0,1200, 900];
    ax = axes;
    iiwa_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
    iiwa_patch.EdgeColor = [0.1, 0.1, 0.1];
    iiwa_patch.FaceAlpha = 0.5;
    iiwa_patch.FaceColor = 'flat';
    iiwa_patch.FaceVertexCData = repmat([0.5 0.5 1], [size(iiwa_patch.Faces,1),1]);
    ax.Visible = 'off';
    view(3);
        camlight HEADLIGHT;
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
%     mark_on_mesh(initial_face, iiwa_patch);
    
    tic
    [result_path_pts, result_path_normals] = integrate_velocity_over_surface(tspan, vspan, initial_position, [0 0 1], -pi+ 1.5, geo_data.merged_iiwa(detail_level));
    toc
    col = lines(10);
    plot3(result_path_pts(:,1), result_path_pts(:,2), result_path_pts(:,3), 'Color', col(2,:), 'LineWidth', 4);
    plot3(result_path_pts(1,1), result_path_pts(1,2), result_path_pts(1,3), '.g', 'MarkerSize', 60);
    plot3(result_path_pts(end,1), result_path_pts(end,2), result_path_pts(end,3), '.r', 'MarkerSize', 60);
    quiv_int = 5;
    quiver3(result_path_pts(1:quiv_int:end,1), result_path_pts(1:quiv_int:end,2), result_path_pts(1:quiv_int:end,3), result_path_normals(1:quiv_int:end,1), result_path_normals(1:quiv_int:end,2), result_path_normals(1:quiv_int:end,3))
   campos([   -0.4916   -1.3838    1.2303]);
    drawnow;
    hold off;
    save2pdf('../../data/images/path_on_foot.pdf', fig, 600);
    break;
end