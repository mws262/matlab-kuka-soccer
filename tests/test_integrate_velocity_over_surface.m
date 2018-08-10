clear all;
addpath ../vis;
addpath ../geometry;
addpath ../;
geo_data = load('../data/iiwa_merged_end_effector.mat');
for detail_level = 1:3 % Try all mesh detail levels
    faces = geo_data.merged_iiwa(detail_level).faces;
    vertices = geo_data.merged_iiwa(detail_level).vertices;
    face_normals = geo_data.merged_iiwa(detail_level).face_normals;
    vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
    
    
    fig = figure(detail_level);
    iiwa_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
    iiwa_patch.EdgeColor = [0.1, 0.1, 0.1];
    iiwa_patch.FaceColor = [1 0 0];
    iiwa_patch.FaceAlpha = 0.85;
    
    camlight HEADLIGHT;
    view(3);
    daspect([1,1,1]);
    hold on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
    hold on;
    
    initial_face = ceil(size(faces,1)/3);
    initial_face_verts = vertices(faces(initial_face,:), :);
    initial_position = [0, 0, 0.3]; %mean(initial_face_verts,1); % Start at a consistent point on a mesh.
    
    tspan = linspace(0, 1, 1000);
    vspan = repmat(0.2*[0.2, -0.8, 0], [length(tspan), 1]);
    tic
    profile on;
    [result_path_pts, result_path_normals] = integrate_velocity_over_surface(tspan, vspan, initial_position, [0 0 1], geo_data.merged_iiwa(detail_level));
    profile off;
    toc
    plot3(result_path_pts(:,1), result_path_pts(:,2), result_path_pts(:,3), '.b', 'MarkerSize', 20);
    quiver3(result_path_pts(:,1), result_path_pts(:,2), result_path_pts(:,3), result_path_normals(:,1), result_path_normals(:,2), result_path_normals(:,3))
    drawnow;
    hold off;
end