function mesh_face_picker(varargin)
%% Lets the user select patch faces by clicking. Face indices are saved after each click.
addpath ../vis;
selected_faces = [];
selected_points = [];
save_file_name = './picked_faces_tmp.mat';
geo_data = load('./iiwa_merged_end_effector.mat');

picker_fig = figure;
picker_fig.Position = [0, 0, 1000, 900];
ax = axes;

if numel(varargin) > 0
    link_patch = varargin{1}; % TODO: haven't tested this since some changes.
else
    detail_level = 1;
    faces = geo_data.merged_iiwa(detail_level).faces;
    vertices = geo_data.merged_iiwa(detail_level).vertices;
    face_normals = geo_data.merged_iiwa(detail_level).face_normals;
    vertex_normals = geo_data.merged_iiwa(detail_level).vertex_normals;
    link_patch = patch('Faces', faces, 'Vertices', vertices, 'FaceNormals', face_normals, 'VertexNormals', vertex_normals);
end

cdata = repmat([0.8, 0.5, 0.5], [size(link_patch.Faces,1), 1]);
link_patch.Parent = ax;
link_patch.FaceAlpha = 0.8;
link_patch.LineStyle = '-';
link_patch.EdgeAlpha = 1;
link_patch.FaceColor = 'flat';
link_patch.FaceVertexCData = cdata;
link_patch.EdgeColor = [0.1 0.1 0.1];
xlabel('x');
ylabel('y');
zlabel('z');
view(3);
daspect([1,1,1]);

% Listens for clicks, calls the specified callback.
link_patch.ButtonDownFcn = @(obj,hit)patchfacefcn(obj,hit, @click_callback,'face');

    function click_callback(obj,hit,face_index)
        disp(face_index);
        link_patch.FaceVertexCData(face_index, :) = [0 1 0];
        selected_points(end + 1,:) = hit.IntersectionPoint;
        selected_faces(end + 1) = face_index;
        selections.points = selected_points;
        selections.faces = selected_faces;
        save(save_file_name, 'selections');
    end
end