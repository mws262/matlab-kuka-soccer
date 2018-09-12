function mesh_face_picker(save_file_name, link_patch)
% MESH_FACE_PICKER Click a mesh to select faces and save these selections
% to file. File is saved after every click, so the window can be terminated
% at any time. Clicked faces are highlighted.
%
%   MESH_FACE_PICKER(save_file_name, link_patch)
%   MESH_FACE_PICKER(save_file_name)
%
%   Inputs:
%       `save_file_name` -- Name of the .MAT file to save selected points
%       to.
%       `link_patch` -- (OPTIONAL) Patch to load up for click-selection. If
%       not specified, then the high-res dummy link mesh is loaded.
%   Outputs: <none>
%   Output files:
%       `save_file_name`.mat -- File with struct named `selections`. Has
%       fields `points` and `faces`. `points` is n x 3 cartesian positions
%       of selected click points. `faces` is n x 1 indices of selected
%       faces.
%
%   See also PATCHFACEFCN, GET_MESH_DATA, LOAD_AND_SAVE_IIWA_MERGED, PATCH.
%

selected_faces = [];
selected_points = [];

picker_fig = figure;
picker_fig.Position = [0, 0, 1000, 900];
ax = axes;

if nargin == 1
    geo_data = get_mesh_data('dummy_manipulator_high_res');
    link_patch = patch('Faces', geo_data.faces, 'Vertices', geo_data.vertices, 'FaceNormals', geo_data.face_normals, 'VertexNormals', geo_data.vertex_normals);
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