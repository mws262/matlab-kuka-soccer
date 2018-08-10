function mesh_face_picker(varargin)
%% Lets the user select patch faces by clicking. Face indices are saved after each click.
addpath ./vis;
selected_faces = [];
save_file_name = './data/picked_faces.mat';

if numel(varargin) > 0
    link_patch = varargin{1};
    temp_fig_active = false;
else
    temp_fig_active = true;
    temp_fig = figure; % For disposably importing the whole robot, then throwing most of it away.
    iiwa = IIWAImporter(temp_fig);
    link_patch = iiwa.link_patches{2}; % 2 is last full link. 1 is the rotating plate at the end.
end

picker_fig = figure;
picker_fig.Position = [0, 0, 1000, 900];
cdata = repmat([0.8, 0.5, 0.5], [size(link_patch.Faces,1), 1]);
ax = axes;
link_patch.Parent = ax;
link_patch.FaceAlpha = 0.8;
link_patch.LineStyle = '-';
link_patch.EdgeAlpha = 1;
link_patch.FaceColor = 'flat';
link_patch.FaceVertexCData = cdata;
link_patch.EdgeColor = [0.1 0.1 0.1];
view(3);
daspect([1,1,1]);

if temp_fig_active
    close(temp_fig);
end

% Listens for clicks, calls the specified callback.
link_patch.ButtonDownFcn = @(obj,hit)patchfacefcn(obj,hit, @click_callback,'face');

    function click_callback(obj,hit,face_index)
        disp(face_index);
        link_patch.FaceVertexCData(face_index, :) = [0 1 0];
        selected_faces(end + 1) = face_index;
        save(save_file_name, 'selected_faces');
    end
end