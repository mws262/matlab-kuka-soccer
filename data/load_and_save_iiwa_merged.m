function load_and_save_iiwa_merged()
% LOAD_AND_SAVE_IIWA_MERGED Load .OBJ meshes from file, prepare them, and
% save to a .MAT file for easy access. Each saved mesh will include face
% and vertex normals.
% Also does some debug drawing to make sure the loaded meshes are as
% expected.
%
%   LOAD_AND_SAVE_IIWA_MERGED()
%
%   Inputs: <none>
%   Function outputs: <none>
%   File output:
%       'iiwa_merged_end_effector.mat' - contains `merged_iiwa` and
%       `banned_regions`. `merged_iiwa` is a struct array with the dummy
%       end effector link in 4 levels of difficulty, 1-high, 2-mid, 3-low,
%       4-a plane (sort of a joke). `banned_regions` is another mesh
%       structure containing regions we want to prohibit on the robot link.
%       
%   See also VALIDATE_MESH_STRUCT, READOBJ, STLREAD, SIMPLE_SHAPE_DATA,
%   PATCH.
%

scale_factor = 100; % cm -> m, artifact of using Maya earlier.

% Unreduced load of mesh.
import_struct = readObj('iiwa_merged_conv_maya.obj');% Stl has the issue of not storing vertex connectivity.
merged_vertices_full = import_struct.v;
merged_faces_full = import_struct.f.v;
merged_vertex_normals_full = import_struct.vn./sqrt(import_struct.vn(:,1).^2 + import_struct.vn(:,2).^2 + import_struct.vn(:,3).^2);
merged_vertices_full = merged_vertices_full/scale_factor; % cm -> meters

% Plot
fig = figure;
iiwa_patch = patch('Faces', merged_faces_full, 'Vertices', merged_vertices_full, 'VertexNormals', merged_vertex_normals_full);
iiwa_patch.EdgeColor = 'none';
iiwa_patch.FaceColor = 'r';
iiwa_patch.FaceNormalsMode = 'auto';

camlight HEADLIGHT;
view(3);
daspect([1,1,1]);
drawnow;
merged_face_normals_full = -iiwa_patch.FaceNormals; % Gets auto-calculated with the drawnow.

% Reduce patch. Mid-res
reducepatch(iiwa_patch,0.5);
drawnow;
merged_faces_mid = iiwa_patch.Faces;
merged_vertices_mid = iiwa_patch.Vertices;
merged_face_normals_mid = -iiwa_patch.FaceNormals;
merged_vertex_normals_mid = get_all_normals(merged_faces_mid, merged_vertices_mid);


% Reduce patch. Low-res
reducepatch(iiwa_patch,0.2); % Reduction is effectively multiplicative with previous reductions.
drawnow;
merged_faces_low = iiwa_patch.Faces;
merged_vertices_low = iiwa_patch.Vertices;
merged_face_normals_low = -iiwa_patch.FaceNormals;
merged_vertex_normals_low = get_all_normals(merged_faces_low, merged_vertices_low);

plane_struct = readObj('horiz_plane.obj'); % Load a basic horizontal plane for debugging.
plane_verts = plane_struct.v;
plane_faces = plane_struct.f.v;
[plane_vn, plane_fn] = get_all_normals(plane_faces, plane_verts);


merged_iiwa = struct('faces', {merged_faces_full, merged_faces_mid, merged_faces_low, plane_faces}, ...
    'vertices', {merged_vertices_full, merged_vertices_mid, merged_vertices_low, plane_verts}, ...
    'face_normals', {merged_face_normals_full, merged_face_normals_mid, merged_face_normals_low, plane_fn}, ...
    'vertex_normals', {merged_vertex_normals_full, merged_vertex_normals_mid, merged_vertex_normals_low, plane_vn});

%% Load banned regions.
banned_region1 = readObj('banned_conv.obj');
banned_faces1 = banned_region1.f.v;
banned_verts1 = banned_region1.v/scale_factor;
[banned_vn1, banned_fn1] = get_all_normals(banned_faces1, banned_verts1);
hold on;
banned1_patch = patch('Faces', banned_faces1, 'Vertices', banned_verts1);
banned1_patch.FaceAlpha = 0.3;
hold off;
banned_regions = struct('faces', banned_faces1, 'vertices', banned_verts1, 'face_normals', banned_fn1, 'vertex_normals', banned_vn1);

%% Save all!
save('iiwa_merged_end_effector.mat', 'merged_iiwa', 'banned_regions');

%% Some plotting.
hold on;
detail = 3;
verts = merged_iiwa(detail).vertices;
faces = merged_iiwa(detail).faces;
fn = merged_iiwa(detail).face_normals;
vn = merged_iiwa(detail).vertex_normals;

vert_n = quiver3(verts(:,1), verts(:,2), verts(:,3), vn(:,1), vn(:,2), vn(:,3));

face_centroids = (verts(faces(:,1),:) + verts(faces(:,2),:) + verts(faces(:,3),:)) / 3;
face_n = quiver3(face_centroids(:,1), face_centroids(:,2), face_centroids(:,3), fn(:,1), fn(:,2), fn(:,3));
end

