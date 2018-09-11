function quiver_object = draw_all_normals(mesh_data, scaling)
% DRAW_ALL_NORMALS Draw the face normal vectors of a mesh. Does not draw
% the mesh itself. Draws these normals as arrows emanating from the
% centroid of each face.
%
%   quiver_object = DRAW_ALL_NORMALS(mesh_data, scaling)
%
%   Inputs:
%       `mesh_data` -- Mesh data structure including faces, vertices, face
%       normals, vertex normals.
%       `scaling` -- Scaling factor for the arrows representing the
%       normals. Should be a real number greater than 0.
%   Outputs:
%       `quiver_object` -- Graphics object representing these arrows. Can
%       be used to further edit the appearance or data.
%
%   See also QUIVER3, VALIDATE_MESH_STRUCT, GET_ALL_NORMALS.
%

validate_mesh_struct(mesh_data);
validateattributes(scaling, {'single', 'double'}, {'positive', 'real', 'scalar'});

centers = (mesh_data.vertices(mesh_data.faces(:,1),:) + mesh_data.vertices(mesh_data.faces(:,2),:) + mesh_data.vertices(mesh_data.faces(:,3),:))/3;
quiver_object = quiver3(centers(:,1)', centers(:,2)', centers(:,3)', ...
    mesh_data.face_normals(:,1)' * scaling, mesh_data.face_normals(:,2)' * scaling, mesh_data.face_normals(:,3)' * scaling);
quiver_object.AutoScale = 'off';
end

