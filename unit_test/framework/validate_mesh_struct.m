function validate_mesh_struct(mesh_struct)
% VALIDATE_MESH_STRUCT Make sure that a structure contains the the general
% fields and data types needed throughout matlab-kuka-soccer. Throws
% exceptions otherwise.
%
%   VALIDATE_MESH_STRUCT(mesh_struct)
%
%   Inputs:
%       `mesh_struct` -- Triangle mesh structure to test. Should contain
%       fields: vertices, faces, face_normals, vertex_normals. May contain
%       extra fields. This is not checked.
%
%   Outputs: <none>
%
%   See also VALIDATEATTRIBUTES, ASSERT, ASSERT_NEAR, DO_TEST,
%   ASSERT_ERROR, THEADER, TNAME, REPORT_EXCEPTIONS.
%

    %% Verify correct fields.
    validateattributes(mesh_struct, {'struct'}, {});
    assert(all(isfield(mesh_struct, {'vertices', 'faces', 'face_normals', 'vertex_normals'})), 'Banned region struct does not contain the correct mesh description fields.');
    
    %% Verify sizes and data types.
    validateattributes(mesh_struct.vertices, {'single', 'double'}, {'real', 'ncols', 3});
    validateattributes(mesh_struct.faces, {'numeric'}, ...
        {'integer', 'positive', 'ncols', 3, '>=', 1, '<=', size(mesh_struct.vertices,1)});
    validateattributes(mesh_struct.face_normals, {'single', 'double'}, ...
        {'real', 'ncols', 3, 'nrows', size(mesh_struct.faces,1)});
    validateattributes(mesh_struct.vertex_normals, {'single', 'double'}, ...
        {'real', 'ncols', 3, 'nrows', size(mesh_struct.vertices,1)});
end

