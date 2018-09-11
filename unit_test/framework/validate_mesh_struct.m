function validate_mesh_struct(mesh_struct)
    validateattributes(mesh_struct, {'struct'}, {});
    assert(all(isfield(mesh_struct, {'vertices', 'faces', 'face_normals'})), 'Banned region struct does not contain the correct mesh description fields.');
    validateattributes(mesh_struct.vertices, {'single', 'double'}, {'real', 'ncols', 3});
    validateattributes(mesh_struct.faces, {'numeric'}, ...
        {'integer', 'positive', 'ncols', 3, '>=', 1, '<=', size(mesh_struct.vertices,1)});
    validateattributes(mesh_struct.face_normals, {'single', 'double'}, ...
        {'real', 'ncols', 3, 'nrows', size(mesh_struct.faces,1)});
    validateattributes(mesh_struct.vertex_normals, {'single', 'double'}, ...
        {'real', 'ncols', 3, 'nrows', size(mesh_struct.vertices,1)});
end

