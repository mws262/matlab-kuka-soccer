function mesh_struct = get_mesh_data(mesh_name)
% GET_MESH_DATA Single place from which all mesh data can be loaded.
%
%   mesh_struct = GET_MESH_DATA(mesh_name)
%
%   Inputs:
%       `mesh_name` -- Name of the mesh to be loaded. Valid mesh names are:
%           'dummy_manipulator_high_res' - edited foot link with convex end.
%           'dummy_manipulator_mid_res' - 50% reduction
%           'dummy_manipulator_low_res' - 90% reduction
%           'vert_plane' - vertical plane centered at 0,0,0. Normals face
%           in the +y direction.
%           'twentyhedron' - twenty-sided platonic polyhedron.
%           'sixhedron' - six-sided platonic polyhedron.
%           'cube' - centered around the origin.
%           'manipulator_banned1' - banned region near the wrist of the 
%           robot end link.
%           'geodesic_sphere' - many-faceted discretized sphere with 
%           equilateral triangle faces. Unit radius.
%       `mesh_struct` -- Loaded mesh data. Contains faces, vertices,
%       face_normals, vertex_normals.
%
%   See also LOAD_AND_SAVE_IIWA_MERGED, READOBJ, STLREAD,
%   SIMPLE_SHAPE_DATA, GET_MESH_DATA.
%

validateattributes(mesh_name, {'string', 'char'}, {});

switch mesh_name
    case 'dummy_manipulator_high_res'
        geo_data = load('iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.merged_iiwa(1);
        return;
    case 'dummy_manipulator_mid_res'
        geo_data = load('iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.merged_iiwa(2);
        return;
    case 'dummy_manipulator_low_res'
        geo_data = load('iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.merged_iiwa(3);
        return;
    case 'vert_plane'
        geo_data = load('iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.merged_iiwa(4);
        return;
    case 'twentyhedron'
        [twentygon, ~, ~] = simple_shape_data();
        mesh_struct = twentygon;
        return;
    case 'sixhedron'
        [~, sixgon, ~] = simple_shape_data();
        mesh_struct = sixgon;
        return;
    case 'cube'
        [~, ~, cube] = simple_shape_data();
        mesh_struct = cube;
        return;
    case 'manipulator_banned1'
        geo_data = load('iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.banned_regions;
        return;
    case 'geodesic_sphere'
        sph_struct = readObj('sphere.obj'); % Load a basic horizontal plane for debugging.
        sph_verts = sph_struct.v;
        sph_faces = sph_struct.f.v;
        [sph_vn, sph_fn] = get_all_normals(sph_faces, sph_verts);
        mesh_struct.faces = sph_faces;
        mesh_struct.vertices = sph_verts;
        mesh_struct.face_normals = sph_fn;
        mesh_struct.vertex_normals = sph_vn;
        return;
    otherwise
        error(['Unknown mesh name. Cannot load: ', mesh_name]);
end
end

