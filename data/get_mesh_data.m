function mesh_struct = get_mesh_data(mesh_name)
%GET_MESH_DATA Single place from which all mesh data can be loaded.
%   All meshes should have fields faces, vertices, face_normals,
%   vertex_normals.
%   Valid meshes:
% 'dummy_manipulator_high_res' - edited foot link with convex end.
% 'dummy_manipulator_mid_res' - 50% reduction
% 'dummy_manipulator_low_res' - 90% reduction
% 'horiz_plane' - horizontal plane centered at 0,0,0.
% 'twentyhedron' - twenty-sided platonic polyhedron.
% 'sixhedron' - six-sided platonic polyhedron.
% 'cube' - centered around the origin.
% 'manipulator_banned1' - banned region near the wrist of the robot end
% link.


switch mesh_name
    case 'dummy_manipulator_high_res'
        geo_data = load('../data/iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.merged_iiwa(1);
        return;
    case 'dummy_manipulator_mid_res'
        geo_data = load('../data/iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.merged_iiwa(2);
        return;
    case 'dummy_manipulator_low_res'
        geo_data = load('../data/iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.merged_iiwa(3);
        return;
    case 'horiz_plane'
        geo_data = load('../data/iiwa_merged_end_effector.mat');
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
        geo_data = load('../data/iiwa_merged_end_effector.mat');
        mesh_struct = geo_data.banned_regions;
        return;     
end
end

