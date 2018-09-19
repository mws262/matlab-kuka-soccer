function exceptions = test_get_mesh_data()
% TEST_GET_MESH_DATA Unit tests for get_mesh_data.
%
%   exceptions = TEST_GET_MESH_DATA()
%
%   Inputs: <none>
%   Outputs:
%       `exceptions` -- Cell array of exceptions generated while running
%       sub-tests.
%
%   See also GET_MESH_DATA, TEST_ALL.
%
theader('Testing get_mesh_data.');

exceptions = {};
exceptions{end+1} = do_test(@dummy_manipulator_high_res);
exceptions{end+1} = do_test(@dummy_manipulator_mid_res);
exceptions{end+1} = do_test(@dummy_manipulator_low_res);
exceptions{end+1} = do_test(@vert_plane);
exceptions{end+1} = do_test(@twentyhedron);
exceptions{end+1} = do_test(@sixhedron);
exceptions{end+1} = do_test(@cube);
exceptions{end+1} = do_test(@manipulator_banned1);
exceptions{end+1} = do_test(@geodesic_sphere);

    function dummy_manipulator_high_res()
        tname('dummy_manipulator_high_res');
        mesh_data = get_mesh_data('dummy_manipulator_high_res');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
    function dummy_manipulator_mid_res()
        tname('dummy_manipulator_mid_res');
        mesh_data = get_mesh_data('dummy_manipulator_mid_res');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
    function dummy_manipulator_low_res()
        tname('dummy_manipulator_low_res');
        mesh_data = get_mesh_data('dummy_manipulator_low_res');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
    function vert_plane()
        tname('vert_plane');
        mesh_data = get_mesh_data('vert_plane');
        validate_mesh_struct(mesh_data);
    end
    function twentyhedron()
        tname('twentyhedron');
        mesh_data = get_mesh_data('twentyhedron');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
    function sixhedron()
        tname('sixhedron');
        mesh_data = get_mesh_data('sixhedron');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
    function cube()
        tname('cube');
        mesh_data = get_mesh_data('cube');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
    function manipulator_banned1()
        tname('manipulator_banned1');
        mesh_data = get_mesh_data('manipulator_banned1');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
    function geodesic_sphere()
        tname('geodesic_sphere');
        mesh_data = get_mesh_data('geodesic_sphere');
        validate_mesh_struct(mesh_data);
        assert(get_signed_mesh_volume(mesh_data) > 0);
    end
end

