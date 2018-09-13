function exceptions = test_integrate_velocity_over_surface()
%TEST_INTEGRATE_VELOCITY_OVER_SURFACE

exceptions = {};
exceptions{end+1} = do_test(@line_on_plane);


    function line_on_plane()
        plane_dat = get_mesh_data('vert_plane');
        prob = integrate_velocity_over_surface('problem');
        opts = integrate_velocity_over_surface('options');
        
        prob.time_vector = linspace(0,5,100);
        prob.velocity_vector = ones(100,3) .* [0.01, 0, 0.01]
        prob.initial_surface_point = [-1, 0, -1];
        prob.normals_to_match = [0 1 0];
        prob.orientations_about_normal = 0;
        prob.mesh_data = plane_dat;
        
        output = integrate_velocity_over_surface(prob, opts);
    end
end

