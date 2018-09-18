function exceptions = test_integrate_velocity_over_surface()
%TEST_INTEGRATE_VELOCITY_OVER_SURFACE Unit tests for
%integrate_velocity_over_surface.
%
%   exceptions = TEST_INTEGRATE_VELOCITY_OVER_SURFACE()
%
%   Inputs: <none>
%   Outputs:
%       `exceptions` -- Cell array of exceptions generated while running
%       sub-tests.
%
%   See also INTEGRATE_VELOCITY_OVER_SURFACE, TEST_ALL.
%

tolerance = 1e-8;

theader('Testing integrate_velocity_over_surface.');

exceptions = {};
exceptions{end+1} = do_test(@line_on_sphere);

    function line_on_plane()
        tname('Linear velocity on planes');
        % No transform.
        line_on_plane_at_rot(eye(3));
        
        % Rotate the plane and try again.
        plane_rot = [    0.0000    1.0000         0
            -1.0000    0.0000         0
            0         0    1.0000]; % 90 degrees around z axis.
        line_on_plane_at_rot(plane_rot);
        
        % Rotate the plane and try again.
        plane_rot = [    0.0000         0   -1.0000
            0    1.0000         0
            1.0000         0    0.0000]; % 90 deg about y-axis      
        line_on_plane_at_rot(plane_rot);
        
        % Rotate the plane and try again.
        plane_rot = [    1.0000         0         0
            0    0.0000    1.0000
            0   -1.0000    0.0000]; % 90 deg about z-axis.       
        line_on_plane_at_rot(plane_rot);
        
        % Arbitrary crazy rotation.
        plane_rot = angle2dcm(-0.54, 1.57, 2.04);
        line_on_plane_at_rot(plane_rot);
        
    end

    function line_on_plane_at_rot(rotation)
        plane_dat = get_mesh_data('vert_plane'); % Normals point in -y direction.
        prob = integrate_velocity_over_surface('problem');
        opts = integrate_velocity_over_surface('options');
        
        rot_plane_dat.faces = plane_dat.faces;
        rot_plane_dat.vertices = (rotation*plane_dat.vertices')';
        rot_plane_dat.face_normals = (rotation*plane_dat.face_normals')';
        rot_plane_dat.vertex_normals = (rotation*plane_dat.vertex_normals')';
        
        % Diagonal across the plane.
        prob.time_vector = linspace(0, sqrt(2), 100)';
        prob.velocity_vector = ones(100,3) .* (rotation*[-1, 0, -1]'/norm([-1, 0, -1]))';
        prob.initial_surface_point = (rotation * [-1, 0, -1]')';
        prob.normals_to_match =  (rotation * [0 -1 0]')';
        prob.orientations_about_normal = 0;
        prob.mesh_data = rot_plane_dat;
        
        output = integrate_velocity_over_surface(prob, opts);
        target_loc =  rotation * [0.5, 0, 0.5]';
        assert_near(output.mesh_surface_path(end,:), target_loc', tolerance, 'Integration over surface did not trace to the expected location.');
        assert_near(sqrt(dot(diff(output.mesh_surface_path),diff(output.mesh_surface_path),2)),diff(prob.time_vector), tolerance, 'Distance traveled should match linear velocity * time.');
          
        % Bottom to top, except rotate up by 45 degrees
        prob.time_vector = linspace(0, 1, 100)';
        prob.velocity_vector = ones(100,3) .* (rotation*[-1, 0, -1]'/norm([-1, 0, -1]))';
        prob.initial_surface_point = (rotation * [-1, 0, -1]')';
        prob.normals_to_match =  (rotation * [0 -1 0]')';
        prob.orientations_about_normal = -pi/4;
        prob.mesh_data = rot_plane_dat;
        
        output = integrate_velocity_over_surface(prob, opts);
        target_loc =  rotation * [-0.5, 0, 0.5]';
        assert_near(output.mesh_surface_path(end,:), target_loc', tolerance, 'Integration over surface did not trace to the expected location.');
        assert_near(sqrt(dot(diff(output.mesh_surface_path),diff(output.mesh_surface_path),2)),diff(prob.time_vector), tolerance, 'Distance traveled should match linear velocity * time.');
        
        % Stay along the bottom. Rotate down by 45 degrees.
        prob.time_vector = linspace(0, 1, 100)';
        prob.velocity_vector = ones(100,3) .* (rotation*[-1, 0, -1]'/norm([-1, 0, -1]))';
        prob.initial_surface_point = (rotation * [-1, 0, -1]')';
        prob.normals_to_match =  (rotation * [0 -1 0]')';
        prob.orientations_about_normal = pi/4;
        prob.mesh_data = rot_plane_dat;
        
        output = integrate_velocity_over_surface(prob, opts);
        target_loc =  rotation * [0.5, 0, -0.5]';
        assert_near(output.mesh_surface_path(end,:), target_loc', tolerance, 'Integration over surface did not trace to the expected location.');
        assert_near(sqrt(dot(diff(output.mesh_surface_path),diff(output.mesh_surface_path),2)),diff(prob.time_vector), tolerance, 'Distance traveled should match linear velocity * time.');      
    end


    function line_on_sphere()
        tname('Linear velocity on spheres');
        
        sph = get_mesh_data('geodesic_sphere');
        prob = integrate_velocity_over_surface('problem');
        opts = integrate_velocity_over_surface('options');
                prob.time_vector = linspace(0, 12*pi, 100)';
        prob.velocity_vector = ones(100,3) .* [-1, 0, -1]/norm([-1, 0, -1]);
        prob.initial_surface_point = [-1, 0, -1];
        prob.normals_to_match =  [0 0 1];
        prob.orientations_about_normal = 0;
        prob.mesh_data = sph;
        
        output = integrate_velocity_over_surface(prob, opts);
        close all;
        figure;
        patch('Faces', sph.faces, 'Vertices', sph.vertices, 'FaceColor', [1,0.5, 0.5], 'EdgeAlpha', 0.5);
        hold on;
        draw_all_normals(sph, 0.1);
        path = output.mesh_surface_path;
        plot3(path(:,1), path(:,2), path(:,3), 'LineWidth', 3);
        
    end
end
