function exceptions = test_find_mesh_contact_tform()
% TEST_FIND_MESH_CONTACT_TFORM Unit tests for find_mesh_contact_tform.
%
%   exceptions = TEST_FIND_MESH_CONTACT_TFORM()
%
%   Inputs: <none>
%   Outputs:
%       `exceptions` -- Cell array of exceptions generated while running
%       sub-tests.
%
%   See also FIND_MESH_CONTACT_TFORM, TEST_ALL.
%

theader('Testing find_mesh_contact_tform.');

tolerance = 1e-12;

exceptions = {};
exceptions{end+1} = do_test(@cube_test);
exceptions{end+1} = do_test(@foot_mesh_test);

    function cube_test()
        tname('Cube transformation.');
        cube_data = get_mesh_data('cube');
        
        pt_near = [0,0.01,1];
        target_normal = [0,1/sqrt(2),1/sqrt(2)];
        target_location = [0.1, -1, 0.5];
        twist_ang = 0;
        
        
        % No destination location.
        [total_tform, current_pt, current_normal] = find_mesh_contact_tform(cube_data, pt_near, target_normal, twist_ang);
        
        % Validate the rotation part of the tform.
        rotation = tform2rotm(total_tform);
        validateattributes(rotation, {'single', 'double'}, {'real', 'size', [3, 3]});
        assert_near(det(rotation), 1, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,1), rotation(:,2)), 0, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,2), rotation(:,3)), 0, tolerance, 'Invalid rotation returned.');
        
        % Make sure it returned the closest point on mesh.
        assert(min(sum((cube_data.vertices - pt_near).*(cube_data.vertices - pt_near),2)) >= sum((pt_near - current_pt).*(pt_near - current_pt),2), 'Nearest point returned was not as close as one or more of the vertices.');
        
        % Make sure that the rotation matches the normals correctly.
        assert_near(target_normal', -rotation*current_normal', tolerance, 'Returned rotation does not transform the original normal vector onto the -target.');
        
        % Make sure that the transformation maps the original point to the
        % target.
        tformed_center = total_tform*[current_pt, 1]';
        tformed_center = tformed_center(1:3); % Column.
        assert_near(tformed_center', current_pt, tolerance, 'Transformed point does not match target destination.');
        
        % With a destination location.
        [total_tform, current_pt, current_normal] = find_mesh_contact_tform(cube_data, pt_near, target_normal, twist_ang, target_location);
        
        % Validate the rotation part of the tform.
        rotation = tform2rotm(total_tform);
        validateattributes(rotation, {'single', 'double'}, {'real', 'size', [3, 3]});
        assert_near(det(rotation), 1, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,1), rotation(:,2)), 0, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,2), rotation(:,3)), 0, tolerance, 'Invalid rotation returned.');
        
        % Make sure it returned the closest point on mesh.
        assert(min(sum((cube_data.vertices - pt_near).*(cube_data.vertices - pt_near),2)) >= sum((pt_near - current_pt).*(pt_near - current_pt),2), 'Nearest point returned was not as close as one or more of the vertices.');
        
        % Make sure that the rotation matches the normals correctly.
        assert_near(target_normal', -rotation*current_normal', tolerance, 'Returned rotation does not transform the original normal vector onto the -target.');
        
        % Make sure that the transformation maps the original point to the
        % target.
        tformed_center = total_tform*[current_pt, 1]';
        tformed_center = tformed_center(1:3); % Column.
        assert_near(tformed_center, target_location', tolerance, 'Transformed point does not match target destination.');

        % See how a nearby point is affected.
        test_pt = [-0, -0.05, 1];
        [ ~, near_pt, near_normal ] = point2trimesh_with_normals( test_pt, cube_data );
        dist = norm(current_pt - near_pt);
        
        % Test for a sweep of twist angles. Original point shouldn't be different
        % for different angles and other points should go in a circle.
        angs = linspace(0, 2*pi, 20);
        tformed_pts_rel_center = zeros(length(angs),3);
        for i = 1:length(angs)
            [tform, alt_surface_pt, ~] = find_mesh_contact_tform(cube_data, pt_near, target_normal, angs(i), target_location);
            % Nearby point after transformation.
            tform_near = tform*[near_pt,1]';
            tform_near = tform_near(1:3);
            
            % Surface point after transformation.
            tform_alt = tform*[alt_surface_pt,1]';
            tform_alt = tform_alt(1:3);
            
            assert_near(tformed_center, tform_alt, tolerance, 'Rotation about normal should not affect the transformed point.');
            
            tformed_pts_rel_center(i,:) = tform_near' - tformed_center';
            % We use smooth normals, so coplanar points might not be coplanar when
            % aligned with the normal direction. We project these down to the plane
            % of the normal. Otherwise the angles won't match in the check below.
            tformed_pts_rel_center(i,:) = tformed_pts_rel_center(i,:) - dot(tformed_pts_rel_center(i,:),target_normal)*target_normal;
            assert_near(norm(tformed_center - tform_near), dist, tolerance, 'Transformed nearby point should be the same distance away after transformation.');
        end
        
        angs_between_tformed = acos(dot(tformed_pts_rel_center(2:end,:)./sqrt(sum(tformed_pts_rel_center(2:end,:).*tformed_pts_rel_center(2:end,:),2)), tformed_pts_rel_center(1:end - 1,:)./sqrt(sum(tformed_pts_rel_center(1:end - 1,:).*tformed_pts_rel_center(1:end - 1,:),2)),2));
        assert_near(angs_between_tformed, diff(angs)', tolerance, 'Rotation about normal should be the same before and after transformation.');
        
    end

    function foot_mesh_test()
        tname('Robot link mesh transformation.');

        foot_data = get_mesh_data('dummy_manipulator_mid_res');
        
        pt_near = [0,0.01,1];
        target_normal = [0.5,0.2,-0.1];
        target_normal = target_normal/norm(target_normal);
        target_location = [0.1, 0.15, -0.05];
        twist_ang = 0;
        
        % No target location.
        [total_tform, current_pt, current_normal] = find_mesh_contact_tform(foot_data, pt_near, target_normal, twist_ang);
        
        % Validate the rotation part of the tform.
        rotation = tform2rotm(total_tform);
        validateattributes(rotation, {'single', 'double'}, {'real', 'size', [3, 3]});
        assert_near(det(rotation), 1, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,1), rotation(:,2)), 0, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,2), rotation(:,3)), 0, tolerance, 'Invalid rotation returned.');
        
        % Make sure it returned the closest point on mesh.
        assert(min(sum((foot_data.vertices - pt_near).*(foot_data.vertices - pt_near),2)) >= sum((pt_near - current_pt).*(pt_near - current_pt),2), 'Nearest point returned was not as close as one or more of the vertices.');
        
        % Make sure that the rotation matches the normals correctly.
        assert_near(target_normal', -rotation*current_normal', tolerance, 'Returned rotation does not transform the original normal vector onto the -target.');
        
        % Make sure that the transformation maps the original point to the
        % target.
        tformed_center = total_tform*[current_pt, 1]';
        tformed_center = tformed_center(1:3); % Column.
        assert_near(tformed_center', current_pt, tolerance, 'Transformed point does not match target destination.');
        
        % With target location.
        [total_tform, current_pt, current_normal] = find_mesh_contact_tform(foot_data, pt_near, target_normal, twist_ang, target_location);
        
        % Validate the rotation part of the tform.
        rotation = tform2rotm(total_tform);
        validateattributes(rotation, {'single', 'double'}, {'real', 'size', [3, 3]});
        assert_near(det(rotation), 1, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,1), rotation(:,2)), 0, tolerance, 'Invalid rotation returned.');
        assert_near(dot(rotation(:,2), rotation(:,3)), 0, tolerance, 'Invalid rotation returned.');
        
        % Make sure it returned the closest point on mesh.
        assert(min(sum((foot_data.vertices - pt_near).*(foot_data.vertices - pt_near),2)) >= sum((pt_near - current_pt).*(pt_near - current_pt),2), 'Nearest point returned was not as close as one or more of the vertices.');
        
        % Make sure that the rotation matches the normals correctly.
        assert_near(target_normal', -rotation*current_normal', tolerance, 'Returned rotation does not transform the original normal vector onto the -target.');
        
        % Make sure that the transformation maps the original point to the
        % target.
        tformed_center = total_tform*[current_pt, 1]';
        tformed_center = tformed_center(1:3); % Column.
        assert_near(tformed_center, target_location', tolerance, 'Transformed point does not match target destination.');
        
        
        % See how a nearby point is affected.
        test_pt = [-0, -0.05, 1];
        [ ~, near_pt, near_normal ] = point2trimesh_with_normals( test_pt, foot_data );
        dist = norm(current_pt - near_pt);
        
        % Test for a sweep of twist angles. Original point shouldn't be different
        % for different angles and other points should go in a circle.
        angs = linspace(0, 2*pi, 20);
        tformed_pts_rel_center = zeros(length(angs),3);
        for i = 1:length(angs)
            [tform, alt_surface_pt, ~] = find_mesh_contact_tform(foot_data, pt_near, target_normal, angs(i), target_location);
            % Nearby point after transformation.
            tform_near = tform*[near_pt,1]';
            tform_near = tform_near(1:3);
            
            % Surface point after transformation.
            tform_alt = tform*[alt_surface_pt,1]';
            tform_alt = tform_alt(1:3);
            
            assert_near(tformed_center, tform_alt, tolerance, 'Rotation about normal should not affect the transformed point.');
            
            tformed_pts_rel_center(i,:) = tform_near' - tformed_center';
            % We use smooth normals, so coplanar points might not be coplanar when
            % aligned with the normal direction. We project these down to the plane
            % of the normal. Otherwise the angles won't match in the check below.
            tformed_pts_rel_center(i,:) = tformed_pts_rel_center(i,:) - dot(tformed_pts_rel_center(i,:),target_normal)*target_normal;
            assert_near(norm(tformed_center - tform_near), dist, tolerance, 'Transformed nearby point should be the same distance away after transformation.');
        end
        
        angs_between_tformed = acos(dot(tformed_pts_rel_center(2:end,:)./sqrt(sum(tformed_pts_rel_center(2:end,:).*tformed_pts_rel_center(2:end,:),2)), tformed_pts_rel_center(1:end - 1,:)./sqrt(sum(tformed_pts_rel_center(1:end - 1,:).*tformed_pts_rel_center(1:end - 1,:),2)),2));
        assert_near(angs_between_tformed, diff(angs)', tolerance, 'Rotation about normal should be the same before and after transformation.');
        
    end
end

