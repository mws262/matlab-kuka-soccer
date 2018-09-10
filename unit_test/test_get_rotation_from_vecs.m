function test_get_rotation_from_vecs()
%TEST_GET_ROTATION_FROM_VECS 
tolerance = 1e-12;

theader('Testing get_rotation_from_vecs.');
exceptions = {};
exceptions{end+1} = do_test(@normal_cases);
exceptions{end+1} = do_test(@special_cases);

    function normal_cases()
        tname('Normal cases.');
        v1 = [0.0673    0.4375    0.3208];
        v2 = [-0.1615    0.0137   -0.3514];
        
        rot = get_rotation_from_vecs(v1,v2);
        % Check SO3-ness of rotation.
        assert_near(det(rot),1, tolerance, 'non-unit rotation matrix row');
        assert_near(cross(rot(:,1), rot(:,2)), rot(:,3), tolerance, 'non-orthogonal rotation matrix');
        assert_near(cross(rot(:,2), rot(:,3)), rot(:,1), tolerance, 'non-orthogonal rotation matrix');

        % Check correctness of transformation.
        assert_near((v2/norm(v2))', rot*(v1/norm(v1))', tolerance, 'Rotation not consistent.'); 
        
        v1 = [0.3694   -0.1138   -0.1287];
        v2 = [0.2315   -0.3901    0.5339];
        
        rot = get_rotation_from_vecs(v1,v2);

        % Check SO3-ness of rotation.
        assert_near(det(rot), 1, tolerance, 'non-unit rotation matrix row');
        assert_near(cross(rot(:,1), rot(:,2)), rot(:,3), tolerance, 'non-orthogonal rotation matrix');
        assert_near(cross(rot(:,2), rot(:,3)), rot(:,1), tolerance, 'non-orthogonal rotation matrix');

        % Check correctness of transformation.
        assert_near((v2/norm(v2))', rot*(v1/norm(v1))', tolerance, 'Rotation not consistent.'); 
        
    end

    function special_cases()
        tname('Special cases.');
        
        v1 = [0.3694   -0.1138   -0.1287];
        v2 = -v1;
        
        rot = get_rotation_from_vecs(v1,v2); % 180 deg
        assert_near((v2/norm(v2))', rot*(v1/norm(v1))', tolerance, 'Rotation not consistent.');
        assert_near(det(rot), 1, tolerance, 'non-unit rotation matrix row');
        assert_near(cross(rot(:,1), rot(:,2)), rot(:,3), tolerance, 'non-orthogonal rotation matrix');
        assert_near(cross(rot(:,2), rot(:,3)), rot(:,1), tolerance, 'non-orthogonal rotation matrix');
        
        
        rot = get_rotation_from_vecs(v1,v1);
        assert_near((v1/norm(v1))', rot*(v1/norm(v1))', tolerance, 'Rotation not consistent.');
        assert_near(det(rot), 1, tolerance, 'non-unit rotation matrix row');
        assert_near(cross(rot(:,1), rot(:,2)), rot(:,3), tolerance, 'non-orthogonal rotation matrix');
        assert_near(cross(rot(:,2), rot(:,3)), rot(:,1), tolerance, 'non-orthogonal rotation matrix');
    end

end

