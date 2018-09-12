function exceptions = test_point2trimesh_with_normals()
%TEST_POINT2TRIMESH_WITH_NORMALS Unit tests for point2trimesh_with_normals
%
%   exceptions = TEST_POINT2TRIMESH_WITH_NORMALS()
%
%   Inputs: <none>
%   Outputs:
%       `exceptions` -- Cell array of exceptions generated while running
%       sub-tests.
%
%   See also POINT2TRIMESH_WITH_NORMALS, POINT2TRIMESH, TEST_ALL.
%

tolerance = 1e-10;

theader('Testing trimesh point projections.');
mesh_data = get_mesh_data('twentyhedron');

exceptions = {};
exceptions{end+1} = do_test(@to_vertex);
exceptions{end+1} = do_test(@to_face_center);
exceptions{end+1} = do_test(@to_edge_center);

    function to_vertex()
        tname('Project to vertex');
        % Pick a point directly off a vertex.
        idx = 10;
        vert = mesh_data.vertices(idx,:);
        vert_norm = mesh_data.vertex_normals(idx,:);
        point = vert + vert_norm * 0.5;
        
        [ distance, surface_point, normal_vec, face_idx ] = point2trimesh_with_normals(point, mesh_data);
        
        assert_near(vert, surface_point, tolerance, 'Did not identify the contact point as being at the vertex.');
        assert_near(vert_norm, normal_vec, tolerance, 'Did not identify correct normal.');
        assert_near(distance, norm(vert_norm * 0.5), tolerance, 'Distance does not match.');
    end

    function to_face_center()
        tname('Project to face center');        
        idx = 10;
        face_center = sum(mesh_data.vertices(mesh_data.faces(idx,:),:),1)/3;
        face_center_normal_vert = sum(mesh_data.vertex_normals(mesh_data.faces(idx,:),:),1);
        face_center_normal_vert = face_center_normal_vert/norm(face_center_normal_vert); % According to the vertex normals.
        face_center_normal_face = mesh_data.face_normals(idx,:); % According to the face normals.
        point = face_center + face_center_normal_face * 0.1;
        
        [ distance, surface_point, normal_vec, face_idx ] = point2trimesh_with_normals(point, mesh_data);
        
        assert_near(face_center, surface_point, tolerance, 'Did not identify the contact point on face.');
        assert_near(face_center_normal_vert, normal_vec, tolerance, 'Did not identify correct normal.');
        assert_near(distance, norm(face_center_normal_face * 0.1), tolerance, 'Distance does not match.');     
        assert(face_idx == idx, 'Face indexes do not match.');
    end

    function to_edge_center()
        tname('Project to edge center');
        face_idx = 10;
        
        v1_idx = mesh_data.faces(face_idx, 1);
        v2_idx = mesh_data.faces(face_idx, 2);
        v1 = mesh_data.vertices(v1_idx,:);
        v2 = mesh_data.vertices(v2_idx,:);
        
        edge_center = (v1 + v2)/2;
        normal = (v1 + v2)/norm(v1 + v2);
        point = edge_center + normal * 0.3;
        
        [ distance, surface_point, normal_vec, face_idx ] = point2trimesh_with_normals(point, mesh_data);
        
        assert_near(edge_center, surface_point, tolerance, 'Did not identify the contact point on face.');
        assert_near(normal, normal_vec, tolerance, 'Did not identify correct normal.');
        assert_near(distance, 0.3, tolerance, 'Distance does not match.');        
    end
end

