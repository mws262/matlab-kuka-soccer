function test_check_pt_inside_mesh()
%TEST_CHECK_PT_INSIDE_MESH

theader('Testing test_check_pt_inside_mesh');

% Load an icosohedron.
hedron = get_mesh_data('twentyhedron');

% Move it away from the origin.
hedron.vertices = hedron.vertices + [0.3, -0.1, -5];

face_center = sum(hedron.vertices(hedron.faces(3,:),:),1)/3;
face_normal = hedron.face_normals(3,:);
inside_pt = face_center - 0.05 * face_normal;

assert(check_pt_inside_mesh([0.3, -0.1, -5], hedron), 'Center should be inside the shape.');
assert(check_pt_inside_mesh([0.3, -0.1, -5]', hedron), 'Center should be inside the shape.');

assert(check_pt_inside_mesh(inside_pt, hedron), 'Inside point did not register as being inside.');

assert(check_pt_inside_mesh(hedron.vertices(5,:), hedron), 'Vertex points should count as being inside.');
assert(check_pt_inside_mesh(face_center, hedron), 'Point on surface should count as being inside.');
assert(~check_pt_inside_mesh([4, -4, 5], hedron), 'Point outside should not register as being inside.');

% Bad size inputs should throw exceptions.
assert_error(@()(check_pt_inside_mesh(rand(2,3))));
assert_error(@()(check_pt_inside_mesh(rand(3,2))));
assert_error(@()(check_pt_inside_mesh(rand(1,4))));
assert_error(@()(check_pt_inside_mesh(rand(1,2))));

end

