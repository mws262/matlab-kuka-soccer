function signed_area = get_signed_mesh_volume(mesh_data)
%GET_SIGNED_MESH_VOLUME

% Use average vertex position as the test point.
test_pt = sum(mesh_data.vertices,1)/size(mesh_data.vertices,1);
a = mesh_data.vertices(mesh_data.faces(:,1),:) - test_pt;
b = mesh_data.vertices(mesh_data.faces(:,2),:) - test_pt;
c = mesh_data.vertices(mesh_data.faces(:,3),:) - test_pt;

signed_area = sum(dot(cross(a, b), c, 2))/6;

end

