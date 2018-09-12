function [twentyhedron, sixhedron, cube] = simple_shape_data()
% SIMPLE_SHAPE_DATA Makes mesh data for several simple convex polyhedra.
%
%   [twentygon, sixgon, cube] = SIMPLE_SHAPE_DATA()
%
%   Inputs: <none>
%   Outputs:
%       `twentygon` -- Icosahedron mesh data. Structure contains faces,
%       vertices, face_normals, and vertex_normals.
%       `sixgon` -- Hexahedron mesh data. Structure contains faces,
%       vertices, face_normals, and vertex_normals.
%       `cube` -- Cube mesh data. Structure contains faces,
%       vertices, face_normals, and vertex_normals.
%
%   See also: GET_MESH_DATA, GET_ALL_NORMALS, PATCH.
%

%% Icosahedron
scale_factor = 0.1;
V1 = [    0.0000   -1.0515   -1.7013
    0.0000   -1.0515    1.7013
    0.0000    1.0515    1.7013
    0.0000    1.0515   -1.7013
   -1.0515   -1.7013         0
   -1.0515    1.7013         0
    1.0515    1.7013         0
    1.0515   -1.7013         0
   -1.7013    0.0000   -1.0515
    1.7013         0   -1.0515
    1.7013         0    1.0515
   -1.7013    0.0000    1.0515] * scale_factor;

F1 = [     9     4     1
     1     5     9
     1     8     5
     10     8    1
     4    10     1
    5     2     12
    12     2     3
    12     3     6
    12     6     9
    12     9     5
    10     7    11
    8    10     11
    2     8     11
    3     2     11
    7     3     11
     2     5     8
    10     4     7
     7     6     3
     6     7     4
     6     4     9];
 [VN1, FN1] = get_all_normals(F1, V1);
 twentyhedron.vertices = V1;
 twentyhedron.faces = F1;
 twentyhedron.vertex_normals = VN1;
 twentyhedron.face_normals = FN1;
 
 %% Hexahedron
 V2 = [   -0.7071   -0.7071         0
    0.7071   -0.7071         0
    0.7071    0.7071         0
   -0.7071    0.7071         0
    0.0000         0   -1.0000
    0.0000         0    1.0000] * scale_factor;

F2 = [      5     2     1
     5     3     2
     5     4     3
     5     1     4
     1     2     6
     2     3     6
     3     4     6
     4     1     6];
 [VN2, FN2] = get_all_normals(F2, V2);
 sixhedron.vertices = V2;
 sixhedron.faces = F2;
 sixhedron.vertex_normals = VN2;
 sixhedron.face_normals = FN2;
 
 %% Cube
 V3 = [ ...
     1 1 1
     1 -1 1
     1 -1 -1
     -1 -1 -1
     -1 1 -1
     -1 1 1
     -1 -1 1
     1 1 -1] * scale_factor;
 F3 = [ ...
     1 6 7
     1 7 2
     3 4 5
     5 8 3
     1 2 3
     1 3 8
     7 4 3 
     2 7 3
     6 5 7
     5 4 7
     1 5 6
     1 8 5];
  [VN3, FN3] = get_all_normals(F3, V3);
 cube.vertices = V3;
 cube.faces = F3;
 cube.vertex_normals = VN3;
 cube.face_normals = FN3;

end