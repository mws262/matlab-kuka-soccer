function [twentygon, sixgon] = simple_shape_data()
% Just some sample face and vertex data.
% platonic_solid by Kevin Moerman: https://www.mathworks.com/matlabcentral/fileexchange/28213-platonic-solid
% helped me generate these numbers
% Note -- I had to fix normal directions myself. Don't use this tool again.

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
 twentygon.vertices = V1;
 twentygon.faces = F1;
 twentygon.vertex_normals = VN1;
 twentygon.face_normals = FN1;
 
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
 sixgon.vertices = V2;
 sixgon.faces = F2;
 sixgon.vertex_normals = VN2;
 sixgon.face_normals = FN2;
end