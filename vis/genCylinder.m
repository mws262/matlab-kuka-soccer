function [cylinder_side_faces, cylinder_lid_faces, cylinder_axis_rotation] = ...
    make_cylinder( bottom_face_location, top_face_location, radius, number_of_faces, face_color )
% MAKE_CYLINDER Draw a faceted cylinder using patch objects.
%
%   MAKE_CYLINDER( bottom_face_location, top_face_location, radius, number_of_faces )
%   MAKE_CYLINDER( bottom_face_location, top_face_location, radius, number_of_faces, face_color )
%
%   Inputs:


ERROROROROROR Finish this.
%


%% Validate inputs.
if isrow(bottom_face_location)
   bottom_face_location = bottom_face_location'; 
end
if isrow(top_face_location)
    top_face_location = top_face_location';
end

validateattributes(bottom_face_location, {'single', 'double'}, {'real', 'vector', 'column', 'numel', 3});
validateattributes(top_face_location, {'single', 'double'}, {'real', 'vector', 'column', 'numel', 3});
validateattributes(radius, {'single', 'double'}, {'real', 'positive', 'scalar'});
validateattributes(number_of_faces, {'numeric'}, {'integer', 'positive', 'scalar'});

if nargin > 4
   validateattributes(face_color, {'single', 'double'}, {'real', 'row', 'numel', 3});
end

% Old code from FigGen (i.e. the nsf flexible arm proposal crap).
[X,Y,Z] = cylinder(radius,number_of_faces);
cylinder_lid_faces = patch(X', Y', Z', 'r');
cylinder_side_faces = patch(surf2patch(X,Y,Z));

% Make correct height
height = norm(top_face_location - bottom_face_location);
cylinder_side_faces.Vertices(:,3) = cylinder_side_faces.Vertices(:,3)*height;
cylinder_lid_faces.Vertices(:,3) = cylinder_lid_faces.Vertices(:,3)*height;
 
% Rotate & translate
% Copied from stack exchange to find the rotation matrix between two
% vectors.
b=(top_face_location - bottom_face_location)/norm(top_face_location - bottom_face_location);
if b(3) == 1
    U = eye(3,3);
else
    GG = @(A,B) [ dot(A,B) -norm(cross(A,B)) 0;
        norm(cross(A,B)) dot(A,B)  0;
        0              0           1];
    FFi = @(A,B) [ A (B-dot(A,B)*A)/norm(B-dot(A,B)*A) cross(B,A) ];
    UU = @(Fi,G) Fi*G*inv(Fi);
     
    a=[0 0 1]';
    U = UU(FFi(a,b), GG(a,b));
end
cylinder_side_faces.Vertices = (U*cylinder_side_faces.Vertices' + repmat(bottom_face_location,[1,size(cylinder_side_faces.Vertices,1)]))';
cylinder_lid_faces.Vertices = (U*cylinder_lid_faces.Vertices' + repmat(bottom_face_location,[1,size(cylinder_lid_faces.Vertices,1)]))';

if nargin > 4
    cylinder_lid_faces.FaceColor = face_color;
    cylinder_side_faces.FaceColor = face_color;
else
    cylinder_lid_faces.FaceColor = [0.6,0.6,0.8];
    cylinder_side_faces.FaceColor = [0.6,0.6,0.8];
end

cylinder_lid_faces.EdgeAlpha = 0.4;
cylinder_side_faces.EdgeAlpha = 0;
cylinder_axis_rotation = U;
end