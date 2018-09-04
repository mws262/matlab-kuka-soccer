function [cylSides, cylLids, rot] = genCylinder( loc1, loc2, rad, faces )

if isrow(loc1)
   loc1 = loc1'; 
end
if isrow(loc2)
    loc2 = loc2';
end

% Old code from FigGen (i.e. the nsf flexible arm proposal crap).
[X,Y,Z] = cylinder(rad,faces);
cylLids = patch(X',Y',Z','r');
cylSides = patch(surf2patch(X,Y,Z));

% Make correct height
height = norm(loc2-loc1);
cylSides.Vertices(:,3) = cylSides.Vertices(:,3)*height;
cylLids.Vertices(:,3) = cylLids.Vertices(:,3)*height;
 
% Rotate & translate
%Copied from stack exchange to find the rotation matrix between two vectors
b=(loc2-loc1)/norm(loc2-loc1);
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
cylSides.Vertices = (U*cylSides.Vertices' + repmat(loc1,[1,size(cylSides.Vertices,1)]))';
cylLids.Vertices = (U*cylLids.Vertices' + repmat(loc1,[1,size(cylLids.Vertices,1)]))';
 
cylLids.FaceColor = [0.6,0.6,0.8];
cylSides.FaceColor = [0.6,0.6,0.8];
cylLids.EdgeAlpha = 0.4;
cylSides.EdgeAlpha = 0;
rot = U;
end