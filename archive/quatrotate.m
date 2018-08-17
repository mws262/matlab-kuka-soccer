function Pt_coord_rotated = quatrotate(q_rot,Pt_coord)
% quatrotate : Computes the rotation of the points with a quaternion representation
% SYNTAX          : Pt_coord_rotated = quatrotate(q_rot,Pt_coord)
% 
% DESCRIPTION     : Computes the rotation of the points with a quaternion representation.
%                   This function is in similitude with the Matlab function "quaternion.m" (aero toolbox).
%                   It seems that quatrotate of Matlab is doing his rotation in the opposite direction as the one dictated by
%                    the right hand rule. Therefore, to agree with this right hand rule, a negative theta should be provided
%                    in q_rot.
%
% EXAMPLES        : 
%
% INPUT PARAMETERS:
%           - q_rot: unit quaternions (Nb_rot x 4) specifying the Nb_rot rotation(s). A quaternion is written as:
%                     q_rot = (s,vx,vy,vz), with:
%                     s = cos(theta/2);
%                     v = u*sin(theta/2);
%                     u: rotation axis (unit vector);
%                     theta: angle of rotation (in radians).
%           - Pt_coord: point(s) coordinates (NP x 3).
%
%           Accepted situations are:
%           - Nb_rot=1, NP=1; (the point is rotated by the quaternion)
%           - Nb_rot=1, NP>1; (all the points are rotated by the quaternion)
%           - Nb_rot>1, NP=1; (the point is rotated by every quaternion)
%           - Nb_rot>1, NP=Nb_rot. (each point is rotated by the corresponding quaternion)
%
% OUTPUT PARAMETERS:
%           - Pt_coord_rotated : Matrix (NP x 3) of coordinates of the points rotated.
%
% RESTRICTIONS    : 
%
% See also        : http://www.genesis3d.com/~kdtop/Quaternions-UsingToRepresentRotation.htm
%                   quatrotate.m (Matlab aero toolbox)
% BUGS            : 
% CALLING MODULES : 
% FUNCTION CALLED : 
%
% VERSION         : 1.00
% AUTHOR          : Mathieu Gendron
% CREATION DATE   : July 09th 2013
% REVISED BY      : 
% MAJOR MODIFICATIONS:
%
% WISHES-LIST     : 
%*******************************************************************************
if nargin~=2
    error(['The function" ',mfilename,'" requires 2 input arguments.'])
end
errmsg = check_quatrotate(q_rot,Pt_coord);
if ~isempty(errmsg)
    error(errmsg)
end

if isempty(q_rot) || isempty(Pt_coord)
    Pt_coord_rotated = zeros(0,3); return
end

% The syntax used here is: P_rotated = q*P*q^(-1).
% q1 = (s1,v1);
% q2 = (s2,v2);
% q1*q2 = (s1s2 - v1.v2, s1v2 + s2v1 + v1xv2);
% With q = (s,v) unit quaternion, q^(-1) = q' = (s,-v)

% To agree with quatrotate of Matlab, we do the rotation in the opposite direction of the right hand rule.
% Setting a -theta is equivalent to take the negative of the vector part of the quaternions.
q_rot(:,2:4) = -1*q_rot(:,2:4);

q_rot_m1 = q_rot; % Initialization of the inverse of q_rot
q_rot_m1(:,2:4) = -1*q_rot_m1(:,2:4); % Conjugate of q_rot, which is equal to the inverse this unit quaternion

NP = size(Pt_coord,1);
Pt_coord_quaternion = [zeros(NP,1) Pt_coord]; % We write the point coordinates as quaternions

Pqm1 = quatprod(Pt_coord_quaternion,q_rot_m1);
qPqm1 = quatprod(q_rot,Pqm1);
Pt_coord_rotated = qPqm1(:,2:4);

% ********** QUATPROD ********** %
function q1prodq2 = quatprod(q1,q2)

N1 = size(q1,1);
N2 = size(q2,1);
F_N1sup1_N2sup1 = N1>1 && N2>1;

s1 = q1(:,1);
s2 = q2(:,1);

v1 = q1(:,2:4);
v2 = q2(:,2:4);

s1s2 = s1.*s2;
if F_N1sup1_N2sup1
    v1dotv2 = sum(v1.*v2,2);
else
    v1dotv2 = v1*v2';
    if N2>1, v1dotv2 = v1dotv2'; end
end

if F_N1sup1_N2sup1
    s1v2 = s1(:,ones(1,3)).*v2;
    s2v1 = s2(:,ones(1,3)).*v1;
else
    s1v2 = s1*v2;
    s2v1 = s2*v1;
end
v1crossv2 = cross_prod_V1_V2(v1,v2);

q1prodq2 = [(s1s2 - v1dotv2)   (s1v2 + s2v1 + v1crossv2)];

function V3 = cross_prod_V1_V2(V1,V2)
% V1 and V2 must be written as line vectors (Nb_vectors x 3).
V3 = [V1(:,2).*V2(:,3)-V1(:,3).*V2(:,2)  V1(:,3).*V2(:,1)-V1(:,1).*V2(:,3)  V1(:,1).*V2(:,2)-V1(:,2).*V2(:,1)];

% ********** CHECK_QUATROTATE ********** %
function [errmsg] = check_quatrotate(q_rot,Pt_coord)
errmsg = '';

if length(size(q_rot))~=2 || size(q_rot,2)~=4
    errmsg = 'The first input argument, "q_rot", must be a "Nb_rot x 4" matrix.'; return
end
% We assume here that the quaternions are already normalized.

if length(size(Pt_coord))~=2 || size(Pt_coord,2)~=3
    errmsg = 'The second input argument, "Pt_coord", must be a Nx3 matrix.'; return
end

Nb_rot = size(q_rot,1);
NP = size(Pt_coord,1);
if Nb_rot>1 && NP>1
    if Nb_rot~=NP
        errmsg = 'When both "q_rot" and "Pt_coord" have multiple elements, they should have the same number of lines.';
        return
    end
end