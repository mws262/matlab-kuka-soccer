function [ distance, surface_point, normal_vec, face_idx ] = point2trimesh_with_normals(point_to_project, mesh_data, use_face_normals)
% POINT2TRIMESH_WITH_NORMALS Projects a point to the nearest face, edge or
% vertex.
%
%   [ distance, surface_point, normal_vec, face_idx ] = POINT2TRIMESH_WITH_NORMALS(point, mesh_data)
%
%   Inputs:
%       `point_to_project` -- Point in 3D space in the coordinates of the
%       given mesh. Point gets projected to the nearest point on the mesh.
%       `mesh_data` -- Structure with faces, vertices, face_normals, and
%       vertex_normals data.
%       `use_face_normals` -- (OPTIONAL) If included, can specify whether
%       to use smoothly interpolated vertex normals or the face normals
%       when the nearest point is on a face. Vertex normals will still be
%       used when an edge or vertex is the nearest. This is so large boxes
%       don't have super-weird non-flat surfaces (according to the
%       normals).
%   Outputs:
%       `distance` -- Distance from the provided point to its projection on
%       the surface of the mesh.
%       `surface_point` -- Point on the surface of the mesh. Corresponds to
%       a vertex, a point on an edge, or a point on the plane of a triangle
%       surface.
%       `normal_vec` -- Outward-facing normal vector of the projected
%       surface location. These are smooth normal vectors, interpolated
%       from the given vertex normals.
%       `face_idx` -- Index of the face that was projected to. If the
%       projection went to a vertex or edge, this will be one of the
%       adjacent faces.
%
%   This is drastically edited and reduced from point2trimesh from the
%   File Exchange.
%
%   See also POINT2TRIMESH, FIND_MESH_CONTACT_TFORM, VALIDATE_MESH_STRUCT,
%   TEST_POINT2TRIMESH_WITH_NORMALS.

% Edited version which has about 1/20th the features. It accepts only one
% point at a time. However, it does return face or vertex normals as
% applicable.
% Note that face idx might represent only an adjacent face if a vertex is
% nearest.

validateattributes(point_to_project, {'numeric'}, {'2d', 'numel', 3, 'real'});
validate_mesh_struct(mesh_data);

if nargin > 2
    face_norms = use_face_normals;
else
    face_norms = false;
end
%% Distance Calculation
[distance,surface_point,face_idx,normal_vec] = processPoint(mesh_data.faces, mesh_data.vertices, point_to_project, mesh_data.face_normals, mesh_data.vertex_normals, @distance_to_vertices, @distance_to_edges, @distance_to_surfaces, face_norms);
end

%% Non-vectorized Distance Functions
%  (can process only one point)
function [D,P,F,N] = processPoint(faces, vertices, point, ...
    face_normals, vertex_normals, distance_to_vertices, distance_to_edges, distance_to_surfaces, use_face_normals)

d = zeros(3,1) + realmax; % (distanceTypes x 1) Made arbitrarily large so "empty" elements never win the min. NaN is oddly slow.
p = zeros(3,3); % (distanceTypes x xyz)
f = zeros(3,1); % (distanceTypes x 1)
n = zeros(3,3);

% find nearest vertex
[d(1),p(1,:),f(1),n(1,:)] = distance_to_vertices(faces,vertices,point,face_normals, vertex_normals);
% d:  (1 x 1) signed distance to surface
% p:  (1 x 3) corresponding point on surface
% v:  (1 x 1) nearest vertex
% connectedFaces: (#connectedFaces x 1) face indices

[d(2),p(2,:),f(2),n(2,:)] = distance_to_edges(faces,vertices,point,face_normals, vertex_normals);

% find nearest point on all surfaces
[d(3),p(3,:),f(3),n(3,:)] = distance_to_surfaces(faces,vertices,point,face_normals, vertex_normals, use_face_normals);

% find minimum distance type
[~,I] = min(abs(d),[],1);
D = d(I);
P = p(I,:);
F = f(I);
N = n(I,:);
N = N/norm(N); % Some of the barycentric weighting leads to non-unit vectors. Correct it!
end

function [D,P,F,N] = distance_to_vertices(faces,vertices,qPoint,normals,vnormals)

% find nearest vertex
[D,nearestVertexID] = min(sum(bsxfun(@minus,vertices,qPoint).^2,2),[],1);
D = sqrt(D);
P = vertices(nearestVertexID,:);
N = vnormals(nearestVertexID,:);

% find faces that belong to the vertex
connectedFaces = find(any(faces == nearestVertexID,2)); % (#connectedFaces x 1) face indices
assert(length(connectedFaces)>=1,'Vertex %u is not connected to any face.',nearestVertexID)
F = connectedFaces(1);
n = normals(connectedFaces,:); % (#connectedFaces x 3) normal vectors

% scalar product between distance vector and normal vectors
coefficients = dot2(n,qPoint-P);
sgn = signOfLargest(coefficients);
D = D*sgn;

end

function [D,P,F,N] = distance_to_edges(faces,vertices,qPoint,normals,vnormals)

% Point-point representation of all edges
edges = [faces(:,[1,2]); faces(:,[1,3]); faces(:,[2,3])]; % (#edges x 2) vertice IDs

% Intersection between tangent of edge lines and query point
r1 = vertices(edges(:,1),:);   % (#edges x 3) first point of every edge
r2 = vertices(edges(:,2),:);   % (#edges x 3) second point of every edge
t = dot( bsxfun(@minus,qPoint,r1), r2-r1, 2) ./ sum((r2-r1).^2,2); % (#edges x 1) location of intersection relative to r1 and r2
t(t<=0) = NaN; % exclude intersections not between the two vertices r1 and r2
t(t>=1) = NaN;

% Distance between intersection and query point
P = r1 + bsxfun(@times,(r2-r1),t); % (#edges x 3) intersection points
D = bsxfun(@minus,qPoint,P); % (#edges x 3)
D = sqrt(sum(D.^2,2));       % (#edges x 1)
[D,I] = min(D,[],1);         % (1 x 1)
P = P(I,:);

% find faces that belong to the edge
inds = edges(I,:);  % (1 x 2)
inds = permute(inds,[3,1,2]);  % (1 x 1 x 2)
inds = bsxfun(@eq,faces,inds); % (#faces x 3 x 2)
inds = any(inds,3);    % (#faces x 3)
inds = sum(inds,2)==2; % (#faces x 1) logical indices which faces belong to the nearest edge of the query point

% Average vertex normals.
unit_dist_between = t(I,:);
N = sum(vnormals(edges(I,:),:).* [1 - unit_dist_between; unit_dist_between], 1);

F = find(inds,1);
n = normals(inds,:); % (#connectedFaces x 3) normal vectors
% N = sum(n,1); % TODO: weight by area if it seems remotely necessary.
% N = N/norm(N);

% scalar product between distance vector and normal vectors
coefficients = dot2(n,qPoint-P); % (#connectedFaces x 1)
sgn = signOfLargest(coefficients);
D = D*sgn;
end

function [D,P,F,N] = distance_to_surfaces(faces,vertices,point,normals,vnormals, use_face_normals)

r1 = vertices(faces(:,1),:);   % (#faces x 3) % 1st vertex of every face
r2 = vertices(faces(:,2),:);   % (#faces x 3) % 2nd vertex of every face
r3 = vertices(faces(:,3),:);   % (#faces x 3) % 3rd vertex of every face

vq = bsxfun(@minus,point,r1);  % (#faces x 3)
D = dot(vq,normals,2);         % (#faces x 1) distance to surface
rD = bsxfun(@times,normals,D); % (#faces x 3) vector from surface to query point
P = bsxfun(@minus,point,rD);   % (#faces x 3) nearest point on surface; can be outside triangle

% find barycentric coordinates (query point as linear combination of two edges)
r31r31 = sum((r3-r1).^2,2);    % (#faces x 1)
r21r21 = sum((r2-r1).^2,2);    % (#faces x 1)
r21r31 = dot(r2-r1,r3-r1,2);   % (#faces x 1)
r31vq = dot(r3-r1,vq,2);       % (#faces x 1)
r21vq = dot(r2-r1,vq,2);       % (#faces x 1)

d = r31r31.*r21r21 - r21r31.^2;               % (#faces x 1)
bary = zeros(size(faces,1),3);                  % (#faces x 3)
bary(:,1) = (r21r21.*r31vq-r21r31.*r21vq)./d; % (#faces x 3)
bary(:,2) = (r31r31.*r21vq-r21r31.*r31vq)./d; % (#faces x 3)
bary(:,3) = 1 - bary(:,1) - bary(:,2);        % (#faces x 3)

% tri = triangulation(faces,vertices);
% bary = tri.cartesianToBarycentric((1:size(faces,1))',P); % (#faces x 3)

% exclude intersections that are outside the triangle. % MATT Removed some
% <='s so edges count as faces.
D( abs(d)<=eps | any(bary<0,2) | any(bary>1,2) ) = realmax;  % (#faces x 1)

% find nearest face for query point
[~,I] = min(abs(D),[],1); % (1 x 1)
D = D(I);       % (1 x 1)
P = P(I,:);     % (1 x 3)
F = I;          % (1 x 1)


verts = faces(I,:);

if nargin > 5 && use_face_normals
    N = normals(I,:);
else
    N = sum(vnormals(verts,:).*fliplr(bary(I,:))',1); % Averaging vertex normals rather than using face normals.
end
end

function sgn = signOfLargest(coeff)
[~,I] = max(abs(coeff));
sgn = sign(coeff(I));
if sgn==0, sgn=1; end
end

function d = dot2(A,B)
% dot product along 2nd dimension with singleton extension
d = sum(bsxfun(@times,A,B),2);
end

