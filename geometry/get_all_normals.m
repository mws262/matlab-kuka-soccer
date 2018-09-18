function [vertex_normals, face_normals] = get_all_normals(f, v, fn)
% GET_ALL_NORMALS Get face and vertex normals given faces and vertices.
%   This will calculate face normals and area-weighted vertex normals.
%   If face normals a given as an argument, then they will be used to just
%   calculate the vertex normals.
%
%   [vertex_normals, face_normals] = GET_ALL_NORMALS(f, v, fn)
%   [vertex_normals, face_normals] = GET_ALL_NORMALS(f, v)
%
%   Inputs:
%       `f` -- Face connectivity. Indices in vertex matrix which are part
%       of a single face.
%       `v` -- Cartesian vertex locations. nx3.
%       `fn` -- (OPTIONAL) Face normals. Will save a little computation if
%       these are provided. The rows should correspond to face definitions
%       in `f`. For best results, these should be unit-length.
%   Outputs:
%       `vertex_normals` -- Normalized vertex normals. These are the 
%       normals of the faces which use a given vertex, weighted by the area 
%       of these faces. In the future, may add an option for angle-weighted
%       vertex normals.
%       `face_normals` -- Normals of the faces. `fn` is provided, then this
%       will be identical (and not necessarily normalized). If `fn` is not
%       provided, then this will be calculated and normalized.
%
%   See also CHECK_PT_INSIDE_MESH, VALIDATE_MESH_STRUCT, GET_MESH_DATA.
%


if nargin < 3
    a = v(f(:,1),:);
    b = v(f(:,2),:);
    c = v(f(:,3),:);
    fn = cross((b-a),(c-a)); % un-normalized face normals
    face_normals = fn./repmat(sqrt(sum(fn.^2,2)),[1,3]); % normalized face normals
else
    face_normals = fn;
end

% TODO: expand this doozy of a one-liner.
vertex_normals = accumarray([f(:),ones(size(f(:)));f(:),2*ones(size(f(:))); f(:),3*ones(size(f(:)))], [repmat(fn(:,1),[3,1]);repmat(fn(:,2),[3,1]);repmat(fn(:,3),[3,1])]);
vertex_normals = vertex_normals./repmat(sqrt(sum(vertex_normals.^2,2)),[1,3]); % normalized vertex normals
