function [vn, face_norm] = get_all_normals(f, v, fn)
% GET_ALL_NORMALS Get mesh normals given faces and vertices.
%   This will calculate face normals and area-weighted vertex normals.
%   If face normals a given as an argument, then they will be used to just
%   calculate the vertex normals.

if ~exist('fn','var')
    a = v(f(:,1),:);
    b = v(f(:,2),:);
    c = v(f(:,3),:);
    fn = cross((b-a),(c-a)); % un-normalized face normals
    face_norm = fn./repmat(sqrt(sum(fn.^2,2)),[1,3]); % normalized face normals
else
    face_norm = fn;
end

% File exchange version I downloaded was STLVertexNormals. It was trash,
% however.
% vn(f(:,1),:) = vn(f(:,1),:) + fn; % add the normals for the first point in each triangle
% vn(f(:,2),:) = vn(f(:,2),:) + fn; % add the normals for the second point in each triangle
% vn(f(:,3),:) = vn(f(:,3),:) + fn; % add the normals for the third point in each triangle
% Issue is that a([1,1]) = a([1,1]) + [5,5] causes element 1 to increment
% by 5 NOT 10.
vn = accumarray([f(:),ones(size(f(:)));f(:),2*ones(size(f(:))); f(:),3*ones(size(f(:)))], [repmat(fn(:,1),[3,1]);repmat(fn(:,2),[3,1]);repmat(fn(:,3),[3,1])]);
vn = vn./repmat(sqrt(sum(vn.^2,2)),[1,3]); % normalized vertex normals
