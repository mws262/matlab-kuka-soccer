function centroids = get_face_centroids(faces, vertices)
% GET_FACE_CENTROIDS Return face centroids.
%   The indices of these align with that of the given faces.
centroids = (vertices(faces(:,1),:) + vertices(faces(:,2),:) + vertices(faces(:,3),:))/3;
end