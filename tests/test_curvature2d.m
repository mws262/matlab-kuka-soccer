pts = rand(8,3).*[1,1,0];
pts = sort(pts);

face_normals = cross(repmat([0,0,1],[size(pts,1) - 1,1]), diff(pts), 2);
face_normals = face_normals./sqrt(sum(face_normals.*face_normals,2));
face_centers = (pts(2:end,:) + pts(1:end-1,:))/2;

side_lengths = sqrt(sum(diff(pts).*diff(pts),2));
weighted_face_normals = side_lengths.*face_normals;
vert_normals = (weighted_face_normals(1:end-1,:) + weighted_face_normals(2:end,:)) / 2;
vert_normals = vert_normals./sqrt(sum(vert_normals.*vert_normals,2));
vert_normals = [face_normals(1,:); vert_normals; face_normals(end,:)];

curvature = dot(diff(vert_normals), diff(pts),2)./side_lengths.^2;
radii = curvature.^-1;

Pc = -sqrt(radii.^2 - (side_lengths/2).^2) .* face_normals + face_centers;
P1c = pts(1:end-1, :) - Pc;
P2c = pts(2:end, :) - Pc;
th1 = atan2(P1c(:,2), P1c(:,1));
th2 = atan2(P2c(:,2), P2c(:,1));

interp_pts = interp1(1:size(pts,1), pts, linspace(1, size(pts,1), size(pts,1) * 10));
interp_normals = interp1(1:size(pts,1), vert_normals, linspace(1, size(pts,1), size(pts,1) * 10));

close all;
figure;
plot(pts(:,1), pts(:,2), 'Marker', '.');
hold on;
plot(Pc(:,1), Pc(:,2), '.', 'MarkerSize', 20);
% quiver(face_centers(:,1), face_centers(:,2), face_normals(:,1), face_normals(:,2));
quiver(pts(:,1), pts(:,2), vert_normals(:,1), vert_normals(:,2));
quiver(interp_pts(:,1), interp_pts(:,2), interp_normals(:,1), interp_normals(:,2));