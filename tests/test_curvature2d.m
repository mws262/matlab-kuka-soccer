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
 
Pc = -sign(radii).*sqrt(radii.^2 - (side_lengths/2).^2) .* face_normals + face_centers;
P1c = pts(1:end-1, :) - Pc;
P2c = pts(2:end, :) - Pc;
th1 = atan2(P1c(:,2), P1c(:,1));
th2 = atan2(P2c(:,2), P2c(:,1));

st_pt = [cos(th1).*abs(radii), sin(th1).*abs(radii), zeros(size(th1,1),1)] + Pc;
end_pt = [cos(th2).*abs(radii), sin(th2).*abs(radii), zeros(size(th2,1),1)] + Pc;

interp_pts = interp1(1:size(pts,1), pts, linspace(1, size(pts,1), size(pts,1) * 10));
interp_normals = interp1(1:size(pts,1), vert_normals, linspace(1, size(pts,1), size(pts,1) * 10));

close all;
figure;
plot(pts(:,1), pts(:,2), 'Marker', '.');
hold on;
for i = 1:length(th1)
    n = 100;
    arc = [cos(linspace(0, 2*pi, n)').*radii(i), sin(linspace(0, 2*pi, n)').*radii(i), zeros(n,1)] + Pc(i,:)
    plot(arc(:,1), arc(:,2));
end
plot([Pc(:,1), st_pt(:,1)]', [Pc(:,2), st_pt(:,2)]');
% plot(end_pt(:,1), end_pt(:,2), '.', 'MarkerSize', 20);
% quiver(face_centers(:,1), face_centers(:,2), face_normals(:,1), face_normals(:,2));
quiver(pts(:,1), pts(:,2), vert_normals(:,1), vert_normals(:,2));
quiver(interp_pts(:,1), interp_pts(:,2), interp_normals(:,1), interp_normals(:,2));
axis equal;