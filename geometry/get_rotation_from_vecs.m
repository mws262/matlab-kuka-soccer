function rotation = get_rotation_from_vecs(v1, v2)
v1 = v1./norm(v1);
v2 = v2./norm(v2);
if (max(abs(v1-v2)) < 1e-10)
    rotation = eye(3);
    return;
end

v = cross(v1, v2);
sk = skew(v);
c = dot(v1, v2);
s = norm(v);

if s < 1e-15 % Could happen with pure 180 degree flip. So unlikely to be intended behavior that we ignore it.
   rotation = eye(3);
   return;
end

rotation = eye(3) + sk + sk^2*(1 - c)/s^2;
if any(any(isnan(rotation)))
    error('Rotation matrix maker got some NaNs all up in it.');
end
end