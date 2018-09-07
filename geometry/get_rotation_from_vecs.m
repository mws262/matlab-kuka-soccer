function rotation = get_rotation_from_vecs(v1, v2)

validateattributes(v1, {'numeric'}, {'2d', 'numel', 3, 'real'});
validateattributes(v2, {'numeric'}, {'2d', 'numel', 3, 'real'});

v1 = v1./norm(v1);
v2 = v2./norm(v2);

v = cross(v1, v2);
sk = skew(v);
c = dot(v1, v2);
s = norm(v);

if s < 1e-15
    if c < 0 % 180 degree flip. WARNING this probably isn't what we want to be happening
        rotation = -eye(3);
    else % 0-degree 'rotation'
        rotation = eye(3);
    end
   return;
end

rotation = eye(3) + sk + sk^2*(1 - c)/s^2;

validateattributes(rotation, {'numeric'}, {'nonnan', '>=', -1, '<=', 1});

end