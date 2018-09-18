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
    if c < 0 % 180 degree flip. WARNING This is kind of sketchy...          
        rotation = axang2rotm([cross(v1,v1+[1,5,2]), pi]);
    else % 0-degree 'rotation'
        rotation = eye(3);
    end
   return;
end

rotation = eye(3) + sk + sk^2*(1 - c)/s^2;

validateattributes(rotation, {'numeric'}, {'nonnan', '>=', -1 - 1e-8, '<=', 1 + 1e-8});

end