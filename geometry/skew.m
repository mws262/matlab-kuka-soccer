function skewmat = skew(vec)
% SKEWMAT = SKEW(VEC) Make a skew-symmetric matrix (i.e. the cross-product
% matrix).

assert(iscolumn(vec) || isrow(vec), 'Input vector was not a row or column vector.');
assert(length(vec) == 3, 'Input vector must have 3 elements. Has: %d. \n', length(vec));

skewmat =[0 -vec(3) vec(2) ; vec(3) 0 -vec(1) ; -vec(2) vec(1) 0 ];
end