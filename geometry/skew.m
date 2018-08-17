function skewmat = skew(vec)
% SKEWMAT = SKEW(VEC) Make a skew-symmetric matrix (i.e. the cross-product
% matrix).
skewmat =[0 -vec(3) vec(2) ; vec(3) 0 -vec(1) ; -vec(2) vec(1) 0 ];
end