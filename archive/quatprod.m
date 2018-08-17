function q1prodq2 = quatprod(q1,q2)

N1 = size(q1,1);
N2 = size(q2,1);
F_N1sup1_N2sup1 = N1>1 && N2>1;

s1 = q1(:,1);
s2 = q2(:,1);

v1 = q1(:,2:4);
v2 = q2(:,2:4);

s1s2 = s1.*s2;
if F_N1sup1_N2sup1
    v1dotv2 = sum(v1.*v2,2);
else
    v1dotv2 = v1*v2';
    if N2>1, v1dotv2 = v1dotv2'; end
end

if F_N1sup1_N2sup1
    s1v2 = s1(:,ones(1,3)).*v2;
    s2v1 = s2(:,ones(1,3)).*v1;
else
    s1v2 = s1*v2;
    s2v1 = s2*v1;
end
v1crossv2 = cross_prod_V1_V2(v1,v2);

q1prodq2 = [(s1s2 - v1dotv2)   (s1v2 + s2v1 + v1crossv2)];

function V3 = cross_prod_V1_V2(V1,V2)
% V1 and V2 must be written as line vectors (Nb_vectors x 3).
V3 = [V1(:,2).*V2(:,3)-V1(:,3).*V2(:,2)  V1(:,3).*V2(:,1)-V1(:,1).*V2(:,3)  V1(:,1).*V2(:,2)-V1(:,2).*V2(:,1)];