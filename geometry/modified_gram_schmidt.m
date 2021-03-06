function [Q,R]=modified_gram_schmidt(A)
% MODIFIED_GRAM_SCHMIDT For re-orthonormalizing rotation matrices.
validateattributes(A, {'single', 'double'}, {'real', 'size', [3,3]});

[m,n]=size(A);
V=A;
Q=zeros(m,n);
R=zeros(n,n);
for i=1:n
  R(i,i)=norm(V(:,i));
  Q(:,i)=V(:,i)/R(i,i);
  for j=i+1:n
    R(i,j)=Q(:,i)'*V(:,j);
    V(:,j)=V(:,j)-R(i,j)*Q(:,i);
  end
end