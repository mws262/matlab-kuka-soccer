syms x y real;
bernsteins = sym('n', [12,1]);
count = 1;
for n = 0:4
    for k = 0:n
        bernsteins(count,1) = nchoosek(n,k)*x^k*(1-x)^(n-k);
        count = count + 1;
    end
end

bernsteins_dx = diff(bernsteins) 