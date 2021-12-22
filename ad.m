function adV = ad(V)

%support ad function for calculating the lie bracket


adV = [skew(V(4:6)) skew(V(1:3)); zeros(3) skew(V(4:6))];


end