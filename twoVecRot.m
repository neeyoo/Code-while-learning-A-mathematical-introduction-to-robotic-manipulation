function R = twoVecRot(a, b)
%This function is for calculation of the rotational matrix of two vector

v = cross(a, b);
c = a' * b;

if c == -1
    %opposite direction, in the case the rotational matrix is not unique
    error('opposite direction, rotational matrix cannot be solved');
else
    R = eye(3) + skew(v) + skew(v)^2 * (1/(1+c));

end