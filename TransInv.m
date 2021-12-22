function Tinv = TransInv(T)
%this function is for calculating the inverse transformation matrix in SE(3)

Tinv = [T(1:3,1:3)' -T(1:3,1:3)'*T(1:3,4); 0 0 0 1];



end