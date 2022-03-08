function AdgInv = Trans2Invadjoint(Trans)

%This function is for calculating the inverse adjoint transformation which is a 6*6
%matrix which transforms twists from one coordinate frame to another.

R = Trans(1:3,1:3);
p = Trans(1:3,4);
AdgInv = [R' -R'*skew(p); zeros(3) R'];


end