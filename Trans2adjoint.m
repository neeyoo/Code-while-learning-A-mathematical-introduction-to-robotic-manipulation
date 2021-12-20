function Adg = Trans2adjoint(Trans)

%This function is for calculating the adjoint transformation which is a 6*6
%matrix which transforms twists from one coordinate frame to another.

R = Trans(1:3,1:3);
p = Trans(1:3,4);
Adg = [R skew(p)*R; zeros(3) R];


end