function omega = rotaxis(R)
%This function is for calculating the rotational axis
%The input is rotational matrix 'R'

omega = (1/(2 * sin(rotangle(R)))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];


end