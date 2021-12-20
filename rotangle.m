function theta = rotangle(R)
% this function is for calculating the rotation angle about rotational axis
% the input is the rotational matrix 'R'


theta = acos((trace(R)-1)/2);


end