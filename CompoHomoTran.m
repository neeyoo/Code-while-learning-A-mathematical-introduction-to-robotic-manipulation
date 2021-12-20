function T = CompoHomoTran(varargin)
%This function is for the compositon of homogeneous transformations
%Inputs are consecutive homogenous transformation matices '0Tn = 0T1 * 1T2 * ... * n-1Tn'

if nargin <=1
    error('Input must have at least two homogeneous transformations')
end

T_0 = varargin{1};
R = T_0(1:3,1:3);
p = T_0(1:3,4);

for i = 2:nargin
    T_temp = varargin{i};
    p = p + R * T_temp(1:3,4);
    R = R * T_temp(1:3,1:3);
end

T = [[R; 0 0 0] [p;1]];

end