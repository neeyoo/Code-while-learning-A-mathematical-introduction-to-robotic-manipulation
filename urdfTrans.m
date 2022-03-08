function T = urdfTrans(Attributes)

rpy = str2num(Attributes.rpy);
p = str2num(Attributes.xyz);

R = exposkew([1 0 0], rpy(1)) * exposkew([0 1 0], rpy(2)) * exposkew([0 0 1], rpy(3));

T = [R p'; 0 0 0 1];

end