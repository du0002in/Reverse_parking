%taking line parameters as input cx+ey+fp=0. return intersection points
%between consective lines. (max no. of lines is 3)
function [x_intersect y_intersect]=lineintersection(C,E,Fp)
if length(C)==2
    y_intersect=(C(2)*Fp(1)-C(1)*Fp(2))/(E(2)*C(1)-C(2)*E(1));
    x_intersect=(E(1)*Fp(2)-E(2)*Fp(1))/(C(1)*E(2)-C(2)*E(1));
elseif length(C)==3
    y_intersect(1)=(C(2)*Fp(1)-C(1)*Fp(2))/(E(2)*C(1)-C(2)*E(1));
    x_intersect(1)=(E(1)*Fp(2)-E(2)*Fp(1))/(C(1)*E(2)-C(2)*E(1));
    y_intersect(2)=(C(2)*Fp(3)-C(3)*Fp(2))/(E(2)*C(3)-C(2)*E(3));
    x_intersect(2)=(E(3)*Fp(2)-E(2)*Fp(3))/(C(3)*E(2)-C(2)*E(3));
    y_intersect(3)=(C(3)*Fp(1)-C(1)*Fp(3))/(E(3)*C(1)-C(3)*E(1));
    x_intersect(3)=(E(1)*Fp(3)-E(3)*Fp(1))/(C(1)*E(3)-C(3)*E(1));
end
end