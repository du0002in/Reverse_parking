function abs_angle=absolueAngle(x,y,R,xc,yc)
x=x-xc;
y=y-yc;
phi1=asin(y/R);
phi2=pi-phi1;
phi=[phi1 phi2];
phi(phi<0)=phi(phi<0)+2*pi;
if phi(1)==phi(2)
    abs_angle=phi(1);
else
    sign_cos_phi=sign(cos(phi));
    sign_x=sign(x);
    delta=sign_cos_phi-sign_x;
    abs_angle=phi(delta==0);
end
end