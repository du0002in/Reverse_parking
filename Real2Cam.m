function [Px, Pz]=Real2Cam(X,Z,theta)
F=717; ox=307; oy=257; Y=1.525; phi=-55.3/180*pi;
Px=round(ox+F*(X*cos(theta)-Z*sin(theta))./(Z*cos(theta)*cos(phi)-Y*sin(phi)+X*cos(phi)*sin(theta)));
Pz=round(oy+F*(Y*cos(phi)+Z*cos(theta)*sin(phi)+X*sin(phi)*sin(theta))./(Z*cos(phi)*cos(theta)-Y*sin(phi)+X*cos(phi)*sin(theta)));
end