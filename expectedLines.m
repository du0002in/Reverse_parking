function [NoLine, slop, location_x, location_z]=expectedLines(theta,d)
NoLine=0;
R=[cos(theta) 0 sin(theta);...
    0 1 0;...
    -sin(theta) 0 cos(theta)];
%camera vision -- the area where the camera can capture.
x_vision=[0.68 1.10 -1.10 -0.68];
z_vision=[0.48 2.13 2.13 0.48];
vision_global=R*[x_vision; zeros(1,4); z_vision];
x_vision_global=vision_global(1,:);
z_vision_global=vision_global(3,:);

line1x=[d(4):0.01:d(2)];
line1z=ones(1,length(line1x))*d(1);
IN=inpolygon(line1x,line1z,x_vision_global,z_vision_global);
if isempty(IN)
    location_x(1)=NaN;
    location_z(1)=NaN;
    slop(1)=NaN;
else
    location_x(1)=mean(line1x(IN));
    location_z(1)=mean(line1z(IN));
    [location_x(1), location_z(1)]=Real2Cam(location_x(1), location_z(1),theta);
    [xtmp1,ztmp1]=Real2Cam(d(4),d(1),theta);
    [xtmp2,ztmp2]=Real2Cam(d(2),d(1),theta);
    slop(1)=atan((ztmp1-ztmp2)/(xtmp1-xtmp2));
    NoLine=NoLine+1;
end

line2z=[d(1):0.01:d(3)];
line2x=ones(1,length(line2z))*d(2);
IN=inpolygon(line2x,line2z,x_vision_global,z_vision_global);
if isempty(IN)
    location_x(2)=NaN;
    location_z(2)=NaN;
    slop(2)=NaN;
else
    location_x(2)=mean(line2x(IN));
    location_z(2)=mean(line2z(IN));
    [location_x(2), location_z(2)]=Real2Cam(location_x(2), location_z(2),theta);
    [xtmp1,ztmp1]=Real2Cam(d(2),d(3),theta);
    [xtmp2,ztmp2]=Real2Cam(d(2),d(1),theta);
    slop(2)=atan((ztmp1-ztmp2)/(xtmp1-xtmp2));
    NoLine=NoLine+1;
end

line3x=[d(4):0.01:d(2)];
line3z=ones(1,length(line2z))*d(3);
IN=inpolygon(line3x,line3z,x_vision_global,z_vision_global);
if isempty(IN)
    location_x(3)=NaN;
    location_z(3)=NaN;
    slop(3)=NaN;
else
    location_x(3)=mean(line3x(IN));
    location_z(3)=mean(line3z(IN));
    [location_x(3), location_z(3)]=Real2Cam(location_x(3), location_z(3),theta);
    [xtmp1,ztmp1]=Real2Cam(d(2),d(3),theta);
    [xtmp2,ztmp2]=Real2Cam(d(4),d(3),theta);
    slop(3)=atan((ztmp1-ztmp2)/(xtmp1-xtmp2));
    NoLine=NoLine+1;
end

line4z=[d(1):0.01:d(3)];
line4x=ones(1,length(line2z))*d(4);
IN=inpolygon(line4x,line4z,x_vision_global,z_vision_global);
if isempty(IN)
    location_x(4)=NaN;
    location_z(4)=NaN;
    slop(4)=NaN;
else
    location_x(4)=mean(line4x(IN));
    location_z(4)=mean(line4z(IN));
    [location_x(4), location_z(4)]=Real2Cam(location_x(4), location_z(4),theta);
    [xtmp1,ztmp1]=Real2Cam(d(4),d(3),theta);
    [xtmp2,ztmp2]=Real2Cam(d(4),d(1),theta);
    slop(4)=atan((ztmp1-ztmp2)/(xtmp1-xtmp2));
    NoLine=NoLine+1;
end
end