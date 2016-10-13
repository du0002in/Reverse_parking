function collitionflag=checkcollition(x, z, theta, buffer)
global_varibles;
collitionflag=0;
theta=-pi/2-theta;
R=[cos(theta) 0 sin(theta);...
    0 1 0;...
    -sin(theta) 0 cos(theta)];

x1=linspace(0.5*W_car,-0.5*W_car,ceil(W_car/0.2));
z1=linspace((L_car-L_wheels)*0.5,(L_car-L_wheels)*0.5,length(x1));
z2=linspace((L_car-L_wheels)*0.5,-L_car+(L_car-L_wheels)*0.5,ceil(L_car/0.2));
x2=linspace(-0.5*W_car, -0.5*W_car, length(z2));
x3=linspace(-0.5*W_car,0.5*W_car,ceil(W_car/0.2));
z3=linspace(-L_car+(L_car-L_wheels)*0.5,-L_car+(L_car-L_wheels)*0.5,length(x1));
z4=linspace(-L_car+(L_car-L_wheels)*0.5,(L_car-L_wheels)*0.5,ceil(L_car/0.2));
x4=linspace(0.5*W_car, 0.5*W_car, length(z4));
x_car=[x1 x2 x3 x4];
z_car=[z1 z2 z3 z4];

car_global=R*[x_car; zeros(1,length(x_car)); z_car];
elementno=size(car_global,2);
for i=1:length(x)
    x_car_global((i-1)*elementno+1:i*elementno)=car_global(1,:)+x(i);
    z_car_global((i-1)*elementno+1:i*elementno)=car_global(3,:)+z(i);
end

ind=find(z_car_global(x_car_global>(W_slot+0.1)/2+buffer+0.05)>buffer+0.05,1);
if isempty(ind)
    ind=find(z_car_global(x_car_global<-(W_slot+0.1)/2-buffer-0.05)>buffer+0.05,1);
    if ~isempty(ind)
        collitionflag=1;
    end
else
    collitionflag=1;
end

if collitionflag==0
    ind=find(z_car_global>L_slot-.35+buffer,1);
    %0.35 is coz the car control point to the car end is abt 0.3m. another 0.05m for buffer
    if ~isempty(ind)
        collitionflag=1;
    else
        collitionflag=0;
    end
end