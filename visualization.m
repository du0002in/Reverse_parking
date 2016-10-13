%car park slot
x_slot=[d(2) d(2) d(4) d(4)];
z_slot=[d(1) d(3) d(3) d(1)];
figure;
fill(x_slot, z_slot, 'g');
axis equal;
hold on;

%car
x_car=[.5 -.5 -.5 .5];
z_car=[0.33 0.33 -1.61 -1.61];
R=[cos(theta) 0 sin(theta);...
    0 1 0;...
    -sin(theta) 0 cos(theta)];
car_global=R*[x_car; zeros(1,4); z_car];
x_car_global=car_global(1,:);
z_car_global=car_global(3,:);
fill(x_car_global, z_car_global,'c');

%camera vision -- the area where the camera can capture.
x_vision=[0.68 1.10 -1.10 -0.68];
z_vision=[0.48 2.13 2.13 0.48];
vision_global=R*[x_vision; zeros(1,4); z_vision];
x_vision_global=vision_global(1,:);
z_vision_global=vision_global(3,:);
plot(x_vision_global, z_vision_global,'b--');

%car and carpark slot angle
z_slot_center=[0 d(3)];
x_slot_center=[(d(2)+d(4))/2 (d(2)+d(4))/2];
plot(x_slot_center, z_slot_center,'k--');
x_car_center=[0 0];
z_car_center=[0 1.5];
car_center_global=R*[x_car_center;[0 0];z_car_center];
x_car_center_global=car_center_global(1,:);
z_car_center_global=car_center_global(3,:);
plot(x_car_center_global,z_car_center_global,'k--');

