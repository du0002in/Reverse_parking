function plotCarPark
global_varibles;
x=[-0.5*W_slot-0.1 0.5*W_slot+0.1 0.5*W_slot+0.1 -0.5*W_slot-0.1];
z=[0 0 L_slot L_slot];

fill(x,z,'g');
x=[-0.5*W_slot .5*W_slot .5*W_slot -.5*W_slot -.5*W_slot];
z=[0.1 0.1 L_slot-.1 L_slot-.1 .1];
axis equal;
axis ([-5 5 -5 3]);
hold on; plot(x,z);
hold off;
end