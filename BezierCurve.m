clear all; close all;
L=3;W=1;slotW=1.4;
delta=.15;
theta=30/180*pi;
t=[0:0.01:1];
n=0;
x_slot=[0.5*slotW+delta -.5*slotW+delta -.5*slotW+delta .5*slotW+delta];
z_slot=[L L L-2 L-2];
fill(x_slot,z_slot,'g');
hold on;
axis equal
for l1=0.1:0.2:L
    for l2=0.1:0.2:L
        P0=[0;0];
        P1=[l1*sin(theta);l1*cos(theta)];
        P2=[delta;L-l2];
        P3=[delta;L];
        B=P0*((1-t).*(1-t).*(1-t))+3*P1*(t.*(1-t).*(1-t))+3*P2*(t.*t.*(1-t))+P3*(t.*t.*t.*t);
        
        D_B=-3*P0*((1-t).*(1-t))+3*P1*(3*t.*t-4*t+1)+3*P2*(2*t-3*t.*t)+3*P3*(t.*t);
        D_D_B=6*P0*(1-t)+6*P1*(3*t-2)+6*P2*(1-3*t)+6*P3*t;
        S_sqr=D_B(1,:).^2+D_B(2,:).^2;
        k=abs(D_B(1,:).*D_D_B(2,:)-D_B(2,:).*D_D_B(1,:))./(S_sqr).^(3/2);
        if ~isempty(find(k>0.43,1))
            continue;
        end
        
        PL0=0.5*W*[-cos(theta);sin(theta)];
        PL1=[l1*sin(theta);l1*cos(theta)]+PL0;
        PL2=[delta-W*0.5;L-l2];
        PL3=[delta-W*0.5;L];
        BL=PL0*((1-t).*(1-t).*(1-t))+3*PL1*(t.*(1-t).*(1-t))+3*PL2*(t.*t.*(1-t))+PL3*(t.*t.*t.*t);
        x=BL(1,:);y=BL(2,:);
        if ~isempty(find(x(y>(L-2))<(-.5*slotW+delta),1))
            continue;
        end
        
        PR0=0.5*W*[cos(theta); -sin(theta)];
        PR1=[l1*sin(theta);l1*cos(theta)]+PR0;
        PR2=[delta+W*0.5;L-l2];
        PR3=[delta+W*0.5;L];
        BR=PR0*((1-t).*(1-t).*(1-t))+3*PR1*(t.*(1-t).*(1-t))+3*PR2*(t.*t.*(1-t))+PR3*(t.*t.*t.*t);
        x=BR(1,:);y=BR(2,:);
        if ~isempty(find(x(y>(L-2))>(0.5*slotW+delta),1))
            continue;
        end
        
        plot(B(1,:),B(2,:));
        plot(BL(1,:),BL(2,:),'r');
        plot(BR(1,:),BR(2,:),'r');
        
        n=n+1;
        Path_x(n,:)=B(1,:);
        Path_y(n,:)=B(2,:);
        Sum_k(n)=sum(k);
        Arc(n)=sum(S_sqr.^0.5);
    end
end
if n~=0
    J=0.5*Arc+0.5*Sum_k; %cost function;
    [~,ind]=min(J);
    FinalPath_x=Path_x(ind,:);
    FinalPath_y=Path_y(ind,:);
    plot(FinalPath_x,FinalPath_y,'y.');
end