function [S_region, Line_In]=getSearchRegion(theta, car, var_predict)
global_varibles;
theta=-pi/2-theta;
cam(1)=car(1)+L_wheel_cam*sin(theta);
cam(2)=car(2)+L_wheel_cam*cos(theta);
theta1=theta+sqrt(var_predict);
theta2=theta-sqrt(var_predict);
x1=[-0.5*W_slot-0.15 0.5*W_slot+0.15 0.5*W_slot+0.15 -0.5*W_slot-0.15 ...
    -0.5*W_slot+0.05 0.5*W_slot-0.05 0.5*W_slot-0.05 -0.5*W_slot+0.05]';
z1=[-0.05 -0.05 L_slot+0.05 L_slot+0.05 0.15 0.15 L_slot-0.15 L_slot-0.15]';
X=x1-cam(1);
Z=z1-cam(2);
Px1=round(ox+focal*(X*cos(theta1)-Z*sin(theta1))./(Z*cos(theta1)*cos(phi)-Y*sin(phi)+X*cos(phi)*sin(theta1)));
Pz1=round(oy+focal*(Y*cos(phi)+Z*cos(theta1)*sin(phi)+X*sin(phi)*sin(theta1))./(Z*cos(phi)*cos(theta1)-Y*sin(phi)+X*cos(phi)*sin(theta1)));
Px2=round(ox+focal*(X*cos(theta2)-Z*sin(theta2))./(Z*cos(theta2)*cos(phi)-Y*sin(phi)+X*cos(phi)*sin(theta2)));
Pz2=round(oy+focal*(Y*cos(phi)+Z*cos(theta2)*sin(phi)+X*sin(phi)*sin(theta2))./(Z*cos(phi)*cos(theta2)-Y*sin(phi)+X*cos(phi)*sin(theta2)));
Px=zeros(size(Px1));
Pz=zeros(size(Pz1));
centroid_x=mean([Px1(1:4);Px2(1:4)]);
centroid_z=mean([Pz1(1:4);Pz2(1:4)]);
for i=1:4
    delta=distance([centroid_x centroid_z],[Px1(i) Pz1(i)])-distance([centroid_x centroid_z],[Px2(i) Pz2(i)]);
    if delta>0
        Px(i)=Px1(i); Pz(i)=Pz1(i);
    else
        Px(i)=Px2(i); Pz(i)=Pz2(i);
    end
end
centroid_x=mean([Px1(5:8) Px2(5:8)]);
centroid_z=mean([Pz1(5:8) Pz2(5:8)]);
for i=5:8
    delta=distance([centroid_x centroid_z],[Px1(i) Pz1(i)])-distance([centroid_x centroid_z],[Px2(i) Pz2(i)]);
    if delta<0
        Px(i)=Px1(i); Pz(i)=Pz1(i);
    else
        Px(i)=Px2(i); Pz(i)=Pz2(i);
    end
end
% S_region{1}=0.5*[Px(1)+Px(5) Px(2)+Px(6);Pz(1)+Pz(5) Pz(2)+Pz(6)];
% S_region{2}=0.5*[Px(2)+Px(6) Px(3)+Px(7);Pz(2)+Pz(6) Pz(3)+Pz(7)];
% S_region{3}=0.5*[Px(3)+Px(7) Px(4)+Px(8);Pz(3)+Pz(7) Pz(4)+Pz(8)];
% S_region{4}=0.5*[Px(4)+Px(8) Px(1)+Px(5);Pz(4)+Pz(8) Pz(1)+Pz(5)];

S_region=0.5*[Px(1)+Px(5) Pz(1)+Pz(5); ...
              Px(2)+Px(6) Pz(2)+Pz(6); ...
              Px(3)+Px(7) Pz(3)+Pz(7); ...
              Px(4)+Px(8) Pz(4)+Pz(8)];
Line_In=zeros(4,1);
for i=1:4
    if i==4
        j=1;
    else j=i+1;
    end
    xy1=(S_region(i,1)-S_region(j,1))/(S_region(i,2)-S_region(j,2))*(1-S_region(i,2))+S_region(i,1);
    xy480=(S_region(i,1)-S_region(j,1))/(S_region(i,2)-S_region(j,2))*(480-S_region(i,2))+S_region(i,1);
    yx1=(S_region(i,2)-S_region(j,2))/(S_region(i,1)-S_region(j,1))*(1-S_region(i,1))+S_region(i,2);
    yx640=(S_region(i,2)-S_region(j,2))/(S_region(i,1)-S_region(j,1))*(640-S_region(i,1))+S_region(i,2);
    if (xy1>-100 && xy1<740) || (xy480>-100 && xy480<740) || (yx1>-100 && yx1<580) || (yx640>-100 && yx640<580)
        Line_In(i)=1;
    end
end
        
    
% for i=1:4
%     if S_region(i,1)>1 && S_region(i,1)<640 && S_region(i,2)>1 && S_region(i,2)<480
%         Line_In(i)=1;
%         if i-1==0
%             Line_In(4)=1;
%         else
%             Line_In(i-1)=1;
%         end
%     end
% end
% S_region{1}=[Px([1 2 6 5]), Pz([1 2 6 5])];
% S_region{2}=[Px([2 3 7 6]), Pz([2 3 7 6])];
% S_region{3}=[Px([3 4 8 7]), Pz([3 4 8 7])];
% S_region{4}=[Px([4 1 5 8]), Pz([4 1 5 8])];
end