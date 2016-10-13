% clear all;
% reverse_parking_status
% 1: program runs okay and exit properly
% 5: initial user input is not valid (obsolated, not in use any longer)
% 6: Not able to initialize camera
% 7: Not able to locate intial position due to lack of information in x or z
% 8: No feasible path available
% 9: Abort by remote driver
reverse_parking_status=1;
folder_ind=1;
folder_exist=exist(['D:\DXX\Self Parking\code\real run\test' int2str(folder_ind)],'dir');
while folder_exist~=0
  folder_ind=folder_ind+1;
  folder_exist=exist(['D:\DXX\Self Parking\code\real run\test' int2str(folder_ind)],'dir');
end
mkdir(['D:\DXX\Self Parking\code\real run\test' int2str(folder_ind)]);
save_dir=['D:\DXX\Self Parking\code\real run\test',int2str(folder_ind)];

RidgeParameters2;
global_varibles;
InitializationGlobalVariable;
buffer=0.0; consec_check=0; pre_check=0;pre_check_d3=0;consec_d3=0;resetcounter=0;
consec_d24=0;pathcounter=0;reset_readjust_flag=0;dist2travel=0;pre_segment=0;
check_readjust=1;
check_collition=1;
targetPhai=atan(L_wheels/Rmin);
% k_s=2; k0_s=0.0; P_s=1; Q_s=.15/pi();
% k_s=2; k0_s=1; P_s=1; Q_s=5/pi();exp_n=10;
% k_s=0.5; k0_s=0; P_s=0.2; Q_s=1;exp_n=10;
k_s=0.5; k0_s=0; P_s=0.2; Q_s=0.5;exp_n=10;
%plot car park slot
% plotCarPark;
%initialize simulation
reach=0;n=1;L_slot_update_flag=1;
v_t=0;
phai_t=0;OMEGA=0.45; TravelDist=0.01;
% PositiveS=0.01; NegtiveS=0.01; PreviousPositiveS=0; PreviousNegtiveS=0;
x_slot_o=0;
z_slot_o=0;
homing;
abt;
prompt='Car Direction? L/R: ';
usr_ip=1;
while (usr_ip==1)
    result=input(prompt,'s');
    if ~isempty(result)
        if result=='l' || result=='L'
            direction=1; theta_pre=-pi/4; usr_ip=0;
        elseif result=='r' || result=='R'
            direction=2; theta_pre=-3*pi/4; usr_ip=0;
        else
            disp('Not valid input');
            disp('Pls re-enter');
%             reverse_parking_status=5; %initial user input is not valid
%             return;
        end
    else
        reverse_parking_status=5; %initial user input is not valid
        disp('Please provide the car direction');
%         return;
    end
end

[cam_status, vid]=InitializVideo;
if cam_status==0
    disp 'Not able to initialize camera';
    reverse_parking_status=6; %Not able to initialize camera
    return;
end
trigger(vid);
Img=getdata(vid);
% imshow(Img,[]);
imwrite(Img,strcat(save_dir,'\FrameLog',int2str(1),'.jpg'),'jpg');
% Img=CamSim(theta_o(1),[x_car_o(1) z_car_o(1)],[x_slot_o(1) z_slot_o(1)]);
%%
Img1=double(max(Img,[],3));
kcsditmp=Ridgeness2(Img1,xfil2,ix,yfil2,iy,sy,GsiCenter, GsiMiddle, GsiCorner,ImgSize1, ImgSize2,smoothsize);
[NoofLines, C, E, Fp, PCount, act_line]=sequentialRANSAC_SelfParking2(kcsditmp, Img1, direction,theta_pre);
log_C(:,1)=C; log_E(:,1)=E; log_Fp(:,1)=Fp;
[theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount, theta_pre);
if act_line==1
    z_m_est=1;
else
    z_m_est=0;
end
if isnan(x_m) || isnan(z_m)
    reverse_parking_status=7; % Not able to locate intial position due to lack of information in x or z
    stop(vid1);
    stoppreview(vid1);
    return;
end
pre_phai_t=0;
x_e=x_m;
z_e=z_m;
theta_e=theta_m;
% theta_a=theta_m;x_a=x_m;z_a=z_m;
% [status, waypoints, trajectory, S, manuver]=pathGeneration8(theta_e, x_e, z_e, buffer);
% if status==0
    [status, waypoints, trajectory, S, manuver]=path_reGenveration(theta_e, x_e, z_e, buffer);
    if status==0
        disp('No path found');stop(vid);
        reverse_parking_status=8; % No feasible path available
        stop(vid1);
        stoppreview(vid1);
        return;
    end
% end
pathcounter=pathcounter+1;
log_path{pathcounter}=drawpath;
tic;
manuver(manuver==4)=[];
if size(manuver,1)==1
    manuver=reshape(manuver,length(manuver)/2,2);
end
if manuver(1,1)==0
    v_T(1)=V_max;
else
    v_T(1)=-V_max;
end
if manuver(1,2)==1
    phai_T(1)=targetPhai;
    currentline=0; %0 means circle
elseif manuver(1,2)==2
    phai_T(1)=-targetPhai;
    currentline=0; %0 means circle
else
    phai_T(1)=0;
    currentline=1; %1 means line
end
currentd=manuver(1,1);
for i=1:size(manuver,1)
    if manuver(i,1)==currentd
        dist2travel=S(i)+dist2travel;
    else
        break;
    end
end
t=toc; T(1)=t;
[v_m, phai_m]=get_measurements;
log_v_m(1)=v_m; log_phai_m(1)=phai_m;
log_x_m(1)=x_m; log_z_m(1)=z_m; log_theta_m(1)=theta_m;
% [v_t acc]=targetV_Generation(v_T(1), v_t, S, t);
[phai_t_ausilary omega]=targetPhai_Generation(phai_T(1), phai_t, t);
if currentline==1
    C=trajectory(1,1); E=trajectory(1,2); F=trajectory(1,3);
    xd=(-C*E*z_e+E*E*x_e-.5*C*F)/(E*E+C*C);
    zd=(C*C*z_e-C*E*x_e-0.5*E*F)/(E*E+C*C);
    if waypoints(1,1)~=waypoints(2,1)
        if (waypoints(1,1)<waypoints(2,1) && xd>waypoints(2,1)) || (waypoints(1,1)>waypoints(2,1) && xd<waypoints(2,1))
            xd=waypoints(2,1);
            zd=waypoints(2,2);
        end
    else
        if (waypoints(1,2)<waypoints(2,2) && zd>waypoints(2,2)) || (waypoints(1,2)>waypoints(2,2) && zd<waypoints(2,2))
            xd=waypoints(2,1);
            zd=waypoints(2,2);
        end
    end
    
    if waypoints(1,1)~=waypoints(2,1)
        if (waypoints(1,1)<waypoints(2,1) && xd<waypoints(1,1)) || (waypoints(1,1)>waypoints(2,1) && xd>waypoints(1,1))
            xd=waypoints(1,1);
            zd=waypoints(1,2);
        end
    else
        if (waypoints(1,2)<waypoints(2,2) && zd<waypoints(1,2)) || (waypoints(1,2)>waypoints(2,2) && zd>waypoints(1,2))
            xd=waypoints(1,1);
            zd=waypoints(1,2);
        end
    end
    
    if v_T(1)>0
        TravelDist=sqrt((xd-waypoints(1,1))^2+(zd-waypoints(1,2))^2);
        if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
            acc=-ACC; v_t=sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
        elseif (TravelDist+pre_segment)<V_max^2/2/ACC
            %             if TravelDist~=0
            %                 acc=ACC; v_t=sqrt(TravelDist*2*ACC);
            %             else
            acc=ACC; v_t=acc*t;
            %             end
        elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
            acc=0; v_t=V_max;
        end
    else
        TravelDist=sqrt((xd-waypoints(1,1))^2+(zd-waypoints(1,2))^2);
        if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
            acc=ACC; v_t=-sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
        elseif (TravelDist+pre_segment)<V_max^2/2/ACC
            %             if TravelDist~=0
            %                 acc=-ACC; v_t=-sqrt(TravelDist*2*ACC);
            %             else
            acc=-ACC; v_t=acc*t;
            %             end
        elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
            acc=0; v_t=V_max;
        end
    end
    if abs(v_t)>V_max
        v_t=sign(v_t)*V_max;
    end
    
    theta_d=atan(-C/E);
    if theta_d>0
        if (v_T(1)>0 && waypoints(1,2)<waypoints(2,2)) || (v_T(1)<0 && waypoints(1,2)>waypoints(2,2))
            theta_d=theta_d;
        elseif (v_T(1)>0 && waypoints(1,2)>waypoints(2,2)) || (v_T(1)<0 && waypoints(1,2)<waypoints(2,2))
            theta_d=-pi+theta_d;
        end
    elseif theta_d<0
        if (v_T(1)>0 && waypoints(1,2)>waypoints(2,2)) || (v_T(1)<0 && waypoints(1,2)<waypoints(2,2))
            theta_d=theta_d;
        elseif (v_T(1)>0 && waypoints(1,1)<waypoints(2,1)) || (v_T(1)<0 && waypoints(1,1)>waypoints(2,1))
            theta_d=pi+theta_d;
        end
    else
        if (v_T(1)>0 && waypoints(1,1)<waypoints(2,1)) || (v_T(1)<0 && waypoints(1,1)>waypoints(2,1))
            theta_d=theta_d;
        elseif (v_T(1)>0 && waypoints(1,1)>waypoints(2,1)) || (v_T(1)<0 && waypoints(1,1)<waypoints(2,1))
            theta_d=-pi+theta_d;
        end
    end
else
    xc=trajectory(1,1); zc=trajectory(1,2); R=trajectory(1,3);
    dumy_x=R*(xc-x_e)/sqrt((xc-x_e)^2+(zc-z_e)^2);
    dumy_z=R*(zc-z_e)/sqrt((xc-x_e)^2+(zc-z_e)^2);
    xd1=xc+dumy_x; zd1=zc+dumy_z;
    xd2=xc-dumy_x; zd2=zc-dumy_z;
    if ((xd1-x_e)^2+(zd1-z_e)^2)<((xd2-x_e)^2+(zd2-z_e)^2)
        xd=xd1; zd=zd1;
    else xd=xd2; zd=zd2;
    end
    theta1=atan2((waypoints(1,2)-zc)/R, (waypoints(1,1)-xc)/R);
    theta2=atan2((waypoints(2,2)-zc)/R, (waypoints(2,1)-xc)/R);
    if abs(theta1-theta2)>pi
        if theta1>theta2
            theta2=2*pi+theta2;
        else
            theta1=2*pi+theta1;
        end
    end
    if theta1>theta2
        theta_f=atan2((zd-zc)/R, (xd-xc)/R);
        if abs(theta_f-theta2)>pi
            if theta_f>theta2
                theta_f=-2*pi+theta_f;
            else
                theta_f=2*pi+theta_f;
            end
        end
        if theta_f<theta2
            xd=waypoints(2,1);
            zd=waypoints(2,2);
            theta_f=theta2;
        end
        
        if abs(theta_f-theta1)>pi
            if theta_f>theta1
                theta_f=-2*pi+theta_f;
            else
                theta_f=2*pi+theta_f;
            end
        end
        if theta_f>theta1
            xd=waypoints(1,1);
            zd=waypoints(1,2);
            theta_f=theta1;
        end
        
    else
        theta_f=atan2((zd-zc)/R, (xd-xc)/R);
        if abs(theta_f-theta2)>pi
            if theta_f>theta2
                theta_f=-2*pi+theta_f;
            else
                theta_f=2*pi+theta_f;
            end
        end
        if theta_f>theta2
            xd=waypoints(2,1);
            zd=waypoints(2,2);
            theta_f=theta2;
        end
        
        if abs(theta_f-theta1)>pi
            if theta_f>theta1
                theta_f=-2*pi+theta_f;
            else
                theta_f=2*pi+theta_f;
            end
        end
        if theta_f<theta1
            xd=waypoints(1,1);
            zd=waypoints(1,2);
            theta_f=theta1;
        end
        
    end
    
    if v_T(1)>0
        TravelDist=abs(theta_f-theta1)*Rmin;
        if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
            acc=-ACC; v_t=sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
        elseif (TravelDist+pre_segment)<V_max^2/2/ACC
            %             if TravelDist~=0
            %                 acc=ACC; v_t=sqrt(TravelDist*2*ACC);
            %             else
            acc=ACC; v_t=acc*t;
            %             end
        elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
            acc=0; v_t=V_max;
        end
    elseif v_T(1)<0
        TravelDist=abs(theta_f-theta1)*Rmin;
        if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
            acc=ACC; v_t=-sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
        elseif (TravelDist+pre_segment)<V_max^2/2/ACC
            %             if TravelDist~=0
            %                 acc=-ACC; v_t=-sqrt(TravelDist*2*ACC);
            %             else
            acc=-ACC; v_t=acc*t;
            %             end
        elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
            acc=0; v_t=-V_max;
        end
    end
    if abs(v_t)>V_max
        v_t=sign(v_t)*V_max;
    end
    
    %     theta_d=atan(-(xd-xc)/(zd-zc));
    if (theta1>theta2 && v_T(1)>0) || (theta1<theta2 && v_T(1)<0)
        theta_d=theta_f-0.5*pi;
    elseif (theta1>theta2 && v_T(1)<0) || (theta1<theta2 && v_T(1)>0)
        theta_d=theta_f+0.5*pi;
    end
    if theta_d>0
        theta_d=theta_d-2*pi;
    elseif theta_d<-pi
        theta_d=theta_d+2*pi;
    end
end

if (waypoints(1,1)~=waypoints(2,1) && xd==waypoints(2,1)) || (waypoints(1,1)==waypoints(2,1) && zd==waypoints(2,2))
    manuver(1,:)=[4 4];
    waypoints(1,:)=[];
    trajectory(1,:)=[];
    if manuver(2,1)==currentd
       pre_segment=pre_segment+S(1);
    else
        currentd=manuver(2,1);
        pre_segment=0;
        dist2travel=0;
        for i=2:size(manuver,1)
            if manuver(i,1)==currentd
                dist2travel=dist2travel+S(i);
            else
                break;
            end
        end
    end
    S(1)=[];
end

% v_t=acc*t;
% v_a=v_t+normrnd(0,sqrt(var_v),[1,1]);
% v_m=v_a+normrnd(0,sqrt(var_w_v),[1,1]);%read from the car in real situation
v_e=v_m;
phai_e=phai_m;

k_s=abs(v_e)/L_wheels*10;
theta_error=theta_e-theta_d;
y_error=-sin(theta_d)*(x_e-xd)+cos(theta_d)*(z_e-zd);
ydot_error=v_e*sin(theta_error);
s=ydot_error+k_s*y_error+k0_s*sign(y_error)*sign(theta_error)*(theta_error);
% phai_t=atan(-Q_s*(exp(exp_n*abs(s))-1)*atan(s/P_s)+tan(phai_T(1)));
phai_t=atan(-Q_s*atan(s/P_s)-k_s*L_car/v_e*tan(theta_error)+tan(phai_T(1)));
phai_t_s(1)=phai_t;
if isnan(phai_t)
    phai_t=phai_T(1);
end
if (phai_t-pre_phai_t)>t*OMEGA
    phai_t=pre_phai_t+t*OMEGA;
elseif (phai_t-pre_phai_t)<-t*OMEGA
    phai_t=pre_phai_t-t*OMEGA;
end 
sendtarget(v_t, phai_t);
pre_phai_t=phai_t;
log_phai_t(1)=phai_t;
log_v_t(1)=v_t;

X_e(:,1)=[x_e;z_e;theta_e;cos(theta_e)*v_e;sin(theta_e)*v_e;tan(phai_e)*v_e/L_wheels;v_e;phai_e];
P=eye(8)*.01; W=eye(8); Q=zeros(8,8); Q(7,7)=var_v; Q(8,8)=var_phai;
X_a(:,1)=X_e(:,1);
I=eye(8);
% plotCar(theta_a,x_a,z_a,phai_a);hold on;
% plot(X_a(1,:), X_a(2,:),'r');
% plot(X_e(1,:), X_e(2,:),'b');
% hold off;
% frame=getframe(gcf);
% im=frame2im(frame);
% imwrite(im,strcat('C:\Users\a0107257\Documents\MATLAB\ImageProcessingLearning\Pictures\Self Parking\code\Simulation\Animation\Frame',int2str(1),'.jpg'),'jpg');

%%
tstart=tic;
stop_all_flag=1;
while reach==0 && stop_all_flag==1
    tloop=tic;
    stop_all_flag=get_stop_all_flag;
    n=n+1;
    x_b=X_e(1,n-1)+X_e(4,n-1)*t;
    z_b=X_e(2,n-1)+X_e(5,n-1)*t;
    theta_b=X_e(3,n-1)+X_e(6,n-1)*t;
    v_b=X_e(7,n-1)+acc*t;
    if (acc>0 && v_b>v_T(n-1)) || (acc<0 && v_b<v_T(n-1))
        v_b=v_T(n-1);
    end
    %     phai_b=X_e(8,n-1)+omega*t;
    %     if (omega>0 && phai_b>phai_T(n-1)) || (omega<0 && phai_b<phai_T(n-1))
    %         phai_b=phai_T(n-1);
    %     end
    phai_b=X_e(8,n-1);
    X_b(:,n)=[x_b; z_b; theta_b; ...
        cos(X_e(3,n-1))*X_e(7,n-1); sin(X_e(3,n-1))*X_e(7,n-1); tan(X_e(8,n-1))*X_e(7,n-1)/L_wheels; ...
        v_b; phai_b;];
    A=zeros(8,8);
    A(1)=1; A(1,4)=t; A(2,2)=1; A(2,5)=t; A(3,3)=1; A(3,6)=t;
    A(4,3)=-X_e(7,n-1)*sin(X_e(3,n-1));A(4,7)=cos(X_e(3,n-1));
    A(5,3)=X_e(7,n-1)*cos(X_e(3,n-1)); A(5,7)=sin(X_e(3,n-1));
    A(6,7)=tan(X_e(8,n-1))/L_wheels; A(6,8)=X_e(7,n-1)/L_wheels/cos(X_e(8,n-1))^2;
    A(7,7)=1; A(8,8)=1;
    P_b=A*P*A'+W*Q*W';
    
    tmp=tic;
    if check_collition==1
        collitionflag=checkcollition(x_b, z_b, theta_b, buffer);
        if collitionflag==1 %collition
            v_t=0;
            sendtarget(0, phai_m);
            buffer=0.2;
            [v_m, ~]=get_measurements;
            while abs(v_m)>0.05
                [v_m, ~]=get_measurements;
            end
            check_collition=0;
            tmp=tic;
            %         [status, waypoints, trajectory, S, manuver]=pathGeneration8(theta_e, x_e, z_e, buffer);
            %         if status==0;
            %             [status, waypoints, trajectory, S, manuver]=path_reGenveration(theta_e, x_e, z_e, buffer);
            %             if status==0
            %                 disp('No path found');
            %                 stop(vid);
            %                 return;
            %             end
            %         end
        end
    else
        collitionflag=0;
    end
    %     v_a1=v_a;
    %     v_a=v_t+normrnd(0,sqrt(var_v),[1,1]);
    %     phai_a1=phai_a;
    %     phai_a=phai_t+normrnd(0,sqrt(var_phai),[1,1]);
    %     x_a=x_a+cos(theta_a)*0.5*(v_a1+v_a)*t;
    %     z_a=z_a+sin(theta_a)*0.5*(v_a1+v_a)*t;
    %     theta_a=theta_a+tan(0.5*(phai_a+phai_a1))*0.5*(v_a1+v_a)/L_wheels*t;
    %     X_a(:,n)=[x_a; z_a; theta_a; ...
    %         cos(theta_a)*0.5*(v_a1+v_a); sin(theta_a)*0.5*(v_a1+v_a); tan(0.5*(phai_a+phai_a1))*0.5*(v_a1+v_a)/L_wheels; ...
    %         v_a; phai_a;];
    %     if mod(n, 10)==0
    
    %     v_m=v_a+normrnd(0,sqrt(var_w_v),[1,1]);%read from the car in real situation
    %     phai_m=phai_a+normrnd(0,sqrt(var_w_phai),[1,1]);%read from the car in real situation
    
    trigger(vid);
    Img=getdata(vid);
    imwrite(Img,strcat(save_dir,'\FrameLog',int2str(n),'.jpg'),'jpg');
    %     imshow(Img,[]);toc
    %     Img=CamSim(theta_a,[x_a z_a],[x_slot_o z_slot_o]);%take the picture in real situation
    Img1=double(max(Img,[],3));
    kcsditmp=Ridgeness2(Img1,xfil2,ix,yfil2,iy,sy,GsiCenter, GsiMiddle, GsiCorner,ImgSize1, ImgSize2,smoothsize);
    reset_flag=0; reset_x_flag=0; reset_z_flag=0;
    if z_m_est==1
        if theta_e>-pi/2
            direction=1;
        else direction=2;
        end
        [NoofLines, C, E, Fp, PCount, act_line]=sequentialRANSAC_SelfParking2(kcsditmp,Img1,direction,theta_e);
        if act_line>1
            if pre_check==1
                consec_check=consec_check+1;
            end
            pre_check=1;
        else pre_check=0;
             consec_check=0;
        end
        if consec_check>5
            [theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount, theta_e);
            consec_check=0; pre_check=0;
            z_m_est=0;
        else
            [S_region, Line_In]=getSearchRegion(theta_b, [x_b z_b], P_b(3,3));
            P_b_debug(n)=P_b(3,3);
            %         kcs_debug{n}=kcsditmp;
            S_region_debug{n}=S_region;
            [NoofLines, C, E, Fp, PCount]=lineAssignment3(kcsditmp, S_region, Line_In);
            log_C(:,n)=C; log_E(:,n)=E; log_Fp(:,n)=Fp;
            if L_slot_update_flag==1 && ~isnan(C(3))
                [theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount, theta_e, x_b, z_b);
                L_slot_update_flag=0;
            else
                [theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount, theta_e);
            end
            reset_flag=0;
        end
    else
        [S_region, Line_In]=getSearchRegion(theta_b, [x_b z_b], P_b(3,3));
        P_b_debug(n)=P_b(3,3);
        %         kcs_debug{n}=kcsditmp;
        S_region_debug{n}=S_region;
        [NoofLines, C, E, Fp, PCount]=lineAssignment3(kcsditmp, S_region, Line_In);
        log_C(:,n)=C; log_E(:,n)=E; log_Fp(:,n)=Fp;
        if ~isnan(C(3))
            pre_check_d3=1;
            if pre_check_d3==1
                consec_d3=consec_d3+1;
            end
        else
            pre_check_d3=0;
            consec_d3=0;
        end
        [theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount, theta_e);
        if ~isnan(C(3)) && consec_d3<=3
            L_slottmp(consec_d3)=PCount(3)*(-z_m+L_slot-0.05+z_b+0.05);
            PCounttmp(consec_d3)=PCount(3);
        end
        if consec_d3==3
            reset_z_flag=1;
            L_slot=sum(L_slottmp)/sum(PCounttmp);
            [theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount, theta_e);
        else
            reset_z_flag=0;
        end
        
        if ~isnan(C(2)) && ~isnan(C(4))
            pre_check_d24=1;
            if pre_check_d24==1
                consec_d24=consec_d24+1;
            end
        else
            pre_check_d24=0;
            consec_d24=0;
        end
        if consec_d24==3
            reset_x_flag=1;
        else
            reset_x_flag=0;
        end
%         if L_slot_update_flag==1 && ~isnan(C(3))
%             [theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount, x_b, z_b);
%             L_slot_update_flag=0;
%         else
%             [theta_m, x_m, z_m]=getPerspective(C, E, Fp, PCount);
%         end
    end
    log_x_m(n)=x_m; log_z_m(n)=z_m; log_theta_m(n)=theta_m;
    
    [v_m, phai_m]=get_measurements;
    log_v_m(n)=v_m; log_phai_m(n)=phai_m;
    
    H=zeros(5,8); H(1,7)=1; H(2,8)=1; H(3,1)=1; H(4,2)=1; H(5,3)=1;
    R=diag([var_w_v,var_w_phai,var_w_x,var_w_y,var_w_theta]);
    z=[v_m; phai_m; x_m; z_m; theta_m];
    Z(:,n)=z;
    indtmp=[1 2 3 4 5];
    if isnan(x_m)
        %         H(3,:)=NaN; R(3,:)=NaN;
        indtmp(3)=NaN;
    end
    if isnan(z_m)
        %         H(4,:)=NaN; R(4,:)=NaN;
        indtmp(4)=NaN;
    end
    if isnan(theta_m)
        %         H(5,:)=NaN; R(5,:)=NaN;
        indtmp(5)=NaN;
    end
    indtmp(isnan(indtmp))=[];
    %     H(isnan(H))=[]; R(isnan(R))=[]; z(isnan(z))=[];
    H=H(indtmp,:); R=R(indtmp,:); R=R(:,indtmp); z=z(indtmp,:);
    V=eye(size(H,1));
    K=P_b*H'*inv((H*P_b*H'+V*R*V'));
    X_e(:,n)=X_b(:,n)+K*(z-H*X_b(:,n));
    P=(I-K*H)*P_b;
    if reset_z_flag==1 || reset_x_flag==1 || reset_flag==1 || collitionflag==1 || reset_readjust_flag==1
        if isnan(x_m)
            x_m=X_e(1,n);
        end
        if isnan(z_m)
            z_m=X_e(2,n);
        end
        if isnan(theta_m)
            theta_m=X_e(3,n);
        end
        X_e(:,n)=[x_m z_m theta_m cos(theta_m)*v_m sin(theta_m)*v_m tan(phai_m)/L_wheels*v_m v_m phai_m]';
        P=eye(8)*.01;
        resetcounter=resetcounter+1;
    end
    x_e=X_e(1,n);
    z_e=X_e(2,n);
    theta_e=X_e(3,n);
    v_e=X_e(7,n);
    phai_e=X_e(8,n);
    if collitionflag==1
        z_e
        %         [status, waypoints, trajectory, S, manuver]=pathGeneration8(theta_e, x_e, z_e, buffer);
        %         if status==0;
        [status, waypoints, trajectory, S, manuver]=path_reGenveration(theta_e, x_e, z_e, buffer);
        if status==0
            disp('No path found');
            reverse_parking_status=8; % No feasible path available
            stop(vid1);
            stoppreview(vid1);
            return;
        end
        %         end
        pathcounter=pathcounter+1;
        log_path{pathcounter}=drawpath;
        dist2travel=0;
        pre_segment=0;
        currentd=manuver(1,1);
        for i=1:size(manuver,1)
            if manuver(i,1)==currentd
                dist2travel=S(i)+dist2travel;
            else
                break;
            end
        end
        tmp=tic;
    end
    if reset_readjust_flag==1
        n
        waypoints=[0 z_e;0 -0.3;0 L_slot-.65];
        trajectory=[1 0 0; 1 0 0];
        S=[abs(-0.3-z_e); L_slot-0.3];
        manuver=[0 3;1 3];
        status=1;
        pathcounter=pathcounter+1;
        log_path{pathcounter}=drawpath;
        dist2travel=0;
        pre_segment=0;
        currentd=manuver(1,1);
        for i=1:size(manuver,1)
            if manuver(i,1)==currentd
                dist2travel=S(i)+dist2travel;
            else
                break;
            end
        end
        tmp=tic;
    end
    %     if z_e<L_slot-0.55
    manuver(manuver==4)=[];
    if size(manuver,1)==1
        manuver=reshape(manuver,length(manuver)/2,2);
    end
    if manuver(1,1)==0
        v_T(n)=V_max;
    else
        v_T(n)=-V_max;
    end
    if manuver(1,2)==1
        phai_T(n)=targetPhai;
        currentline=0; %0 means circle
    elseif manuver(1,2)==2
        phai_T(n)=-targetPhai;
        currentline=0; %0 means circle
    else
        phai_T(n)=0;
        currentline=1; %1 means line
    end
    t=toc(tmp);
    T(n)=t;
    %                 t=0.35;
    %         [v_t acc]=targetV_Generation(v_T(n), v_t, S, t);
    %         [phai_t_ausilary omega]=targetPhai_Generation(phai_T(n), phai_t, t);
    if currentline==1
        C=trajectory(1,1); E=trajectory(1,2); F=trajectory(1,3);
        xd=(-C*E*z_e+E*E*x_e-.5*C*F)/(E*E+C*C);
        zd=(C*C*z_e-C*E*x_e-0.5*E*F)/(E*E+C*C);
        if waypoints(1,1)~=waypoints(2,1)
            if (waypoints(1,1)<waypoints(2,1) && xd>waypoints(2,1)) || (waypoints(1,1)>waypoints(2,1) && xd<waypoints(2,1))
                xd=waypoints(2,1);
                zd=waypoints(2,2);
            end
        else
            if (waypoints(1,2)<waypoints(2,2) && zd>waypoints(2,2)) || (waypoints(1,2)>waypoints(2,2) && zd<waypoints(2,2))
                xd=waypoints(2,1);
                zd=waypoints(2,2);
            end
        end
        
        if waypoints(1,1)~=waypoints(2,1)
            if (waypoints(1,1)<waypoints(2,1) && xd<waypoints(1,1)) || (waypoints(1,1)>waypoints(2,1) && xd>waypoints(1,1))
                xd=waypoints(1,1);
                zd=waypoints(1,2);
            end
        else
            if (waypoints(1,2)<waypoints(2,2) && zd<waypoints(1,2)) || (waypoints(1,2)>waypoints(2,2) && zd>waypoints(1,2))
                xd=waypoints(1,1);
                zd=waypoints(1,2);
            end
        end
        
        if v_T(n)>0
            TravelDist=sqrt((xd-waypoints(1,1))^2+(zd-waypoints(1,2))^2);
            if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
                acc=-ACC; v_t=sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
            elseif (TravelDist+pre_segment)<V_max^2/2/ACC
                %                     if TravelDist~=0
                %                         acc=ACC; v_t=sqrt(TravelDist*2*ACC);
                %                     else
                acc=ACC; v_t=v_t+acc*t;
                %                     end
            elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
                acc=0; v_t=V_max;
            end
        elseif v_T(n)<0
            TravelDist=sqrt((xd-waypoints(1,1))^2+(zd-waypoints(1,2))^2);
            if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
                acc=ACC; v_t=-sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
            elseif (TravelDist+pre_segment)<V_max^2/2/ACC
                %                     if TravelDist~=0
                %                         acc=-ACC; v_t=-sqrt(TravelDist*2*ACC);
                %                     else
                acc=-ACC; v_t=v_t+acc*t;
                %                     end
            elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
                acc=0; v_t=-V_max;
            end
        end
        if abs(v_t)>V_max
            v_t=sign(v_t)*V_max;
        end
        
        theta_d=atan(-C/E);
        if theta_d>0
            if (v_T(n)>0 && waypoints(1,2)<waypoints(2,2)) || (v_T(n)<0 && waypoints(1,2)>waypoints(2,2))
                theta_d=theta_d;
            elseif (v_T(n)>0 && waypoints(1,2)>waypoints(2,2)) || (v_T(n)<0 && waypoints(1,2)<waypoints(2,2))
                theta_d=-pi+theta_d;
            end
        elseif theta_d<0
            if (v_T(n)>0 && waypoints(1,2)>waypoints(2,2)) || (v_T(n)<0 && waypoints(1,2)<waypoints(2,2))
                theta_d=theta_d;
            elseif (v_T(n)>0 && waypoints(1,1)<waypoints(2,1)) || (v_T(n)<0 && waypoints(1,1)>waypoints(2,1))
                theta_d=pi+theta_d;
            end
        else
            if (v_T(n)>0 && waypoints(1,1)<waypoints(2,1)) || (v_T(n)<0 && waypoints(1,1)>waypoints(2,1))
                theta_d=theta_d;
            elseif (v_T(n)>0 && waypoints(1,1)>waypoints(2,1)) || (v_T(n)<0 && waypoints(1,1)<waypoints(2,1))
                theta_d=-pi+theta_d;
            end
        end
    else
        xc=trajectory(1,1); zc=trajectory(1,2); R=trajectory(1,3);
        dumy_x=R*(xc-x_e)/sqrt((xc-x_e)^2+(zc-z_e)^2);
        dumy_z=R*(zc-z_e)/sqrt((xc-x_e)^2+(zc-z_e)^2);
        xd1=xc+dumy_x; zd1=zc+dumy_z;
        xd2=xc-dumy_x; zd2=zc-dumy_z;
        if ((xd1-x_e)^2+(zd1-z_e)^2)<((xd2-x_e)^2+(zd2-z_e)^2)
            xd=xd1; zd=zd1;
        else xd=xd2; zd=zd2;
        end
        theta1=atan2((waypoints(1,2)-zc)/R, (waypoints(1,1)-xc)/R);
        theta2=atan2((waypoints(2,2)-zc)/R, (waypoints(2,1)-xc)/R);
        if abs(theta1-theta2)>pi
            if theta1>theta2
                theta2=2*pi+theta2;
            else
                theta1=2*pi+theta1;
            end
        end
        if theta1>theta2
            theta_f=atan2((zd-zc)/R, (xd-xc)/R);
            if abs(theta_f-theta2)>pi
                if theta_f>theta2
                    theta_f=-2*pi+theta_f;
                else
                    theta_f=2*pi+theta_f;
                end
            end
            if theta_f<theta2
                xd=waypoints(2,1);
                zd=waypoints(2,2);
                theta_f=theta2;
            end
            
            if abs(theta_f-theta1)>pi
                if theta_f>theta1
                    theta_f=-2*pi+theta_f;
                else
                    theta_f=2*pi+theta_f;
                end
            end
            if theta_f>theta1
                xd=waypoints(1,1);
                zd=waypoints(1,2);
                theta_f=theta1;
            end
            
        else
            theta_f=atan2((zd-zc)/R, (xd-xc)/R);
            if abs(theta_f-theta2)>pi
                if theta_f>theta2
                    theta_f=-2*pi+theta_f;
                else
                    theta_f=2*pi+theta_f;
                end
            end
            if theta_f>theta2
                xd=waypoints(2,1);
                zd=waypoints(2,2);
                theta_f=theta2;
            end
            
            if abs(theta_f-theta1)>pi
                if theta_f>theta1
                    theta_f=-2*pi+theta_f;
                else
                    theta_f=2*pi+theta_f;
                end
            end
            if theta_f<theta1
                xd=waypoints(1,1);
                zd=waypoints(1,2);
                theta_f=theta1;
            end
            
        end
        
        if v_T(n)>0
            TravelDist=abs(theta_f-theta1)*Rmin;
            if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
                acc=-ACC; v_t=sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
            elseif (TravelDist+pre_segment)<V_max^2/2/ACC
                %                     if TravelDist~=0
                %                         acc=ACC; v_t=sqrt(TravelDist*2*ACC);
                %                     else
                acc=ACC; v_t=v_t+acc*t;
                %                     end
            elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
                acc=0; v_t=V_max;
            end
        elseif v_T(n)<0
            TravelDist=abs(theta_f-theta1)*Rmin;
            if abs(dist2travel-pre_segment-TravelDist)<=(v_t^2/2/ACC)
                acc=ACC; v_t=-sqrt(abs(dist2travel-pre_segment-TravelDist)*2*ACC);
            elseif (TravelDist+pre_segment)<V_max^2/2/ACC
                %                     if TravelDist~=0
                %                         acc=-ACC; v_t=-sqrt(TravelDist*2*ACC);
                %                     else
                acc=-ACC; v_t=v_t+acc*t;
                %                     end
            elseif (TravelDist+pre_segment)>=V_max^2/2/ACC
                acc=0; v_t=-V_max;
            end
        end
        if abs(v_t)>V_max
            v_t=sign(v_t)*V_max;
        end
        %     theta_d=atan(-(xd-xc)/(zd-zc));
        if (theta1>theta2 && v_T(n)>0) || (theta1<theta2 && v_T(n)<0)
            theta_d=theta_f-0.5*pi;
        elseif (theta1>theta2 && v_T(n)<0) || (theta1<theta2 && v_T(n)>0)
            theta_d=theta_f+0.5*pi;
        end
        if theta_d>0
            theta_d=theta_d-2*pi;
        elseif theta_d<-pi
            theta_d=theta_d+2*pi;
        end
    end
    if (waypoints(1,1)~=waypoints(2,1) && xd==waypoints(2,1)) || (waypoints(1,1)==waypoints(2,1) && zd==waypoints(2,2))
        manuver(1,:)=[4 4];
        waypoints(1,:)=[];
        trajectory(1,:)=[];
        if manuver(2,1)==currentd
            pre_segment=pre_segment+S(1);
        else
            currentd=manuver(2,1);
            pre_segment=0;
            dist2travel=0;
            for i=2:size(manuver,1)
                if manuver(i,1)==currentd
                    dist2travel=dist2travel+S(i);
                else
                    break;
                end
            end
        end
        S(1)=[];
        buffer=0;
        if length(S)==1
            S(1)=L_slot-0.35-waypoints(1,2);
        end
        check_readjust=1;
        check_collition=1;
    end
    k_s=abs(v_e)/L_wheels*3;
    theta_error=theta_e-theta_d;
    y_error=-sin(theta_d)*(x_e-xd)+cos(theta_d)*(z_e-zd);
    ydot_error=v_e*sin(theta_error);
    s=ydot_error+k_s*y_error+k0_s*sign(y_error)*sign(theta_error)*theta_error;
    %         phai_t=atan(1/v_e*(L_car*(-Q_s*s-P_s*sign(s)-k_s*v_e*sin(theta_error))/(v_e*cos(theta_error)+k0_s*sign(theta_error)*sign(y_error))+v_t*tan(phai_T(n))));
    %         phai_t=atan(1/v_e*(L_car*(-Q_s*atan(s/P_s)-k_s*v_e*sin(theta_error))/(v_e*cos(theta_error)+k0_s*sign(theta_error)*sign(y_error))+v_e*tan(phai_T(n))));
    %         phai_t=atan(-Q_s*(exp(exp_n*abs(s))-1)*atan(s/P_s)+tan(phai_T(n)));
%     phai_t=atan(-Q_s*atan(s/P_s)-k_s*L_car/v_e*tan(theta_error)+tan(phai_T(n)));
    phai_t=atan(-Q_s*atan(s/P_s)-k_s*L_wheels/v_e*tan(theta_error)+tan(phai_T(n)));
    phai_t_s(n)=phai_t;
    if isnan(phai_t)
        phai_t=phai_T(n);
    end
    if (phai_t-pre_phai_t)>t*OMEGA
        phai_t=pre_phai_t+t*OMEGA;
    elseif (phai_t-pre_phai_t)<-t*OMEGA
        phai_t=pre_phai_t-t*OMEGA;
    end
    sendtarget(v_t, phai_t);
    pre_phai_t=phai_t;
    log_phai_t(n)=phai_t;
    log_v_t(n)=v_t;
    log_W_slot(n)=W_slot;
    log_L_slot(n)=L_slot;
    %         pre_phai_t=phai_t;
    %         phai_t_max(n)=phai_t;
    Y_error(n)=y_error; YDOT_error(n)=ydot_error; S_debug(n)=s; THETA_error(n)=theta_error; k_s_Debug(n)=k_s;
    Sdot(n)=-Q_s*s-P_s*sign(s);
    
    %         phai_t=phai_t_ausilary;
    %         plotCarPark;
    %         plotCar(theta_a,x_a,z_a, phai_a);
    %         hold on;
    %         plot(drawpath(:,1),drawpath(:,2),'c');
    %         plot(X_a(1,:), X_a(2,:),'r');
    %         plot(X_e(1,:), X_e(2,:),'b');
    %         hold off;
    %         frame=getframe(gcf);
    %         im=frame2im(frame);
    %         imwrite(im,strcat('C:\Users\a0107257\Documents\MATLAB\ImageProcessingLearning\Pictures\Self Parking\code\Simulation\Animation\Frame',int2str(n),'.jpg'),'jpg');
    %     end
    if z_e<L_slot-0.8+buffer
        reset_readjust_flag=0;
    else
        if abs(x_e)>0.1 || theta_e>pi*(-0.5+4/180) || theta_e<pi*(-.5-4/180)
            if check_readjust==1
                v_t=0;
                sendtarget(0, phai_m);
                buffer=0.2;
                [v_m, ~]=get_measurements;
                while abs(v_m)>0.05
                    [v_m, ~]=get_measurements;
                end
                reset_readjust_flag=1;
                check_readjust=0;
            else reset_readjust_flag=0;
            end
%             [status, waypoints, trajectory, S, manuver]=pathGeneration8(theta_e, x_e, z_e, buffer);
%             if status==0;
%                 [status, waypoints, trajectory, S, manuver]=path_reGenveration(theta_e, x_e, z_e, buffer);
%                 if status==0
%                     disp('No path found');
%                     stop(vid);
%                     return;
%                 end
%             end
%             pathcounter=pathcounter+1;
%             log_path{pathcounter}=drawpath;
        else
            %         plotCarPark;
            %         plotCar(theta_a,x_a,z_a, phai_a); hold on;
            %         plot(drawpath(:,1),drawpath(:,2),'c');
            %         plot(X_a(1,:), X_a(2,:),'r');
            %         plot(X_e(1,:), X_e(2,:),'b');
            %         hold off;
            %         frame=getframe(gcf);
            %         im=frame2im(frame);
            %         imwrite(im,strcat('C:\Users\a0107257\Documents\MATLAB\ImageProcessingLearning\Pictures\Self Parking\code\Simulation\Animation\Frame',int2str(n),'.jpg'),'jpg');
            t=toc(tmp);
            T(n)=t;
            %         t=0.35;
            abt;
            reach=1;
            
        end
    end
    Tloop(n)=toc(tloop);
end
abt;
if reach==1
    reverse_parking_status=1; % parked successfully
else
    reverse_parking_status=9; % Abort by remote driver
end
stop(vid1);
stoppreview(vid1);
save([save_dir '\runningdata.mat']);
tend=toc(tstart);