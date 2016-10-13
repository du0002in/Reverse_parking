function [status, waypoints, trajectory, S, manuver]=pathGeneration7(theta,x,z,buffer)

global_varibles;

min_z=-5; %5meter is based on Rmin=3.7, if Rmin changes, the min_z also need to change.
theta=-pi/2-theta;
d(1)=-z+0.05;
d(2)=-x+W_slot*0.5+0.05;
d(3)=-z+L_slot-0.05;
d(4)=-x-W_slot*0.5-0.05;
L=d(3)-0.5;%0.5 is the distance from car stopping point to line 3
delta=(d(2)+d(4))/2; %car center to car park slot center distance along x
% Rmin=2.33;
% W_slot=d(2)-d(4);
side_torlerence=0.25;
%from car to global
R=[cos(theta) 0 sin(theta);...
    0 1 0;...
    -sin(theta) 0 cos(theta)];

%manuver
%1st column: Driving direction (0 1 4 => forward reverse ignore)
%2nd column: steering direction.
%           (1 2 3 4=> left right straight ignore)
manuver=ones(4,2)*4;
manuver(4,:)=[1 3];

%check whether the last segment can do
x_car=[0.5*W_car -0.5*W_car 0 -0.5*W_car 0.5*W_car];
z_car=[(L_car-L_wheels)*0.5 (L_car-L_wheels)*0.5 0 -L_car+(L_car-L_wheels)*0.5 -L_car+(L_car-L_wheels)*0.5];
if abs(theta)<=0.01
    dist_to_travel=abs(L/cos(theta));
    z_car=z_car+dist_to_travel;
    car_global=R*[x_car; zeros(1,5); z_car];
    x_car_global=car_global(1,:);
    if isempty(find(x_car_global>d(2)-side_torlerence,1)) && isempty(find(x_car_global<d(4)+side_torlerence,1))
        status=1;
        manuver(1:3,:)=4;
        SP=[0 0]; EP=[x_car_global(3) L_slot+0.5-z];
        B=[0 0];C=[0 0];D=[0 0];
        waypoints=[SP+[x z];EP+[x z]];
        if isempty(find(waypoints(:,2)<min_z,1))
            drawpath=waypoints;
            %     [cc ee ff]=lines(SP+[x z],EP+[x z]);
            [cc ee ff]=lines([SP(1) EP(1)]+x,[SP(2) EP(2)]+z);
            trajectory=[cc ee ff];
            %     S=[distance(SP,EP),0];
            S(1)=distance(SP,EP);
            
        else
            status=0;
        end
    else
        status=0;
    end
else status=0;
end

%check using last two segment as path
if status==0
    if theta>=0
        xc0=-Rmin*cos(theta);
        zc0=Rmin*sin(theta);
        left_cut_point=zc0-sqrt((Rmin-W_car/2)^2-(d(4)-xc0)^2);
        if ~isreal(left_cut_point)
            status=0;
        else
            if left_cut_point>(d(1)-buffer)
                status=0;
            else
                D=[xc0+Rmin, zc0];
                if (D(1)-W_car/2)>=(d(4)+side_torlerence) && (D(1)+W_car/2)<=(d(2)-side_torlerence)
                    status=1;
                    if distance([0 0],D)<0.1
                        SP=[0 0]; EP=[D(1), L_slot+0.5-z];
                        manuver(1:2,:)=4;
                        manuver(3,:)=[4 4];
                        status=1;
                        waypoints=[SP+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=waypoints;
                            %                         [cc ee ff]=lines(SP+[x z],EP+[x z]);
                            [cc ee ff]=lines([SP(1) EP(1)]+x,[SP(2) EP(2)]+z);
                            trajectory=[cc ee ff];
                            %                         S=[distance(SP,EP),0];
                            S(1)=distance(SP,EP);
                        else status=0;
                        end
                    else
                        manuver(1:2,:)=4;
                        manuver(3,:)=[1 2];
                        SP=[0 0]; EP=[D(1), L_slot+0.5-z];
                        C=[0 0]; B=[0 0];
                        angleCD=asin(0.5*distance(C,D)/Rmin)*2;
                        angle=(-angleCD:0.01:0)';
                        xp=Rmin*cos(angle); zp=Rmin*sin(angle);
                        arc_CD=[xc0+xp+x zc0+zp+z];
                        waypoints=[SP+[x z];D+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=[SP+[x z];arc_CD;D+[x z];EP+[x z]];
                            %                         [cc ee ff]=lines(D+[x z],EP+[x z]);
                            [cc ee ff]=lines([D(1) EP(1)]+x,[D(2) EP(2)]+z);
                            trajectory=[xc0+x zc0+z Rmin;cc ee ff];
                            %                         S=[distance(D,EP)+Rmin*abs(angleCD),0];
                            S=[Rmin*abs(angleCD);distance(D,EP)];
                        else status=0;
                        end
                    end
                else
                    status=0;
                end
            end
        end
    else
        xc0=Rmin*cos(theta);
        zc0=-Rmin*sin(theta);
        right_cut_point=zc0-sqrt((Rmin-W_car/2)^2-(d(2)-xc0)^2);
        if ~isreal(right_cut_point)
            status=0;
        else
            if right_cut_point>(d(1)-buffer)
                status=0;
            else
                D=[xc0-Rmin, zc0];
                if (D(1)-W_car/2)>=(d(4)+side_torlerence) && (D(1)+W_car/2)<=(d(2)-side_torlerence)
                    if distance([0 0],D)<0.1
                        SP=[0 0];EP=[D(1), L_slot+0.5-z];
                        manuver(1:2,:)=4;
                        manuver(3,:)=[4 4];
                        status=1;
                        waypoints=[SP+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=waypoints;
                            %                         [cc ee ff]=lines(SP+[x z],EP+[x z]);
                            [cc ee ff]=lines([SP(1) EP(1)]+x,[SP(2) EP(2)]+z);
                            trajectory=[cc ee ff];
                            %                         S=[distance(SP,EP),0];
                            S(1)=distance(SP,EP);
                        else status=0;
                        end
                    else
                        status=1;
                        manuver(1:2,:)=4;
                        manuver(3,:)=[1 1];
                        SP=[0 0];EP=[D(1) L_slot+0.5-z];
                        C=[0 0];B=[0 0];
                        angleCD=asin(0.5*distance(C,D)/Rmin)*2;
                        angle=(pi+angleCD:-0.01:pi)';
                        xp=Rmin*cos(angle); zp=Rmin*sin(angle);
                        arc_CD=[xc0+xp+x zc0+zp+z];
                        waypoints=[SP+[x z];D+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=[SP+[x z];arc_CD;D+[x z];EP+[x z]];
                            %                         [cc ee ff]=lines(D+[x z],EP+[x z]);
                            [cc ee ff]=lines([D(1) EP(1)]+x,[D(2) EP(2)]+z);
                            trajectory=[xc0+x zc0+z Rmin;cc ee ff];
                            %                         S=[distance(D,EP)+Rmin*abs(angleCD),0];
                            S=[Rmin*abs(angleCD);distance(D,EP)];
                        else status=0;
                        end
                    end
                else
                    status=0;
                end
            end
        end
    end
end

if status==0
    if theta>=0
        status=1;
        manuver(1:2,:)=4;
        manuver(3,:)=[0 1];
        xc0=x-Rmin*cos(theta);
        zc0=z-Rmin*sin(theta);
        D=[xc0-Rmin, zc0];
        if abs(D(1))<0.05
            EP=[D(1), L_slot+0.5];
            B=[x, z]; C=[x, z];SP=[x z];
            angle=(theta:0.01:pi)';
            xp=Rmin*cos(angle); zp=Rmin*sin(angle);
            arc_CD=[xc0+xp zc0+zp];
            waypoints=[SP; D; EP];
            drawpath=[SP; arc_CD; D; EP];
            [cc ee ff]=lines([D(1) EP(1)], [D(2) EP(2)]);
            trajectory=[xc0 zc0 Rmin; cc ee ff];
            S=[Rmin*abs(pi-theta); distance(D, EP)];
        else status=0;
        end
    else
       status=1;
       manuver(1:2,:)=4;
       manuver(3,:)=[0 2];
       xc0=x-Rmin*cos(theta);
       zc0=z+Rmin*sin(theta);
       D=[xc0+Rmin zc0];
       if abs(D(1))<0.05
           EP=[D(1) L_slot+0.5];
           B=[x z]; C=[x z];SP=[x z];
           angle=(-theta:-0.01:0)';
           xp=Rmin*cos(angle); zp=Rmin*sin(angle);
           arc_CD=[xc0+xp zc0+zp];
           waypoints=[SP; D; EP];
           drawpath=[SP; arc_CD; D; EP];
           [cc ee ff]=lines([D(1) EP(1)], [D(2) EP(2)]);
           trajectory=[xc0 zc0 Rmin; cc ee ff];
           S=[Rmin*abs(theta); distance(D, EP)];
       else status=0;
       end
    end
end

if status==0
    if theta<=0
        a=1/tan(theta);
        if isinf(a)
            status=0;
        else
            b=z-a*x;
            N=b+a*Rmin+Rmin*sqrt(a*a+1);
            if N<sqrt((Rmin-0.5*W_car)^2-(0.5*(W_slot+0.1)-side_torlerence-Rmin)^2)+buffer
%                 status=1;
                SP=[0 0];
                C=[(Rmin-a*(b-N))/(1+a*a)-x a*(Rmin-a*(b-N))/(1+a*a)+b-z];
                xdumy=linspace(SP(1), C(1), ceil(abs(C(1)-SP(1))/0.2));
                zdumy=linspace(SP(2),C(2),length(xdumy));
                flag=checkcollition(xdumy,zdumy,-pi/2-theta, buffer);
                if flag==1
                    status=0;
                else
                    status=1;
                    D=[0-x, N-z];
                    EP=[D(1), L_slot+0.5-z];
                    angleCD=asin(0.5*distance(C,D)/Rmin)*2;
                    angle=(pi+angleCD:-0.01:pi)';
                    xp=Rmin*cos(angle); zp=Rmin*sin(angle);
                    arc_CD=[Rmin-x+xp+x N-z+zp+z];
                    if distance(SP,C)<0.1
                        SP=[0 0];
                        waypoints=[SP+[x z];D+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=[SP+[x z];arc_CD;D+[x z];EP+[x z]];
                            manuver(1:2,:)=4;
                            manuver(3,:)=[1 1];
                            manuver(4,:)=[1 3];
                            %                     [cc ee ff]=lines(D+[x z],EP+[x z]);
                            [cc ee ff]=lines([D(1) EP(1)]+x,[D(2) EP(2)]+z);
                            trajectory=[Rmin N Rmin;cc ee ff];
                            %                     S=[distance(D,EP)+Rmin*abs(angleCD), 0];
                            S=[Rmin*abs(angleCD);distance(D,EP)];
                        else status=0;
                        end
                    else
                        waypoints=[SP+[x z];C+[x z];D+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=[SP+[x z];C+[x z];arc_CD;D+[x z];EP+[x z]];
                            %                     [cc1 ee1 ff1]=lines(SP+[x z],C+[x z]);
                            %                     [cc2 ee2 ff2]=lines(D+[x z],EP+[x z]);
                            [cc1 ee1 ff1]=lines([SP(1) C(1)]+x,[SP(2) C(2)]+z);
                            [cc2 ee2 ff2]=lines([D(1) EP(1)]+x,[D(2) EP(2)]+z);
                            trajectory=[[cc1 ee1 ff1]; ...
                                Rmin N Rmin; ...
                                [cc2 ee2 ff2]];
                            if 0>C(1)
                                manuver(2,:)=[1 3];
                                %                         S=[distance(SP,C)+Rmin*abs(angleCD)+distance(D,EP), 0];
                                S=[distance(SP,C);Rmin*abs(angleCD);distance(D,EP)];
                            else
                                manuver(2,:)=[0 3];
                                %                         S=[Rmin*abs(angleCD)+distance(D,EP), distance(SP,C)];
                                S=[distance(SP,C);Rmin*abs(angleCD);distance(D,EP)];
                            end
                            manuver(1,:)=4;
                            manuver(3,:)=[1 1];
                            manuver(4,:)=[1 3];
                        else status=0;
                        end
                    end
                end
            else
                status=0;
            end
        end
    else
        a=1/tan(theta);
        if isinf(a)
            status=0;
        else
            b=z-a*x;
            N=b-a*Rmin+Rmin*sqrt(a*a+1);
            if N<sqrt((Rmin-0.5*W_car)^2-(0.5*(W_slot+0.1)-side_torlerence-Rmin)^2)+buffer
                %                 status=1;
                SP=[0 0];
                C=[(-Rmin-a*(b-N))/(1+a*a)-x a*(-Rmin-a*(b-N))/(1+a*a)+b-z];
                xdumy=linspace(SP(1), C(1), ceil(abs(C(1)-SP(1))/0.2));
                zdumy=linspace(SP(2),C(2),length(xdumy));
                flag=checkcollition(xdumy,zdumy,-pi/2-theta, buffer);
                if flag==1
                    status=0;
                else
                    status=1;
                    D=[0-x N-z];
                    EP=[D(1) L_slot+0.5-z];
                    angleCD=asin(0.5*distance(C,D)/Rmin)*2;
                    angle=(-angleCD:0.01:0)';
                    xp=Rmin*cos(angle); zp=Rmin*sin(angle);
                    arc_CD=[-Rmin-x+xp+x N-z+zp+z];
                    if distance(SP,C)<0.1
                        SP=[0 0];
                        waypoints=[SP+[x z];D+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=[SP+[x z];arc_CD;D+[x z];EP+[x z]];
                            manuver(1:2,:)=4;
                            manuver(3,:)=[1 -1];
                            manuver(4,:)=[1 3];
                            [cc ee ff]=lines(D+[x z],EP+[x z]);
                            [cc ee ff]=lines([D(1) EP(1)]+x,[D(2) EP(2)]+z);
                            trajectory=[-Rmin N Rmin;[cc ee ff]];
                            %                     S=[distance(D,EP)+Rmin*abs(angleCD), 0];
                            S=[Rmin*abs(angleCD);distance(D,EP)];
                        else status=0;
                        end
                    else
                        waypoints=[SP+[x z];C+[x z];D+[x z];EP+[x z]];
                        if isempty(find(waypoints(:,2)<min_z,1))
                            drawpath=[SP+[x z];C+[x z];arc_CD;D+[x z];EP+[x z]];
                            %                     [cc1 ee1 ff1]=lines(SP+[x z],C+[x z]);
                            %                     [cc2 ee2 ff2]=lines(D+[x z],EP+[x z]);
                            [cc1 ee1 ff1]=lines([SP(1) C(1)]+x,[SP(2) C(2)]+z);
                            [cc2 ee2 ff2]=lines([D(1) EP(1)]+x,[D(2) EP(2)]+z);
                            trajectory=[[cc1 ee1 ff1]; ...
                                -Rmin N Rmin; ...
                                [cc2 ee2 ff2]];
                            if 0<C(1)
                                manuver(2,:)=[1 3];
                                %                         S=[distance(SP,C)+Rmin*abs(angleCD)+distance(D,EP), 0];
                                S=[distance(SP,C);Rmin*abs(angleCD);distance(D,EP)];
                            else
                                manuver(2,:)=[0 3];
                                %                         S=[Rmin*abs(angleCD)+distance(D,EP), distance(SP,C)];
                                S=[distance(SP,C);Rmin*abs(angleCD);distance(D,EP)];
                            end
                            manuver(1,:)=4;
                            manuver(3,:)=[1 2];
                            manuver(4,:)=[1 3];
                        else status=0;
                        end
                    end
                end
            else
                status=0;
            end
        end
    end
end

if status==0
    centers=R*[Rmin -Rmin; 0 0; 0 0];
    x10l=centers(1,1)+x;
    z10l=centers(3,1)+z;
    x10r=centers(1,2)+x;
    z10r=centers(3,2)+z;
    manuver(1,:)=[4 4];
    if (x10r-Rmin)^2<=4*Rmin^2
        x10=x10r;
        z10=z10r;
        manuver(2,:)=[4 2];
        status=1;
        x20=Rmin;
        z20=z10-sqrt(4*Rmin^2-(x10-x20)^2);
        SP=[x z];B=[x z];
        C=0.5*[x10+x20 z10+z20];
        D=[0 z20];
        EP=[0 L_slot+0.5];
        paraB=absolueAngle(B(1), B(2), Rmin, x10, z10);
        paraCc=absolueAngle(C(1), C(2), Rmin, x10, z10);
        if abs(paraCc-paraB)>pi
            if paraCc>paraB
                paraCc=paraCc-2*pi;
            else paraB=paraB-2*pi;
            end
        end
        if paraB<=paraCc
            angle=(paraB:0.01:paraCc)';
            manuver(2,1)=1;
        else
            angle=(paraB:-0.01:paraCc)';
            manuver(2,1)=0;
        end
        xp=Rmin*cos(angle); zp=Rmin*sin(angle);
        arc_BC=[x10+xp z10+zp];
        angletmp=-angle; %convert to car frame
        angletmp=-pi/2-angletmp; %convert to global frame
        for i=1:length(angletmp)
            flag=checkcollition(arc_BC(i,1), arc_BC(i,2), angletmp(i), buffer);
            if flag==1
                status=0;
                break;
            end
        end
        if status==1
            angleCD=asin(0.5*distance(C,D)/Rmin)*2;
            angle=(pi-angleCD:0.01:pi)';
            angletmp=pi-angle; %convert to car frame
            angletmp=-pi/2-angletmp; %convert to global frame
            xp=Rmin*cos(angle); zp=Rmin*sin(angle);
            arc_CD=[x20+xp z20+zp];
            for i=1:length(angletmp)
                flag=checkcollition(arc_CD(i,1), arc_CD(i,2), angletmp(i), buffer);
                if flag==1
                    status=0;
                    break;
                end
            end
            if status==1
                manuver(3,:)=[0 1];
                waypoints=[SP;C;D;EP];
                drawpath=[SP; arc_BC; C; arc_CD; D; EP];
                if min(drawpath(:,2))<-5
                    status=0;
                end
                [cc ee ff]=lines([D(1) EP(1)],[D(2) EP(2)]);
                trajectory=[x10 z10 Rmin; x20 z20 Rmin; cc ee ff;];
                BC=Rmin*abs(paraB-paraCc);
                CD=Rmin*abs(angleCD);
                D_EP=distance(D,EP);
                S=[BC;CD;D_EP];
            end
        end    
    elseif (x10l+Rmin)^2<=4*Rmin^2
        x10=x10l;
        z10=z10l;
        manuver(2,:)=[4 1];
        status=1;
        x20=-Rmin;
        z20=z10-sqrt(4*Rmin^2-(x10-x20)^2);
        SP=[x z];B=[x z];
        C=0.5*[x10+x20 z10+z20];
        D=[0 z20];
        EP=[0 L_slot+0.5];
        paraB=absolueAngle(B(1), B(2), Rmin, x10, z10);
        paraCc=absolueAngle(C(1), C(2), Rmin, x10, z10);
        if abs(paraCc-paraB)>pi
            if paraCc>paraB
                paraCc=paraCc-2*pi;
            else paraB=paraB-2*pi;
            end
        end
        if paraB<=paraCc
            angle=(paraB:0.01:paraCc)';
            manuver(2,1)=0;
        else
            angle=(paraB:-0.01:paraCc)';
            manuver(2,1)=1;
        end
        xp=Rmin*cos(angle); zp=Rmin*sin(angle);
        arc_BC=[x10+xp z10+zp];
        angletmp=pi-angle;
        angletmp=-pi/2-angletmp;
        for i=1:length(angletmp)
            flag=checkcollition(arc_BC(i,1), arc_BC(i,2), angletmp(i), buffer);
            if flag==1
                status=0;
                break;
            end
        end
        if status==1
            angleCD=asin(0.5*distance(C,D)/Rmin)*2;
            angle=(angleCD:-0.01:0)';
            angletmp=-angle;
            angletmp=-pi/2-angletmp;
            xp=Rmin*cos(angle); zp=Rmin*sin(angle);
            arc_CD=[x20+xp z20+zp];
            for i=1:length(angletmp)
                flag=checkcollition(arc_CD(i,1), arc_CD(i,2), angletmp(i), buffer);
                if flag==1
                    status=0;
                    break;
                end
            end
            if status==1
                manuver(3,:)=[0 2];
                waypoints=[SP;C;D;EP];
                drawpath=[SP; arc_BC; C; arc_CD; D; EP];
                if min(drawpath(:,2))<-5
                    status=0;
                end
                [cc ee ff]=lines([D(1) EP(1)],[D(2) EP(2)]);
                trajectory=[x10 z10 Rmin; x20 z20 Rmin; cc ee ff;];
                BC=Rmin*abs(paraB-paraCc);
                CD=Rmin*abs(angleCD);
                D_EP=distance(D,EP);
                S=[BC;CD;D_EP];
            end
        end       
    else status=0;
    end
end


if status==0
    %get the max value for the distacne tt the car can reverse. To prevent the
    %side of the car cutting into the car park edges (left and right).
    tcmax=Inf;
    if sin(theta)~=0
        tctmp(1)=(d(4)-(-0.5*cos(theta)))/sin(theta);
        yctmp(1)=tctmp(1)*cos(theta)+0.5*sin(theta);
        tctmp(2)=(d(2)-(-0.5*cos(theta)))/sin(theta);
        yctmp(2)=tctmp(2)*cos(theta)+0.5*sin(theta);
        tctmp(3)=(d(4)-(0.5*cos(theta)))/sin(theta);
        yctmp(3)=tctmp(3)*cos(theta)-0.5*sin(theta);
        tctmp(4)=(d(2)-(0.5*cos(theta)))/sin(theta);
        yctmp(4)=tctmp(4)*cos(theta)-0.5*sin(theta);
        tcmax=min(tctmp(yctmp>d(1)));
        if isempty(tcmax)
            tcmax=Inf;
        end
    end
    min_Path_length=ones(1,4)*Inf;
    min_tctmp=ones(1,4)*tcmax;
    min_tstmp=ones(1,4)*Inf;
    xc0tmp1=-Rmin*cos(theta);
    zc0tmp1=Rmin*sin(theta);
    xstmp1=d(4)+(W_slot+0.1)/2+Rmin;
    ts=.8:-0.2:-3;
    tc1=-(sin(theta)*(xc0tmp1-xstmp1)+cos(theta)*(zc0tmp1-d(1)-ts))+sqrt((sin(theta)*(xc0tmp1-xstmp1)+cos(theta)*(zc0tmp1-d(1)-ts)).^2-((xc0tmp1-xstmp1)^2+(zc0tmp1-d(1)-ts).^2-4*Rmin^2));
    tc2=-(sin(theta)*(xc0tmp1-xstmp1)+cos(theta)*(zc0tmp1-d(1)-ts))-sqrt((sin(theta)*(xc0tmp1-xstmp1)+cos(theta)*(zc0tmp1-d(1)-ts)).^2-((xc0tmp1-xstmp1)^2+(zc0tmp1-d(1)-ts).^2-4*Rmin^2));
    x_in1=0.5*(sin(theta)*tc1+xc0tmp1+xstmp1);
    y_in1=0.5*(cos(theta)*tc1+zc0tmp1+d(1)+ts);
    path_indx1=find(x_in1<=xstmp1);
    path_indy1=find(y_in1<=(d(1)+ts));
    valid_path_ind1tmp=intersect(path_indx1,path_indy1);
    valid_path_ind1=intersect(valid_path_ind1tmp,find(tc1<=tcmax));
    if ~isempty(valid_path_ind1)
        chordC1=sqrt((tc1(valid_path_ind1)*sin(theta)-x_in1(valid_path_ind1)).^2+(tc1(valid_path_ind1)*cos(theta)-y_in1(valid_path_ind1)).^2);
        chordS1=sqrt((d(4)+0.5*(W_slot+0.1)-x_in1(valid_path_ind1)).^2+(d(1)+ts(valid_path_ind1)-y_in1(valid_path_ind1)).^2);
        Path_length1=abs(tc1(valid_path_ind1))+abs(asin(0.5*chordC1/Rmin)*2*Rmin)+abs(asin(0.5*chordS1/Rmin)*2*Rmin)+abs(d(3)-0.5-d(1)-ts(valid_path_ind1));
        [min_Path_length(1),ind]=min(Path_length1);
        min_tctmp(1)=tc1(valid_path_ind1(ind));
        min_tstmp(1)=ts(valid_path_ind1(ind));
    end
    x_in2=0.5*(sin(theta)*tc2+xc0tmp1+xstmp1);
    y_in2=0.5*(cos(theta)*tc2+zc0tmp1+d(1)+ts);
    path_indx2=find(x_in2<=xstmp1);
    path_indy2=find(y_in2<=(d(1)+ts));
    valid_path_ind2tmp=intersect(path_indx2,path_indy2);
    valid_path_ind2=intersect(valid_path_ind2tmp,find(tc2<=tcmax));
    if ~isempty(valid_path_ind2)
        chordC2=sqrt((tc2(valid_path_ind2)*sin(theta)-x_in2(valid_path_ind2)).^2+(tc2(valid_path_ind2)*cos(theta)-y_in2(valid_path_ind2)).^2);
        chordS2=sqrt((d(4)+0.5*(W_slot+0.1)-x_in2(valid_path_ind2)).^2+(d(1)+ts(valid_path_ind2)-y_in2(valid_path_ind2)).^2);
        Path_length2=abs(tc2(valid_path_ind2))+abs(asin(0.5*chordC2/Rmin)*2*Rmin)+abs(asin(0.5*chordS2/Rmin)*2*Rmin)+abs(d(3)-0.5-d(1)-ts(valid_path_ind2));
        [min_Path_length(2),ind]=min(Path_length2);
        min_tctmp(2)=tc2(valid_path_ind2(ind));
        min_tstmp(2)=ts(valid_path_ind2(ind));
    end
    
    xc0tmp2=Rmin*cos(theta);
    zc0tmp2=-Rmin*sin(theta);
    xstmp2=d(4)+(W_slot+0.1)/2-Rmin;
    ts=.8:-0.2:-3;
    tc1=-(sin(theta)*(xc0tmp2-xstmp2)+cos(theta)*(zc0tmp2-d(1)-ts))+sqrt((sin(theta)*(xc0tmp2-xstmp2)+cos(theta)*(zc0tmp2-d(1)-ts)).^2-((xc0tmp2-xstmp2)^2+(zc0tmp2-d(1)-ts).^2-4*Rmin^2));
    tc2=-(sin(theta)*(xc0tmp2-xstmp2)+cos(theta)*(zc0tmp2-d(1)-ts))-sqrt((sin(theta)*(xc0tmp2-xstmp2)+cos(theta)*(zc0tmp2-d(1)-ts)).^2-((xc0tmp2-xstmp2)^2+(zc0tmp2-d(1)-ts).^2-4*Rmin^2));
    x_in1=0.5*(sin(theta)*tc1+xc0tmp2+xstmp2);
    y_in1=0.5*(cos(theta)*tc1+zc0tmp2+d(1)+ts);
    path_indx1=find(x_in1>=xstmp2);
    path_indy1=find(y_in1<=(d(1)+ts));
    valid_path_ind1tmp=intersect(path_indx1,path_indy1);
    valid_path_ind1=intersect(valid_path_ind1tmp,find(tc1<=tcmax));
    if ~isempty(valid_path_ind1)
        chordC1=sqrt((tc1(valid_path_ind1)*sin(theta)-x_in1(valid_path_ind1)).^2+(tc1(valid_path_ind1)*cos(theta)-y_in1(valid_path_ind1)).^2);
        chordS1=sqrt((d(4)+0.5*(W_slot+0.1)-x_in1(valid_path_ind1)).^2+(d(1)+ts(valid_path_ind1)-y_in1(valid_path_ind1)).^2);
        Path_length1=abs(tc1(valid_path_ind1))+abs(asin(0.5*chordC1/Rmin)*2*Rmin)+abs(asin(0.5*chordS1/Rmin)*2*Rmin)+abs(d(3)-0.5-d(1)-ts(valid_path_ind1));
        [min_Path_length(3),ind]=min(Path_length1);
        min_tctmp(3)=tc1(valid_path_ind1(ind));
        min_tstmp(3)=ts(valid_path_ind1(ind));
    end
    x_in2=0.5*(sin(theta)*tc2+xc0tmp2+xstmp2);
    y_in2=0.5*(cos(theta)*tc2+zc0tmp2+d(1)+ts);
    path_indx2=find(x_in2>=xstmp2);
    path_indy2=find(y_in2<=(d(1)+ts));
    valid_path_ind2tmp=intersect(path_indx2,path_indy2);
    valid_path_ind2=intersect(valid_path_ind2tmp,find(tc2<=tcmax));
    if ~isempty(valid_path_ind2)
        chordC2=sqrt((tc2(valid_path_ind2)*sin(theta)-x_in2(valid_path_ind2)).^2+(tc2(valid_path_ind2)*cos(theta)-y_in2(valid_path_ind2)).^2);
        chordS2=sqrt((d(4)+0.5*(W_slot+0.1)-x_in2(valid_path_ind2)).^2+(d(1)+ts(valid_path_ind2)-y_in2(valid_path_ind2)).^2);
        Path_length2=abs(tc2(valid_path_ind2))+abs(asin(0.5*chordC2/Rmin)*2*Rmin)+abs(asin(0.5*chordS2/Rmin)*2*Rmin)+abs(d(3)-0.5-d(1)-ts(valid_path_ind2));
        [min_Path_length(4),ind]=min(Path_length2);
        min_tctmp(4)=tc2(valid_path_ind2(ind));
        min_tstmp(4)=ts(valid_path_ind2(ind));
    end
    
    [min_length,ind]=min(min_Path_length);
    if ~isinf(min_length)
        status=1;
        min_tc=min_tctmp(ind);
        min_ts=min_tstmp(ind);
        if ind<=2
            xc0=xc0tmp1;
            zc0=zc0tmp1;
            xs=xstmp1;
            manuver(2,2)=2;
            manuver(3,:)=[1 1];
        else
            xc0=xc0tmp2;
            zc0=zc0tmp2;
            xs=xstmp2;
            manuver(2,2)=1;
            manuver(3,:)=[1 2];
        end
    else status=0;
    end

    if status==1
        if min_tc<0
            manuver(1,:)=[0 3];
        else manuver(1,:)=[1 3];
        end
        SP=[0 0];
        B=[min_tc*sin(theta) min_tc*cos(theta)];
        xdumy=linspace(SP(1)+x, B(1)+x, ceil(abs(B(1)-SP(1))/0.2));
        zdumy=linspace(SP(2)+z,B(2)+z,length(xdumy));
        flag=checkcollition(xdumy,zdumy,-pi/2-theta, buffer);
        if flag==1
            status=0;
        else
            status=1;
            C=[0.5*(sin(theta)*min_tc+xc0+xs) 0.5*(cos(theta)*min_tc+zc0+d(1)+min_ts)];
            D=[d(4)+0.5*(W_slot+0.1) d(1)+min_ts];
            EP=[D(1) L_slot+0.5-z];
            %     angleBC=asin(0.5*distance(B,C)/Rmin)*2;
            angleCD=asin(0.5*distance(C,D)/Rmin)*2;
            paraB=absolueAngle(B(1),B(2),Rmin,sin(theta)*min_tc+xc0,cos(theta)*min_tc+zc0);
            paraCc=absolueAngle(C(1),C(2),Rmin,sin(theta)*min_tc+xc0,cos(theta)*min_tc+zc0);
            if abs(paraCc-paraB)>pi
                if paraCc>paraB
                    paraCc=paraCc-2*pi;
                else paraB=paraB-2*pi;
                end
            end
            if manuver(2,2)==2
                if paraB<=paraCc
                    angle=(paraB:0.01:paraCc)';
                    manuver(2,1)=1;
                else
                    angle=(paraB:-0.01:paraCc)';
                    manuver(2,1)=0;
                end
            else
                if paraB<=paraCc
                    angle=(paraB:0.01:paraCc)';
                    manuver(2,1)=0;
                else
                    angle=(paraB:-0.01:paraCc)';
                    manuver(2,1)=1;
                end
            end
            xp=Rmin*cos(angle); zp=Rmin*sin(angle);
            arc_BC=[xc0+sin(theta)*min_tc+xp+x zc0+cos(theta)*min_tc+zp+z];
            if manuver(2,2)==2
                angletmp=-angle; %convert to car frame
                angletmp=-pi/2-angletmp; %convert to global frame
            else
                angletmp=pi-angle;
                angletmp=-pi/2-angletmp;
            end
            for i=1:length(angletmp)
                flag=checkcollition(arc_BC(i,1), arc_BC(i,2), angletmp(i), buffer);
                if flag==1
                    status=0;
                    break;
                end
            end
            if status==1
                
                if manuver(3,2)==1
                    angle=(pi+angleCD:-0.01:pi)';
                    angletmp=pi-angle;
                    angletmp=-pi/2-angletmp;
                else
                    angle=(-angleCD:0.01:0)';
                    angletmp=-angle; %convert to car frame
                    angletmp=-pi/2-angletmp; %convert to global frame
                end
                xp=Rmin*cos(angle); zp=Rmin*sin(angle);
                arc_CD=[xs+xp+x d(1)+min_ts+zp+z];
                for i=1:length(angletmp)
                    flag=checkcollition(arc_CD(i,1), arc_CD(i,2), angletmp(i), buffer);
                    if flag==1
                        status=0;
                        break;
                    end
                end
                if status==1
                    waypoints=[SP+[x z];B+[x z];C+[x z];D+[x z];EP+[x z]];
                    if isempty(find(waypoints(:,2)<min_z,1))
                        drawpath=[[SP+[x z];B+[x z];arc_BC;C+[x z];arc_CD;D+[x z];EP+[x z]];];
                        %         [cc1 ee1 ff1]=lines(SP+[x z],B+[x z]);
                        %         [cc2 ee2 ff2]=lines(D+[x z],EP+[x z]);
                        [cc1 ee1 ff1]=lines([SP(1) B(1)]+x,[SP(2) B(2)]+z);
                        [cc2 ee2 ff2]=lines([D(1) EP(1)]+x,[D(2) EP(2)]+z);
                        trajectory=[[cc1 ee1 ff1]; ...
                            xc0+sin(theta)*min_tc+x zc0+cos(theta)*min_tc+z Rmin; ...
                            xs+x d(1)+min_ts+z Rmin; ...
                            [cc2 ee2 ff2]];
                        SP_B=distance(SP,B); BC=Rmin*abs(paraB-paraCc);
                        CD=Rmin*abs(angleCD);D_EP=distance(D,EP);
                        S=[SP_B;BC;CD;D_EP];
                    else status=0;
                    end
                end
            end
        end
    else
        waypoints=[];
        S=[];
        trajectory=[];
%         manuver=[];
    end
end

    

if status==0
    manuver=ones(4,2)*4;
    waypoints=[];
    S=[];
    trajectory=[];
end
if status==1
    hold on;
    plot(drawpath(:,1),drawpath(:,2),'r-','LineWidth',2);
%     plot(waypoints(:,1), waypoints(:,2),'dr','MarkerSize',10);
    hold off;
end
% % % 
% % % %car park slot
% % % x_slot=[d(2) d(2) d(4) d(4)];
% % % z_slot=[d(1) d(3) d(3) d(1)];
% % % hold on;
% % % fill(x_slot, z_slot, 'g');
% % % axis equal;
% % % hold on;
% % % 
% % % %car
% % % x_car=[0.5*W_car -0.5*W_car -0.5*W_car 0.5*W_car];
% % % z_car=[(L_car-L_wheels)*0.5 (L_car-L_wheels)*0.5 -L_car+(L_car-L_wheels)*0.5 -L_car+(L_car-L_wheels)*0.5];
% % % car_global=R*[x_car; zeros(1,4); z_car];
% % % x_car_global=car_global(1,:);
% % % z_car_global=car_global(3,:);
% % % fill(x_car_global, z_car_global,'c');
% % % 
% % % %camera vision -- the area where the camera can capture.
% % % x_vision=[0.68 1.10 -1.10 -0.68];
% % % z_vision=[0.48 2.13 2.13 0.48];
% % % vision_global=R*[x_vision; zeros(1,4); z_vision];
% % % x_vision_global=vision_global(1,:);
% % % z_vision_global=vision_global(3,:);
% % % plot(x_vision_global, z_vision_global,'b--');
% % % 
% % % %car and carpark slot angle
% % % z_slot_center=[0 d(3)];
% % % x_slot_center=[(d(2)+d(4))/2 (d(2)+d(4))/2];
% % % plot(x_slot_center, z_slot_center,'k--');
% % % x_car_center=[0 0];
% % % z_car_center=[0 1.5];
% % % car_center_global=R*[x_car_center;[0 0];z_car_center];
% % % x_car_center_global=car_center_global(1,:);
% % % z_car_center_global=car_center_global(3,:);
% % % plot(x_car_center_global,z_car_center_global,'k--');
% % % 
% % % if status==1
% % % % %     %plot circles
% % % % %     if manuver(3,2)==2
% % % % %         angle=0:0.01:2*pi;
% % % % %         xp=Rmin*cos(angle);zp=Rmin*sin(angle);
% % % % %         plot(xc0+xp,zc0+zp,'--');
% % % % %         angle=pi:0.01:3/2*pi;
% % % % %         xp=Rmin*cos(angle);zp=Rmin*sin(angle);
% % % % %         plot(xs+xp,d(1)+.8+zp,'--');
% % % % %     elseif manuver(3,2)==1
% % % % %         angle=0:0.01:2*pi;
% % % % %         xp=Rmin*cos(angle);zp=Rmin*sin(angle);
% % % % %         plot(xc0+xp,zc0+zp,'--');
% % % % %         angle=-0.5*pi:0.01:0;
% % % % %         xp=Rmin*cos(angle);zp=Rmin*sin(angle);
% % % % %         plot(xs+xp,d(1)+.8+zp,'--');
% % % % %     end
% % %     plot(path(:,1),path(:,2),'r-','LineWidth',2);
% % %     dumy=[SP;B;C;D;EP];
% % %     plot(dumy(:,1),dumy(:,2),'dr','MarkerSize',10);
% % % end
end