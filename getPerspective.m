function [theta, x, z]=getPerspective(C, E, Fp, PCount, theta_pre, varargin)
%theta is the angle with repect to frame Global. Or it is the angle rotated
%from Global to car frame about y axis.
global_varibles;
theta_pre=-pi/2-theta_pre;
if isnan(C(1))
    T(1)=NaN; d(1)=NaN;
else
    if E(1)==1
        xtmp=[1 640];
        ytmp=-C(1)*xtmp-Fp(1);
    elseif C(1)==1
        ytmp=[1 480];
        xtmp=-E(1)*ytmp-Fp(1);
    end
    xtmp=xtmp-ox;
    ytmp=ytmp-oy;
    T(1)=atan((focal*(ytmp(2)-ytmp(1)))/((xtmp(1)*ytmp(2)-xtmp(2)*ytmp(1))*cos(phi)+(xtmp(2)-xtmp(1))*focal*sin(phi)));
    if T(1)>0
        congT=T(1)-pi; %conguate of T, shift T by 1 period
    else congT=T(1)+pi;
    end
    if abs(theta_pre-T(1))>abs(theta_pre-congT)
        T(1)=congT;
    end
    d(1)=mean(Y*(-xtmp*sin(T(1))+ytmp*cos(T(1))*sin(phi)+focal*cos(phi)*cos(T(1)))/(ytmp*cos(phi)-focal*sin(phi)));
    d(1)=d(1)+L_wheel_cam*cos(T(1));
end

if isnan(C(2))
    T(2)=NaN; d(2)=NaN;
else
    if E(2)==1
        xtmp=[1 640];
        ytmp=-C(2)*xtmp-Fp(2);
    elseif C(2)==1
        ytmp=[1 480];
        xtmp=-E(2)*ytmp-Fp(2);
    end
    xtmp=xtmp-ox;
    ytmp=ytmp-oy;
    T(2)=atan(((xtmp(1)*ytmp(2)-xtmp(2)*ytmp(1))*cos(phi)+(xtmp(2)-xtmp(1))*focal*sin(phi))/(focal*(ytmp(1)-ytmp(2))));
    if T(2)>0
        congT=T(2)-pi; %conguate of T, shift T by 1 period
    else congT=T(2)+pi;
    end
    if abs(theta_pre-T(2))>abs(theta_pre-congT)
        T(2)=congT;
    end
    d(2)=mean(Y*(xtmp*cos(T(2))+ytmp*sin(T(2))*sin(phi)+focal*cos(phi)*sin(T(2)))/(ytmp*cos(phi)-focal*sin(phi)));
    d(2)=d(2)+L_wheel_cam*sin(T(2));
end

if isnan(C(3))
    T(3)=NaN; d(3)=NaN;
else
    if E(3)==1
        xtmp=[1 640];
        ytmp=-C(3)*xtmp-Fp(3);
    elseif C(3)==1
        ytmp=[1 480];
        xtmp=-E(3)*ytmp-Fp(3);
    end
    xtmp=xtmp-ox;
    ytmp=ytmp-oy;
    T(3)=atan((focal*(ytmp(2)-ytmp(1)))/((xtmp(1)*ytmp(2)-xtmp(2)*ytmp(1))*cos(phi)+(xtmp(2)-xtmp(1))*focal*sin(phi)));
    if T(3)>0
        congT=T(3)-pi; %conguate of T, shift T by 1 period
    else congT=T(3)+pi;
    end
    if abs(theta_pre-T(3))>abs(theta_pre-congT)
        T(3)=congT;
    end
    d(3)=mean(Y*(-xtmp*sin(T(3))+ytmp*cos(T(3))*sin(phi)+focal*cos(phi)*cos(T(3)))/(ytmp*cos(phi)-focal*sin(phi)));
    d(3)=d(3)+L_wheel_cam*cos(T(3));
end

if isnan(C(4))
    T(4)=NaN; d(4)=NaN;
else
    if E(4)==1
        xtmp=[1 640];
        ytmp=-C(4)*xtmp-Fp(4);
    elseif C(4)==1
        ytmp=[1 480];
        xtmp=-E(4)*ytmp-Fp(4);
    end
    xtmp=xtmp-ox;
    ytmp=ytmp-oy;
    T(4)=atan(((xtmp(1)*ytmp(2)-xtmp(2)*ytmp(1))*cos(phi)+(xtmp(2)-xtmp(1))*focal*sin(phi))/(focal*(ytmp(1)-ytmp(2))));
    if T(4)>0
        congT=T(4)-pi; %conguate of T, shift T by 1 period
    else congT=T(4)+pi;
    end
    if abs(theta_pre-T(4))>abs(theta_pre-congT)
        T(4)=congT;
    end
    d(4)=mean(Y*(xtmp*cos(T(4))+ytmp*sin(T(4))*sin(phi)+focal*cos(phi)*sin(T(4)))/(ytmp*cos(phi)-focal*sin(phi)));
    d(4)=d(4)+L_wheel_cam*sin(T(4));
end

ind=~isnan(T);
Ttmp=T(ind);
if length(Ttmp)==2
    if abs(Ttmp(1)-Ttmp(2))>10/180*pi
        T(1:4)=NaN;
        d(1:4)=NaN;
    end
elseif length(Ttmp)==3
    if abs(Ttmp(1)-Ttmp(2))>10/180*pi || abs(Ttmp(1)-Ttmp(3))>10/180*pi || abs(Ttmp(3)-Ttmp(2))>10/180*pi
        T(1:4)=NaN;
        d(1:4)=NaN;
    end
elseif length(Ttmp)==4
    if abs(Ttmp(1)-Ttmp(2))>10/180*pi || abs(Ttmp(1)-Ttmp(3))>10/180*pi || abs(Ttmp(1)-Ttmp(4))>10/180*pi || abs(Ttmp(2)-Ttmp(3))>10/180*pi || abs(Ttmp(2)-Ttmp(4))>10/180*pi || abs(Ttmp(3)-Ttmp(4))>10/180*pi
        T(1:4)=NaN;
        d(1:4)=NaN;
    end
end

ind=~isnan(T);
if sum(ind)~=0
    theta=sum(T(ind).*PCount(ind)')/sum(PCount(ind));
    theta=-pi/2-theta;
else theta=NaN;
end
% if ~isnan(d(1)) && ~isnan(d(3))
%     L_slot=(L_slot*L_support_points+(d(3)-d(1)+0.1)*sum(PCount([1 3])));
%     L_support_points=L_support_points+sum(PCount([1 3]));
%     L_slot=L_slot/L_support_points;
% end

if ~isnan(d(2)) && ~isnan(d(4))
    W_slot=(W_slot*W_support_points+(d(2)-d(4)-0.1)*sum(PCount([2 4])));
    W_support_points=W_support_points+sum(PCount([2 4]));
    W_slot=W_slot/W_support_points;
end

no_ip=nargin;
if ~isnan(d(1))
    z=-(d(1)-0.05);
elseif ~isnan(d(3))
    if no_ip==5
        z=-(d(3)-L_slot+0.05);
    else
        L_slot=(L_slot*L_support_points+(d(3)+varargin{2}+0.05)*PCount(3));
        L_support_points=L_support_points+PCount(3);
        L_slot=L_slot/L_support_points;
        z=-(d(3)-L_slot+0.05);
    end
else
    z=NaN;
end
if ~isnan(d(2))
    x=-(d(2)-W_slot*0.5-0.05);
elseif ~isnan(d(4))
    x=-(d(4)+W_slot*0.5+0.05);
else
    x=NaN;
end
% L_slot=2.25;
% if isnan(d(1)) && isnan(d(3))
% %     d(1)=0; d(3)=L_slot+d(1);
% elseif isnan(d(1))
%     d(1)=d(3)-L_slot;
% elseif isnan(d(3))
% %     d(3)=L_slot+d(1);
%     z_slot_car=d(1)-0.05;
% else
%     L_slot=d(3)-d(1)+0.1;
%     z_slot_car=((d(1)-0.05)*PCount(1)+(d(3)-L_slot+0.05)*PCount(3))/(PCount(1)+PCount(3));
% end
% if isnan(d(2)) && isnan(d(4))
% %     d(2)=W_slot/2.0;
% %     d(4)=d(2)-W_slot;
%     x_slot_car=NaN;
% elseif isnan(d(2))
%     d(2)=W_slot+d(4);
% elseif isnan(d(4))
%     d(4)=d(2)-W_slot;
% else
%     W_slot=d(4)-d(2)-0.1;
%     x_slot_car=((d(2)-0.05-0.5*W_slot)*PCount(2)+(d(4)+0.05+0.5*W_slot)*PCount(4))/(PCount(2)+PCount(4));
% end
% distance=d;
end