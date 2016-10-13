function [C, E, Fp, PCount]=templatematching(img, Ctmp, Etmp, Fptmp, PCounttmp, direction)
%direction 1=>left, 2=>right
global_varibles;
C=NaN(4,1);
E=NaN(4,1);
Fp=NaN(4,1);
PCount=NaN(4,1);
count=0;
comb_C=[Ctmp(1) Ctmp(2) Ctmp(1) Ctmp(2); ...
        Ctmp(2) Ctmp(1) NaN NaN; ...
        NaN NaN NaN NaN; ...
        NaN NaN Ctmp(2) Ctmp(1);];
comb_E=[Etmp(1) Etmp(2) Etmp(1) Etmp(2); ...
        Etmp(2) Etmp(1) NaN NaN; ...
        NaN NaN NaN NaN; ...
        NaN NaN Etmp(2) Etmp(1);];    
comb_Fp=[Fptmp(1) Fptmp(2) Fptmp(1) Fptmp(2); ...
        Fptmp(2) Fptmp(1) NaN NaN; ...
        NaN NaN NaN NaN; ...
        NaN NaN Fptmp(2) Fptmp(1);];
comb_PCount=[PCounttmp(1) PCounttmp(2) PCounttmp(1) PCounttmp(2); ...
        PCounttmp(2) PCounttmp(1) NaN NaN; ...
        NaN NaN NaN NaN; ...
        NaN NaN PCounttmp(2) PCounttmp(1);]; 
for i=1:4
    C=comb_C(:,i);
    E=comb_E(:,i);
    Fp=comb_Fp(:,i);
    PCount=comb_PCount(:,i);
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
        if direction==1 && T(1)>0 && abs(T(1)-pi/2)<2*pi/180
            T(1)=-T(1);
        elseif direction==2 && T(1)<0 && abs(T(1)+pi/2)<2*pi/180
            T(1)=-T(1);
        end
        d(1)=mean(Y*(-xtmp*sin(T(1))+ytmp*cos(T(1))*sin(phi)+focal*cos(phi)*cos(T(1)))/(ytmp*cos(phi)-focal*sin(phi)));
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
        if direction==1 && T(2)>0 && abs(T(2)-pi/2)<2*pi/180
            T(2)=-T(2);
        elseif direction==2 && T(2)<0 && abs(T(2)+pi/2)<2*pi/180
            T(2)=-T(2);
        end
        d(2)=mean(Y*(xtmp*cos(T(2))+ytmp*sin(T(2))*sin(phi)+focal*cos(phi)*sin(T(2)))/(ytmp*cos(phi)-focal*sin(phi)));
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
        if direction==1 && T(3)>0 && abs(T(3)-pi/2)<2*pi/180
            T(3)=-T(3);
        elseif direction==2 && T(3)<0 && abs(T(3)+pi/2)<2*pi/180
            T(3)=-T(3);
        end
        d(3)=mean(Y*(-xtmp*sin(T(3))+ytmp*cos(T(3))*sin(phi)+focal*cos(phi)*cos(T(3)))/(ytmp*cos(phi)-focal*sin(phi)));
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
        if direction==1 && T(4)>0 && abs(T(4)-pi/2)<2*pi/180
            T(4)=-T(4);
        elseif direction==2 && T(4)<0 && abs(T(4)+pi/2)<2*pi/180
            T(4)=-T(4);
        end
        d(4)=mean(Y*(xtmp*cos(T(4))+ytmp*sin(T(4))*sin(phi)+focal*cos(phi)*sin(T(4)))/(ytmp*cos(phi)-focal*sin(phi)));
    end
    ind=~isnan(T);
    Ttmp=T(ind);
%     if peak2peak(Ttmp)>pi/2
%         if direction==1
%             Ttmp(Ttmp>0)=-pi+Ttmp(Ttmp>0);
%         else
%             Ttmp(Ttmp<0)=pi+Ttmp(Tmp<0);
%         end
%     end
    theta=mean(Ttmp);
    theta=-pi/2-theta;
    if direction==1 && theta>-pi/2-2/180*pi
        count=count+1;
        Cf(:,count)=C; Ef(:,count)=E; Fpf(:,count)=Fp; PCountf(:,count)=PCount;
        thetaf(count)=theta;
        z(count)=-(d(1)-0.05);
        if ~isnan(d(2))
            x(count)=-(d(2)-W_slot*0.5-0.05);
        elseif ~isnan(d(4))
            x(count)=-(d(4)+W_slot*0.5+0.05);
        end
    elseif direction==2 && theta<-pi/2+2/180*pi
        count=count+1;
        Cf(:,count)=C; Ef(:,count)=E; Fpf(:,count)=Fp;PCountf(:,count)=PCount;
        thetaf(count)=theta;
        z(count)=-(d(1)-0.05);
        if ~isnan(d(2))
            x(count)=-(d(2)-W_slot*0.5-0.05);
        elseif ~isnan(d(4))
            x(count)=-(d(4)+W_slot*0.5+0.05);
        end
    end
end
if count~=0
    for i=1:count
        temp_img=CamSim(thetaf(i),[x(i) z(i)], [0 0]);
        matchingPercent(i)=sum(sum(img.*temp_img));
    end
    [~,ind]=max(matchingPercent);
    C=Cf(:,ind); E=Ef(:,ind); Fp=Fpf(:,ind); PCount=PCountf(:,ind);
else
    C=NaN(4,1);
    E=NaN(4,1);
    Fp=NaN(4,1);
    PCount=NaN(4,1);
end
    
    
    
    
    