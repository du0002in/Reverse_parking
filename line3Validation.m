function pass=line3Validation(Ctmp, Etmp, Fptmp, theta_pre)

Combtmp=npermutek(1:4,3);
Counter=0;
for i=1:length(Combtmp)
    if (length(unique(Combtmp(i,:)))==length(Combtmp(i,:)))
        Counter=1+Counter;
        ind(Counter)=i;
    end
end
Comb=Combtmp(ind,:);
global_varibles;
theta_pre=-pi/2-theta_pre;

for i=1:length(Comb)
    C=NaN(4,1);
    E=NaN(4,1);
    Fp=NaN(4,1);
    C(Comb(i,1))=Ctmp(1);
    C(Comb(i,2))=Ctmp(2);
    C(Comb(i,3))=Ctmp(3);
    E(Comb(i,1))=Etmp(1);
    E(Comb(i,2))=Etmp(2);
    E(Comb(i,3))=Etmp(3);
    Fp(Comb(i,1))=Fptmp(1);
    Fp(Comb(i,2))=Fptmp(2);
    Fp(Comb(i,3))=Fptmp(3);
    
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
    end
    
    if abs(T(Comb(i,1))-T(Comb(i,2)))<10*pi/180 && abs(T(Comb(i,1))-T(Comb(i,3)))<10*pi/180 && abs(T(Comb(i,3))-T(Comb(i,2)))<10*pi/180
        C1=intersect(Comb(i,:),1);
        C2=intersect(Comb(i,:),2);
        C3=intersect(Comb(i,:),3);
        C4=intersect(Comb(i,:),4);
        if (~isempty(C1) && ~isempty(C3))
            if abs(d(1)-d(3))>1.5 && abs(d(1)-d(3))<3
                pass=1;
                return;
            end
        elseif (~isempty(C2) && ~isempty(C4))
            if abs(d(2)-d(4))>1 && abs(d(2)-d(4))<2
                pass=1;
                return;
            end
        end
    end
end
pass=0;
end
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        