function [NoofLines, C, E, Fp, PCount]=lineAssignment3(BW, S_region, Line_In)
C=NaN(4,1);
E=NaN(4,1);
Fp=NaN(4,1);
PCount=NaN(4,1);
NoofLines=0;
if sum(Line_In)==0
    return;
else NoLine=sum(Line_In);
end
% MaxIteration=50;
[indv, indu]=find(BW==1);
% indv=-indv;
Datapoint=[indu';indv']; %diff from the previous defination which is Datapoint=[indv,indu];
NoPoint=length(Datapoint);
if NoPoint<50
    return;
end
for n=1:NoLine
    indextmp=[];
    indextmp=RANSAC(Datapoint);
    if sum(indextmp)>50
        NoofLines=NoofLines+1;
        FinalLinetmp{NoofLines}=Datapoint(:,(indextmp==1));
    end
    Datapoint(:,(indextmp==1))=[];
    if size(Datapoint,2)<0.1*NoPoint
        break;
    end
end

% p=[Datapoint(:,2)'; Datapoint(:,1)'; ones(1,length(Datapoint))];%3xn
% for n=1:NoLine %max 3 lines appear in the image
%     tcon_pre=0;indextmp=[];
%     for ntrials=1:MaxIteration
%         Randpointtmp=randperm(length(Datapoint));
%         Randpoint=Randpointtmp(1:2);
%         v=Datapoint(Randpoint,1);
%         u=Datapoint(Randpoint,2);
%         if u(1)==u(2)
%             e=0; c=1; f=-2*u(1); fp=f/2;
%         else
%             c=-(v(1)-v(2))/(u(1)-u(2));
%             e=1;
%             f=2*(-c*u(1)-v(1)); fp=f/2;
%         end
%         Cr=[0 0 c;0 0 e;c e f;];
%         Cp=Cr*p;
%         dstmp=(diag(transpose(p)*Cr*p))';
%         ds=(dstmp.*dstmp)/4.0./(Cp(1,:).*Cp(1,:)+Cp(2,:).*Cp(2,:));
%         [~,index]=find(ds<10);
%         linetmp=p(:,index)';
%         line=[linetmp(:,2),linetmp(:,1)];
%         tcon=length(line);
%         if tcon<50
%             continue;
%         end
%         if tcon>tcon_pre
%             tcon_pre=tcon;
%             NoofLines=n;
%             FinalLine{NoofLines}=line;
%             indextmp=index;
%             if tcon>0.9*length(Datapoint)
%                 break;
%             end
%         end
%     end
%     Datapoint(indextmp,:)=[];
% %     indextmp=[];
%     if length(Datapoint)<0.1*NoPoint
%         break;
%     else
%         p=[Datapoint(:,2)'; Datapoint(:,1)'; ones(1,length(Datapoint))];%3xn
%     end
% end
if NoofLines==0
    return;
end
for i=1:NoofLines
    xtmp=FinalLinetmp{i}(1,:);
    ytmp=FinalLinetmp{i}(2,:);
    FinalLine{i}=[ytmp',xtmp'];
    xtmp=[];ytmp=[];
end

S_region_angle=zeros(4,1);
for i=1:4
    if i==4
        j=1;
    else j=i+1;
    end
    S_region_angle(i)=atan((S_region(i,2)-S_region(j,2))/(S_region(i,1)-S_region(j,1)));
    [S_region_C(i) S_region_E(i) S_region_F(i)]=lines([S_region(i,1) S_region(j,1)], [S_region(i,2) S_region(j,2)]);
end
for i=1:NoofLines
    if length(FinalLine{i})>200
        Ytmp=[2*FinalLine{i}(:,2) ones(length(FinalLine{i}),1)];
        d=-2*FinalLine{i}(:,1);
        cf=(Ytmp'*Ytmp)\(Ytmp')*d;
        ds_sqr1=(cf(1)*FinalLine{i}(:,2)+FinalLine{i}(:,1)+cf(2)/2.0).*(cf(1)*FinalLine{i}(:,2)+FinalLine{i}(:,1)+cf(2)/2.0)/(cf(1)*cf(1)+1);
        ds_sqr1_total=sum(ds_sqr1);
        Ytmp=[2*FinalLine{i}(:,1) ones(length(FinalLine{i}),1)];
        d=-2*FinalLine{i}(:,2);
        ef=(Ytmp'*Ytmp)\(Ytmp')*d;
        ds_sqr2=(FinalLine{i}(:,2)+ef(1)*FinalLine{i}(:,1)+ef(2)/2.0).*(FinalLine{i}(:,2)+ef(1)*FinalLine{i}(:,1)+ef(2)/2.0)/(1+ef(1)*ef(1));
        ds_sqr2_total=sum(ds_sqr2);
        if ds_sqr1_total<=ds_sqr2_total
            Ctmp(i)=cf(1);Ftmp(i)=cf(2);Etmp(i)=1;Fptmp(i)=Ftmp(i)/2; angletmp=atan(-Ctmp(i)/Etmp(i));
        else
            Ctmp(i)=1; Ftmp(i)=ef(2); Etmp(i)=ef(1); Fptmp(i)=Ftmp(i)/2; angletmp=atan(-Ctmp(i)/Etmp(i));
        end
        angle_delta=abs(angletmp-S_region_angle);
        angle_delta(angle_delta>pi/2)=pi-angle_delta(angle_delta>pi/2);
        [~, ind]=min(angle_delta);
        if ind==1 || ind==3
            ds1=mean((S_region_C(1)*FinalLine{i}(:,2)+S_region_E(1)*FinalLine{i}(:,1)+S_region_F(1)/2).^2/(S_region_C(1)^2+S_region_E(1)^2));
            ds3=mean((S_region_C(3)*FinalLine{i}(:,2)+S_region_E(3)*FinalLine{i}(:,1)+S_region_F(3)/2).^2/(S_region_C(3)^2+S_region_E(3)^2));
            if ds1<ds3
                if Line_In(1)==1 && ds1<12500
                    if isnan(C(1))
                        C(1)=Ctmp(i); E(1)=Etmp(i); Fp(1)=Fptmp(i); PCount(1)=length(FinalLine{i}); dssqr_tmp(1)=ds1;
                    else
                        if PCount(1)>2*length(FinalLine{i})
                        elseif length(FinalLine{i})>2*PCount(1)
                            C(1)=Ctmp(i); E(1)=Etmp(i); Fp(1)=Fptmp(i); PCount(1)=length(FinalLine{i}); dssqr_tmp(1)=ds1;
                        elseif ds1<dssqr_tmp(1)
                            C(1)=Ctmp(i); E(1)=Etmp(i); Fp(1)=Fptmp(i); PCount(1)=length(FinalLine{i}); dssqr_tmp(1)=ds1;
                        end
                    end
                end
            else
                if Line_In(3)==1 && ds3<12500
                    if isnan(C(3))
                        C(3)=Ctmp(i); E(3)=Etmp(i); Fp(3)=Fptmp(i); PCount(3)=length(FinalLine{i}); dssqr_tmp(3)=ds3;
                    else
                        if PCount(3)>2*length(FinalLine{i})
                        elseif length(FinalLine{i})>2*PCount(3)
                            C(3)=Ctmp(i); E(3)=Etmp(i); Fp(3)=Fptmp(i); PCount(3)=length(FinalLine{i}); dssqr_tmp(3)=ds3;
                        elseif ds3<dssqr_tmp(3)
                            C(3)=Ctmp(i); E(3)=Etmp(i); Fp(3)=Fptmp(i); PCount(3)=length(FinalLine{i}); dssqr_tmp(3)=ds3;
                        end
                    end
                end
            end
        elseif ind==2 || ind==4
            ds2=mean((S_region_C(2)*FinalLine{i}(:,2)+S_region_E(2)*FinalLine{i}(:,1)+S_region_F(2)/2).^2/(S_region_C(2)^2+S_region_E(2)^2));
            ds4=mean((S_region_C(4)*FinalLine{i}(:,2)+S_region_E(4)*FinalLine{i}(:,1)+S_region_F(4)/2).^2/(S_region_C(4)^2+S_region_E(4)^2));
            if ds2<ds4
                if Line_In(2)==1 && ds2<12500
                    if isnan(C(2))
                        C(2)=Ctmp(i); E(2)=Etmp(i); Fp(2)=Fptmp(i); PCount(2)=length(FinalLine{i}); dssqr_tmp(2)=ds2;
                    else
                        if PCount(2)>2*length(FinalLine{i})
                        elseif length(FinalLine{i})>2*PCount(2)
                            C(2)=Ctmp(i); E(2)=Etmp(i); Fp(2)=Fptmp(i); PCount(2)=length(FinalLine{i}); dssqr_tmp(2)=ds2;
                        elseif ds2<dssqr_tmp(2)
                            C(2)=Ctmp(i); E(2)=Etmp(i); Fp(2)=Fptmp(i); PCount(2)=length(FinalLine{i}); dssqr_tmp(2)=ds2;
                        end
                    end
                end
            else
                if Line_In(4)==1 && ds4<12500
                    if isnan(C(4))
                        C(4)=Ctmp(i); E(4)=Etmp(i); Fp(4)=Fptmp(i); PCount(4)=length(FinalLine{i}); dssqr_tmp(4)=ds4;
                    else
                        if PCount(4)>2*length(FinalLine{i})
                        elseif length(FinalLine{i})>2*PCount(4)
                            C(4)=Ctmp(i); E(4)=Etmp(i); Fp(4)=Fptmp(i); PCount(4)=length(FinalLine{i}); dssqr_tmp(4)=ds4;
                        elseif ds4<dssqr_tmp(4)
                            C(4)=Ctmp(i); E(4)=Etmp(i); Fp(4)=Fptmp(i); PCount(4)=length(FinalLine{i}); dssqr_tmp(4)=ds4;
                        end
                    end
                end
            end
        end
    end
end
end