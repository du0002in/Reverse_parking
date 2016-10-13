function [NoofLines, C, E, Fp,PCount, act_line]=sequentialRANSAC_SelfParking2(BW, img, direction,theta_pre)

global_varibles;
C=NaN(4,1);
E=NaN(4,1);
Fp=NaN(4,1);
PCount=NaN(4,1);
MaxIteration=100;
[indv, indu]=find(BW==1);
% indv=-indv;
Datapoint=[indu';indv']; %diff from the previous defination which is Datapoint=[indv,indu];
NoPoint=length(Datapoint);
NoofLines=0;act_line=0;
if NoPoint<50
    return;
end
for n=1:3
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

if NoofLines==0
    return;
end
for i=1:NoofLines
    xtmp=FinalLinetmp{i}(1,:);
    ytmp=FinalLinetmp{i}(2,:);
    FinalLine{i}=[ytmp',xtmp'];
    xtmp=[];ytmp=[];
end
tmpNo=NoofLines;
for i=1:tmpNo
    if ~isempty(FinalLine{i})
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
        if ~isnan(ds_sqr1_total) && ~isnan(ds_sqr2_total)
            if ds_sqr1_total<=ds_sqr2_total
                Ctmp(i)=cf(1);Ftmp(i)=cf(2);Etmp(i)=1;Fptmp(i)=Ftmp(i)/2; PCounttmp(i)=length(FinalLine{i});
            else
                Ctmp(i)=1; Ftmp(i)=ef(2); Etmp(i)=ef(1); Fptmp(i)=Ftmp(i)/2;PCounttmp(i)=length(FinalLine{i});
            end
        elseif ~isnan(ds_sqr1_total)
            Ctmp(i)=cf(1);Ftmp(i)=cf(2);Etmp(i)=1;Fptmp(i)=Ftmp(i)/2; PCounttmp(i)=length(FinalLine{i});
        elseif ~isnan(ds_sqr2_total)
            Ctmp(i)=1; Ftmp(i)=ef(2); Etmp(i)=ef(1); Fptmp(i)=Ftmp(i)/2;PCounttmp(i)=length(FinalLine{i});
        else
            Ctmp(i)=NaN; Ftmp(i)=NaN; Etmp(i)=NaN; Fptmp(i)=NaN;PCounttmp(i)=NaN;
        end
    else NoofLines=NoofLines-1;    
    end
end

if NoofLines==2
    pass=line2Validation(Ctmp, Etmp, Fptmp,theta_pre);
    if pass==0
        NoofLines=1;
        [~,indtmp]=max(PCounttmp);
        Ctmp(1)=Ctmp(indtmp);
        Etmp(1)=Etmp(indtmp);
        Fptmp(1)=Fptmp(indtmp);
    end
elseif NoofLines==3
    pass=line3Validation(Ctmp, Etmp, Fptmp,theta_pre);
    if pass==0
        passtmp(1)=line2Validation(Ctmp([1,2]), Etmp([1,2]), Fptmp([1,2]),theta_pre);
        passtmp(2)=line2Validation(Ctmp([1,3]), Etmp([1,3]), Fptmp([1,3]),theta_pre);
        passtmp(3)=line2Validation(Ctmp([3,2]), Etmp([3,2]), Fptmp([3,2]),theta_pre);
        indtmp2=find(passtmp==1);
        NoofLines=2;
        if isempty(indtmp2)
            NoofLines=1;
            [~,indtmp]=max(PCounttmp);
            Ctmp(1)=Ctmp(indtmp);
            Etmp(1)=Etmp(indtmp);
            Fptmp(1)=Fptmp(indtmp);
        elseif length(indtmp2)==1
            
            if indtmp2==1
                Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
            elseif indtmp2==2
                Ctmp(2)=Ctmp(3);Etmp(2)=Etmp(3);Fptmp(2)=Fptmp(3);
                Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
            elseif indtmp2==3
                Ctmp(1)=Ctmp(3);Etmp(1)=Etmp(3);Fptmp(1)=Fptmp(3);
                Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
            end
        elseif length(indtmp2)==2
            indtmp3=find(passtmp==0);
            if indtmp3==1
                if PCounttmp(1)>PCounttmp(2)
                    Ctmp(2)=Ctmp(3);Etmp(2)=Etmp(3);Fptmp(2)=Fptmp(3);
                    Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
                else Ctmp(1)=Ctmp(3);Etmp(1)=Etmp(3);Fptmp(1)=Fptmp(3);
                    Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
                end
            elseif indtmp3==2
                if PCounttmp(1)>PCounttmp(3)
%                     Ctmp(2)=Ctmp(3);Etmp(2)=Etmp(3);Fptmp(2)=Fptmp(3);
                    Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
                else Ctmp(1)=Ctmp(3);Etmp(1)=Etmp(3);Fptmp(1)=Fptmp(3);
                    Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
                end
            elseif indtmp3==3
                if PCounttmp(3)>PCounttmp(2)
                    Ctmp(2)=Ctmp(3);Etmp(2)=Etmp(3);Fptmp(2)=Fptmp(3);
                    Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
                else
%                     Ctmp(1)=Ctmp(3);Etmp(1)=Etmp(3);Fptmp(1)=Fptmp(3);
                    Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
                end
            end
        elseif length(indtmp2)==3
            [~,indtmp4]=max([PCounttmp(1)+PCounttmp(2),PCounttmp(1)+PCounttmp(3),PCounttmp(3)+PCounttmp(2)]);
            if indtmp4==1
                Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
            elseif indtmp4==2
                Ctmp(2)=Ctmp(3);Etmp(2)=Etmp(3);Fptmp(2)=Fptmp(3);
                Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
            elseif indtmp4==3
                Ctmp(1)=Ctmp(3);Etmp(1)=Etmp(3);Fptmp(1)=Fptmp(3);
                Ctmp(3)=[];Etmp(3)=[];Fptmp(3)=[];
            end
        end
    end
end
        
if NoofLines==1
    act_line=1;
    C(1)=Ctmp(1); E(1)=Etmp(1); Fp(1)=Fptmp(1); PCount(1)=PCounttmp(1);
    if E(1)==1
        xtmp=[1 640];
        ytmp=-C(1)*xtmp-Fp(1);
    elseif C(1)==1
        ytmp=[1 480];
        xtmp=-E(1)*ytmp-Fp(1);
    end
    xtmp=xtmp-ox;
    ytmp=ytmp-oy;
    theta=atan((focal*(ytmp(2)-ytmp(1)))/((xtmp(1)*ytmp(2)-xtmp(2)*ytmp(1))*cos(phi)+(xtmp(2)-xtmp(1))*focal*sin(phi)));
    [x_inst(1), y_inst(1)]=lineintersection([Ctmp(1);1], [Etmp(1); 0], [Fptmp(1); -1]);
    [x_inst(2), y_inst(2)]=lineintersection([Ctmp(1);1], [Etmp(1); 0], [Fptmp(1); -640]);
    [x_inst(3), y_inst(3)]=lineintersection([Ctmp(1);0], [Etmp(1); 1], [Fptmp(1); -1]);
    [x_inst(4), y_inst(4)]=lineintersection([Ctmp(1);0], [Etmp(1); 1], [Fptmp(1); -480]);
    ind_x=intersect(find(x_inst<=640), find(x_inst>=1));
    ind_y=intersect(find(y_inst<=480), find(y_inst>=1));
    ind=intersect(ind_x, ind_y);
    center_x=mean(x_inst(ind))-ox;
    center_y=mean(y_inst(ind))-oy;
    X=Y*(center_x*cos(theta)+center_y*sin(phi)*sin(theta)+focal*cos(phi)*sin(theta))/(center_y*cos(phi)-focal*sin(phi));
    Z=Y*(-center_x*sin(theta)+center_y*cos(theta)*sin(phi)+focal*cos(phi)*cos(theta))/(center_y*cos(phi)-focal*sin(phi));
    X1=X+(W_slot+0.1)/2;
    X2=X1;
    Z1=Z;
    Z2=Z1+L_slot;
    x(1)=ox+focal*(X1*cos(theta)-Z1*sin(theta))/(Z1*cos(phi)*cos(theta)-Y*sin(phi)+X1*cos(phi)*sin(theta));
    y(1)=oy+focal*(Y*cos(phi)+Z1*cos(theta)*sin(phi)+X1*sin(phi)*sin(theta))/(Z1*cos(phi)*cos(theta)-Y*sin(phi)+X1*cos(phi)*sin(theta));
    x(2)=ox+focal*(X2*cos(theta)-Z2*sin(theta))/(Z2*cos(phi)*cos(theta)-Y*sin(phi)+X2*cos(phi)*sin(theta));
    y(2)=oy+focal*(Y*cos(phi)+Z2*cos(theta)*sin(phi)+X2*sin(phi)*sin(theta))/(Z2*cos(phi)*cos(theta)-Y*sin(phi)+X2*cos(phi)*sin(theta));
    [C(2) E(2) Fp(2)]=lines(x,y);
    Fp(2)=Fp(2)/2;
    PCount(1)=ceil(0.5*PCount(1));
    PCount(2)=PCount(1);
    NoofLines=2;
elseif NoofLines==2
    [~, y_inst]=lineintersection(Ctmp, Etmp, Fptmp);
    if y_inst>=680 || y_inst<=-40 %line 2 4
        if -Fptmp(1)/Ctmp(1)>-Fptmp(2)/Ctmp(2)
            C(2)=Ctmp(1); E(2)=Etmp(1); Fp(2)=Fptmp(1); PCount(2)=PCounttmp(1);
            C(4)=Ctmp(2); E(4)=Etmp(2); Fp(4)=Fptmp(2); PCount(4)=PCounttmp(2);
        else
            C(2)=Ctmp(2); E(2)=Etmp(2); Fp(2)=Fptmp(2); PCount(2)=PCounttmp(2);
            C(4)=Ctmp(1); E(4)=Etmp(1); Fp(4)=Fptmp(1); PCount(4)=PCounttmp(1);
        end
    else
        [C, E, Fp, PCount]=templatematching(img, Ctmp, Etmp, Fptmp, PCounttmp, direction);
    end
    act_line=2;
elseif NoofLines==3
    act_line=3;
    [x_inst, y_inst]=lineintersection(Ctmp, Etmp, Fptmp);
    dist_sqr(1)=(x_inst(1)-x_inst(3))^2+(y_inst(1)-y_inst(3))^2;
    dist_sqr(2)=(x_inst(1)-x_inst(2))^2+(y_inst(1)-y_inst(2))^2;
    dist_sqr(3)=(x_inst(2)-x_inst(3))^2+(y_inst(2)-y_inst(3))^2;
    [~,IX]=sort(dist_sqr); %accending order
    meanX=mean(FinalLine{IX(2)}(:,2));
    meanY=mean(FinalLine{IX(2)}(:,1));
    checktmp=Ctmp(IX(1))*meanX+Etmp(IX(1))*meanY+Fptmp(IX(1));
    if checktmp<=0
        C(1)=Ctmp(IX(1)); E(1)=Etmp(IX(1)); Fp(1)=Fptmp(IX(1)); PCount(1)=PCounttmp(IX(1));
    else
        C(3)=Ctmp(IX(1)); E(3)=Etmp(IX(1)); Fp(3)=Fptmp(IX(1)); PCount(3)=PCounttmp(IX(1));
    end
    if IX(1)==1
        if x_inst(1)>=x_inst(3)
            C(2)=Ctmp(2); E(2)=Etmp(2); Fp(2)=Fptmp(2); PCount(2)=PCounttmp(2);
            C(4)=Ctmp(3); E(4)=Etmp(3); Fp(4)=Fptmp(3); PCount(4)=PCounttmp(3);
        else
            C(2)=Ctmp(3); E(2)=Etmp(3); Fp(2)=Fptmp(3); PCount(2)=PCounttmp(3);
            C(4)=Ctmp(2); E(4)=Etmp(2); Fp(4)=Fptmp(2); PCount(4)=PCounttmp(2);
        end
    elseif IX(1)==2
        if x_inst(1)>=x_inst(2)
            C(2)=Ctmp(1); E(2)=Etmp(1); Fp(2)=Fptmp(1); PCount(2)=PCounttmp(1);
            C(4)=Ctmp(3); E(4)=Etmp(3); Fp(4)=Fptmp(3); PCount(4)=PCounttmp(3);
        else
            C(2)=Ctmp(3); E(2)=Etmp(3); Fp(2)=Fptmp(3); PCount(2)=PCounttmp(3);
            C(4)=Ctmp(1); E(4)=Etmp(1); Fp(4)=Fptmp(1); PCount(4)=PCounttmp(1);
        end
    else
        if x_inst(2)>=x_inst(3)
            C(2)=Ctmp(2); E(2)=Etmp(2); Fp(2)=Fptmp(2); PCount(2)=PCounttmp(2);
            C(4)=Ctmp(1); E(4)=Etmp(1); Fp(4)=Fptmp(1); PCount(4)=PCounttmp(1);
        else
            C(2)=Ctmp(1); E(2)=Etmp(1); Fp(2)=Fptmp(1); PCount(2)=PCounttmp(1);
            C(4)=Ctmp(2); E(4)=Etmp(2); Fp(4)=Fptmp(2); PCount(4)=PCounttmp(2);
        end
    end
end
