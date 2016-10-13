function [status, waypoints, trajectory, S, manuver]=path_re_reGenveration(theta, x, z, buffer)
global_varibles;
% InitializationGlobalVariable;
stmp=0;PreStmp=Inf;status=0;
[status, waypoints, trajectory, S, manuver]=pathGeneration8(theta, x, z, buffer,1);
if status==1
    PreStmp=sum(S);
    drawpath2=[drawpath];
else
[status, waypoints, trajectory, S, manuver]=path_reGenveration(theta, x, z, buffer);
if status==1
    PreStmp=sum(S);
    drawpath2=[drawpath];
else
    t1=0.1:0.1:6; t2=-t1;
    t3=zeros(1,2*length(t1));
    t3(1:2:end)=t1;t3(2:2:end)=t2;
for i=1:length(t3)
    t=t3(i);
    x1=x+t*cos(theta);
    z1=z+t*sin(theta);
    flag=checkcollition(x1, z1, theta, buffer);
    if flag==1
        continue;
    end
    [stmp,wptmp,trjtmp,Stmp,manuvertmp]=path_reGenveration(theta, x1, z1, buffer);
    if stmp==1
        status=1;
        S1=abs(t);
        if S1+sum(Stmp)<PreStmp
%             angle=(theta:-0.01:theta1)';
%             drawpath1=[x0+Rmin*sin(angle) z0-Rmin*cos(angle)];
            drawpath2=[[x z]; drawpath];
            if t~=0
                waypoints=[[x z];wptmp];
                trajectory=[sin(theta) -cos(theta) 2*(cos(theta)*z-sin(theta)*x);trjtmp];
                S=[S1;Stmp];
                if t>0
                    manuver=[[0 3];manuvertmp];
                else
                    manuver=[[1 3];manuvertmp];
                end
                PreStmp=sum(S);
            else
                waypoints=[wptmp];
                trajectory=[trjtmp];
                S=[Stmp];
                manuver=[manuvertmp];
                PreStmp=sum(S);
            end
            %                 plot(drawpath2(:,1),drawpath2(:,2),'r-','LineWidth',2);
        end
        break;
    end
end
end
end
if status==0
    manuver=ones(4,2)*4;
    waypoints=[];
    S=[];
    trajectory=[];
end
if status==1
    drawpath=drawpath2;
% % % %     hold on;
% % % %     plot(drawpath(:,1),drawpath(:,2),'r-','LineWidth',2);
% % % %     hold off;
end