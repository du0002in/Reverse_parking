function [status, waypoints, trajectory, S, manuver, num_ite]=path_reGenveration(theta, x, z, buffer)
global_varibles;
% InitializationGlobalVariable;
con=1; num_ite=0;
if theta>-pi/2
    direction=1;
else direction=2;
end
stmp=0;PreStmp=Inf;status=0;
%
[status, waypoints, trajectory, S, manuver]=pathGeneration8(theta, x, z, buffer,1);
if status==1
    PreStmp=sum(S);
    drawpath2=[drawpath];
end
%
if direction==1
    theta1=theta;
    x0=x-Rmin*sin(theta); z0=z+Rmin*cos(theta);
    while con==1
        theta1=theta1-0.1;
        if theta1<-pi
            break;
        end
        num_ite=num_ite+1;
        x1=x0+Rmin*sin(theta1);
        z1=z0-Rmin*cos(theta1);
        flag=checkcollition(x1, z1, theta1, buffer);
        if flag==1
            break;
        end
        [stmp,wptmp,trjtmp,Stmp,manuvertmp]=pathGeneration8(theta1,x1,z1,buffer,1);
        if stmp==1
            status=1;
            S1=Rmin*abs(theta1-theta);
            if S1+sum(Stmp)<PreStmp
                angle=(theta:-0.01:theta1)';
                drawpath1=[x0+Rmin*sin(angle) z0-Rmin*cos(angle)];
                drawpath2=[drawpath1; drawpath];
                waypoints=[[x z];wptmp];
                trajectory=[x0 z0 Rmin;trjtmp];
                S=[S1;Stmp];
                manuver=[[1 1];manuvertmp];
                PreStmp=sum(S);
            end
        end
    end
    
    theta1=theta;
    x0=x+Rmin*sin(theta); z0=z-Rmin*cos(theta);
    while con==1
        theta1=theta1-0.1;
        if theta1<-pi
            break;
        end
        num_ite=num_ite+1;
        x1=x0-Rmin*sin(theta1);
        z1=z0+Rmin*cos(theta1);
        flag=checkcollition(x1, z1, theta1, buffer);
        if flag==1
            break;
        end
        [stmp,wptmp,trjtmp,Stmp,manuvertmp]=pathGeneration8(theta1,x1,z1,buffer,1);
        if stmp==1
            status=1;
            S1=Rmin*abs(theta1-theta);
            if S1+sum(Stmp)<PreStmp
                angle=(theta:-0.01:theta1)';
                drawpath1=[x0-Rmin*sin(angle) z0+Rmin*cos(angle)];
                drawpath2=[drawpath1; drawpath];
                waypoints=[[x z];wptmp];
                trajectory=[x0 z0 Rmin;trjtmp];
                S=[S1;Stmp];
                manuver=[[0 2];manuvertmp];
                PreStmp=sum(S);
            end
        end
    end
else
    theta1=theta;
    x0=x+Rmin*sin(theta); z0=z-Rmin*cos(theta);
    while con==1
        theta1=theta1+0.1;
        if theta1>0
            break;
        end
        num_ite=num_ite+1;
        x1=x0-Rmin*sin(theta1);
        z1=z0+Rmin*cos(theta1);
        flag=checkcollition(x1, z1, theta1, buffer);
        if flag==1
            break;
        end
        [stmp,wptmp,trjtmp,Stmp,manuvertmp]=pathGeneration8(theta1,x1,z1,buffer,1);
        if stmp==1
            status=1;
            S1=Rmin*abs(theta1-theta);
            if S1+sum(Stmp)<PreStmp
                angle=(theta:0.01:theta1)';
                drawpath1=[x0-Rmin*sin(angle) z0+Rmin*cos(angle)];
                drawpath2=[drawpath1; drawpath];
                waypoints=[[x z];wptmp];
                trajectory=[x0 z0 Rmin;trjtmp];
                S=[S1;Stmp];
                manuver=[[1 2];manuvertmp];
                PreStmp=sum(S);
            end
        end
    end
    theta1=theta;
    x0=x-Rmin*sin(theta); z0=z+Rmin*cos(theta);
    while con==1
        theta1=theta1+0.1;
        if theta1>0
            break;
        end
        num_ite=num_ite+1;
        x1=x0+Rmin*sin(theta1);
        z1=z0-Rmin*cos(theta1);
        flag=checkcollition(x1, z1, theta1, 0);
        if flag==1
            break;
        end
        [stmp,wptmp,trjtmp,Stmp,manuvertmp]=pathGeneration8(theta1,x1,z1,buffer,1);
        if stmp==1
            status=1;
            S1=Rmin*abs(theta1-theta);
            if S1+sum(Stmp)<PreStmp
                angle=(theta:0.01:theta1)';
                drawpath1=[x0+Rmin*sin(angle) z0-Rmin*cos(angle)];
                drawpath2=[drawpath1; drawpath];
                waypoints=[[x z];wptmp];
                trajectory=[x0 z0 Rmin;trjtmp];
                S=[S1;Stmp];
                manuver=[[0 1];manuvertmp];
                PreStmp=sum(S);
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
%     if ~isempty(find(drawpath(:,2)<-3.8,1))
%         dummy=1;
%     end
%     hold on;
%     plot(drawpath(:,1),drawpath(:,2),'r-','LineWidth',2);
% %     plot(waypoints(:,1), waypoints(:,2),'dr','MarkerSize',10);
%     hold off;
end

