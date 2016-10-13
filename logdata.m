%process the log data for debug purpose
% clear all;
path='D:\DXX\Self Parking\code\real run\test127\FrameLog';
% load test13.mat;
figure;
for ii=1:(n-1)
    Itmp1=imread(strcat(path,int2str(ii),'.jpg'),'jpg');
    imshow(Itmp1);
    C=log_C(:,ii); E=log_E(:,ii); Fp=log_Fp(:,ii);
    for i=1:4
        if isnan(C(i))
            continue;
        end
        
        if E(i)~=0
            x(1)=1; x(2)=640;
            y(1)=(-Fp(i)-C(i)*1)/E(i);
            y(2)=(-Fp(i)-C(i)*640)/E(i);
        else
            x(1)=-Fp(i)/C(i);
            x(2)=-Fp(i)/C(i);
            y(1)=1; y(2)=480;
        end
        if C(i)~=0
            x(3)=(-E(i)*1-Fp(i))/C(i);
            x(4)=(-E(i)*480-Fp(i))/C(i);
            y(3)=1; y(4)=480;
        else
            x(3)=1; x(4)=640;
            y(3)=Fp(i)/E(i);
            y(4)=y(3);
        end
        indx1=find(x<641);
        indx2=find(x>0);
        indx=intersect(indx1,indx2);
        indy1=find(y<481);
        indy2=find(y>0);
        indy=intersect(indy1,indy2);
        comxy=intersect(indx,indy);
        if length(comxy)>1
            if i==1
                color=[1 0 0];
            elseif i==2
                color=[0 1 0];
            elseif i==3
                color=[0 0 1];
            else
                color=[0 0 0];
            end
            line(x(comxy),y(comxy),'Color',color);
            hold on;
        end
    end
    saveas(gcf,strcat(path,'New',int2str(ii),'.jpg'),'jpg');
    hold off;
end
% ind_T=T;
% for i=2:(n-1)
%     T(i)=T(i-1)+ind_T(i);
% end
% 
% figure;
% subplot(5,1,1); plot(T(1:(n-2)),log_v_m(1:(n-2))); hold on; plot(T(1:(n-2)),X_e(7,(1:(n-2))),'r'); 
% plot(T(1:(n-2)),log_v_t(1:(n-2)),'g'); hold off; title('v'); 
% legend('Measurement','Kalman estimation', 'Target');
% subplot(5,1,2); plot(T(1:(n-2)),log_phai_m(1:(n-2))); hold on; plot(T(1:(n-2)),X_e(8,(1:(n-2))),'r');
% plot(T(1:(n-2)),log_phai_t(1:(n-2)),'g'); plot(T(1:(n-2)),phai_T(1:(n-2)),'k'); hold off;
% title('phai'); legend('Measurement','Kalman estimation', 'Sliding control target','Path gen. target');
% subplot(5,1,3); plot(T(1:(n-2)),log_x_m(1:(n-2))); hold on; plot(T(1:(n-2)),X_e(1,(1:(n-2))),'r'); hold off;title('x'); legend('Measurement','Kalman estimation');
% subplot(5,1,4); plot(T(1:(n-2)),log_z_m(1:(n-2))); hold on; plot(T(1:(n-2)),X_e(2,(1:(n-2))),'r'); hold off;title('z'); legend('Measurement','Kalman estimation');
% subplot(5,1,5); plot(T(1:(n-2)),log_theta_m(1:(n-2))); hold on; plot(T(1:(n-2)),X_e(3,(1:(n-2))),'r'); hold off;title('theta'); legend('Measurement','Kalman estimation');
% figure;
% plot(drawpath(:,1),drawpath(:,2),'c');hold on;
% plot(log_x_m(1:(n-2)),log_z_m(1:(n-2)));
% plot(X_e(1,:),X_e(2,:),'r');
% title('Path Tracking');
% legend('Path Generation','Measurement','Kalman estimation');
% hold off; axis equal

figure;
subplot(5,1,1); plot(log_v_m(2:(n-2))); hold on; plot(X_e(7,(2:(n-2))),'r'); 
plot(log_v_t(1:(n-2)),'g'); hold off; title('v'); 
legend('Measurement','Kalman estimation', 'Target');
subplot(5,1,2); plot(log_phai_m(2:(n-2))); hold on; plot(X_e(8,(2:(n-2))),'r');
plot(log_phai_t(1:(n-2)),'g'); plot(phai_T(1:(n-2)),'k'); hold off;
title('phai'); legend('Measurement','Kalman estimation', 'Sliding control target','Path gen. target');
subplot(5,1,3); plot(log_x_m(1:(n-2))); hold on; plot(X_e(1,(1:(n-2))),'r'); hold off;title('x'); legend('Measurement','Kalman estimation');
subplot(5,1,4); plot(log_z_m(1:(n-2))); hold on; plot(X_e(2,(1:(n-2))),'r'); hold off;title('z'); legend('Measurement','Kalman estimation');
subplot(5,1,5); plot(log_theta_m(1:(n-2))); hold on; plot(X_e(3,(1:(n-2))),'r'); hold off;title('theta'); legend('Measurement','Kalman estimation');
figure;
plot(drawpath(:,1),drawpath(:,2),'c');hold on;
for i=1:pathcounter
    plot(log_path{i}(:,1),log_path{i}(:,2),'c');
end
plot(log_x_m(1:(n-2)),log_z_m(1:(n-2)));
plot(X_e(1,:),X_e(2,:),'r');
title('Path Tracking');
legend('Path Generation','Measurement','Kalman estimation');
hold off; axis equal
% % 
% % 
% % 




