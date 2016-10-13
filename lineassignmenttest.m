path='C:\Users\a0107257\Documents\MATLAB\ImageProcessingLearning\Pictures\Self Parking\code\real run\test93\FrameLog';
figure;
% for  ii=2:(n-1)
   for ii=7
    Itmp1=imread(strcat(path,int2str(ii),'.jpg'),'jpg');
    imshow(Itmp1);
    Img1=double(max(Itmp1,[],3));
    kcsditmp=Ridgeness2(Img1,xfil2,ix,yfil2,iy,sy,GsiCenter, GsiMiddle, GsiCorner,ImgSize1, ImgSize2,smoothsize);
    [S_region, Line_In]=getSearchRegion(X_b(3,ii), [X_b(1,ii) X_b(2,ii)], P_b_debug(ii));
    [NoofLines, C, E, Fp, PCount]=lineAssignment3(kcsditmp, S_region, Line_In);
    log_C2(:,n)=C; log_E2(:,n)=E; log_Fp2(:,n)=Fp;
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
    saveas(gcf,strcat(path,'NewNew',int2str(ii),'.jpg'),'jpg');
    hold off;
end
