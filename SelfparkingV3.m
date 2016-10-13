%Road Detection with Ridge versioin 3.0 with larger smoothing matrix
%Using HSI-(intensity) value to further filtering the result
%Remove components with less pixels than minstructure based on it row
%Sequence Sunny-Shadows After-Rain NUS-Road1
%-------------- START ---------------------%
clear all;
warning('off','all');
DatabasePath='\MATLAB\'; %File folder of image series
Files = dir(DatabasePath);
Img_Number = 0;
for i = 1:size(Files,1)
    if (not(strcmp(Files(i).name,'.')||strcmp(Files(i).name,'..')||strcmp(Files(i).name,'Thumbs.db')))
        q=strfind(Files(i).name,'.jpg');
        if q
            Img_Number = Img_Number + 1; % Number of all images in the  database
            Imgs(Img_Number)=Files(i);
        end
    end
end
cmdPath='C:\Users\a0107257\Documents\MATLAB\ImageProcessingLearning\Pictures\Self Parking\code\';
%ImgSize1==>row ImgSize2==>column
ImgSize1=480; ImgSize2=640;
%Define Gaussian smoothing kernel
%Kernel1 is differiciation scale. the size of kernel is determined by half
%of the lane line width in terms of number of pixels. e.g. if lane width is
%20 pixels, then smoothsize should be around 10. sigx should be adjusted as
%accordingly.
K=inline('exp(-0.5*x.^2/sigx^2-0.5*y.^2/sigy^2)');
smoothsize=20;
weight=exp((-0.5*(-smoothsize:smoothsize).^2/25^2));
weight=weight/(sum(weight));

%For converlution use
%Kernel2 is integration scale. we set it to 3x3;
[dx2,dy2]=meshgrid([-1:1]);
Kernel2=K(0.5,0.5,dx2,dy2);
weight2=Kernel2/sum(Kernel2(:));
GsiCenter=weight2(2,2);
GsiMiddle=weight2(1,2);
GsiCorner=weight2(1,1);
minstructure=15;
for ii=4718:4718
    f1=strcat('0d 1.jpg');
    tic
    %read in picture and take v value in HSV color map
    Img=double(max(imread(f1),[],3));t(2)=toc;
    kcsditmp=Ridgeness(Img,weight,GsiCenter, GsiMiddle, GsiCorner, minstructure, ImgSize1, ImgSize2,smoothsize); t(3)=toc;
    [NoofLines, C, E, Fp]=sequentialRANSAC_SelfParking(kcsditmp);t(4)=toc;
    [theta, d]=getPerspective(C, E, Fp);t(5)=toc;
    if d(3)>0.5
        [status, Path, manuver]=pathGeneration3(theta, d);t(6)=toc;t(6)
    else
        manuver=ones(4,2)*4;
    end
    fileID=fopen(strcat(cmdPath,'manuver.txt'),'w');
    fprintf(fileID,'%d',manuver');
    fclose(fileID);
end