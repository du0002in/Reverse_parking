ImgSize1=480; ImgSize2=640;
%Define Gaussian smoothing kernel
%Kernel1 is differiciation scale. the size of kernel is determined by half
%of the lane line width in terms of number of pixels. e.g. if lane width is
%20 pixels, then smoothsize should be around 10. sigx should be adjusted as
%accordingly.
%Define the differiciation scale. The smooth size of x-direction kernel
%equals to half of the lane width in terms of pixel and std deviation
%equals to smooth size exactly. Same for y-direction
tdx=round(15+[0:1:(480-1)]*10.0/(480-1)); %the furtherest vertical line width is 15*2 pixel and the nearest is (15+10)*2;
[~,ix,~]=unique(tdx,'stable');ix=ix';
j=1;
for i=ix
    weight3=exp((-0.5*(-tdx(i):tdx(i)).^2/(tdx(i))^2));
    weight3=weight3/sum(weight3);
    xfil2{j}=weight3;
    j=j+1;
end
ix(length(ix)+1)=ImgSize1+1;

sy=round(10+[0:1:(480-1)]*10.0/(480-1));%the furtherest horizontal line width is 10*2 pixel and the nearest is (10+10)*2;
[~,iy,~]=unique(sy,'stable');iy=iy';
j=1;
for i=iy
    weight2y=exp((-0.5*(-sy(i):sy(i))'.^2/sy(i)^2));
    weight2y=weight2y/sum(weight2y);
    yfil2{j}=weight2y;
    j=j+1;
end
iy(length(iy)+1)=ImgSize1+1;

smoothsize=(15+10);%a fake one, it equals to max of the 4 smooth size
%For converlution use
%Kernel2 is integration scale. we set it to 3x3;
K=inline('exp(-0.5*x.^2/sigx^2-0.5*y.^2/sigy^2)');
[dx2,dy2]=meshgrid([-1:1]);
Kernel2=K(0.5,0.5,dx2,dy2);
weight2=Kernel2/sum(Kernel2(:));
GsiCenter=weight2(2,2);
GsiMiddle=weight2(1,2);
GsiCorner=weight2(1,1);
minstructure=15; reach=0;