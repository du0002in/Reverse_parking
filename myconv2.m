function ImgSmooth=myconv2(Img,xfil2,ix,yfil2,iy,sy,smoothsize,ImgSize1,ImgSize2)
Imgtmp=Img;
ImgSmooth=zeros(ImgSize1,ImgSize2);
for i=1:(length(ix)-1)
    ImgSmooth(ix(i):(ix(i+1)-1),:)=conv2(Imgtmp(ix(i):(ix(i+1)-1),:),xfil2{i},'same');
end
%to be conservative, we pad the array with the max smooth size
Imgtmp=[zeros(25,ImgSize2); ImgSmooth; zeros(25,ImgSize2)];
for i=1:(length(iy)-1)
    ImgSmooth(iy(i):(iy(i+1)-1),:)=conv2(Imgtmp([iy(i)+smoothsize-sy(iy(i)):iy(i+1)-1+smoothsize+sy(iy(i))],:),yfil2{i},'valid');
end
end