path='C:\Users\a0107257\Documents\MATLAB\ImageProcessingLearning\Pictures\Self Parking\code\real run\test13\FrameLog'
RidgeParameters;figure;
for ii=83:83
Itmp1=imread(strcat(path,int2str(ii),'.jpg'),'jpg');
Img1=double(max(Itmp1,[],3));
subplot(1,3,1);imshow(Img1,[]);
kcsditmp=Ridgeness(Img1,weight,GsiCenter, GsiMiddle, GsiCorner, minstructure, ImgSize1, ImgSize2,smoothsize);
subplot(1,3,3); imshow(kcsditmp);
end