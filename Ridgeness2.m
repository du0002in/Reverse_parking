function kcsditmp=Ridgeness2(Img,xfil2,ix,yfil2,iy,sy,GsiCenter, GsiMiddle, GsiCorner,ImgSize1, ImgSize2,smoothsize)
ImgSmooth1=myconv2(Img,xfil2,ix,yfil2,iy,sy,smoothsize,ImgSize1,ImgSize2);
%compute the gradient vector field
[Gr_Lsd_x, Gr_Lsd_y]=C_Gradient(ImgSmooth1);
%compute the structure tensor field Ssdi
ssd11=Gr_Lsd_x.^2;
ssd12=Gr_Lsd_x.*Gr_Lsd_y;
ssd22=Gr_Lsd_y.^2;
Ssdi12=ssd12*GsiCenter+(ssd11+ssd22)*GsiMiddle+ssd12*GsiCorner;
Ssdi11_22=(ssd11-ssd22).*(GsiCenter-GsiCorner);
%calculate the max eignvector of Ssdi
EigenValue_22D12=0.5*(Ssdi11_22+sqrt((Ssdi11_22).^2+4*Ssdi12.^2))./Ssdi12;
EigenValue_22D12(isnan(EigenValue_22D12))=0;
wpsdiV=sqrt(1./(1+EigenValue_22D12.^2));
wpsdiU=wpsdiV.*EigenValue_22D12;
%     wpsdiV(isnan(wpsdiV))=0;
%     wpsdiU(isnan(wpsdiU))=0;
psdi=sign(wpsdiU.*Gr_Lsd_x+wpsdiV.*Gr_Lsd_y);
U=psdi.*wpsdiU;
V=psdi.*wpsdiV;
%ridgeness measurement calculate negtive divergence kcsdi
kcsdi=C_Divergence(U,V);
kcsditmp=kcsdi;
%threshhold the ridgeness and binarize the ridgeness image
kcsditmp(kcsdi>=0.7)=1;
kcsditmp(kcsdi<0.7)=0;
kcsditmp([1:smoothsize+4,ImgSize1-smoothsize-4:ImgSize1],:)=0;
kcsditmp(:,[1:smoothsize+4,ImgSize2-smoothsize-4:ImgSize2])=0;
labelCC=bwconncomp(kcsditmp);
bb=cell(1,labelCC.NumObjects);
bb(:)={[ImgSize1 ImgSize2]};
[r c]=cellfun(@ind2sub,bb,labelCC.PixelIdxList,'UniformOutput',false);
deltar=cellfun(@peak2peak,r);
deltac=cellfun(@peak2peak,c);
lensqr=deltar.^2+deltac.^2;
LenCheck=labelCC.PixelIdxList(lensqr<=625);
kcsditmp(cat(1,LenCheck{:}))=0;
end