RidgeParameters2;
global_varibles;
InitializationGlobalVariable;
buffer=0.0; consec_check=0; pre_check=0;pre_check_d3=0;consec_d3=0;resetcounter=0;
consec_d24=0;pathcounter=0;reset_readjust_flag=0;dist2travel=0;pre_segment=0;
check_readjust=1;
check_collition=1;
targetPhai=atan(L_wheels/Rmin);
direction=1; theta_pre=-pi/4; usr_ip=0;
C_sendtargetParking2('Ini',[0,0.4],0.02);
for i=1:130
    tic;
    Img=imread(['D:\DXX\Self Parking\code\real run\test1\left' int2str(i) '.jpg']);
    Img1=double(max(Img,[],3));
    kcsditmp=Ridgeness2(Img1,xfil2,ix,yfil2,iy,sy,GsiCenter, GsiMiddle, GsiCorner,ImgSize1, ImgSize2,smoothsize);
    [NoofLines, C, E, Fp, PCount, act_line]=sequentialRANSAC_SelfParking2(kcsditmp, Img1, direction,theta_pre);
    Img=imread(['D:\DXX\Self Parking\code\real run\test1\left' int2str(i) '.jpg']);
    Img1=double(max(Img,[],3));
    kcsditmp=Ridgeness2(Img1,xfil2,ix,yfil2,iy,sy,GsiCenter, GsiMiddle, GsiCorner,ImgSize1, ImgSize2,smoothsize);
    [NoofLines, C, E, Fp, PCount, act_line]=sequentialRANSAC_SelfParking2(kcsditmp, Img1, direction,theta_pre);
    V_Phi_t=[0.1, rand(1)];
    C_sendtargetParking2('Con',V_Phi_t,0.02);
    logt(i)=toc;
end
C_sendtargetParking2('End',V_Phi_t,0.02);
    