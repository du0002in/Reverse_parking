function sendtarget(v_t, phai_t)
fid=fopen('D:\DXX\Self Parking\code\manuver.txt','wt');
formatSpec='tv %4.3f\ntp %4.3f';
fprintf(fid,formatSpec,v_t, phai_t);
fclose(fid);
end