function set_comm_to_Linux_status(i)
%0 means initial state, prior to starting communication
%1 means communication established successfully
%2 means port not able to open
%3 means not able to get into STEER Remote Control mode
%4 means not able to get into DRIVE Remote Control mode
fid=fopen('D:\DXX\Self Parking\code\comm_to_Linux_status.txt','wt');
formatSpec='%d';
fprintf(fid,formatSpec,i);
fclose(fid);
end