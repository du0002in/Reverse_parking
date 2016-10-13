function status=get_comm_to_Linux_status
%0 means initial state, prior to starting communication
%1 means communication established successfully
%2 means port not able to open
%3 means not able to get into STEER Remote Control mode
%4 means not able to get into DRIVE Remote Control mode
ele_no=0;
while(ele_no==0) %in case when the file is being written, it will read in nothing
    fid=fopen('D:\DXX\Self Parking\code\comm_to_Linux_status.txt','rt');
    C=textscan(fid, '%d');
    ele_no=numel(C{1});
    fclose(fid);
end
status=C{1};
end