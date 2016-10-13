function set_comm_to_Linux(i)
%1 means starts communication.
%0 means close communication.
fid=fopen('D:\DXX\Self Parking\code\comm_to_Linux.txt','wt');
formatSpec='%d';
fprintf(fid,formatSpec,i);
fclose(fid);
end