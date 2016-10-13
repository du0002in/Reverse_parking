function [status,fdir,fdirnum]=makefolder
prompt='Folder number ';
result=input(prompt,'s');
% folderpath=strcat('D:\DXX\Self Parking\code\real run\daytime\test',result);
folderpath=strcat('D:\DXX\Self Parking\code\real run\horizontal\test',result);
existflag=exist(folderpath);
if existflag==0
    mkdirflag=mkdir(folderpath);
    if mkdirflag~=1
        disp('Folder creation fails');
        status=0;
        fdir='';fdirnum='';
        return;
    else status=1;
        fdir=folderpath;fdirnum=result;
    end
else
    disp('Folder exist');
    status=0;
    fdir='';fdirnum='';
    return;
end