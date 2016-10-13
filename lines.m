%Function to generate all lines with input points
function [c e f]=lines(x,y)
Lines=combnk(1:length(x),2);
NoLines=size(Lines,1);
c=ones(NoLines,1);
e=ones(NoLines,1);
f=ones(NoLines,1);
for i=1:NoLines
    if x(Lines(i,1))==x(Lines(i,2))
        e(i)=0; c(i)=1; f(i)=-2*x(Lines(i,1));
    else
        c(i)=-(y(Lines(i,1))-y(Lines(i,2)))/(x(Lines(i,1))-x(Lines(i,2)));
        f(i)=2*(-c(i)*x(Lines(i,1))-y(Lines(i,1)));
    end
end
end