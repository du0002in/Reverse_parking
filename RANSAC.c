// mex file to run RANSAC to improve speed
// input: 2xn matrix with 1st row being x and 2nd row being y. n refers to the number of points
// ouput: 1xn matrix (or row vector) to indicate whether the corresponding points are inliers (1) or not (0)
#include "mex.h"
#include <math.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double *Datapoint, v[2], u[2], e, c, f, fp, deno, *FinalLine, *distsqr_ptr, *line_ptr;
    size_t mrows, ncols;
    size_t Randpoint[2];
    int i,j, tcon=0, tcon_pre=0;
    mxArray *line, *distsqr;
    
	srand(time(NULL));
    Datapoint=mxGetPr(prhs[0]);
    ncols=mxGetN(prhs[0]);
    plhs[0]=mxCreateDoubleMatrix(1,ncols,mxREAL);
    FinalLine=mxGetPr(plhs[0]);
    Randpoint[0]=0; Randpoint[1]=0;
    distsqr=mxCreateDoubleMatrix(1,(int)ncols,mxREAL);
    distsqr_ptr=mxGetPr(distsqr);
    line=mxCreateDoubleMatrix(1,ncols,mxREAL);
    line_ptr=mxGetPr(line);
    for(j=0;j<50;j++)
    {
		tcon=0;
		Randpoint[0]=rand() % ncols;
		Randpoint[1]=rand() % ncols;
        while(Randpoint[0]==Randpoint[1])
            Randpoint[1]=rand() % ncols;
        u[0]=*(Datapoint+2*Randpoint[0]);
        v[0]=*(Datapoint+2*Randpoint[0]+1);
        u[1]=*(Datapoint+2*Randpoint[1]);
        v[1]=*(Datapoint+2*Randpoint[1]+1);
        if (u[0]==u[1])
        {
            e=0; c=1; f=-2*u[0]; fp=f/2;
        }
        else
        {
            c=-(v[0]-v[1])/(u[0]-u[1]);
            e=1;
            f=2*(-c*u[0]-v[0]);
            fp=f/2;
        }
        deno=c*c+e*e;

        for (i=0;i<ncols;i++)
        {
            *(distsqr_ptr+i)=pow(c**(Datapoint+2*i)+e*(*(Datapoint+2*i+1))+fp,2)/deno;
            if (*(distsqr_ptr+i)<10){
				tcon++;
				*(line_ptr+i)=1;}
            else
                *(line_ptr+i)=0;
        }
		if (tcon<50)
			continue;
		if (tcon>tcon_pre)
		{
			tcon_pre=tcon;
			for (i=0;i<ncols;i++)
				*(FinalLine+i)=*(line_ptr+i);
			if (tcon>0.9*ncols)
				break;
		}    
    }
	mxDestroyArray(line);
    mxDestroyArray(distsqr);

}

            