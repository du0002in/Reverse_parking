// mex file to calculate gradient
// input:   mxn gray image
// output:  gradient along x (mxn) and y (mxn)
#include "mex.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double *ImgPtr, *Grad_xPtr, *Grad_yPtr;
    mwSize mrows, curcols, precols, nextcols;
    mwSize ncols;
    int i, j;
    
    ImgPtr=mxGetPr(prhs[0]);
    mrows=mxGetM(prhs[0]);
    ncols=mxGetN(prhs[0]);
    plhs[0]=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    plhs[1]=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    Grad_xPtr=mxGetPr(plhs[0]);
    Grad_yPtr=mxGetPr(plhs[1]);
    //Grad_xPtr calculation
    for (i=1;i<(ncols-1);i++)
    {
        curcols=i*mrows; precols=(i-1)*mrows; nextcols=(i+1)*mrows;
        for (j=0;j<mrows;j++)
            *(Grad_xPtr+curcols+j)=0.5*(*(ImgPtr+nextcols+j)-*(ImgPtr+precols+j));
    }
    curcols=(ncols-1)*mrows; precols=(ncols-2)*mrows;
    for (j=0;j<mrows;j++)
    {
        *(Grad_xPtr+j)=*(ImgPtr+mrows+j)-*(ImgPtr+j);
        *(Grad_xPtr+curcols+j)=*(ImgPtr+curcols+j)-*(ImgPtr+precols+j);
    }
    //Grad_yPtr calculation
    for (i=0;i<ncols;i++)
    {
        curcols=i*mrows;
        for (j=1;j<mrows-1;j++)
            *(Grad_yPtr+curcols+j)=0.5*(*(ImgPtr+curcols+j+1)-*(ImgPtr+curcols+j-1));
    }
    for (i=0;i<ncols;i++)
    {
        curcols=i*mrows; nextcols=(i+1)*mrows;
        *(Grad_yPtr+curcols)=*(ImgPtr+curcols+1)-*(ImgPtr+curcols);
        *(Grad_yPtr+nextcols-1)=*(ImgPtr+nextcols-1)-*(ImgPtr+nextcols-2);
    }
}