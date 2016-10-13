// mex file to calculate gradient
// input:   2 mxn structure tensor field matrix U and V
// output:  mxn divergence matrix
#include "mex.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double *UPtr, *VPtr, *DivPtr, *dUPtr, *dVPtr;
    mxArray *dU, *dV;
    mwSize mrows, curcols, precols, nextcols;
    mwSize ncols;
    int i, j;
    
    UPtr=mxGetPr(prhs[0]);
    mrows=mxGetM(prhs[0]);
    ncols=mxGetN(prhs[0]);
    VPtr=mxGetPr(prhs[1]);
    plhs[0]=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    dU=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    dV=mxCreateDoubleMatrix(mrows, ncols, mxREAL);
    DivPtr=mxGetPr(plhs[0]);
    dUPtr=mxGetPr(dU);
    dVPtr=mxGetPr(dV);
    //dU calculation
    for (i=1;i<(ncols-1);i++)
    {
        curcols=i*mrows; precols=(i-1)*mrows; nextcols=(i+1)*mrows;
        for (j=0;j<mrows;j++)
            *(dUPtr+curcols+j)=0.5*(*(UPtr+nextcols+j)-*(UPtr+precols+j));
    }
    curcols=(ncols-1)*mrows; precols=(ncols-2)*mrows;
    for (j=0;j<mrows;j++)
    {
        *(dUPtr+j)=*(UPtr+mrows+j)-*(UPtr+j);
        *(dUPtr+curcols+j)=*(UPtr+curcols+j)-*(UPtr+precols+j);
    }
    //dV calculation
    for (i=0;i<ncols;i++)
    {
        curcols=i*mrows;
        for (j=1;j<mrows-1;j++)
        {
            *(dVPtr+curcols+j)=0.5*(*(VPtr+curcols+j+1)-*(VPtr+curcols+j-1));
            *(DivPtr+curcols+j)=-(*(dVPtr+curcols+j)+*(dUPtr+curcols+j));
        }
    }
    for (i=0;i<ncols;i++)
    {
        curcols=i*mrows; nextcols=(i+1)*mrows;
        *(dVPtr+curcols)=*(VPtr+curcols+1)-*(VPtr+curcols);
        *(DivPtr+curcols)=-(*(dVPtr+curcols)+*(dUPtr+curcols));
        *(dVPtr+nextcols-1)=*(VPtr+nextcols-1)-*(VPtr+nextcols-2);
        *(DivPtr+nextcols-1)=-(*(dVPtr+nextcols-1)+*(dUPtr+nextcols-1));
    }
}