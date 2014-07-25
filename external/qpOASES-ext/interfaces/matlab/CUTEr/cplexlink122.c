/* This is a wrapper for the Cplex MEX file.
 * It is necessary to get accurate timing results.
 */

#include <dlfcn.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include "mex.h"
#include "matrix.h"

char *cplexLibName = "/home/potschka/extra/cplex122/matlab/cplexlink122.mexa64";

static void *cplexHandle = NULL;
static void (*mexCplex)(int, mxArray *a[], int, const mxArray *b[]) = NULL;
static double cumTimeAll = 0.0;
static double cumTimeWOConstr = 0.0;
static double cumTimeSolve = 0.0;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	int i;
	struct timespec now;
	double sooner, later;
	char command[100];

	if (cplexHandle == NULL || mexCplex == NULL)
	{
		cplexHandle = dlopen(cplexLibName, RTLD_NOW | RTLD_LOCAL);
		if (cplexHandle == NULL)
		{
			mexErrMsgTxt("Could not open Cplex library.");
			return;
		}
		mexCplex = dlsym(cplexHandle, "mexFunction");
		if (mexCplex == NULL)
		{
			mexErrMsgTxt("Could not load Cplex MEX function.");
			return;
		}
	}

	if (nrhs == 0) /* we hook in if there are no right hand side arguments */
	{
		if (nlhs >= 1)
			plhs[0] = mxCreateDoubleScalar(cumTimeAll);
		if (nlhs >= 2)
			plhs[1] = mxCreateDoubleScalar(cumTimeWOConstr);
		if (nlhs >= 3)
			plhs[2] = mxCreateDoubleScalar(cumTimeSolve);

		/* reset */
		cumTimeAll = 0.0;
		cumTimeWOConstr = 0.0;
		cumTimeSolve = 0.0;
	}
	else 
	{
		clock_gettime(CLOCK_REALTIME, &now);
		sooner = (double)(now.tv_sec) + 1.0e-9 * (double)(now.tv_nsec);

		mexCplex(nlhs, plhs, nrhs, prhs); /* call original Cplex */

		clock_gettime(CLOCK_REALTIME, &now);
		later = (double)(now.tv_sec) + 1.0e-9 * (double)(now.tv_nsec);

		cumTimeAll += later - sooner;

		mxGetString(prhs[1], command, 99);
		if (strncmp(command, "constructor", 99) != 0)
			cumTimeWOConstr += later - sooner;
		if (strncmp(command, "solve", 99) == 0)
			cumTimeSolve += later - sooner;
	}
}

