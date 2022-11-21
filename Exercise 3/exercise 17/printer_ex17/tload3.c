 
/*
	TLOAD (Matlab R11/R12 version)

	(c) Rene' van de Molengraft, 2003

	revision history: September, 24th, 2003
			  October, 13th, 2003

	Input: u[0]	  = trigger signal
	       u[1]       = signal to be loaded
*/

#define S_FUNCTION_NAME tload3
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */
#define LEVEL(S) ssGetSFcnParam(S,0)

#define PHD		ssGetSFcnParam(S,0) 
#define NPARAMS   	1
 
#define NSAMP		150000
#define NSIGNALS	2

#define NINPUTS		1
#define NDSTATES	NSIGNALS*NSAMP
#define NOUTPUTS	NSIGNALS
#define NRWRK		2
#define NIWRK		2

#define wrk_istart	prwrk[piwrk[0]]
#define wrk_idx		prwrk[piwrk[1]]

/*#include "stdio.h"*/





/* init globals in work-space for re-entrancy */

int_T rwrk_init_var3(int_T *pivar, int_T *pidx, int_T nrw, SimStruct *S)
{
	int_T *piwrk=ssGetIWork(S);

	piwrk[pivar[0]]=pidx[0];

	pivar[0]++;
	pidx[0]=pidx[0]+nrw;

	return 1;
}





int_T rwrk_init_all3(SimStruct *S)
{
	int_T ivar,idx;

	ivar=0;
	idx=0;

	rwrk_init_var3(&ivar,&idx,1,S);
	rwrk_init_var3(&ivar,&idx,1,S);

/*	mexPrintf("tsave3: ivar=%d     idx=%d\n",ivar,idx); */

	return 1;
}





/*====================*
 * S-function methods *
 *====================*/

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
/*	check parameter */
}
#endif /* MDL_CHECK_PARAMETERS */

static void mdlInitializeSizes(SimStruct *S)
{
	ssSetNumSFcnParams(S,NPARAMS);

	ssSetNumDiscStates(S,0);
	ssSetNumDiscStates(S,NDSTATES);

	if (!ssSetNumInputPorts(S,1)) return;
	ssSetInputPortWidth(S,0,NINPUTS);
	ssSetInputPortDirectFeedThrough(S,0,NINPUTS);

	if (!ssSetNumOutputPorts(S,1)) return;
	ssSetOutputPortWidth(S,0,NOUTPUTS);

	ssSetNumSampleTimes(S,1);
	ssSetNumRWork(S,NRWRK);
	ssSetNumIWork(S,NIWRK);
	ssSetNumPWork(S,0);
	ssSetNumModes(S,0);
	ssSetNumNonsampledZCs(S,0);
}





static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S,0,CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S,0,0.0);
}





#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
	real_T *x=ssGetDiscStates(S);

	int_T *piwrk=ssGetIWork(S);
	real_T *prwrk=ssGetRWork(S);

	int_T i,nitems;

	FILE *fp;
	double ttt1,ttt2;

	rwrk_init_all3(S);

	wrk_istart=0.0;
	wrk_idx=0.0;

	for (i=0;i<NSAMP;i++) {
		x[i]=0.0;
	}

	fp=fopen("loaddata.dat","r");
	nitems=1;
	i=0;
	while (nitems>0 & i<NSAMP) {
		nitems=fscanf(fp,"%lf   %lf\n",&ttt1,&ttt2);
		*(x+i)=ttt1;
		*(x+NSAMP+i)=ttt2;
		i++;
	}
/*	printf("%d lines scanned.\n",i); */
	fclose(fp);

	wrk_istart=0.0;
	wrk_idx=0.0;
}





#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
}





static void mdlOutputs(SimStruct *S, int_T tid)
{
	real_T *plevel=mxGetPr(LEVEL(S)); 
	real_T *x=ssGetDiscStates(S);
	real_T *y=ssGetOutputPortRealSignal(S,0);

	InputRealPtrsType uPtrs=ssGetInputPortRealSignalPtrs(S,0);

	int_T *piwrk=ssGetIWork(S);
	real_T *prwrk=ssGetRWork(S);

	int_T istart,idx;

/*	retrieve istart and idx from workspace */
	istart=(int_T) wrk_istart;
	idx=(int_T) wrk_idx;

	if (U(0)>=(*plevel)) {
		if (istart==0) {
/*			printf("started.\n");*/
		}
		istart=1;
	}

	if (istart==1) {
		if (idx<NSAMP) {
			y[0]=x[idx];
			y[1]=x[NSAMP+idx];
			idx++;
		}
	}

/* 	store istart and idx to workspace */
	wrk_istart=(real_T) istart;
	wrk_idx=(real_T) idx;
}





static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
