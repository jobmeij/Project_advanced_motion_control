
/*
	TSAVE (Matlab R11/R12 version)

	(c) Rene' van de Molengraft, 2003

	revision history: September, 23th, 2003
			  October, 7th, 2003
			  October, 13th, 2003: moved time series to state

	Input: u[0]	= trigger signal
	       u[1..3]	= signals to be saved
*/

#define S_FUNCTION_NAME tsave3
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define U(element) (*uPtrs[element])  /* Pointer to Input Port0 */
#define LEVEL(S) ssGetSFcnParam(S,0)

#define NSAMP		150000
#define NSIGNALS	3

#define NINPUTS		1+NSIGNALS
#define NDSTATES	(1+NSIGNALS)*NSAMP
#define NOUTPUTS	1
#define NPARAMS		1

#define NRWRK		3
#define NIWRK		3

#define wrk_istart	prwrk[piwrk[0]]
#define wrk_idx		prwrk[piwrk[1]]
#define wrk_t0		prwrk[piwrk[2]]

/*#include "stdio.h" */





/* init globals in work-space for re-entrancy */

int_T rwrk_init_var2(int_T *pivar, int_T *pidx, int_T nrw, SimStruct *S)
{
	int_T *piwrk=ssGetIWork(S);

	piwrk[pivar[0]]=pidx[0];

	pivar[0]++;
	pidx[0]=pidx[0]+nrw;

	return 1;
}





int_T rwrk_init_all2(SimStruct *S)
{
	int_T ivar,idx;

	ivar=0;
	idx=0;

	rwrk_init_var2(&ivar,&idx,1,S);
	rwrk_init_var2(&ivar,&idx,1,S);
	rwrk_init_var2(&ivar,&idx,1,S);

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
	ssSetNumSFcnParams(S,NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
	if (ssGetNumSFcnParams(S)==ssGetSFcnParamsCount(S)) {
	  mdlCheckParameters(S);
	  if (ssGetErrorStatus(S)!=NULL) {
	    return;
	  }
	} else {
	  return; /* Parameter mismatch will be reported by Simulink */
	}
#endif

	ssSetNumContStates(S,0);
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

	int_T i;

	rwrk_init_all2(S);

	wrk_istart=0.0;
	wrk_idx=0.0;
	wrk_t0=0.0;

	for (i=0;i<3*NSAMP;i++) {
		x[i]=0.0;
	}
}





#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
	real_T *x=ssGetDiscStates(S);
	InputRealPtrsType uPtrs=ssGetInputPortRealSignalPtrs(S,0);
	real_T *plevel=mxGetPr(LEVEL(S));

	int_T *piwrk=ssGetIWork(S);
	real_T *prwrk=ssGetRWork(S);

	int_T istart,idx;
	real_T time;

/*	current time */

	time=ssGetT(S);
/*	mexPrintf("t=%f\n",time); */

/*	retrieve istart and idx from workspace */
	istart=(int_T) wrk_istart;
	idx=(int_T) wrk_idx;
/*	mexPrintf("%d   %d\n",istart,idx); */

	if (U(0)>=(*plevel)) {
		if (istart==0) {
/*			mexPrintf("Triggered at level %f\n",U(0)); */
			wrk_t0=U(0);
		}
		istart=1;
	}


	if (istart==1 ) {
		if (idx<NSAMP) {
			x[idx]=U(0)-wrk_t0;
			x[NSAMP+idx]=U(1);
			x[2*NSAMP+idx]=U(2);
			x[3*NSAMP+idx]=U(3);
			idx++;
		}
	}

/*	store istart and idx to workspace */
	wrk_istart=(real_T) istart;
	wrk_idx=(real_T) idx;
}





static void mdlOutputs(SimStruct *S, int_T tid)
{
	real_T *x=ssGetDiscStates(S);
	real_T *y=ssGetOutputPortRealSignal(S,0);

	InputRealPtrsType uPtrs=ssGetInputPortRealSignalPtrs(S,0);

	int_T *piwrk=ssGetIWork(S);
	real_T *prwrk=ssGetRWork(S);

	int_T istart,idx;
	real_T time;

/*	current time */

	time=ssGetT(S);
/*	mexPrintf("t=%f\n",time); */

/*	retrieve istart from workspace */
	istart=(int_T) wrk_istart;

	y[0]=(real_T) istart;
}





static void mdlTerminate(SimStruct *S)
{
	real_T *x=ssGetDiscStates(S);

	FILE *fp;

	int_T i;

	fp=fopen("expdata.dat","w");
	for (i=0;i<NSAMP;i++) {
		if ( (i==0) | (x[i]>0.0) ) {
		    fprintf(fp,"%25.18f   %25.18f   %25.18f   %25.18f\n",x[i],x[NSAMP+i],x[2*NSAMP+i],x[3*NSAMP+i]); 
	    }
	}
	fclose(fp);
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

