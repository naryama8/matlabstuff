#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME DC_MOTOR
#include "simstruc.h"
#include <math.h>

#define U(element) (*uPtrs[element]) /*Pointer to Input Port0*/

static void mdlInitializeSizes(SimStruct *S){
    ssSetNumContStates(S, 3);
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 2);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 3);
    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE); }

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0); }

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S) {

    real_T *X0 = ssGetContStates(S);
    int_T nStates = ssGetNumContStates(S);
    int_T i;

    /* initialize the states to 0.0 */
    for (i=0; i < nStates; i++) {X0[i] = 0.0;} }

static void mdlOutputs(SimStruct *S, int_T tid) {
    real_T *Y = ssGetOutputPortRealSignal(S,0);
    real_T *X = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    Y[0] = X[0];
    Y[1] = X[1];
    Y[2] = X[2];

}

#define MDL_DERIVATIVES
static void mdlDerivatives(SimStruct *S) {
    real_T *dX = ssGetdX(S);
    real_T *X = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    real_T Kt, Ke, R, J, L, theta, theta_dot, omega, omega_dot, current, current_dot, Ct, Tl;

    Kt = 0.235;
    Ke = 0.235;
    R = 2;
    J = 0.000052;
    L = 0.23;

    Ct = U(0);
    Tl = U(1);

    theta = X[0];
    omega = X[1];
    current = X[2];

    theta_dot = omega;
    omega_dot = (Kt / J) * current -(1 / J) * Tl;
    current_dot = -(Ke / L) * omega - (R / L) * current + (1 / L) * Ct;

    dX[0] = theta_dot;
    dX[1] = omega_dot;
    dX[2] = current_dot;

}

static void mdlTerminate(SimStruct *S)
{} /*Keep this function empty since no memory is allocated*/

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /*Code generation registration function*/
#endif
