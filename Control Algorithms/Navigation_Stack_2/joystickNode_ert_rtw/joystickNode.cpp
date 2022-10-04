/*
 * joystickNode.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "joystickNode".
 *
 * Model version              : 1.0
 * Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
 * C++ source code generated on : Tue Oct  4 14:18:52 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "joystickNode.h"
#include "joystickNode_private.h"

/* Block signals (default storage) */
B_joystickNode_T joystickNode_B;

/* Block states (default storage) */
DW_joystickNode_T joystickNode_DW;

/* Real-time model */
RT_MODEL_joystickNode_T joystickNode_M_ = RT_MODEL_joystickNode_T();
RT_MODEL_joystickNode_T *const joystickNode_M = &joystickNode_M_;

/* Forward declaration for local functions */
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* Model step function */
void joystickNode_step(void)
{
  int32_T i;

  /* S-Function (joyinput): '<Root>/Joystick Input' */

  /* Level2 S-Function Block: '<Root>/Joystick Input' (joyinput) */
  {
    SimStruct *rts = joystickNode_M->childSfunctions[0];
    sfcnOutputs(rts,0);
  }

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  Constant: '<S1>/Constant'
   *  DataTypeConversion: '<Root>/Data Type Conversion1'
   */
  joystickNode_B.BusAssignment = joystickNode_P.Constant_Value;
  memcpy(&joystickNode_B.BusAssignment.Data[0],
         &joystickNode_B.JoystickInput_o1[0], sizeof(real_T) << 3U);
  for (i = 0; i < 13; i++) {
    joystickNode_B.BusAssignment.Data[i + 8] = joystickNode_B.JoystickInput_o2[i];
  }

  joystickNode_B.BusAssignment.Data_SL_Info.CurrentLength =
    joystickNode_B.ProbeWidth;

  /* End of BusAssignment: '<Root>/Bus Assignment' */

  /* Outputs for Atomic SubSystem: '<Root>/Publish' */
  /* MATLABSystem: '<S2>/SinkBlock' */
  Pub_joystickNode_8.publish(&joystickNode_B.BusAssignment);

  /* End of Outputs for SubSystem: '<Root>/Publish' */

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++joystickNode_M->Timing.clockTick0)) {
    ++joystickNode_M->Timing.clockTickH0;
  }

  joystickNode_M->Timing.t[0] = joystickNode_M->Timing.clockTick0 *
    joystickNode_M->Timing.stepSize0 + joystickNode_M->Timing.clockTickH0 *
    joystickNode_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void joystickNode_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  rtsiSetSolverName(&joystickNode_M->solverInfo,"FixedStepDiscrete");
  joystickNode_M->solverInfoPtr = (&joystickNode_M->solverInfo);

  /* Initialize timing info */
  {
    int_T *mdlTsMap = joystickNode_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    joystickNode_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    joystickNode_M->Timing.sampleTimes =
      (&joystickNode_M->Timing.sampleTimesArray[0]);
    joystickNode_M->Timing.offsetTimes =
      (&joystickNode_M->Timing.offsetTimesArray[0]);

    /* task periods */
    joystickNode_M->Timing.sampleTimes[0] = (0.2);

    /* task offsets */
    joystickNode_M->Timing.offsetTimes[0] = (0.0);
  }

  rtmSetTPtr(joystickNode_M, &joystickNode_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = joystickNode_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    joystickNode_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(joystickNode_M, -1);
  joystickNode_M->Timing.stepSize0 = 0.2;
  joystickNode_M->solverInfoPtr = (&joystickNode_M->solverInfo);
  joystickNode_M->Timing.stepSize = (0.2);
  rtsiSetFixedStepSize(&joystickNode_M->solverInfo, 0.2);
  rtsiSetSolverMode(&joystickNode_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  (void) memset((static_cast<void *>(&joystickNode_B)), 0,
                sizeof(B_joystickNode_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&joystickNode_DW), 0,
                sizeof(DW_joystickNode_T));

  /* child S-Function registration */
  {
    RTWSfcnInfo *sfcnInfo = &joystickNode_M->NonInlinedSFcns.sfcnInfo;
    joystickNode_M->sfcnInfo = (sfcnInfo);
    rtssSetErrorStatusPtr(sfcnInfo, (&rtmGetErrorStatus(joystickNode_M)));
    rtssSetNumRootSampTimesPtr(sfcnInfo, &joystickNode_M->Sizes.numSampTimes);
    joystickNode_M->NonInlinedSFcns.taskTimePtrs[0] = &(rtmGetTPtr
      (joystickNode_M)[0]);
    rtssSetTPtrPtr(sfcnInfo,joystickNode_M->NonInlinedSFcns.taskTimePtrs);
    rtssSetTStartPtr(sfcnInfo, &rtmGetTStart(joystickNode_M));
    rtssSetTFinalPtr(sfcnInfo, &rtmGetTFinal(joystickNode_M));
    rtssSetTimeOfLastOutputPtr(sfcnInfo, &rtmGetTimeOfLastOutput(joystickNode_M));
    rtssSetStepSizePtr(sfcnInfo, &joystickNode_M->Timing.stepSize);
    rtssSetStopRequestedPtr(sfcnInfo, &rtmGetStopRequested(joystickNode_M));
    rtssSetDerivCacheNeedsResetPtr(sfcnInfo,
      &joystickNode_M->derivCacheNeedsReset);
    rtssSetZCCacheNeedsResetPtr(sfcnInfo, &joystickNode_M->zCCacheNeedsReset);
    rtssSetContTimeOutputInconsistentWithStateAtMajorStepPtr(sfcnInfo,
      &joystickNode_M->CTOutputIncnstWithState);
    rtssSetSampleHitsPtr(sfcnInfo, &joystickNode_M->Timing.sampleHits);
    rtssSetPerTaskSampleHitsPtr(sfcnInfo,
      &joystickNode_M->Timing.perTaskSampleHits);
    rtssSetSimModePtr(sfcnInfo, &joystickNode_M->simMode);
    rtssSetSolverInfoPtr(sfcnInfo, &joystickNode_M->solverInfoPtr);
  }

  joystickNode_M->Sizes.numSFcns = (1);

  /* register each child */
  {
    (void) memset(static_cast<void *>
                  (&joystickNode_M->NonInlinedSFcns.childSFunctions[0]), 0,
                  1*sizeof(SimStruct));
    joystickNode_M->childSfunctions =
      (&joystickNode_M->NonInlinedSFcns.childSFunctionPtrs[0]);
    joystickNode_M->childSfunctions[0] =
      (&joystickNode_M->NonInlinedSFcns.childSFunctions[0]);

    /* Level2 S-Function Block: joystickNode/<Root>/Joystick Input (joyinput) */
    {
      SimStruct *rts = joystickNode_M->childSfunctions[0];

      /* timing info */
      time_T *sfcnPeriod = joystickNode_M->NonInlinedSFcns.Sfcn0.sfcnPeriod;
      time_T *sfcnOffset = joystickNode_M->NonInlinedSFcns.Sfcn0.sfcnOffset;
      int_T *sfcnTsMap = joystickNode_M->NonInlinedSFcns.Sfcn0.sfcnTsMap;
      (void) memset(static_cast<void*>(sfcnPeriod), 0,
                    sizeof(time_T)*1);
      (void) memset(static_cast<void*>(sfcnOffset), 0,
                    sizeof(time_T)*1);
      ssSetSampleTimePtr(rts, &sfcnPeriod[0]);
      ssSetOffsetTimePtr(rts, &sfcnOffset[0]);
      ssSetSampleTimeTaskIDPtr(rts, sfcnTsMap);

      {
        ssSetBlkInfo2Ptr(rts, &joystickNode_M->NonInlinedSFcns.blkInfo2[0]);
      }

      _ssSetBlkInfo2PortInfo2Ptr(rts,
        &joystickNode_M->NonInlinedSFcns.inputOutputPortInfo2[0]);

      /* Set up the mdlInfo pointer */
      ssSetRTWSfcnInfo(rts, joystickNode_M->sfcnInfo);

      /* Allocate memory of model methods 2 */
      {
        ssSetModelMethods2(rts, &joystickNode_M->NonInlinedSFcns.methods2[0]);
      }

      /* Allocate memory of model methods 3 */
      {
        ssSetModelMethods3(rts, &joystickNode_M->NonInlinedSFcns.methods3[0]);
      }

      /* Allocate memory of model methods 4 */
      {
        ssSetModelMethods4(rts, &joystickNode_M->NonInlinedSFcns.methods4[0]);
      }

      /* Allocate memory for states auxilliary information */
      {
        ssSetStatesInfo2(rts, &joystickNode_M->NonInlinedSFcns.statesInfo2[0]);
        ssSetPeriodicStatesInfo(rts,
          &joystickNode_M->NonInlinedSFcns.periodicStatesInfo[0]);
      }

      /* outputs */
      {
        ssSetPortInfoForOutputs(rts,
          &joystickNode_M->NonInlinedSFcns.Sfcn0.outputPortInfo[0]);
        _ssSetNumOutputPorts(rts, 2);
        _ssSetPortInfo2ForOutputUnits(rts,
          &joystickNode_M->NonInlinedSFcns.Sfcn0.outputPortUnits[0]);
        ssSetOutputPortUnit(rts, 0, 0);
        ssSetOutputPortUnit(rts, 1, 0);
        _ssSetPortInfo2ForOutputCoSimAttribute(rts,
          &joystickNode_M->NonInlinedSFcns.Sfcn0.outputPortCoSimAttribute[0]);
        ssSetOutputPortIsContinuousQuantity(rts, 0, 0);
        ssSetOutputPortIsContinuousQuantity(rts, 1, 0);

        /* port 0 */
        {
          _ssSetOutputPortNumDimensions(rts, 0, 1);
          ssSetOutputPortWidth(rts, 0, 8);
          ssSetOutputPortSignal(rts, 0, ((real_T *)
            joystickNode_B.JoystickInput_o1));
        }

        /* port 1 */
        {
          _ssSetOutputPortNumDimensions(rts, 1, 1);
          ssSetOutputPortWidth(rts, 1, 13);
          ssSetOutputPortSignal(rts, 1, ((boolean_T *)
            joystickNode_B.JoystickInput_o2));
        }
      }

      /* path info */
      ssSetModelName(rts, "Joystick Input");
      ssSetPath(rts, "joystickNode/Joystick Input");
      ssSetRTModel(rts,joystickNode_M);
      ssSetParentSS(rts, (NULL));
      ssSetRootSS(rts, rts);
      ssSetVersion(rts, SIMSTRUCT_VERSION_LEVEL2);

      /* parameters */
      {
        mxArray **sfcnParams = (mxArray **)
          &joystickNode_M->NonInlinedSFcns.Sfcn0.params;
        ssSetSFcnParamsCount(rts, 3);
        ssSetSFcnParamsPtr(rts, &sfcnParams[0]);
        ssSetSFcnParam(rts, 0, (mxArray*)joystickNode_P.JoystickInput_P1_Size);
        ssSetSFcnParam(rts, 1, (mxArray*)joystickNode_P.JoystickInput_P2_Size);
        ssSetSFcnParam(rts, 2, (mxArray*)joystickNode_P.JoystickInput_P3_Size);
      }

      /* work vectors */
      ssSetIWork(rts, (int_T *) &joystickNode_DW.JoystickInput_IWORK[0]);
      ssSetPWork(rts, (void **) &joystickNode_DW.JoystickInput_PWORK);

      {
        struct _ssDWorkRecord *dWorkRecord = (struct _ssDWorkRecord *)
          &joystickNode_M->NonInlinedSFcns.Sfcn0.dWork;
        struct _ssDWorkAuxRecord *dWorkAuxRecord = (struct _ssDWorkAuxRecord *)
          &joystickNode_M->NonInlinedSFcns.Sfcn0.dWorkAux;
        ssSetSFcnDWork(rts, dWorkRecord);
        ssSetSFcnDWorkAux(rts, dWorkAuxRecord);
        _ssSetNumDWork(rts, 2);

        /* IWORK */
        ssSetDWorkWidth(rts, 0, 14);
        ssSetDWorkDataType(rts, 0,SS_INTEGER);
        ssSetDWorkComplexSignal(rts, 0, 0);
        ssSetDWork(rts, 0, &joystickNode_DW.JoystickInput_IWORK[0]);

        /* PWORK */
        ssSetDWorkWidth(rts, 1, 1);
        ssSetDWorkDataType(rts, 1,SS_POINTER);
        ssSetDWorkComplexSignal(rts, 1, 0);
        ssSetDWork(rts, 1, &joystickNode_DW.JoystickInput_PWORK);
      }

      /* registration */
      joyinput(rts);
      sfcnInitializeSizes(rts);
      sfcnInitializeSampleTimes(rts);

      /* adjust sample time */
      ssSetSampleTime(rts, 0, 0.2);
      ssSetOffsetTime(rts, 0, 0.0);
      sfcnTsMap[0] = 0;

      /* set compiled values of dynamic vector attributes */
      ssSetNumNonsampledZCs(rts, 0);

      /* Update connectivity flags for each port */
      _ssSetOutputPortConnected(rts, 0, 1);
      _ssSetOutputPortConnected(rts, 1, 1);
      _ssSetOutputPortBeingMerged(rts, 0, 0);
      _ssSetOutputPortBeingMerged(rts, 1, 0);

      /* Update the BufferDstPort flags for each input port */
    }
  }

  {
    char_T tmp[14];
    int32_T i;
    static const char_T tmp_0[13] = { '/', 'a', 'm', 'r', '/', 'j', 'o', 'y',
      's', 't', 'i', 'c', 'k' };

    /* Start for S-Function (joyinput): '<Root>/Joystick Input' */
    /* Level2 S-Function Block: '<Root>/Joystick Input' (joyinput) */
    {
      SimStruct *rts = joystickNode_M->childSfunctions[0];
      sfcnStart(rts);
      if (ssGetErrorStatus(rts) != (NULL))
        return;
    }

    /* Start for Probe: '<Root>/Probe Width' */
    joystickNode_B.ProbeWidth = 21U;

    /* Start for Atomic SubSystem: '<Root>/Publish' */
    /* Start for MATLABSystem: '<S2>/SinkBlock' */
    joystickNode_DW.obj.matlabCodegenIsDeleted = false;
    joystickNode_DW.objisempty = true;
    joystickNode_DW.obj.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp[i] = tmp_0[i];
    }

    tmp[13] = '\x00';
    Pub_joystickNode_8.createPublisher(tmp, 1);
    joystickNode_DW.obj.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S2>/SinkBlock' */
    /* End of Start for SubSystem: '<Root>/Publish' */
  }
}

/* Model terminate function */
void joystickNode_terminate(void)
{
  /* Terminate for S-Function (joyinput): '<Root>/Joystick Input' */
  /* Level2 S-Function Block: '<Root>/Joystick Input' (joyinput) */
  {
    SimStruct *rts = joystickNode_M->childSfunctions[0];
    sfcnTerminate(rts);
  }

  /* Terminate for Atomic SubSystem: '<Root>/Publish' */
  /* Terminate for MATLABSystem: '<S2>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&joystickNode_DW.obj);

  /* End of Terminate for SubSystem: '<Root>/Publish' */
}
