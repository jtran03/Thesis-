/*
 * nav3v2.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "nav3v2".
 *
 * Model version              : 1.57
 * Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
 * C++ source code generated on : Fri Oct 14 15:28:34 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "nav3v2.h"
#include "nav3v2_private.h"

/* Named constants for Chart: '<S2>/State_Machine' */
const uint8_T IN_FinishedReachingAllWaypoin_g = 1U;
const uint8_T IN_FinishedReachingAllWaypoints = 1U;
const uint8_T na_IN_PurePursuitMoveToWaypoint = 4U;
const uint8_T na_IN_ReverseRotateToWaypoint_o = 7U;
const uint8_T nav3_IN_PreciseRotateToWaypoint = 3U;
const uint8_T nav3_IN_ReverseMoveToWaypoint_a = 6U;
const uint8_T nav3_IN_ReverseRotateToWaypoint = 5U;
const uint8_T nav3v2_IN_AUTONOMOUS_CONTROL = 1U;
const uint8_T nav3v2_IN_EMERGENCY = 2U;
const uint8_T nav3v2_IN_INITIALISATION = 3U;
const uint8_T nav3v2_IN_MANUAL_CONTROL = 4U;
const uint8_T nav3v2_IN_MOVEBASE = 5U;
const uint8_T nav3v2_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T nav3v2_IN_PreciseMoveToWaypoint = 2U;
const uint8_T nav3v2_IN_REVERSE = 6U;
const uint8_T nav3v2_IN_ReachedWaypoint = 3U;
const uint8_T nav3v2_IN_ReachedWaypoint_b = 5U;
const uint8_T nav3v2_IN_ReverseMoveToWaypoint = 4U;
const uint8_T nav3v2_IN_RotateToWaypoint = 8U;
const uint8_T nav3v2_IN_WAITING = 7U;
const uint8_T nav_IN_PreciseRotateToWaypoint1 = 2U;

/* Block signals (default storage) */
B_nav3v2_T nav3v2_B;

/* Block states (default storage) */
DW_nav3v2_T nav3v2_DW;

/* Real-time model */
RT_MODEL_nav3v2_T nav3v2_M_ = RT_MODEL_nav3v2_T();
RT_MODEL_nav3v2_T *const nav3v2_M = &nav3v2_M_;

/* Forward declaration for local functions */
static boolean_T nav3v2_anyNonFinite(const real_T x[16]);
static real_T nav3v2_rt_hypotd_snf(real_T u0, real_T u1);
static real_T nav3v2_xzlangeM(const creal_T x[16]);
static void nav3v2_xzlascl(real_T cfrom, real_T cto, creal_T A[16]);
static real_T nav3v2_xzlanhs(const creal_T A[16], int32_T ilo, int32_T ihi);
static void nav3v2_xzlartg_g(const creal_T f, const creal_T g, real_T *cs,
  creal_T *sn);
static void nav3v2_xzlartg(const creal_T f, const creal_T g, real_T *cs, creal_T
  *sn, creal_T *r);
static void nav3v2_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi, creal_T Z[16],
  int32_T *info, creal_T alpha1[4], creal_T beta1[4]);
static void nav3v2_xztgevc(const creal_T A[16], creal_T V[16]);
static void nav3v2_xzggev(creal_T A[16], int32_T *info, creal_T alpha1[4],
  creal_T beta1[4], creal_T V[16]);
static real_T nav3v2_xnrm2(int32_T n, const real_T x[16], int32_T ix0);
static void nav3v2_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau, real_T
  C[16], int32_T ic0, real_T work[4]);
static void nav3v2_xgehrd(real_T a[16], real_T tau[3]);
static real_T nav3v2_xnrm2_j(int32_T n, const real_T x[3]);
static real_T nav3v2_xzlarfg(int32_T n, real_T *alpha1, real_T x[3]);
static void nav3v2_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *d, real_T
  *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *sn);
static void nav3v2_xrot(int32_T n, real_T x[16], int32_T ix0, int32_T iy0,
  real_T c, real_T s);
static void nav3v2_xrot_o(int32_T n, real_T x[16], int32_T ix0, int32_T iy0,
  real_T c, real_T s);
static int32_T nav3v2_eml_dlahqr(real_T h[16], real_T z[16]);
static void nav3v2_eig(const real_T A[16], creal_T V[16], creal_T D[4]);
static real_T nav3v2_rt_atan2d_snf(real_T u0, real_T u1);
static real_T nav3v2_CalculateHeading(const real_T startPose[3], const real_T
  endPose[2]);
static real_T nav3v2_CalculateReverseHeading(const real_T startPose[3], const
  real_T endPose[2]);
static real_T nav3v2_norm(const real_T x[2]);
static real_T nav3v2_closestPointOnLine(const real_T pt1[2], real_T pt2[2],
  const real_T refPt[2]);
static void controllerPurePursuit_stepImpl(controllerPurePursuit_nav3v2_T *obj,
  const real_T curPose[3], real_T *v, real_T *w, real_T lookaheadPoint[2]);
static void nav3v2_purePursuit(real_T lookAhead, real_T maxLinVel, real_T
  maxAngVel, const real_T waypoints[6], const real_T pose[3], real_T
  targetHeading[3]);
static void exit_internal_AUTONOMOUS_CONTRO(void);
static void nav3v2_AUTONOMOUS_CONTROL(void);
static void matlabCodegenHandle_mat_ikxi5sl(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabC_ikx(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void nav3v2_matlabCodegenHa_f0(ros_slros_internal_block_SetP_T *obj);
int32_T div_nzp_s32(int32_T numerator, int32_T denominator)
{
  uint32_T tempAbsQuotient;
  tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
                     static_cast<uint32_T>(numerator)) / (denominator < 0 ? ~
    static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>(denominator));
  return (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>
    (tempAbsQuotient) : static_cast<int32_T>(tempAbsQuotient);
}

/* System initialize for function-call system: '<S19>/checkAtGoal' */
void nav3v2_checkAtGoal_Init(B_checkAtGoal_nav3v2_T *localB,
  P_checkAtGoal_nav3v2_T *localP)
{
  /* SystemInitialize for Outport: '<S21>/atGoal' */
  localB->LessThan = localP->atGoal_Y0;
}

/* Output and update for function-call system: '<S19>/checkAtGoal' */
void nav3v2_checkAtGoal(const real_T rtu_pose[3], const real_T rtu_goalWaypoint
  [2], real_T rtu_goalTolerance, B_checkAtGoal_nav3v2_T *localB)
{
  real_T rtb_Subtract_0;
  real_T rtb_Subtract;

  /* Sum: '<S21>/Subtract' */
  rtb_Subtract = rtu_pose[0] - rtu_goalWaypoint[0];

  /* DotProduct: '<S21>/Dot Product' */
  rtb_Subtract_0 = rtb_Subtract * rtb_Subtract;

  /* Sum: '<S21>/Subtract' */
  rtb_Subtract = rtu_pose[1] - rtu_goalWaypoint[1];

  /* DotProduct: '<S21>/Dot Product' */
  rtb_Subtract_0 += rtb_Subtract * rtb_Subtract;

  /* RelationalOperator: '<S21>/Less Than' incorporates:
   *  DotProduct: '<S21>/Dot Product'
   *  Sqrt: '<S21>/Sqrt'
   */
  localB->LessThan = (sqrt(rtb_Subtract_0) >= rtu_goalTolerance);
}

/*
 * Output and update for atomic system:
 *    '<S20>/Normalise_Goal_Heading'
 *    '<S20>/Normalise_Heading'
 */
void nav3v2_Normalise_Goal_Heading(real_T rtu_headingAngle,
  B_Normalise_Goal_Heading_nav3_T *localB)
{
  localB->bearingAngle = rtu_headingAngle;
  if (rtu_headingAngle < 0.0) {
    localB->bearingAngle = (3.1415926535897931 - fabs(rtu_headingAngle)) +
      3.1415926535897931;
  }
}

/* System initialize for function-call system: '<S19>/PID' */
void nav3v2_PID_Init(B_PID_nav3v2_T *localB, DW_PID_nav3v2_T *localDW,
                     P_PID_nav3v2_T *localP)
{
  /* InitializeConditions for DiscreteIntegrator: '<S59>/Integrator' */
  localDW->Integrator_DSTATE = localP->DiscreteVaryingPID_InitialCondi;
  localDW->Integrator_PREV_U = 0.0;

  /* InitializeConditions for Delay: '<S52>/UD' */
  localDW->UD_DSTATE = localP->DiscreteVaryingPID_Differentiat;

  /* SystemInitialize for Outport: '<S20>/angVel' */
  localB->Saturation = localP->angVel_Y0;
}

/* Enable for function-call system: '<S19>/PID' */
void nav3v2_PID_Enable(DW_PID_nav3v2_T *localDW)
{
  localDW->PID_RESET_ELAPS_T = true;

  /* Enable for DiscreteIntegrator: '<S59>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE = 1U;
}

/* Output and update for function-call system: '<S19>/PID' */
void nav3v2_PID(RT_MODEL_nav3v2_T * const nav3v2_M, const real_T rtu_pose[3],
                real_T rtu_desiredHeading, real_T rtu_P, real_T rtu_I, real_T
                rtu_D, B_PID_nav3v2_T *localB, DW_PID_nav3v2_T *localDW,
                P_PID_nav3v2_T *localP)
{
  uint32_T elapseT_H;
  real_T rtb_error;
  real_T rtb_Tsamp;
  boolean_T rtb_NotEqual;
  uint32_T PID_ELAPS_T_tmp;
  real_T tmp;
  if (localDW->PID_RESET_ELAPS_T) {
    localDW->PID_ELAPS_T[0] = 0U;
    localDW->PID_ELAPS_T[1] = 0U;
  } else {
    PID_ELAPS_T_tmp = nav3v2_M->Timing.clockTick0;
    localDW->PID_ELAPS_T[0] = PID_ELAPS_T_tmp - localDW->PID_PREV_T[0];
    elapseT_H = nav3v2_M->Timing.clockTickH0 - localDW->PID_PREV_T[1];
    if (localDW->PID_PREV_T[0] > PID_ELAPS_T_tmp) {
      elapseT_H--;
    }

    localDW->PID_ELAPS_T[1] = elapseT_H;
  }

  localDW->PID_PREV_T[0] = nav3v2_M->Timing.clockTick0;
  localDW->PID_PREV_T[1] = nav3v2_M->Timing.clockTickH0;
  localDW->PID_RESET_ELAPS_T = false;

  /* MATLAB Function: '<S20>/Normalise_Goal_Heading' */
  nav3v2_Normalise_Goal_Heading(rtu_desiredHeading,
    &localB->sf_Normalise_Goal_Heading);

  /* MATLAB Function: '<S20>/Normalise_Heading' */
  nav3v2_Normalise_Goal_Heading(rtu_pose[2], &localB->sf_Normalise_Heading);

  /* MATLAB Function: '<S20>/Calculate_Error' */
  rtb_error = localB->sf_Normalise_Goal_Heading.bearingAngle -
    localB->sf_Normalise_Heading.bearingAngle;
  if (rtb_error < -3.1415926535897931) {
    rtb_error += 6.2831853071795862;
  } else {
    if (rtb_error > 3.1415926535897931) {
      rtb_error -= 6.2831853071795862;
    }
  }

  /* End of MATLAB Function: '<S20>/Calculate_Error' */

  /* DiscreteIntegrator: '<S59>/Integrator' */
  if (localDW->Integrator_SYSTEM_ENABLE == 0) {
    localDW->Integrator_DSTATE += localP->Integrator_gainval *
      static_cast<real_T>(localDW->PID_ELAPS_T[0]) * localDW->Integrator_PREV_U;
  }

  /* End of DiscreteIntegrator: '<S59>/Integrator' */

  /* SampleTimeMath: '<S54>/Tsamp' incorporates:
   *  Product: '<S51>/DProd Out'
   *
   * About '<S54>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Tsamp = rtb_error * rtu_D / ((static_cast<real_T>(localDW->PID_ELAPS_T[0])
    * 0.2 + static_cast<real_T>(localDW->PID_ELAPS_T[1]) * 8.589934592E+8) *
    localP->Tsamp_WtEt);

  /* Sum: '<S68>/Sum' incorporates:
   *  Delay: '<S52>/UD'
   *  Product: '<S64>/PProd Out'
   *  Sum: '<S52>/Diff'
   */
  localB->Saturation = (rtb_error * rtu_P + localDW->Integrator_DSTATE) +
    (rtb_Tsamp - localDW->UD_DSTATE);

  /* DeadZone: '<S50>/DeadZone' */
  if (localB->Saturation > localP->DiscreteVaryingPID_UpperSaturat) {
    localDW->Integrator_PREV_U = localB->Saturation -
      localP->DiscreteVaryingPID_UpperSaturat;
  } else if (localB->Saturation >= localP->DiscreteVaryingPID_LowerSaturat) {
    localDW->Integrator_PREV_U = 0.0;
  } else {
    localDW->Integrator_PREV_U = localB->Saturation -
      localP->DiscreteVaryingPID_LowerSaturat;
  }

  /* End of DeadZone: '<S50>/DeadZone' */

  /* RelationalOperator: '<S50>/NotEqual' incorporates:
   *  Gain: '<S50>/ZeroGain'
   */
  rtb_NotEqual = (localP->ZeroGain_Gain * localB->Saturation !=
                  localDW->Integrator_PREV_U);

  /* Signum: '<S50>/SignPreSat' */
  if (localDW->Integrator_PREV_U < 0.0) {
    localDW->Integrator_PREV_U = -1.0;
  } else if (localDW->Integrator_PREV_U > 0.0) {
    localDW->Integrator_PREV_U = 1.0;
  } else if (localDW->Integrator_PREV_U == 0.0) {
    localDW->Integrator_PREV_U = 0.0;
  } else {
    localDW->Integrator_PREV_U = (rtNaN);
  }

  /* End of Signum: '<S50>/SignPreSat' */

  /* DataTypeConversion: '<S50>/DataTypeConv1' */
  if (rtIsNaN(localDW->Integrator_PREV_U)) {
    tmp = 0.0;
  } else {
    tmp = fmod(localDW->Integrator_PREV_U, 256.0);
  }

  /* Product: '<S56>/IProd Out' */
  localDW->Integrator_PREV_U = rtb_error * rtu_I;

  /* Saturate: '<S66>/Saturation' */
  if (localB->Saturation > localP->DiscreteVaryingPID_UpperSaturat) {
    localB->Saturation = localP->DiscreteVaryingPID_UpperSaturat;
  } else {
    if (localB->Saturation < localP->DiscreteVaryingPID_LowerSaturat) {
      localB->Saturation = localP->DiscreteVaryingPID_LowerSaturat;
    }
  }

  /* End of Saturate: '<S66>/Saturation' */

  /* Update for DiscreteIntegrator: '<S59>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE = 0U;

  /* Signum: '<S50>/SignPreIntegrator' */
  if (localDW->Integrator_PREV_U < 0.0) {
    /* DataTypeConversion: '<S50>/DataTypeConv2' */
    rtb_error = -1.0;
  } else if (localDW->Integrator_PREV_U > 0.0) {
    /* DataTypeConversion: '<S50>/DataTypeConv2' */
    rtb_error = 1.0;
  } else if (localDW->Integrator_PREV_U == 0.0) {
    /* DataTypeConversion: '<S50>/DataTypeConv2' */
    rtb_error = 0.0;
  } else {
    /* DataTypeConversion: '<S50>/DataTypeConv2' */
    rtb_error = (rtNaN);
  }

  /* End of Signum: '<S50>/SignPreIntegrator' */

  /* DataTypeConversion: '<S50>/DataTypeConv2' */
  if (rtIsNaN(rtb_error)) {
    rtb_error = 0.0;
  } else {
    rtb_error = fmod(rtb_error, 256.0);
  }

  /* Switch: '<S50>/Switch' incorporates:
   *  DataTypeConversion: '<S50>/DataTypeConv1'
   *  DataTypeConversion: '<S50>/DataTypeConv2'
   *  Logic: '<S50>/AND3'
   *  RelationalOperator: '<S50>/Equal1'
   */
  if (rtb_NotEqual && (static_cast<int8_T>(tmp < 0.0 ? static_cast<int32_T>(
         static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-tmp)))) :
        static_cast<int32_T>(static_cast<int8_T>(static_cast<uint8_T>(tmp)))) ==
                       (rtb_error < 0.0 ? static_cast<int32_T>
                        (static_cast<int8_T>(-static_cast<int8_T>
          (static_cast<uint8_T>(-rtb_error)))) : static_cast<int32_T>(
         static_cast<int8_T>(static_cast<uint8_T>(rtb_error)))))) {
    /* Update for DiscreteIntegrator: '<S59>/Integrator' incorporates:
     *  Constant: '<S50>/Constant1'
     */
    localDW->Integrator_PREV_U = localP->Constant1_Value;
  }

  /* End of Switch: '<S50>/Switch' */

  /* Update for Delay: '<S52>/UD' */
  localDW->UD_DSTATE = rtb_Tsamp;
}

/*
 * System initialize for enable system:
 *    '<S96>/Enabled Subsystem'
 *    '<S97>/Enabled Subsystem'
 */
void nav3v2_EnabledSubsystem_Init(B_EnabledSubsystem_nav3v2_T *localB,
  P_EnabledSubsystem_nav3v2_T *localP)
{
  /* SystemInitialize for Outport: '<S98>/Out1' */
  localB->In1 = localP->Out1_Y0;
}

/*
 * Output and update for enable system:
 *    '<S96>/Enabled Subsystem'
 *    '<S97>/Enabled Subsystem'
 */
void nav3v2_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_nav3v2_std_msgs_Float32 *rtu_In1, B_EnabledSubsystem_nav3v2_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S96>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S98>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S98>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S96>/Enabled Subsystem' */
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Output and update for atomic system:
 *    '<S100>/Quaternion_2_Euler'
 *    '<S107>/Quaternion_2_Euler'
 *    '<S112>/Quaternion_2_Euler'
 */
void nav3v2_Quaternion_2_Euler(real_T rtu_x, real_T rtu_y, real_T rtu_z, real_T
  rtu_w, B_Quaternion_2_Euler_nav3v2_T *localB)
{
  real_T q_idx_0;
  real_T q_idx_1;
  real_T q_idx_2;
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  localB->c_b = 1.0 / sqrt(((rtu_x * rtu_x + rtu_y * rtu_y) + rtu_z * rtu_z) +
    rtu_w * rtu_w);
  q_idx_0 = rtu_x * localB->c_b;
  q_idx_1 = rtu_y * localB->c_b;
  q_idx_2 = rtu_z * localB->c_b;
  localB->c_b *= rtu_w;
  localB->aSinInput = (q_idx_1 * localB->c_b - q_idx_0 * q_idx_2) * -2.0;
  if (localB->aSinInput > 1.0) {
    localB->aSinInput = 1.0;
  }

  if (localB->aSinInput < -1.0) {
    localB->aSinInput = -1.0;
  }

  tmp = q_idx_0 * q_idx_0;
  tmp_0 = q_idx_1 * q_idx_1;
  tmp_1 = q_idx_2 * q_idx_2;
  tmp_2 = localB->c_b * localB->c_b;
  localB->theta = rt_atan2d_snf((q_idx_2 * localB->c_b + q_idx_0 * q_idx_1) *
    2.0, ((tmp - tmp_0) - tmp_1) + tmp_2);
  if ((rtIsNaN(rt_atan2d_snf((q_idx_1 * q_idx_2 + q_idx_0 * localB->c_b) * 2.0,
         ((tmp + tmp_0) - tmp_1) - tmp_2)) + rtIsNaN(asin(localB->aSinInput))) +
      rtIsNaN(localB->theta) == 3) {
    localB->theta = 0.0;
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static boolean_T nav3v2_anyNonFinite(const real_T x[16])
{
  boolean_T b_p;
  int32_T k;
  b_p = true;
  for (k = 0; k < 16; k++) {
    if (b_p && ((!rtIsInf(x[k])) && (!rtIsNaN(x[k])))) {
    } else {
      b_p = false;
    }
  }

  return !b_p;
}

static real_T nav3v2_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  nav3v2_B.a = fabs(u0);
  y = fabs(u1);
  if (nav3v2_B.a < y) {
    nav3v2_B.a /= y;
    y *= sqrt(nav3v2_B.a * nav3v2_B.a + 1.0);
  } else if (nav3v2_B.a > y) {
    y /= nav3v2_B.a;
    y = sqrt(y * y + 1.0) * nav3v2_B.a;
  } else {
    if (!rtIsNaN(y)) {
      y = nav3v2_B.a * 1.4142135623730951;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static real_T nav3v2_xzlangeM(const creal_T x[16])
{
  real_T y;
  real_T absxk;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    absxk = nav3v2_rt_hypotd_snf(x[k].re, x[k].im);
    if (rtIsNaN(absxk)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (absxk > y) {
        y = absxk;
      }

      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xzlascl(real_T cfrom, real_T cto, creal_T A[16])
{
  nav3v2_B.cfromc = cfrom;
  nav3v2_B.ctoc = cto;
  nav3v2_B.notdone = true;
  while (nav3v2_B.notdone) {
    nav3v2_B.cfrom1 = nav3v2_B.cfromc * 2.0041683600089728E-292;
    nav3v2_B.cto1 = nav3v2_B.ctoc / 4.9896007738368E+291;
    if ((fabs(nav3v2_B.cfrom1) > fabs(nav3v2_B.ctoc)) && (nav3v2_B.ctoc != 0.0))
    {
      nav3v2_B.mul_e = 2.0041683600089728E-292;
      nav3v2_B.cfromc = nav3v2_B.cfrom1;
    } else if (fabs(nav3v2_B.cto1) > fabs(nav3v2_B.cfromc)) {
      nav3v2_B.mul_e = 4.9896007738368E+291;
      nav3v2_B.ctoc = nav3v2_B.cto1;
    } else {
      nav3v2_B.mul_e = nav3v2_B.ctoc / nav3v2_B.cfromc;
      nav3v2_B.notdone = false;
    }

    for (nav3v2_B.i3 = 0; nav3v2_B.i3 < 16; nav3v2_B.i3++) {
      A[nav3v2_B.i3].re *= nav3v2_B.mul_e;
      A[nav3v2_B.i3].im *= nav3v2_B.mul_e;
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static real_T nav3v2_xzlanhs(const creal_T A[16], int32_T ilo, int32_T ihi)
{
  real_T f;
  f = 0.0;
  if (ilo <= ihi) {
    nav3v2_B.scale_da = 0.0;
    nav3v2_B.sumsq = 0.0;
    nav3v2_B.firstNonZero = true;
    nav3v2_B.j_o = ilo;
    while (nav3v2_B.j_o <= ihi) {
      nav3v2_B.b_o = nav3v2_B.j_o + 1;
      if (ihi < nav3v2_B.j_o + 1) {
        nav3v2_B.b_o = ihi;
      }

      nav3v2_B.i_ip = ilo;
      while (nav3v2_B.i_ip <= nav3v2_B.b_o) {
        nav3v2_B.reAij_tmp = (((nav3v2_B.j_o - 1) << 2) + nav3v2_B.i_ip) - 1;
        if (A[nav3v2_B.reAij_tmp].re != 0.0) {
          nav3v2_B.temp1 = fabs(A[nav3v2_B.reAij_tmp].re);
          if (nav3v2_B.firstNonZero) {
            nav3v2_B.sumsq = 1.0;
            nav3v2_B.scale_da = nav3v2_B.temp1;
            nav3v2_B.firstNonZero = false;
          } else if (nav3v2_B.scale_da < nav3v2_B.temp1) {
            nav3v2_B.temp2 = nav3v2_B.scale_da / nav3v2_B.temp1;
            nav3v2_B.sumsq = nav3v2_B.sumsq * nav3v2_B.temp2 * nav3v2_B.temp2 +
              1.0;
            nav3v2_B.scale_da = nav3v2_B.temp1;
          } else {
            nav3v2_B.temp2 = nav3v2_B.temp1 / nav3v2_B.scale_da;
            nav3v2_B.sumsq += nav3v2_B.temp2 * nav3v2_B.temp2;
          }
        }

        if (A[nav3v2_B.reAij_tmp].im != 0.0) {
          nav3v2_B.temp1 = fabs(A[nav3v2_B.reAij_tmp].im);
          if (nav3v2_B.firstNonZero) {
            nav3v2_B.sumsq = 1.0;
            nav3v2_B.scale_da = nav3v2_B.temp1;
            nav3v2_B.firstNonZero = false;
          } else if (nav3v2_B.scale_da < nav3v2_B.temp1) {
            nav3v2_B.temp2 = nav3v2_B.scale_da / nav3v2_B.temp1;
            nav3v2_B.sumsq = nav3v2_B.sumsq * nav3v2_B.temp2 * nav3v2_B.temp2 +
              1.0;
            nav3v2_B.scale_da = nav3v2_B.temp1;
          } else {
            nav3v2_B.temp2 = nav3v2_B.temp1 / nav3v2_B.scale_da;
            nav3v2_B.sumsq += nav3v2_B.temp2 * nav3v2_B.temp2;
          }
        }

        nav3v2_B.i_ip++;
      }

      nav3v2_B.j_o++;
    }

    f = nav3v2_B.scale_da * sqrt(nav3v2_B.sumsq);
  }

  return f;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xzlartg_g(const creal_T f, const creal_T g, real_T *cs,
  creal_T *sn)
{
  boolean_T guard1 = false;
  nav3v2_B.f2s_o = fabs(f.re);
  nav3v2_B.di_b = fabs(f.im);
  nav3v2_B.scale_dy = nav3v2_B.f2s_o;
  if (nav3v2_B.di_b > nav3v2_B.f2s_o) {
    nav3v2_B.scale_dy = nav3v2_B.di_b;
  }

  nav3v2_B.gs_re_h = fabs(g.re);
  nav3v2_B.gs_im_b = fabs(g.im);
  if (nav3v2_B.gs_im_b > nav3v2_B.gs_re_h) {
    nav3v2_B.gs_re_h = nav3v2_B.gs_im_b;
  }

  if (nav3v2_B.gs_re_h > nav3v2_B.scale_dy) {
    nav3v2_B.scale_dy = nav3v2_B.gs_re_h;
  }

  nav3v2_B.fs_re_b = f.re;
  nav3v2_B.fs_im_l = f.im;
  nav3v2_B.gs_re_h = g.re;
  nav3v2_B.gs_im_b = g.im;
  guard1 = false;
  if (nav3v2_B.scale_dy >= 7.4428285367870146E+137) {
    do {
      nav3v2_B.fs_re_b *= 1.3435752215134178E-138;
      nav3v2_B.fs_im_l *= 1.3435752215134178E-138;
      nav3v2_B.gs_re_h *= 1.3435752215134178E-138;
      nav3v2_B.gs_im_b *= 1.3435752215134178E-138;
      nav3v2_B.scale_dy *= 1.3435752215134178E-138;
    } while (!(nav3v2_B.scale_dy < 7.4428285367870146E+137));

    guard1 = true;
  } else if (nav3v2_B.scale_dy <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        nav3v2_B.fs_re_b *= 7.4428285367870146E+137;
        nav3v2_B.fs_im_l *= 7.4428285367870146E+137;
        nav3v2_B.gs_re_h *= 7.4428285367870146E+137;
        nav3v2_B.gs_im_b *= 7.4428285367870146E+137;
        nav3v2_B.scale_dy *= 7.4428285367870146E+137;
      } while (!(nav3v2_B.scale_dy > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    nav3v2_B.scale_dy = nav3v2_B.fs_re_b * nav3v2_B.fs_re_b + nav3v2_B.fs_im_l *
      nav3v2_B.fs_im_l;
    nav3v2_B.g2_l = nav3v2_B.gs_re_h * nav3v2_B.gs_re_h + nav3v2_B.gs_im_b *
      nav3v2_B.gs_im_b;
    nav3v2_B.x_n = nav3v2_B.g2_l;
    if (1.0 > nav3v2_B.g2_l) {
      nav3v2_B.x_n = 1.0;
    }

    if (nav3v2_B.scale_dy <= nav3v2_B.x_n * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        nav3v2_B.f2s_o = nav3v2_rt_hypotd_snf(nav3v2_B.gs_re_h, nav3v2_B.gs_im_b);
        sn->re = nav3v2_B.gs_re_h / nav3v2_B.f2s_o;
        sn->im = -nav3v2_B.gs_im_b / nav3v2_B.f2s_o;
      } else {
        nav3v2_B.scale_dy = sqrt(nav3v2_B.g2_l);
        *cs = nav3v2_rt_hypotd_snf(nav3v2_B.fs_re_b, nav3v2_B.fs_im_l) /
          nav3v2_B.scale_dy;
        if (nav3v2_B.di_b > nav3v2_B.f2s_o) {
          nav3v2_B.f2s_o = nav3v2_B.di_b;
        }

        if (nav3v2_B.f2s_o > 1.0) {
          nav3v2_B.f2s_o = nav3v2_rt_hypotd_snf(f.re, f.im);
          nav3v2_B.fs_re_b = f.re / nav3v2_B.f2s_o;
          nav3v2_B.fs_im_l = f.im / nav3v2_B.f2s_o;
        } else {
          nav3v2_B.fs_re_b = 7.4428285367870146E+137 * f.re;
          nav3v2_B.di_b = 7.4428285367870146E+137 * f.im;
          nav3v2_B.f2s_o = nav3v2_rt_hypotd_snf(nav3v2_B.fs_re_b, nav3v2_B.di_b);
          nav3v2_B.fs_re_b /= nav3v2_B.f2s_o;
          nav3v2_B.fs_im_l = nav3v2_B.di_b / nav3v2_B.f2s_o;
        }

        nav3v2_B.gs_re_h /= nav3v2_B.scale_dy;
        nav3v2_B.gs_im_b = -nav3v2_B.gs_im_b / nav3v2_B.scale_dy;
        sn->re = nav3v2_B.fs_re_b * nav3v2_B.gs_re_h - nav3v2_B.fs_im_l *
          nav3v2_B.gs_im_b;
        sn->im = nav3v2_B.fs_re_b * nav3v2_B.gs_im_b + nav3v2_B.fs_im_l *
          nav3v2_B.gs_re_h;
      }
    } else {
      nav3v2_B.f2s_o = sqrt(nav3v2_B.g2_l / nav3v2_B.scale_dy + 1.0);
      nav3v2_B.fs_re_b *= nav3v2_B.f2s_o;
      nav3v2_B.fs_im_l *= nav3v2_B.f2s_o;
      *cs = 1.0 / nav3v2_B.f2s_o;
      nav3v2_B.f2s_o = nav3v2_B.scale_dy + nav3v2_B.g2_l;
      nav3v2_B.fs_re_b /= nav3v2_B.f2s_o;
      nav3v2_B.fs_im_l /= nav3v2_B.f2s_o;
      sn->re = nav3v2_B.fs_re_b * nav3v2_B.gs_re_h - nav3v2_B.fs_im_l *
        -nav3v2_B.gs_im_b;
      sn->im = nav3v2_B.fs_re_b * -nav3v2_B.gs_im_b + nav3v2_B.fs_im_l *
        nav3v2_B.gs_re_h;
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xzlartg(const creal_T f, const creal_T g, real_T *cs, creal_T
  *sn, creal_T *r)
{
  boolean_T guard1 = false;
  nav3v2_B.f2s = fabs(f.re);
  nav3v2_B.di = fabs(f.im);
  nav3v2_B.scale_g = nav3v2_B.f2s;
  if (nav3v2_B.di > nav3v2_B.f2s) {
    nav3v2_B.scale_g = nav3v2_B.di;
  }

  nav3v2_B.gs_re = fabs(g.re);
  nav3v2_B.gs_im = fabs(g.im);
  if (nav3v2_B.gs_im > nav3v2_B.gs_re) {
    nav3v2_B.gs_re = nav3v2_B.gs_im;
  }

  if (nav3v2_B.gs_re > nav3v2_B.scale_g) {
    nav3v2_B.scale_g = nav3v2_B.gs_re;
  }

  nav3v2_B.fs_re = f.re;
  nav3v2_B.fs_im = f.im;
  nav3v2_B.gs_re = g.re;
  nav3v2_B.gs_im = g.im;
  nav3v2_B.count = -1;
  nav3v2_B.rescaledir = 0;
  guard1 = false;
  if (nav3v2_B.scale_g >= 7.4428285367870146E+137) {
    do {
      nav3v2_B.count++;
      nav3v2_B.fs_re *= 1.3435752215134178E-138;
      nav3v2_B.fs_im *= 1.3435752215134178E-138;
      nav3v2_B.gs_re *= 1.3435752215134178E-138;
      nav3v2_B.gs_im *= 1.3435752215134178E-138;
      nav3v2_B.scale_g *= 1.3435752215134178E-138;
    } while (!(nav3v2_B.scale_g < 7.4428285367870146E+137));

    nav3v2_B.rescaledir = 1;
    guard1 = true;
  } else if (nav3v2_B.scale_g <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        nav3v2_B.count++;
        nav3v2_B.fs_re *= 7.4428285367870146E+137;
        nav3v2_B.fs_im *= 7.4428285367870146E+137;
        nav3v2_B.gs_re *= 7.4428285367870146E+137;
        nav3v2_B.gs_im *= 7.4428285367870146E+137;
        nav3v2_B.scale_g *= 7.4428285367870146E+137;
      } while (!(nav3v2_B.scale_g > 1.3435752215134178E-138));

      nav3v2_B.rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    nav3v2_B.scale_g = nav3v2_B.fs_re * nav3v2_B.fs_re + nav3v2_B.fs_im *
      nav3v2_B.fs_im;
    nav3v2_B.g2 = nav3v2_B.gs_re * nav3v2_B.gs_re + nav3v2_B.gs_im *
      nav3v2_B.gs_im;
    nav3v2_B.x = nav3v2_B.g2;
    if (1.0 > nav3v2_B.g2) {
      nav3v2_B.x = 1.0;
    }

    if (nav3v2_B.scale_g <= nav3v2_B.x * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = nav3v2_rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        nav3v2_B.f2s = nav3v2_rt_hypotd_snf(nav3v2_B.gs_re, nav3v2_B.gs_im);
        sn->re = nav3v2_B.gs_re / nav3v2_B.f2s;
        sn->im = -nav3v2_B.gs_im / nav3v2_B.f2s;
      } else {
        nav3v2_B.scale_g = sqrt(nav3v2_B.g2);
        *cs = nav3v2_rt_hypotd_snf(nav3v2_B.fs_re, nav3v2_B.fs_im) /
          nav3v2_B.scale_g;
        if (nav3v2_B.di > nav3v2_B.f2s) {
          nav3v2_B.f2s = nav3v2_B.di;
        }

        if (nav3v2_B.f2s > 1.0) {
          nav3v2_B.f2s = nav3v2_rt_hypotd_snf(f.re, f.im);
          nav3v2_B.fs_re = f.re / nav3v2_B.f2s;
          nav3v2_B.fs_im = f.im / nav3v2_B.f2s;
        } else {
          nav3v2_B.fs_re = 7.4428285367870146E+137 * f.re;
          nav3v2_B.di = 7.4428285367870146E+137 * f.im;
          nav3v2_B.f2s = nav3v2_rt_hypotd_snf(nav3v2_B.fs_re, nav3v2_B.di);
          nav3v2_B.fs_re /= nav3v2_B.f2s;
          nav3v2_B.fs_im = nav3v2_B.di / nav3v2_B.f2s;
        }

        nav3v2_B.gs_re /= nav3v2_B.scale_g;
        nav3v2_B.gs_im = -nav3v2_B.gs_im / nav3v2_B.scale_g;
        sn->re = nav3v2_B.fs_re * nav3v2_B.gs_re - nav3v2_B.fs_im *
          nav3v2_B.gs_im;
        sn->im = nav3v2_B.fs_re * nav3v2_B.gs_im + nav3v2_B.fs_im *
          nav3v2_B.gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      nav3v2_B.f2s = sqrt(nav3v2_B.g2 / nav3v2_B.scale_g + 1.0);
      r->re = nav3v2_B.f2s * nav3v2_B.fs_re;
      r->im = nav3v2_B.f2s * nav3v2_B.fs_im;
      *cs = 1.0 / nav3v2_B.f2s;
      nav3v2_B.f2s = nav3v2_B.scale_g + nav3v2_B.g2;
      nav3v2_B.fs_re = r->re / nav3v2_B.f2s;
      nav3v2_B.f2s = r->im / nav3v2_B.f2s;
      sn->re = nav3v2_B.fs_re * nav3v2_B.gs_re - nav3v2_B.f2s * -nav3v2_B.gs_im;
      sn->im = nav3v2_B.fs_re * -nav3v2_B.gs_im + nav3v2_B.f2s * nav3v2_B.gs_re;
      if (nav3v2_B.rescaledir > 0) {
        nav3v2_B.rescaledir = 0;
        while (nav3v2_B.rescaledir <= nav3v2_B.count) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
          nav3v2_B.rescaledir++;
        }
      } else {
        if (nav3v2_B.rescaledir < 0) {
          nav3v2_B.rescaledir = 0;
          while (nav3v2_B.rescaledir <= nav3v2_B.count) {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
            nav3v2_B.rescaledir++;
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi, creal_T Z[16],
  int32_T *info, creal_T alpha1[4], creal_T beta1[4])
{
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  int32_T exitg1;
  boolean_T exitg2;
  *info = 0;
  alpha1[0].re = 0.0;
  alpha1[0].im = 0.0;
  beta1[0].re = 1.0;
  beta1[0].im = 0.0;
  alpha1[1].re = 0.0;
  alpha1[1].im = 0.0;
  beta1[1].re = 1.0;
  beta1[1].im = 0.0;
  alpha1[2].re = 0.0;
  alpha1[2].im = 0.0;
  beta1[2].re = 1.0;
  beta1[2].im = 0.0;
  alpha1[3].re = 0.0;
  alpha1[3].im = 0.0;
  beta1[3].re = 1.0;
  beta1[3].im = 0.0;
  nav3v2_B.eshift_re = 0.0;
  nav3v2_B.eshift_im = 0.0;
  nav3v2_B.ctemp.re = 0.0;
  nav3v2_B.ctemp.im = 0.0;
  nav3v2_B.anorm = nav3v2_xzlanhs(A, ilo, ihi);
  nav3v2_B.shift_re = 2.2204460492503131E-16 * nav3v2_B.anorm;
  nav3v2_B.b_atol = 2.2250738585072014E-308;
  if (nav3v2_B.shift_re > 2.2250738585072014E-308) {
    nav3v2_B.b_atol = nav3v2_B.shift_re;
  }

  nav3v2_B.shift_re = 2.2250738585072014E-308;
  if (nav3v2_B.anorm > 2.2250738585072014E-308) {
    nav3v2_B.shift_re = nav3v2_B.anorm;
  }

  nav3v2_B.anorm = 1.0 / nav3v2_B.shift_re;
  nav3v2_B.failed = true;
  nav3v2_B.ilast = ihi;
  while (nav3v2_B.ilast + 1 < 5) {
    alpha1[nav3v2_B.ilast] = A[(nav3v2_B.ilast << 2) + nav3v2_B.ilast];
    nav3v2_B.ilast++;
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    nav3v2_B.ifirst = ilo;
    nav3v2_B.istart = ilo;
    nav3v2_B.ilast = ihi - 1;
    nav3v2_B.ilastm1 = ihi - 2;
    nav3v2_B.iiter = 0;
    nav3v2_B.goto60 = false;
    nav3v2_B.goto70 = false;
    nav3v2_B.goto90 = false;
    nav3v2_B.jiter = 0;
    do {
      exitg1 = 0;
      if (nav3v2_B.jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (nav3v2_B.ilast + 1 == ilo) {
          nav3v2_B.goto60 = true;
        } else {
          nav3v2_B.jp1 = (nav3v2_B.ilastm1 << 2) + nav3v2_B.ilast;
          if (fabs(A[nav3v2_B.jp1].re) + fabs(A[nav3v2_B.jp1].im) <=
              nav3v2_B.b_atol) {
            A[nav3v2_B.jp1].re = 0.0;
            A[nav3v2_B.jp1].im = 0.0;
            nav3v2_B.goto60 = true;
          } else {
            nav3v2_B.j = nav3v2_B.ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (nav3v2_B.j + 1 >= ilo)) {
              if (nav3v2_B.j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                nav3v2_B.jp1 = ((nav3v2_B.j - 1) << 2) + nav3v2_B.j;
                if (fabs(A[nav3v2_B.jp1].re) + fabs(A[nav3v2_B.jp1].im) <=
                    nav3v2_B.b_atol) {
                  A[nav3v2_B.jp1].re = 0.0;
                  A[nav3v2_B.jp1].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  nav3v2_B.j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              nav3v2_B.ifirst = nav3v2_B.j + 1;
              nav3v2_B.goto70 = true;
            }
          }
        }

        if ((!nav3v2_B.goto60) && (!nav3v2_B.goto70)) {
          alpha1[0].re = (rtNaN);
          alpha1[0].im = 0.0;
          beta1[0].re = (rtNaN);
          beta1[0].im = 0.0;
          alpha1[1].re = (rtNaN);
          alpha1[1].im = 0.0;
          beta1[1].re = (rtNaN);
          beta1[1].im = 0.0;
          alpha1[2].re = (rtNaN);
          alpha1[2].im = 0.0;
          beta1[2].re = (rtNaN);
          beta1[2].im = 0.0;
          alpha1[3].re = (rtNaN);
          alpha1[3].im = 0.0;
          beta1[3].re = (rtNaN);
          beta1[3].im = 0.0;
          for (nav3v2_B.jp1 = 0; nav3v2_B.jp1 < 16; nav3v2_B.jp1++) {
            Z[nav3v2_B.jp1].re = (rtNaN);
            Z[nav3v2_B.jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (nav3v2_B.goto60) {
          nav3v2_B.goto60 = false;
          alpha1[nav3v2_B.ilast] = A[(nav3v2_B.ilast << 2) + nav3v2_B.ilast];
          nav3v2_B.ilast = nav3v2_B.ilastm1;
          nav3v2_B.ilastm1--;
          if (nav3v2_B.ilast + 1 < ilo) {
            nav3v2_B.failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            nav3v2_B.iiter = 0;
            nav3v2_B.eshift_re = 0.0;
            nav3v2_B.eshift_im = 0.0;
            nav3v2_B.jiter++;
          }
        } else {
          if (nav3v2_B.goto70) {
            nav3v2_B.goto70 = false;
            nav3v2_B.iiter++;
            if (nav3v2_B.iiter - div_nzp_s32(nav3v2_B.iiter, 10) * 10 != 0) {
              nav3v2_B.j = (nav3v2_B.ilastm1 << 2) + nav3v2_B.ilastm1;
              nav3v2_B.ar = A[nav3v2_B.j].re * nav3v2_B.anorm;
              nav3v2_B.ai = A[nav3v2_B.j].im * nav3v2_B.anorm;
              if (nav3v2_B.ai == 0.0) {
                nav3v2_B.shift_re = nav3v2_B.ar / 0.5;
                nav3v2_B.shift_im = 0.0;
              } else if (nav3v2_B.ar == 0.0) {
                nav3v2_B.shift_re = 0.0;
                nav3v2_B.shift_im = nav3v2_B.ai / 0.5;
              } else {
                nav3v2_B.shift_re = nav3v2_B.ar / 0.5;
                nav3v2_B.shift_im = nav3v2_B.ai / 0.5;
              }

              nav3v2_B.j = (nav3v2_B.ilast << 2) + nav3v2_B.ilast;
              nav3v2_B.ar = A[nav3v2_B.j].re * nav3v2_B.anorm;
              nav3v2_B.ai = A[nav3v2_B.j].im * nav3v2_B.anorm;
              if (nav3v2_B.ai == 0.0) {
                nav3v2_B.ad22.re = nav3v2_B.ar / 0.5;
                nav3v2_B.ad22.im = 0.0;
              } else if (nav3v2_B.ar == 0.0) {
                nav3v2_B.ad22.re = 0.0;
                nav3v2_B.ad22.im = nav3v2_B.ai / 0.5;
              } else {
                nav3v2_B.ad22.re = nav3v2_B.ar / 0.5;
                nav3v2_B.ad22.im = nav3v2_B.ai / 0.5;
              }

              nav3v2_B.t1_re = (nav3v2_B.shift_re + nav3v2_B.ad22.re) * 0.5;
              nav3v2_B.t1_im = (nav3v2_B.shift_im + nav3v2_B.ad22.im) * 0.5;
              nav3v2_B.j = (nav3v2_B.ilast << 2) + nav3v2_B.ilastm1;
              nav3v2_B.ar = A[nav3v2_B.j].re * nav3v2_B.anorm;
              nav3v2_B.ai = A[nav3v2_B.j].im * nav3v2_B.anorm;
              if (nav3v2_B.ai == 0.0) {
                nav3v2_B.absxr = nav3v2_B.ar / 0.5;
                nav3v2_B.absxi = 0.0;
              } else if (nav3v2_B.ar == 0.0) {
                nav3v2_B.absxr = 0.0;
                nav3v2_B.absxi = nav3v2_B.ai / 0.5;
              } else {
                nav3v2_B.absxr = nav3v2_B.ar / 0.5;
                nav3v2_B.absxi = nav3v2_B.ai / 0.5;
              }

              nav3v2_B.j = (nav3v2_B.ilastm1 << 2) + nav3v2_B.ilast;
              nav3v2_B.ar = A[nav3v2_B.j].re * nav3v2_B.anorm;
              nav3v2_B.ai = A[nav3v2_B.j].im * nav3v2_B.anorm;
              if (nav3v2_B.ai == 0.0) {
                nav3v2_B.ar /= 0.5;
                nav3v2_B.ai = 0.0;
              } else if (nav3v2_B.ar == 0.0) {
                nav3v2_B.ar = 0.0;
                nav3v2_B.ai /= 0.5;
              } else {
                nav3v2_B.ar /= 0.5;
                nav3v2_B.ai /= 0.5;
              }

              nav3v2_B.shift_im_g = nav3v2_B.shift_re * nav3v2_B.ad22.im +
                nav3v2_B.shift_im * nav3v2_B.ad22.re;
              nav3v2_B.shift_re = ((nav3v2_B.t1_re * nav3v2_B.t1_re -
                                    nav3v2_B.t1_im * nav3v2_B.t1_im) +
                                   (nav3v2_B.absxr * nav3v2_B.ar -
                                    nav3v2_B.absxi * nav3v2_B.ai)) -
                (nav3v2_B.shift_re * nav3v2_B.ad22.re - nav3v2_B.shift_im *
                 nav3v2_B.ad22.im);
              nav3v2_B.shift_im = nav3v2_B.t1_re * nav3v2_B.t1_im;
              nav3v2_B.shift_im = ((nav3v2_B.shift_im + nav3v2_B.shift_im) +
                                   (nav3v2_B.absxr * nav3v2_B.ai +
                                    nav3v2_B.absxi * nav3v2_B.ar)) -
                nav3v2_B.shift_im_g;
              if (nav3v2_B.shift_im == 0.0) {
                if (nav3v2_B.shift_re < 0.0) {
                  nav3v2_B.absxr = 0.0;
                  nav3v2_B.absxi = sqrt(-nav3v2_B.shift_re);
                } else {
                  nav3v2_B.absxr = sqrt(nav3v2_B.shift_re);
                  nav3v2_B.absxi = 0.0;
                }
              } else if (nav3v2_B.shift_re == 0.0) {
                if (nav3v2_B.shift_im < 0.0) {
                  nav3v2_B.absxr = sqrt(-nav3v2_B.shift_im / 2.0);
                  nav3v2_B.absxi = -nav3v2_B.absxr;
                } else {
                  nav3v2_B.absxr = sqrt(nav3v2_B.shift_im / 2.0);
                  nav3v2_B.absxi = nav3v2_B.absxr;
                }
              } else if (rtIsNaN(nav3v2_B.shift_re)) {
                nav3v2_B.absxr = nav3v2_B.shift_re;
                nav3v2_B.absxi = nav3v2_B.shift_re;
              } else if (rtIsNaN(nav3v2_B.shift_im)) {
                nav3v2_B.absxr = nav3v2_B.shift_im;
                nav3v2_B.absxi = nav3v2_B.shift_im;
              } else if (rtIsInf(nav3v2_B.shift_im)) {
                nav3v2_B.absxr = fabs(nav3v2_B.shift_im);
                nav3v2_B.absxi = nav3v2_B.shift_im;
              } else if (rtIsInf(nav3v2_B.shift_re)) {
                if (nav3v2_B.shift_re < 0.0) {
                  nav3v2_B.absxr = 0.0;
                  nav3v2_B.absxi = nav3v2_B.shift_im * -nav3v2_B.shift_re;
                } else {
                  nav3v2_B.absxr = nav3v2_B.shift_re;
                  nav3v2_B.absxi = 0.0;
                }
              } else {
                nav3v2_B.absxr = fabs(nav3v2_B.shift_re);
                nav3v2_B.absxi = fabs(nav3v2_B.shift_im);
                if ((nav3v2_B.absxr > 4.4942328371557893E+307) ||
                    (nav3v2_B.absxi > 4.4942328371557893E+307)) {
                  nav3v2_B.absxr *= 0.5;
                  nav3v2_B.absxi *= 0.5;
                  nav3v2_B.absxi = nav3v2_rt_hypotd_snf(nav3v2_B.absxr,
                    nav3v2_B.absxi);
                  if (nav3v2_B.absxi > nav3v2_B.absxr) {
                    nav3v2_B.absxr = sqrt(nav3v2_B.absxr / nav3v2_B.absxi + 1.0)
                      * sqrt(nav3v2_B.absxi);
                  } else {
                    nav3v2_B.absxr = sqrt(nav3v2_B.absxi) * 1.4142135623730951;
                  }
                } else {
                  nav3v2_B.absxr = sqrt((nav3v2_rt_hypotd_snf(nav3v2_B.absxr,
                    nav3v2_B.absxi) + nav3v2_B.absxr) * 0.5);
                }

                if (nav3v2_B.shift_re > 0.0) {
                  nav3v2_B.absxi = nav3v2_B.shift_im / nav3v2_B.absxr * 0.5;
                } else {
                  if (nav3v2_B.shift_im < 0.0) {
                    nav3v2_B.absxi = -nav3v2_B.absxr;
                  } else {
                    nav3v2_B.absxi = nav3v2_B.absxr;
                  }

                  nav3v2_B.absxr = nav3v2_B.shift_im / nav3v2_B.absxi * 0.5;
                }
              }

              if ((nav3v2_B.t1_re - nav3v2_B.ad22.re) * nav3v2_B.absxr +
                  (nav3v2_B.t1_im - nav3v2_B.ad22.im) * nav3v2_B.absxi <= 0.0) {
                nav3v2_B.shift_re = nav3v2_B.t1_re + nav3v2_B.absxr;
                nav3v2_B.shift_im = nav3v2_B.t1_im + nav3v2_B.absxi;
              } else {
                nav3v2_B.shift_re = nav3v2_B.t1_re - nav3v2_B.absxr;
                nav3v2_B.shift_im = nav3v2_B.t1_im - nav3v2_B.absxi;
              }
            } else {
              nav3v2_B.j = (nav3v2_B.ilastm1 << 2) + nav3v2_B.ilast;
              nav3v2_B.ar = A[nav3v2_B.j].re * nav3v2_B.anorm;
              nav3v2_B.ai = A[nav3v2_B.j].im * nav3v2_B.anorm;
              if (nav3v2_B.ai == 0.0) {
                nav3v2_B.absxr = nav3v2_B.ar / 0.5;
                nav3v2_B.absxi = 0.0;
              } else if (nav3v2_B.ar == 0.0) {
                nav3v2_B.absxr = 0.0;
                nav3v2_B.absxi = nav3v2_B.ai / 0.5;
              } else {
                nav3v2_B.absxr = nav3v2_B.ar / 0.5;
                nav3v2_B.absxi = nav3v2_B.ai / 0.5;
              }

              nav3v2_B.eshift_re += nav3v2_B.absxr;
              nav3v2_B.eshift_im += nav3v2_B.absxi;
              nav3v2_B.shift_re = nav3v2_B.eshift_re;
              nav3v2_B.shift_im = nav3v2_B.eshift_im;
            }

            nav3v2_B.j = nav3v2_B.ilastm1;
            nav3v2_B.jp1 = nav3v2_B.ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (nav3v2_B.j + 1 > nav3v2_B.ifirst)) {
              nav3v2_B.istart = nav3v2_B.j + 1;
              nav3v2_B.ctemp_tmp_tmp = nav3v2_B.j << 2;
              nav3v2_B.ctemp_tmp = nav3v2_B.ctemp_tmp_tmp + nav3v2_B.j;
              nav3v2_B.ctemp.re = A[nav3v2_B.ctemp_tmp].re * nav3v2_B.anorm -
                nav3v2_B.shift_re * 0.5;
              nav3v2_B.ctemp.im = A[nav3v2_B.ctemp_tmp].im * nav3v2_B.anorm -
                nav3v2_B.shift_im * 0.5;
              nav3v2_B.t1_re = fabs(nav3v2_B.ctemp.re) + fabs(nav3v2_B.ctemp.im);
              nav3v2_B.jp1 += nav3v2_B.ctemp_tmp_tmp;
              nav3v2_B.t1_im = (fabs(A[nav3v2_B.jp1].re) + fabs(A[nav3v2_B.jp1].
                im)) * nav3v2_B.anorm;
              nav3v2_B.absxr = nav3v2_B.t1_re;
              if (nav3v2_B.t1_im > nav3v2_B.t1_re) {
                nav3v2_B.absxr = nav3v2_B.t1_im;
              }

              if ((nav3v2_B.absxr < 1.0) && (nav3v2_B.absxr != 0.0)) {
                nav3v2_B.t1_re /= nav3v2_B.absxr;
                nav3v2_B.t1_im /= nav3v2_B.absxr;
              }

              nav3v2_B.jp1 = ((nav3v2_B.j - 1) << 2) + nav3v2_B.j;
              if ((fabs(A[nav3v2_B.jp1].re) + fabs(A[nav3v2_B.jp1].im)) *
                  nav3v2_B.t1_im <= nav3v2_B.t1_re * nav3v2_B.b_atol) {
                nav3v2_B.goto90 = true;
                exitg2 = true;
              } else {
                nav3v2_B.jp1 = nav3v2_B.j;
                nav3v2_B.j--;
              }
            }

            if (!nav3v2_B.goto90) {
              nav3v2_B.istart = nav3v2_B.ifirst;
              nav3v2_B.ctemp_tmp = (((nav3v2_B.ifirst - 1) << 2) +
                                    nav3v2_B.ifirst) - 1;
              nav3v2_B.ctemp.re = A[nav3v2_B.ctemp_tmp].re * nav3v2_B.anorm -
                nav3v2_B.shift_re * 0.5;
              nav3v2_B.ctemp.im = A[nav3v2_B.ctemp_tmp].im * nav3v2_B.anorm -
                nav3v2_B.shift_im * 0.5;
            }

            nav3v2_B.goto90 = false;
            nav3v2_B.j = ((nav3v2_B.istart - 1) << 2) + nav3v2_B.istart;
            nav3v2_B.ascale.re = A[nav3v2_B.j].re * nav3v2_B.anorm;
            nav3v2_B.ascale.im = A[nav3v2_B.j].im * nav3v2_B.anorm;
            nav3v2_xzlartg_g(nav3v2_B.ctemp, nav3v2_B.ascale, &nav3v2_B.t1_re,
                             &nav3v2_B.ad22);
            nav3v2_B.j = nav3v2_B.istart;
            nav3v2_B.jp1 = nav3v2_B.istart - 2;
            while (nav3v2_B.j < nav3v2_B.ilast + 1) {
              if (nav3v2_B.j > nav3v2_B.istart) {
                nav3v2_xzlartg(A[(nav3v2_B.j + (nav3v2_B.jp1 << 2)) - 1],
                               A[nav3v2_B.j + (nav3v2_B.jp1 << 2)],
                               &nav3v2_B.t1_re, &nav3v2_B.ad22, &A[(nav3v2_B.j +
                  (nav3v2_B.jp1 << 2)) - 1]);
                nav3v2_B.jp1 = nav3v2_B.j + (nav3v2_B.jp1 << 2);
                A[nav3v2_B.jp1].re = 0.0;
                A[nav3v2_B.jp1].im = 0.0;
              }

              nav3v2_B.ctemp_tmp = nav3v2_B.j - 1;
              while (nav3v2_B.ctemp_tmp + 1 < 5) {
                nav3v2_B.jp1 = (nav3v2_B.ctemp_tmp << 2) + nav3v2_B.j;
                nav3v2_B.ctemp_tmp_tmp = nav3v2_B.jp1 - 1;
                nav3v2_B.shift_re = A[nav3v2_B.ctemp_tmp_tmp].re *
                  nav3v2_B.t1_re + (A[nav3v2_B.jp1].re * nav3v2_B.ad22.re -
                                    A[nav3v2_B.jp1].im * nav3v2_B.ad22.im);
                nav3v2_B.shift_im = A[nav3v2_B.ctemp_tmp_tmp].im *
                  nav3v2_B.t1_re + (A[nav3v2_B.jp1].im * nav3v2_B.ad22.re +
                                    A[nav3v2_B.jp1].re * nav3v2_B.ad22.im);
                nav3v2_B.t1_im = A[nav3v2_B.ctemp_tmp_tmp].im;
                nav3v2_B.absxr = A[nav3v2_B.ctemp_tmp_tmp].re;
                A[nav3v2_B.jp1].re = A[nav3v2_B.jp1].re * nav3v2_B.t1_re -
                  (A[nav3v2_B.ctemp_tmp_tmp].re * nav3v2_B.ad22.re +
                   A[nav3v2_B.ctemp_tmp_tmp].im * nav3v2_B.ad22.im);
                A[nav3v2_B.jp1].im = A[nav3v2_B.jp1].im * nav3v2_B.t1_re -
                  (nav3v2_B.ad22.re * nav3v2_B.t1_im - nav3v2_B.ad22.im *
                   nav3v2_B.absxr);
                A[nav3v2_B.ctemp_tmp_tmp].re = nav3v2_B.shift_re;
                A[nav3v2_B.ctemp_tmp_tmp].im = nav3v2_B.shift_im;
                nav3v2_B.ctemp_tmp++;
              }

              nav3v2_B.ad22.re = -nav3v2_B.ad22.re;
              nav3v2_B.ad22.im = -nav3v2_B.ad22.im;
              nav3v2_B.ctemp_tmp = nav3v2_B.j;
              if (nav3v2_B.ilast + 1 < nav3v2_B.j + 2) {
                nav3v2_B.ctemp_tmp = nav3v2_B.ilast - 1;
              }

              nav3v2_B.i_o = 0;
              while (nav3v2_B.i_o + 1 <= nav3v2_B.ctemp_tmp + 2) {
                nav3v2_B.jp1 = ((nav3v2_B.j - 1) << 2) + nav3v2_B.i_o;
                nav3v2_B.ctemp_tmp_tmp = (nav3v2_B.j << 2) + nav3v2_B.i_o;
                nav3v2_B.shift_re = (A[nav3v2_B.jp1].re * nav3v2_B.ad22.re -
                                     A[nav3v2_B.jp1].im * nav3v2_B.ad22.im) +
                  A[nav3v2_B.ctemp_tmp_tmp].re * nav3v2_B.t1_re;
                nav3v2_B.shift_im = (A[nav3v2_B.jp1].im * nav3v2_B.ad22.re +
                                     A[nav3v2_B.jp1].re * nav3v2_B.ad22.im) +
                  A[nav3v2_B.ctemp_tmp_tmp].im * nav3v2_B.t1_re;
                nav3v2_B.t1_im = A[nav3v2_B.ctemp_tmp_tmp].im;
                nav3v2_B.absxr = A[nav3v2_B.ctemp_tmp_tmp].re;
                A[nav3v2_B.jp1].re = A[nav3v2_B.jp1].re * nav3v2_B.t1_re -
                  (A[nav3v2_B.ctemp_tmp_tmp].re * nav3v2_B.ad22.re +
                   A[nav3v2_B.ctemp_tmp_tmp].im * nav3v2_B.ad22.im);
                A[nav3v2_B.jp1].im = A[nav3v2_B.jp1].im * nav3v2_B.t1_re -
                  (nav3v2_B.ad22.re * nav3v2_B.t1_im - nav3v2_B.ad22.im *
                   nav3v2_B.absxr);
                A[nav3v2_B.ctemp_tmp_tmp].re = nav3v2_B.shift_re;
                A[nav3v2_B.ctemp_tmp_tmp].im = nav3v2_B.shift_im;
                nav3v2_B.i_o++;
              }

              nav3v2_B.jp1 = (nav3v2_B.j - 1) << 2;
              nav3v2_B.ctemp_tmp_tmp = nav3v2_B.j << 2;
              nav3v2_B.shift_re = (Z[nav3v2_B.jp1].re * nav3v2_B.ad22.re -
                                   Z[nav3v2_B.jp1].im * nav3v2_B.ad22.im) +
                Z[nav3v2_B.ctemp_tmp_tmp].re * nav3v2_B.t1_re;
              nav3v2_B.shift_im = (Z[nav3v2_B.jp1].im * nav3v2_B.ad22.re +
                                   Z[nav3v2_B.jp1].re * nav3v2_B.ad22.im) +
                Z[nav3v2_B.ctemp_tmp_tmp].im * nav3v2_B.t1_re;
              nav3v2_B.t1_im = Z[nav3v2_B.ctemp_tmp_tmp].im;
              nav3v2_B.absxr = Z[nav3v2_B.ctemp_tmp_tmp].re;
              Z[nav3v2_B.jp1].re = Z[nav3v2_B.jp1].re * nav3v2_B.t1_re -
                (Z[nav3v2_B.ctemp_tmp_tmp].re * nav3v2_B.ad22.re +
                 Z[nav3v2_B.ctemp_tmp_tmp].im * nav3v2_B.ad22.im);
              Z[nav3v2_B.jp1].im = Z[nav3v2_B.jp1].im * nav3v2_B.t1_re -
                (nav3v2_B.ad22.re * nav3v2_B.t1_im - nav3v2_B.ad22.im *
                 nav3v2_B.absxr);
              Z[nav3v2_B.ctemp_tmp_tmp].re = nav3v2_B.shift_re;
              Z[nav3v2_B.ctemp_tmp_tmp].im = nav3v2_B.shift_im;
              nav3v2_B.ctemp_tmp = nav3v2_B.jp1 + 1;
              nav3v2_B.i_o = nav3v2_B.ctemp_tmp_tmp + 1;
              nav3v2_B.shift_re = (Z[nav3v2_B.ctemp_tmp].re * nav3v2_B.ad22.re -
                                   Z[nav3v2_B.ctemp_tmp].im * nav3v2_B.ad22.im)
                + Z[nav3v2_B.i_o].re * nav3v2_B.t1_re;
              nav3v2_B.shift_im = (Z[nav3v2_B.ctemp_tmp].im * nav3v2_B.ad22.re +
                                   Z[nav3v2_B.ctemp_tmp].re * nav3v2_B.ad22.im)
                + Z[nav3v2_B.i_o].im * nav3v2_B.t1_re;
              nav3v2_B.t1_im = Z[nav3v2_B.i_o].im;
              nav3v2_B.absxr = Z[nav3v2_B.i_o].re;
              Z[nav3v2_B.ctemp_tmp].re = Z[nav3v2_B.ctemp_tmp].re *
                nav3v2_B.t1_re - (Z[nav3v2_B.i_o].re * nav3v2_B.ad22.re +
                                  Z[nav3v2_B.i_o].im * nav3v2_B.ad22.im);
              Z[nav3v2_B.ctemp_tmp].im = Z[nav3v2_B.ctemp_tmp].im *
                nav3v2_B.t1_re - (nav3v2_B.ad22.re * nav3v2_B.t1_im -
                                  nav3v2_B.ad22.im * nav3v2_B.absxr);
              Z[nav3v2_B.i_o].re = nav3v2_B.shift_re;
              Z[nav3v2_B.i_o].im = nav3v2_B.shift_im;
              nav3v2_B.ctemp_tmp = nav3v2_B.jp1 + 2;
              nav3v2_B.i_o = nav3v2_B.ctemp_tmp_tmp + 2;
              nav3v2_B.shift_re = (Z[nav3v2_B.ctemp_tmp].re * nav3v2_B.ad22.re -
                                   Z[nav3v2_B.ctemp_tmp].im * nav3v2_B.ad22.im)
                + Z[nav3v2_B.i_o].re * nav3v2_B.t1_re;
              nav3v2_B.shift_im = (Z[nav3v2_B.ctemp_tmp].im * nav3v2_B.ad22.re +
                                   Z[nav3v2_B.ctemp_tmp].re * nav3v2_B.ad22.im)
                + Z[nav3v2_B.i_o].im * nav3v2_B.t1_re;
              nav3v2_B.t1_im = Z[nav3v2_B.i_o].im;
              nav3v2_B.absxr = Z[nav3v2_B.i_o].re;
              Z[nav3v2_B.ctemp_tmp].re = Z[nav3v2_B.ctemp_tmp].re *
                nav3v2_B.t1_re - (Z[nav3v2_B.i_o].re * nav3v2_B.ad22.re +
                                  Z[nav3v2_B.i_o].im * nav3v2_B.ad22.im);
              Z[nav3v2_B.ctemp_tmp].im = Z[nav3v2_B.ctemp_tmp].im *
                nav3v2_B.t1_re - (nav3v2_B.ad22.re * nav3v2_B.t1_im -
                                  nav3v2_B.ad22.im * nav3v2_B.absxr);
              Z[nav3v2_B.i_o].re = nav3v2_B.shift_re;
              Z[nav3v2_B.i_o].im = nav3v2_B.shift_im;
              nav3v2_B.jp1 += 3;
              nav3v2_B.ctemp_tmp_tmp += 3;
              nav3v2_B.shift_re = (Z[nav3v2_B.jp1].re * nav3v2_B.ad22.re -
                                   Z[nav3v2_B.jp1].im * nav3v2_B.ad22.im) +
                Z[nav3v2_B.ctemp_tmp_tmp].re * nav3v2_B.t1_re;
              nav3v2_B.shift_im = (Z[nav3v2_B.jp1].im * nav3v2_B.ad22.re +
                                   Z[nav3v2_B.jp1].re * nav3v2_B.ad22.im) +
                Z[nav3v2_B.ctemp_tmp_tmp].im * nav3v2_B.t1_re;
              nav3v2_B.t1_im = Z[nav3v2_B.ctemp_tmp_tmp].im;
              nav3v2_B.absxr = Z[nav3v2_B.ctemp_tmp_tmp].re;
              Z[nav3v2_B.jp1].re = Z[nav3v2_B.jp1].re * nav3v2_B.t1_re -
                (Z[nav3v2_B.ctemp_tmp_tmp].re * nav3v2_B.ad22.re +
                 Z[nav3v2_B.ctemp_tmp_tmp].im * nav3v2_B.ad22.im);
              Z[nav3v2_B.jp1].im = Z[nav3v2_B.jp1].im * nav3v2_B.t1_re -
                (nav3v2_B.ad22.re * nav3v2_B.t1_im - nav3v2_B.ad22.im *
                 nav3v2_B.absxr);
              Z[nav3v2_B.ctemp_tmp_tmp].re = nav3v2_B.shift_re;
              Z[nav3v2_B.ctemp_tmp_tmp].im = nav3v2_B.shift_im;
              nav3v2_B.jp1 = nav3v2_B.j - 1;
              nav3v2_B.j++;
            }
          }

          nav3v2_B.jiter++;
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (nav3v2_B.failed) {
      *info = nav3v2_B.ilast + 1;
      nav3v2_B.ifirst = 0;
      while (nav3v2_B.ifirst <= nav3v2_B.ilast) {
        alpha1[nav3v2_B.ifirst].re = (rtNaN);
        alpha1[nav3v2_B.ifirst].im = 0.0;
        beta1[nav3v2_B.ifirst].re = (rtNaN);
        beta1[nav3v2_B.ifirst].im = 0.0;
        nav3v2_B.ifirst++;
      }

      for (nav3v2_B.jp1 = 0; nav3v2_B.jp1 < 16; nav3v2_B.jp1++) {
        Z[nav3v2_B.jp1].re = (rtNaN);
        Z[nav3v2_B.jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    nav3v2_B.ilast = 0;
    while (nav3v2_B.ilast <= ilo - 2) {
      alpha1[nav3v2_B.ilast] = A[(nav3v2_B.ilast << 2) + nav3v2_B.ilast];
      nav3v2_B.ilast++;
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xztgevc(const creal_T A[16], creal_T V[16])
{
  nav3v2_B.rworka[0] = 0.0;
  nav3v2_B.rworka[2] = 0.0;
  nav3v2_B.rworka[3] = 0.0;
  nav3v2_B.anorm_g = fabs(A[0].re) + fabs(A[0].im);
  nav3v2_B.rworka[1] = fabs(A[4].re) + fabs(A[4].im);
  nav3v2_B.ascale_l = (fabs(A[5].re) + fabs(A[5].im)) + nav3v2_B.rworka[1];
  if (nav3v2_B.ascale_l > nav3v2_B.anorm_g) {
    nav3v2_B.anorm_g = nav3v2_B.ascale_l;
  }

  nav3v2_B.i_p = 0;
  while (nav3v2_B.i_p <= 1) {
    nav3v2_B.rworka[2] += fabs(A[nav3v2_B.i_p + 8].re) + fabs(A[nav3v2_B.i_p + 8]
      .im);
    nav3v2_B.i_p++;
  }

  nav3v2_B.ascale_l = (fabs(A[10].re) + fabs(A[10].im)) + nav3v2_B.rworka[2];
  if (nav3v2_B.ascale_l > nav3v2_B.anorm_g) {
    nav3v2_B.anorm_g = nav3v2_B.ascale_l;
  }

  nav3v2_B.i_p = 0;
  while (nav3v2_B.i_p <= 2) {
    nav3v2_B.rworka[3] += fabs(A[nav3v2_B.i_p + 12].re) + fabs(A[nav3v2_B.i_p +
      12].im);
    nav3v2_B.i_p++;
  }

  nav3v2_B.ascale_l = (fabs(A[15].re) + fabs(A[15].im)) + nav3v2_B.rworka[3];
  if (nav3v2_B.ascale_l > nav3v2_B.anorm_g) {
    nav3v2_B.anorm_g = nav3v2_B.ascale_l;
  }

  nav3v2_B.ascale_l = nav3v2_B.anorm_g;
  if (2.2250738585072014E-308 > nav3v2_B.anorm_g) {
    nav3v2_B.ascale_l = 2.2250738585072014E-308;
  }

  nav3v2_B.ascale_l = 1.0 / nav3v2_B.ascale_l;
  for (nav3v2_B.i_p = 0; nav3v2_B.i_p < 4; nav3v2_B.i_p++) {
    nav3v2_B.c_x_tmp_tmp = (3 - nav3v2_B.i_p) << 2;
    nav3v2_B.c_x_tmp = (nav3v2_B.c_x_tmp_tmp - nav3v2_B.i_p) + 3;
    nav3v2_B.salpha_re = (fabs(A[nav3v2_B.c_x_tmp].re) + fabs(A[nav3v2_B.c_x_tmp]
      .im)) * nav3v2_B.ascale_l;
    if (1.0 > nav3v2_B.salpha_re) {
      nav3v2_B.salpha_re = 1.0;
    }

    nav3v2_B.temp = 1.0 / nav3v2_B.salpha_re;
    nav3v2_B.salpha_re = A[nav3v2_B.c_x_tmp].re * nav3v2_B.temp *
      nav3v2_B.ascale_l;
    nav3v2_B.salpha_im = A[nav3v2_B.c_x_tmp].im * nav3v2_B.temp *
      nav3v2_B.ascale_l;
    nav3v2_B.acoeff = nav3v2_B.temp * nav3v2_B.ascale_l;
    nav3v2_B.lscalea = ((nav3v2_B.temp >= 2.2250738585072014E-308) &&
                        (nav3v2_B.acoeff < 4.0083367200179456E-292));
    nav3v2_B.dmin = fabs(nav3v2_B.salpha_re) + fabs(nav3v2_B.salpha_im);
    if ((nav3v2_B.dmin >= 2.2250738585072014E-308) && (nav3v2_B.dmin <
         4.0083367200179456E-292)) {
      nav3v2_B.lscaleb = true;
    } else {
      nav3v2_B.lscaleb = false;
    }

    nav3v2_B.scale_d = 1.0;
    if (nav3v2_B.lscalea) {
      nav3v2_B.scale_d = nav3v2_B.anorm_g;
      if (2.4948003869184E+291 < nav3v2_B.anorm_g) {
        nav3v2_B.scale_d = 2.4948003869184E+291;
      }

      nav3v2_B.scale_d *= 4.0083367200179456E-292 / nav3v2_B.temp;
    }

    if (nav3v2_B.lscaleb) {
      nav3v2_B.work2_idx_2_im = 4.0083367200179456E-292 / nav3v2_B.dmin;
      if (nav3v2_B.work2_idx_2_im > nav3v2_B.scale_d) {
        nav3v2_B.scale_d = nav3v2_B.work2_idx_2_im;
      }
    }

    if (nav3v2_B.lscalea || nav3v2_B.lscaleb) {
      nav3v2_B.work2_idx_2_im = nav3v2_B.acoeff;
      if (1.0 > nav3v2_B.acoeff) {
        nav3v2_B.work2_idx_2_im = 1.0;
      }

      if (nav3v2_B.dmin > nav3v2_B.work2_idx_2_im) {
        nav3v2_B.work2_idx_2_im = nav3v2_B.dmin;
      }

      nav3v2_B.dmin = 1.0 / (2.2250738585072014E-308 * nav3v2_B.work2_idx_2_im);
      if (nav3v2_B.dmin < nav3v2_B.scale_d) {
        nav3v2_B.scale_d = nav3v2_B.dmin;
      }

      if (nav3v2_B.lscalea) {
        nav3v2_B.acoeff = nav3v2_B.scale_d * nav3v2_B.temp * nav3v2_B.ascale_l;
      } else {
        nav3v2_B.acoeff *= nav3v2_B.scale_d;
      }

      nav3v2_B.salpha_re *= nav3v2_B.scale_d;
      nav3v2_B.salpha_im *= nav3v2_B.scale_d;
    }

    memset(&nav3v2_B.work1[0], 0, sizeof(creal_T) << 2U);
    nav3v2_B.work1[3 - nav3v2_B.i_p].re = 1.0;
    nav3v2_B.work1[3 - nav3v2_B.i_p].im = 0.0;
    nav3v2_B.dmin = 2.2204460492503131E-16 * nav3v2_B.acoeff * nav3v2_B.anorm_g;
    nav3v2_B.temp = (fabs(nav3v2_B.salpha_re) + fabs(nav3v2_B.salpha_im)) *
      2.2204460492503131E-16;
    if (nav3v2_B.temp > nav3v2_B.dmin) {
      nav3v2_B.dmin = nav3v2_B.temp;
    }

    if (2.2250738585072014E-308 > nav3v2_B.dmin) {
      nav3v2_B.dmin = 2.2250738585072014E-308;
    }

    nav3v2_B.c_x_tmp = 0;
    while (nav3v2_B.c_x_tmp <= 2 - nav3v2_B.i_p) {
      nav3v2_B.d_re_tmp = nav3v2_B.c_x_tmp_tmp + nav3v2_B.c_x_tmp;
      nav3v2_B.work1[nav3v2_B.c_x_tmp].re = A[nav3v2_B.d_re_tmp].re *
        nav3v2_B.acoeff;
      nav3v2_B.work1[nav3v2_B.c_x_tmp].im = A[nav3v2_B.d_re_tmp].im *
        nav3v2_B.acoeff;
      nav3v2_B.c_x_tmp++;
    }

    nav3v2_B.work1[3 - nav3v2_B.i_p].re = 1.0;
    nav3v2_B.work1[3 - nav3v2_B.i_p].im = 0.0;
    nav3v2_B.c_x_tmp = static_cast<int32_T>(((-1.0 - ((-static_cast<real_T>
      (nav3v2_B.i_p) + 4.0) - 1.0)) + 1.0) / -1.0);
    nav3v2_B.c_j_p = 0;
    while (nav3v2_B.c_j_p <= nav3v2_B.c_x_tmp - 1) {
      nav3v2_B.work2_idx_1_re_tmp = 2 - (nav3v2_B.i_p + nav3v2_B.c_j_p);
      nav3v2_B.d_re_tmp_tmp = nav3v2_B.work2_idx_1_re_tmp << 2;
      nav3v2_B.d_re_tmp = nav3v2_B.d_re_tmp_tmp + nav3v2_B.work2_idx_1_re_tmp;
      nav3v2_B.work2_idx_3_re = A[nav3v2_B.d_re_tmp].re * nav3v2_B.acoeff -
        nav3v2_B.salpha_re;
      nav3v2_B.scale_d = A[nav3v2_B.d_re_tmp].im * nav3v2_B.acoeff -
        nav3v2_B.salpha_im;
      if (fabs(nav3v2_B.work2_idx_3_re) + fabs(nav3v2_B.scale_d) <=
          nav3v2_B.dmin) {
        nav3v2_B.work2_idx_3_re = nav3v2_B.dmin;
        nav3v2_B.scale_d = 0.0;
      }

      nav3v2_B.work2_idx_2_im = fabs(nav3v2_B.work2_idx_3_re);
      nav3v2_B.work2_idx_3_im = fabs(nav3v2_B.scale_d);
      nav3v2_B.temp = nav3v2_B.work2_idx_2_im + nav3v2_B.work2_idx_3_im;
      if (nav3v2_B.temp < 1.0) {
        nav3v2_B.f_y = fabs(nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re) +
          fabs(nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im);
        if (nav3v2_B.f_y >= nav3v2_B.temp * 1.1235582092889474E+307) {
          nav3v2_B.temp = 1.0 / nav3v2_B.f_y;
          nav3v2_B.d_re_tmp = 0;
          while (nav3v2_B.d_re_tmp <= 3 - nav3v2_B.i_p) {
            nav3v2_B.work1[nav3v2_B.d_re_tmp].re *= nav3v2_B.temp;
            nav3v2_B.work1[nav3v2_B.d_re_tmp].im *= nav3v2_B.temp;
            nav3v2_B.d_re_tmp++;
          }
        }
      }

      if (nav3v2_B.scale_d == 0.0) {
        if (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im == 0.0) {
          nav3v2_B.temp = -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re /
            nav3v2_B.work2_idx_3_re;
          nav3v2_B.scale_d = 0.0;
        } else if (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re == 0.0) {
          nav3v2_B.temp = 0.0;
          nav3v2_B.scale_d = -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im /
            nav3v2_B.work2_idx_3_re;
        } else {
          nav3v2_B.temp = -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re /
            nav3v2_B.work2_idx_3_re;
          nav3v2_B.scale_d = -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im /
            nav3v2_B.work2_idx_3_re;
        }
      } else if (nav3v2_B.work2_idx_3_re == 0.0) {
        if (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re == 0.0) {
          nav3v2_B.temp = -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im /
            nav3v2_B.scale_d;
          nav3v2_B.scale_d = 0.0;
        } else if (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im == 0.0) {
          nav3v2_B.temp = 0.0;
          nav3v2_B.scale_d = -(-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re /
                               nav3v2_B.scale_d);
        } else {
          nav3v2_B.temp = -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im /
            nav3v2_B.scale_d;
          nav3v2_B.scale_d = -(-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re /
                               nav3v2_B.scale_d);
        }
      } else if (nav3v2_B.work2_idx_2_im > nav3v2_B.work2_idx_3_im) {
        nav3v2_B.work2_idx_2_im = nav3v2_B.scale_d / nav3v2_B.work2_idx_3_re;
        nav3v2_B.scale_d = nav3v2_B.work2_idx_2_im * nav3v2_B.scale_d +
          nav3v2_B.work2_idx_3_re;
        nav3v2_B.temp = (nav3v2_B.work2_idx_2_im *
                         -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im +
                         -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re) /
          nav3v2_B.scale_d;
        nav3v2_B.scale_d = (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im -
                            nav3v2_B.work2_idx_2_im *
                            -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re) /
          nav3v2_B.scale_d;
      } else if (nav3v2_B.work2_idx_3_im == nav3v2_B.work2_idx_2_im) {
        nav3v2_B.work2_idx_3_re = nav3v2_B.work2_idx_3_re > 0.0 ? 0.5 : -0.5;
        nav3v2_B.scale_d = nav3v2_B.scale_d > 0.0 ? 0.5 : -0.5;
        nav3v2_B.temp = (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re *
                         nav3v2_B.work2_idx_3_re +
                         -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im *
                         nav3v2_B.scale_d) / nav3v2_B.work2_idx_2_im;
        nav3v2_B.scale_d = (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im *
                            nav3v2_B.work2_idx_3_re -
                            -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re *
                            nav3v2_B.scale_d) / nav3v2_B.work2_idx_2_im;
      } else {
        nav3v2_B.work2_idx_2_im = nav3v2_B.work2_idx_3_re / nav3v2_B.scale_d;
        nav3v2_B.scale_d += nav3v2_B.work2_idx_2_im * nav3v2_B.work2_idx_3_re;
        nav3v2_B.temp = (nav3v2_B.work2_idx_2_im *
                         -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re +
                         -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im) /
          nav3v2_B.scale_d;
        nav3v2_B.scale_d = (nav3v2_B.work2_idx_2_im *
                            -nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im -
                            (-nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re)) /
          nav3v2_B.scale_d;
      }

      nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re = nav3v2_B.temp;
      nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im = nav3v2_B.scale_d;
      if (nav3v2_B.work2_idx_1_re_tmp + 1 > 1) {
        if (fabs(nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re) + fabs
            (nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im) > 1.0) {
          nav3v2_B.temp = 1.0 / (fabs(nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp]
            .re) + fabs(nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im));
          if (nav3v2_B.acoeff * nav3v2_B.rworka[nav3v2_B.work2_idx_1_re_tmp] >=
              1.1235582092889474E+307 * nav3v2_B.temp) {
            nav3v2_B.d_re_tmp = 0;
            while (nav3v2_B.d_re_tmp <= 3 - nav3v2_B.i_p) {
              nav3v2_B.work1[nav3v2_B.d_re_tmp].re *= nav3v2_B.temp;
              nav3v2_B.work1[nav3v2_B.d_re_tmp].im *= nav3v2_B.temp;
              nav3v2_B.d_re_tmp++;
            }
          }
        }

        nav3v2_B.work2_idx_3_re = nav3v2_B.acoeff *
          nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].re;
        nav3v2_B.scale_d = nav3v2_B.acoeff *
          nav3v2_B.work1[nav3v2_B.work2_idx_1_re_tmp].im;
        nav3v2_B.e_jr = 0;
        while (nav3v2_B.e_jr <= nav3v2_B.work2_idx_1_re_tmp - 1) {
          nav3v2_B.d_re_tmp = nav3v2_B.d_re_tmp_tmp + nav3v2_B.e_jr;
          nav3v2_B.work1[nav3v2_B.e_jr].re += A[nav3v2_B.d_re_tmp].re *
            nav3v2_B.work2_idx_3_re - A[nav3v2_B.d_re_tmp].im * nav3v2_B.scale_d;
          nav3v2_B.work1[nav3v2_B.e_jr].im += A[nav3v2_B.d_re_tmp].im *
            nav3v2_B.work2_idx_3_re + A[nav3v2_B.d_re_tmp].re * nav3v2_B.scale_d;
          nav3v2_B.e_jr++;
        }
      }

      nav3v2_B.c_j_p++;
    }

    nav3v2_B.salpha_re = 0.0;
    nav3v2_B.salpha_im = 0.0;
    nav3v2_B.acoeff = 0.0;
    nav3v2_B.dmin = 0.0;
    nav3v2_B.scale_d = 0.0;
    nav3v2_B.work2_idx_2_im = 0.0;
    nav3v2_B.work2_idx_3_re = 0.0;
    nav3v2_B.work2_idx_3_im = 0.0;
    nav3v2_B.c_x_tmp = 0;
    while (nav3v2_B.c_x_tmp <= 3 - nav3v2_B.i_p) {
      nav3v2_B.c_j_p = nav3v2_B.c_x_tmp << 2;
      nav3v2_B.salpha_re += V[nav3v2_B.c_j_p].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re - V[nav3v2_B.c_j_p].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im;
      nav3v2_B.salpha_im += V[nav3v2_B.c_j_p].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im + V[nav3v2_B.c_j_p].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re;
      nav3v2_B.work2_idx_1_re_tmp = nav3v2_B.c_j_p + 1;
      nav3v2_B.acoeff += V[nav3v2_B.work2_idx_1_re_tmp].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re - V[nav3v2_B.work2_idx_1_re_tmp].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im;
      nav3v2_B.dmin += V[nav3v2_B.work2_idx_1_re_tmp].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im + V[nav3v2_B.work2_idx_1_re_tmp].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re;
      nav3v2_B.work2_idx_1_re_tmp = nav3v2_B.c_j_p + 2;
      nav3v2_B.scale_d += V[nav3v2_B.work2_idx_1_re_tmp].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re - V[nav3v2_B.work2_idx_1_re_tmp].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im;
      nav3v2_B.work2_idx_2_im += V[nav3v2_B.work2_idx_1_re_tmp].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im + V[nav3v2_B.work2_idx_1_re_tmp].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re;
      nav3v2_B.c_j_p += 3;
      nav3v2_B.work2_idx_3_re += V[nav3v2_B.c_j_p].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re - V[nav3v2_B.c_j_p].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im;
      nav3v2_B.work2_idx_3_im += V[nav3v2_B.c_j_p].re *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].im + V[nav3v2_B.c_j_p].im *
        nav3v2_B.work1[nav3v2_B.c_x_tmp].re;
      nav3v2_B.c_x_tmp++;
    }

    nav3v2_B.temp = fabs(nav3v2_B.salpha_re) + fabs(nav3v2_B.salpha_im);
    nav3v2_B.f_y = fabs(nav3v2_B.acoeff) + fabs(nav3v2_B.dmin);
    if (nav3v2_B.f_y > nav3v2_B.temp) {
      nav3v2_B.temp = nav3v2_B.f_y;
    }

    nav3v2_B.f_y = fabs(nav3v2_B.scale_d) + fabs(nav3v2_B.work2_idx_2_im);
    if (nav3v2_B.f_y > nav3v2_B.temp) {
      nav3v2_B.temp = nav3v2_B.f_y;
    }

    nav3v2_B.f_y = fabs(nav3v2_B.work2_idx_3_re) + fabs(nav3v2_B.work2_idx_3_im);
    if (nav3v2_B.f_y > nav3v2_B.temp) {
      nav3v2_B.temp = nav3v2_B.f_y;
    }

    if (nav3v2_B.temp > 2.2250738585072014E-308) {
      nav3v2_B.temp = 1.0 / nav3v2_B.temp;
      V[nav3v2_B.c_x_tmp_tmp].re = nav3v2_B.temp * nav3v2_B.salpha_re;
      V[nav3v2_B.c_x_tmp_tmp].im = nav3v2_B.temp * nav3v2_B.salpha_im;
      nav3v2_B.d_re_tmp = ((3 - nav3v2_B.i_p) << 2) + 1;
      V[nav3v2_B.d_re_tmp].re = nav3v2_B.temp * nav3v2_B.acoeff;
      V[nav3v2_B.d_re_tmp].im = nav3v2_B.temp * nav3v2_B.dmin;
      nav3v2_B.d_re_tmp = ((3 - nav3v2_B.i_p) << 2) + 2;
      V[nav3v2_B.d_re_tmp].re = nav3v2_B.temp * nav3v2_B.scale_d;
      V[nav3v2_B.d_re_tmp].im = nav3v2_B.temp * nav3v2_B.work2_idx_2_im;
      nav3v2_B.d_re_tmp = ((3 - nav3v2_B.i_p) << 2) + 3;
      V[nav3v2_B.d_re_tmp].re = nav3v2_B.temp * nav3v2_B.work2_idx_3_re;
      V[nav3v2_B.d_re_tmp].im = nav3v2_B.temp * nav3v2_B.work2_idx_3_im;
    } else {
      V[nav3v2_B.c_x_tmp_tmp].re = 0.0;
      V[nav3v2_B.c_x_tmp_tmp].im = 0.0;
      nav3v2_B.d_re_tmp = nav3v2_B.c_x_tmp_tmp + 1;
      V[nav3v2_B.d_re_tmp].re = 0.0;
      V[nav3v2_B.d_re_tmp].im = 0.0;
      nav3v2_B.d_re_tmp = nav3v2_B.c_x_tmp_tmp + 2;
      V[nav3v2_B.d_re_tmp].re = 0.0;
      V[nav3v2_B.d_re_tmp].im = 0.0;
      nav3v2_B.d_re_tmp = nav3v2_B.c_x_tmp_tmp + 3;
      V[nav3v2_B.d_re_tmp].re = 0.0;
      V[nav3v2_B.d_re_tmp].im = 0.0;
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xzggev(creal_T A[16], int32_T *info, creal_T alpha1[4],
  creal_T beta1[4], creal_T V[16])
{
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *info = 0;
  nav3v2_B.anrm = nav3v2_xzlangeM(A);
  if (rtIsInf(nav3v2_B.anrm) || rtIsNaN(nav3v2_B.anrm)) {
    alpha1[0].re = (rtNaN);
    alpha1[0].im = 0.0;
    beta1[0].re = (rtNaN);
    beta1[0].im = 0.0;
    alpha1[1].re = (rtNaN);
    alpha1[1].im = 0.0;
    beta1[1].re = (rtNaN);
    beta1[1].im = 0.0;
    alpha1[2].re = (rtNaN);
    alpha1[2].im = 0.0;
    beta1[2].re = (rtNaN);
    beta1[2].im = 0.0;
    alpha1[3].re = (rtNaN);
    alpha1[3].im = 0.0;
    beta1[3].re = (rtNaN);
    beta1[3].im = 0.0;
    for (nav3v2_B.k = 0; nav3v2_B.k < 16; nav3v2_B.k++) {
      V[nav3v2_B.k].re = (rtNaN);
      V[nav3v2_B.k].im = 0.0;
    }
  } else {
    nav3v2_B.ilascl = false;
    nav3v2_B.anrmto = nav3v2_B.anrm;
    if ((nav3v2_B.anrm > 0.0) && (nav3v2_B.anrm < 6.7178761075670888E-139)) {
      nav3v2_B.anrmto = 6.7178761075670888E-139;
      nav3v2_B.ilascl = true;
      nav3v2_xzlascl(nav3v2_B.anrm, nav3v2_B.anrmto, A);
    } else {
      if (nav3v2_B.anrm > 1.4885657073574029E+138) {
        nav3v2_B.anrmto = 1.4885657073574029E+138;
        nav3v2_B.ilascl = true;
        nav3v2_xzlascl(nav3v2_B.anrm, nav3v2_B.anrmto, A);
      }
    }

    nav3v2_B.rscale[0] = 1;
    nav3v2_B.rscale[1] = 1;
    nav3v2_B.rscale[2] = 1;
    nav3v2_B.rscale[3] = 1;
    nav3v2_B.c_i = 0;
    nav3v2_B.ihi = 4;
    do {
      exitg2 = 0;
      nav3v2_B.i_i = 0;
      nav3v2_B.jcol = 0;
      nav3v2_B.found = false;
      nav3v2_B.ii = nav3v2_B.ihi;
      exitg3 = false;
      while ((!exitg3) && (nav3v2_B.ii > 0)) {
        nav3v2_B.nzcount = 0;
        nav3v2_B.i_i = nav3v2_B.ii;
        nav3v2_B.jcol = nav3v2_B.ihi;
        nav3v2_B.jj = 0;
        exitg4 = false;
        while ((!exitg4) && (nav3v2_B.jj <= nav3v2_B.ihi - 1)) {
          nav3v2_B.k = ((nav3v2_B.jj << 2) + nav3v2_B.ii) - 1;
          if ((A[nav3v2_B.k].re != 0.0) || (A[nav3v2_B.k].im != 0.0) ||
              (nav3v2_B.jj + 1 == nav3v2_B.ii)) {
            if (nav3v2_B.nzcount == 0) {
              nav3v2_B.jcol = nav3v2_B.jj + 1;
              nav3v2_B.nzcount = 1;
              nav3v2_B.jj++;
            } else {
              nav3v2_B.nzcount = 2;
              exitg4 = true;
            }
          } else {
            nav3v2_B.jj++;
          }
        }

        if (nav3v2_B.nzcount < 2) {
          nav3v2_B.found = true;
          exitg3 = true;
        } else {
          nav3v2_B.ii--;
        }
      }

      if (!nav3v2_B.found) {
        exitg2 = 2;
      } else {
        if (nav3v2_B.i_i != nav3v2_B.ihi) {
          nav3v2_B.atmp_re = A[nav3v2_B.i_i - 1].re;
          nav3v2_B.atmp_im = A[nav3v2_B.i_i - 1].im;
          A[nav3v2_B.i_i - 1] = A[nav3v2_B.ihi - 1];
          A[nav3v2_B.ihi - 1].re = nav3v2_B.atmp_re;
          A[nav3v2_B.ihi - 1].im = nav3v2_B.atmp_im;
          nav3v2_B.atmp_re = A[nav3v2_B.i_i + 3].re;
          nav3v2_B.atmp_im = A[nav3v2_B.i_i + 3].im;
          A[nav3v2_B.i_i + 3] = A[nav3v2_B.ihi + 3];
          A[nav3v2_B.ihi + 3].re = nav3v2_B.atmp_re;
          A[nav3v2_B.ihi + 3].im = nav3v2_B.atmp_im;
          nav3v2_B.atmp_re = A[nav3v2_B.i_i + 7].re;
          nav3v2_B.atmp_im = A[nav3v2_B.i_i + 7].im;
          A[nav3v2_B.i_i + 7] = A[nav3v2_B.ihi + 7];
          A[nav3v2_B.ihi + 7].re = nav3v2_B.atmp_re;
          A[nav3v2_B.ihi + 7].im = nav3v2_B.atmp_im;
          nav3v2_B.atmp_re = A[nav3v2_B.i_i + 11].re;
          nav3v2_B.atmp_im = A[nav3v2_B.i_i + 11].im;
          A[nav3v2_B.i_i + 11] = A[nav3v2_B.ihi + 11];
          A[nav3v2_B.ihi + 11].re = nav3v2_B.atmp_re;
          A[nav3v2_B.ihi + 11].im = nav3v2_B.atmp_im;
        }

        if (nav3v2_B.jcol != nav3v2_B.ihi) {
          nav3v2_B.ii = 0;
          while (nav3v2_B.ii <= nav3v2_B.ihi - 1) {
            nav3v2_B.i_i = ((nav3v2_B.jcol - 1) << 2) + nav3v2_B.ii;
            nav3v2_B.atmp_re = A[nav3v2_B.i_i].re;
            nav3v2_B.atmp_im = A[nav3v2_B.i_i].im;
            nav3v2_B.k = ((nav3v2_B.ihi - 1) << 2) + nav3v2_B.ii;
            A[nav3v2_B.i_i] = A[nav3v2_B.k];
            A[nav3v2_B.k].re = nav3v2_B.atmp_re;
            A[nav3v2_B.k].im = nav3v2_B.atmp_im;
            nav3v2_B.ii++;
          }
        }

        nav3v2_B.rscale[nav3v2_B.ihi - 1] = nav3v2_B.jcol;
        nav3v2_B.ihi--;
        if (nav3v2_B.ihi == 1) {
          nav3v2_B.rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        nav3v2_B.ii = 0;
        nav3v2_B.jcol = 0;
        nav3v2_B.found = false;
        nav3v2_B.i_i = nav3v2_B.c_i + 1;
        exitg3 = false;
        while ((!exitg3) && (nav3v2_B.i_i <= nav3v2_B.ihi)) {
          nav3v2_B.nzcount = 0;
          nav3v2_B.ii = nav3v2_B.ihi;
          nav3v2_B.jcol = nav3v2_B.i_i;
          nav3v2_B.jj = nav3v2_B.c_i + 1;
          exitg4 = false;
          while ((!exitg4) && (nav3v2_B.jj <= nav3v2_B.ihi)) {
            nav3v2_B.k = (((nav3v2_B.i_i - 1) << 2) + nav3v2_B.jj) - 1;
            if ((A[nav3v2_B.k].re != 0.0) || (A[nav3v2_B.k].im != 0.0) ||
                (nav3v2_B.jj == nav3v2_B.i_i)) {
              if (nav3v2_B.nzcount == 0) {
                nav3v2_B.ii = nav3v2_B.jj;
                nav3v2_B.nzcount = 1;
                nav3v2_B.jj++;
              } else {
                nav3v2_B.nzcount = 2;
                exitg4 = true;
              }
            } else {
              nav3v2_B.jj++;
            }
          }

          if (nav3v2_B.nzcount < 2) {
            nav3v2_B.found = true;
            exitg3 = true;
          } else {
            nav3v2_B.i_i++;
          }
        }

        if (!nav3v2_B.found) {
          exitg1 = 1;
        } else {
          if (nav3v2_B.c_i + 1 != nav3v2_B.ii) {
            nav3v2_B.nzcount = nav3v2_B.c_i;
            while (nav3v2_B.nzcount + 1 < 5) {
              nav3v2_B.k = nav3v2_B.nzcount << 2;
              nav3v2_B.i_i = (nav3v2_B.k + nav3v2_B.ii) - 1;
              nav3v2_B.atmp_re = A[nav3v2_B.i_i].re;
              nav3v2_B.atmp_im = A[nav3v2_B.i_i].im;
              nav3v2_B.k += nav3v2_B.c_i;
              A[nav3v2_B.i_i] = A[nav3v2_B.k];
              A[nav3v2_B.k].re = nav3v2_B.atmp_re;
              A[nav3v2_B.k].im = nav3v2_B.atmp_im;
              nav3v2_B.nzcount++;
            }
          }

          if (nav3v2_B.c_i + 1 != nav3v2_B.jcol) {
            nav3v2_B.ii = 0;
            while (nav3v2_B.ii <= nav3v2_B.ihi - 1) {
              nav3v2_B.i_i = ((nav3v2_B.jcol - 1) << 2) + nav3v2_B.ii;
              nav3v2_B.atmp_re = A[nav3v2_B.i_i].re;
              nav3v2_B.atmp_im = A[nav3v2_B.i_i].im;
              nav3v2_B.k = (nav3v2_B.c_i << 2) + nav3v2_B.ii;
              A[nav3v2_B.i_i] = A[nav3v2_B.k];
              A[nav3v2_B.k].re = nav3v2_B.atmp_re;
              A[nav3v2_B.k].im = nav3v2_B.atmp_im;
              nav3v2_B.ii++;
            }
          }

          nav3v2_B.rscale[nav3v2_B.c_i] = nav3v2_B.jcol;
          nav3v2_B.c_i++;
          if (nav3v2_B.c_i + 1 == nav3v2_B.ihi) {
            nav3v2_B.rscale[nav3v2_B.c_i] = nav3v2_B.c_i + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (nav3v2_B.k = 0; nav3v2_B.k < 16; nav3v2_B.k++) {
      nav3v2_B.b_I[nav3v2_B.k] = 0;
    }

    nav3v2_B.b_I[0] = 1;
    nav3v2_B.b_I[5] = 1;
    nav3v2_B.b_I[10] = 1;
    nav3v2_B.b_I[15] = 1;
    for (nav3v2_B.k = 0; nav3v2_B.k < 16; nav3v2_B.k++) {
      V[nav3v2_B.k].re = nav3v2_B.b_I[nav3v2_B.k];
      V[nav3v2_B.k].im = 0.0;
    }

    if (nav3v2_B.ihi >= nav3v2_B.c_i + 3) {
      nav3v2_B.jcol = nav3v2_B.c_i;
      while (nav3v2_B.jcol + 1 < nav3v2_B.ihi - 1) {
        nav3v2_B.ii = nav3v2_B.ihi - 1;
        while (nav3v2_B.ii + 1 > nav3v2_B.jcol + 2) {
          nav3v2_xzlartg(A[(nav3v2_B.ii + (nav3v2_B.jcol << 2)) - 1],
                         A[nav3v2_B.ii + (nav3v2_B.jcol << 2)], &nav3v2_B.mul,
                         &nav3v2_B.s, &A[(nav3v2_B.ii + (nav3v2_B.jcol << 2)) -
                         1]);
          nav3v2_B.k = nav3v2_B.ii + (nav3v2_B.jcol << 2);
          A[nav3v2_B.k].re = 0.0;
          A[nav3v2_B.k].im = 0.0;
          nav3v2_B.nzcount = nav3v2_B.jcol + 1;
          while (nav3v2_B.nzcount + 1 < 5) {
            nav3v2_B.i_i = (nav3v2_B.nzcount << 2) + nav3v2_B.ii;
            nav3v2_B.k = nav3v2_B.i_i - 1;
            nav3v2_B.atmp_re = A[nav3v2_B.k].re * nav3v2_B.mul + (A[nav3v2_B.i_i]
              .re * nav3v2_B.s.re - A[nav3v2_B.i_i].im * nav3v2_B.s.im);
            nav3v2_B.atmp_im = A[nav3v2_B.k].im * nav3v2_B.mul + (A[nav3v2_B.i_i]
              .im * nav3v2_B.s.re + A[nav3v2_B.i_i].re * nav3v2_B.s.im);
            nav3v2_B.d1 = A[nav3v2_B.k].im;
            nav3v2_B.d = A[nav3v2_B.k].re;
            A[nav3v2_B.i_i].re = A[nav3v2_B.i_i].re * nav3v2_B.mul -
              (A[nav3v2_B.k].re * nav3v2_B.s.re + A[nav3v2_B.k].im *
               nav3v2_B.s.im);
            A[nav3v2_B.i_i].im = A[nav3v2_B.i_i].im * nav3v2_B.mul -
              (nav3v2_B.s.re * nav3v2_B.d1 - nav3v2_B.s.im * nav3v2_B.d);
            A[nav3v2_B.k].re = nav3v2_B.atmp_re;
            A[nav3v2_B.k].im = nav3v2_B.atmp_im;
            nav3v2_B.nzcount++;
          }

          nav3v2_B.s.re = -nav3v2_B.s.re;
          nav3v2_B.s.im = -nav3v2_B.s.im;
          nav3v2_B.nzcount = 0;
          while (nav3v2_B.nzcount + 1 <= nav3v2_B.ihi) {
            nav3v2_B.i_i = ((nav3v2_B.ii - 1) << 2) + nav3v2_B.nzcount;
            nav3v2_B.k = (nav3v2_B.ii << 2) + nav3v2_B.nzcount;
            nav3v2_B.atmp_re = (A[nav3v2_B.i_i].re * nav3v2_B.s.re -
                                A[nav3v2_B.i_i].im * nav3v2_B.s.im) +
              A[nav3v2_B.k].re * nav3v2_B.mul;
            nav3v2_B.atmp_im = (A[nav3v2_B.i_i].im * nav3v2_B.s.re +
                                A[nav3v2_B.i_i].re * nav3v2_B.s.im) +
              A[nav3v2_B.k].im * nav3v2_B.mul;
            nav3v2_B.d1 = A[nav3v2_B.k].im;
            nav3v2_B.d = A[nav3v2_B.k].re;
            A[nav3v2_B.i_i].re = A[nav3v2_B.i_i].re * nav3v2_B.mul -
              (A[nav3v2_B.k].re * nav3v2_B.s.re + A[nav3v2_B.k].im *
               nav3v2_B.s.im);
            A[nav3v2_B.i_i].im = A[nav3v2_B.i_i].im * nav3v2_B.mul -
              (nav3v2_B.s.re * nav3v2_B.d1 - nav3v2_B.s.im * nav3v2_B.d);
            A[nav3v2_B.k].re = nav3v2_B.atmp_re;
            A[nav3v2_B.k].im = nav3v2_B.atmp_im;
            nav3v2_B.nzcount++;
          }

          nav3v2_B.i_i = (nav3v2_B.ii - 1) << 2;
          nav3v2_B.k = nav3v2_B.ii << 2;
          nav3v2_B.atmp_re = (V[nav3v2_B.i_i].re * nav3v2_B.s.re -
                              V[nav3v2_B.i_i].im * nav3v2_B.s.im) + V[nav3v2_B.k]
            .re * nav3v2_B.mul;
          nav3v2_B.atmp_im = (V[nav3v2_B.i_i].im * nav3v2_B.s.re +
                              V[nav3v2_B.i_i].re * nav3v2_B.s.im) + V[nav3v2_B.k]
            .im * nav3v2_B.mul;
          nav3v2_B.d1 = V[nav3v2_B.k].re;
          V[nav3v2_B.i_i].re = V[nav3v2_B.i_i].re * nav3v2_B.mul - (V[nav3v2_B.k]
            .re * nav3v2_B.s.re + V[nav3v2_B.k].im * nav3v2_B.s.im);
          V[nav3v2_B.i_i].im = V[nav3v2_B.i_i].im * nav3v2_B.mul - (V[nav3v2_B.k]
            .im * nav3v2_B.s.re - nav3v2_B.s.im * nav3v2_B.d1);
          V[nav3v2_B.k].re = nav3v2_B.atmp_re;
          V[nav3v2_B.k].im = nav3v2_B.atmp_im;
          nav3v2_B.nzcount = nav3v2_B.i_i + 1;
          nav3v2_B.jj = nav3v2_B.k + 1;
          nav3v2_B.atmp_re = (V[nav3v2_B.nzcount].re * nav3v2_B.s.re -
                              V[nav3v2_B.nzcount].im * nav3v2_B.s.im) +
            V[nav3v2_B.jj].re * nav3v2_B.mul;
          nav3v2_B.atmp_im = (V[nav3v2_B.nzcount].im * nav3v2_B.s.re +
                              V[nav3v2_B.nzcount].re * nav3v2_B.s.im) +
            V[nav3v2_B.jj].im * nav3v2_B.mul;
          nav3v2_B.d1 = V[nav3v2_B.jj].re;
          V[nav3v2_B.nzcount].re = V[nav3v2_B.nzcount].re * nav3v2_B.mul -
            (V[nav3v2_B.jj].re * nav3v2_B.s.re + V[nav3v2_B.jj].im *
             nav3v2_B.s.im);
          V[nav3v2_B.nzcount].im = V[nav3v2_B.nzcount].im * nav3v2_B.mul -
            (V[nav3v2_B.jj].im * nav3v2_B.s.re - nav3v2_B.s.im * nav3v2_B.d1);
          V[nav3v2_B.jj].re = nav3v2_B.atmp_re;
          V[nav3v2_B.jj].im = nav3v2_B.atmp_im;
          nav3v2_B.nzcount = nav3v2_B.i_i + 2;
          nav3v2_B.jj = nav3v2_B.k + 2;
          nav3v2_B.atmp_re = (V[nav3v2_B.nzcount].re * nav3v2_B.s.re -
                              V[nav3v2_B.nzcount].im * nav3v2_B.s.im) +
            V[nav3v2_B.jj].re * nav3v2_B.mul;
          nav3v2_B.atmp_im = (V[nav3v2_B.nzcount].im * nav3v2_B.s.re +
                              V[nav3v2_B.nzcount].re * nav3v2_B.s.im) +
            V[nav3v2_B.jj].im * nav3v2_B.mul;
          nav3v2_B.d1 = V[nav3v2_B.jj].re;
          V[nav3v2_B.nzcount].re = V[nav3v2_B.nzcount].re * nav3v2_B.mul -
            (V[nav3v2_B.jj].re * nav3v2_B.s.re + V[nav3v2_B.jj].im *
             nav3v2_B.s.im);
          V[nav3v2_B.nzcount].im = V[nav3v2_B.nzcount].im * nav3v2_B.mul -
            (V[nav3v2_B.jj].im * nav3v2_B.s.re - nav3v2_B.s.im * nav3v2_B.d1);
          V[nav3v2_B.jj].re = nav3v2_B.atmp_re;
          V[nav3v2_B.jj].im = nav3v2_B.atmp_im;
          nav3v2_B.i_i += 3;
          nav3v2_B.k += 3;
          nav3v2_B.atmp_re = (V[nav3v2_B.i_i].re * nav3v2_B.s.re -
                              V[nav3v2_B.i_i].im * nav3v2_B.s.im) + V[nav3v2_B.k]
            .re * nav3v2_B.mul;
          nav3v2_B.atmp_im = (V[nav3v2_B.i_i].im * nav3v2_B.s.re +
                              V[nav3v2_B.i_i].re * nav3v2_B.s.im) + V[nav3v2_B.k]
            .im * nav3v2_B.mul;
          nav3v2_B.d1 = V[nav3v2_B.k].re;
          V[nav3v2_B.i_i].re = V[nav3v2_B.i_i].re * nav3v2_B.mul - (V[nav3v2_B.k]
            .re * nav3v2_B.s.re + V[nav3v2_B.k].im * nav3v2_B.s.im);
          V[nav3v2_B.i_i].im = V[nav3v2_B.i_i].im * nav3v2_B.mul - (V[nav3v2_B.k]
            .im * nav3v2_B.s.re - nav3v2_B.s.im * nav3v2_B.d1);
          V[nav3v2_B.k].re = nav3v2_B.atmp_re;
          V[nav3v2_B.k].im = nav3v2_B.atmp_im;
          nav3v2_B.ii--;
        }

        nav3v2_B.jcol++;
      }
    }

    nav3v2_xzhgeqz(A, nav3v2_B.c_i + 1, nav3v2_B.ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      nav3v2_xztgevc(A, V);
      if (nav3v2_B.c_i + 1 > 1) {
        nav3v2_B.c_i--;
        while (nav3v2_B.c_i + 1 >= 1) {
          nav3v2_B.k = nav3v2_B.rscale[nav3v2_B.c_i] - 1;
          if (nav3v2_B.c_i + 1 != nav3v2_B.rscale[nav3v2_B.c_i]) {
            nav3v2_B.atmp_re = V[nav3v2_B.c_i].re;
            nav3v2_B.atmp_im = V[nav3v2_B.c_i].im;
            V[nav3v2_B.c_i] = V[nav3v2_B.k];
            V[nav3v2_B.k].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k].im = nav3v2_B.atmp_im;
            nav3v2_B.atmp_re = V[nav3v2_B.c_i + 4].re;
            nav3v2_B.atmp_im = V[nav3v2_B.c_i + 4].im;
            V[nav3v2_B.c_i + 4] = V[nav3v2_B.k + 4];
            V[nav3v2_B.k + 4].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k + 4].im = nav3v2_B.atmp_im;
            nav3v2_B.atmp_re = V[nav3v2_B.c_i + 8].re;
            nav3v2_B.atmp_im = V[nav3v2_B.c_i + 8].im;
            V[nav3v2_B.c_i + 8] = V[nav3v2_B.k + 8];
            V[nav3v2_B.k + 8].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k + 8].im = nav3v2_B.atmp_im;
            nav3v2_B.atmp_re = V[nav3v2_B.c_i + 12].re;
            nav3v2_B.atmp_im = V[nav3v2_B.c_i + 12].im;
            V[nav3v2_B.c_i + 12] = V[nav3v2_B.k + 12];
            V[nav3v2_B.k + 12].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k + 12].im = nav3v2_B.atmp_im;
          }

          nav3v2_B.c_i--;
        }
      }

      if (nav3v2_B.ihi < 4) {
        while (nav3v2_B.ihi + 1 < 5) {
          nav3v2_B.k = nav3v2_B.rscale[nav3v2_B.ihi] - 1;
          if (nav3v2_B.ihi + 1 != nav3v2_B.rscale[nav3v2_B.ihi]) {
            nav3v2_B.atmp_re = V[nav3v2_B.ihi].re;
            nav3v2_B.atmp_im = V[nav3v2_B.ihi].im;
            V[nav3v2_B.ihi] = V[nav3v2_B.k];
            V[nav3v2_B.k].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k].im = nav3v2_B.atmp_im;
            nav3v2_B.atmp_re = V[nav3v2_B.ihi + 4].re;
            nav3v2_B.atmp_im = V[nav3v2_B.ihi + 4].im;
            V[nav3v2_B.ihi + 4] = V[nav3v2_B.k + 4];
            V[nav3v2_B.k + 4].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k + 4].im = nav3v2_B.atmp_im;
            nav3v2_B.atmp_re = V[nav3v2_B.ihi + 8].re;
            nav3v2_B.atmp_im = V[nav3v2_B.ihi + 8].im;
            V[nav3v2_B.ihi + 8] = V[nav3v2_B.k + 8];
            V[nav3v2_B.k + 8].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k + 8].im = nav3v2_B.atmp_im;
            nav3v2_B.atmp_re = V[nav3v2_B.ihi + 12].re;
            nav3v2_B.atmp_im = V[nav3v2_B.ihi + 12].im;
            V[nav3v2_B.ihi + 12] = V[nav3v2_B.k + 12];
            V[nav3v2_B.k + 12].re = nav3v2_B.atmp_re;
            V[nav3v2_B.k + 12].im = nav3v2_B.atmp_im;
          }

          nav3v2_B.ihi++;
        }
      }

      for (nav3v2_B.ihi = 0; nav3v2_B.ihi < 4; nav3v2_B.ihi++) {
        nav3v2_B.c_i = nav3v2_B.ihi << 2;
        nav3v2_B.atmp_re = fabs(V[nav3v2_B.c_i].re) + fabs(V[nav3v2_B.c_i].im);
        nav3v2_B.k = nav3v2_B.c_i + 1;
        nav3v2_B.atmp_im = fabs(V[nav3v2_B.k].re) + fabs(V[nav3v2_B.k].im);
        if (nav3v2_B.atmp_im > nav3v2_B.atmp_re) {
          nav3v2_B.atmp_re = nav3v2_B.atmp_im;
        }

        nav3v2_B.i_i = nav3v2_B.c_i + 2;
        nav3v2_B.atmp_im = fabs(V[nav3v2_B.i_i].re) + fabs(V[nav3v2_B.i_i].im);
        if (nav3v2_B.atmp_im > nav3v2_B.atmp_re) {
          nav3v2_B.atmp_re = nav3v2_B.atmp_im;
        }

        nav3v2_B.jcol = nav3v2_B.c_i + 3;
        nav3v2_B.atmp_im = fabs(V[nav3v2_B.jcol].re) + fabs(V[nav3v2_B.jcol].im);
        if (nav3v2_B.atmp_im > nav3v2_B.atmp_re) {
          nav3v2_B.atmp_re = nav3v2_B.atmp_im;
        }

        if (nav3v2_B.atmp_re >= 6.7178761075670888E-139) {
          nav3v2_B.atmp_re = 1.0 / nav3v2_B.atmp_re;
          V[nav3v2_B.c_i].re *= nav3v2_B.atmp_re;
          V[nav3v2_B.c_i].im *= nav3v2_B.atmp_re;
          V[nav3v2_B.k].re *= nav3v2_B.atmp_re;
          V[nav3v2_B.k].im *= nav3v2_B.atmp_re;
          V[nav3v2_B.i_i].re *= nav3v2_B.atmp_re;
          V[nav3v2_B.i_i].im *= nav3v2_B.atmp_re;
          V[nav3v2_B.jcol].re *= nav3v2_B.atmp_re;
          V[nav3v2_B.jcol].im *= nav3v2_B.atmp_re;
        }
      }

      if (nav3v2_B.ilascl) {
        nav3v2_B.ilascl = true;
        while (nav3v2_B.ilascl) {
          nav3v2_B.atmp_re = nav3v2_B.anrmto * 2.0041683600089728E-292;
          nav3v2_B.atmp_im = nav3v2_B.anrm / 4.9896007738368E+291;
          if ((fabs(nav3v2_B.atmp_re) > fabs(nav3v2_B.anrm)) && (nav3v2_B.anrm
               != 0.0)) {
            nav3v2_B.mul = 2.0041683600089728E-292;
            nav3v2_B.anrmto = nav3v2_B.atmp_re;
          } else if (fabs(nav3v2_B.atmp_im) > fabs(nav3v2_B.anrmto)) {
            nav3v2_B.mul = 4.9896007738368E+291;
            nav3v2_B.anrm = nav3v2_B.atmp_im;
          } else {
            nav3v2_B.mul = nav3v2_B.anrm / nav3v2_B.anrmto;
            nav3v2_B.ilascl = false;
          }

          alpha1[0].re *= nav3v2_B.mul;
          alpha1[0].im *= nav3v2_B.mul;
          alpha1[1].re *= nav3v2_B.mul;
          alpha1[1].im *= nav3v2_B.mul;
          alpha1[2].re *= nav3v2_B.mul;
          alpha1[2].im *= nav3v2_B.mul;
          alpha1[3].re *= nav3v2_B.mul;
          alpha1[3].im *= nav3v2_B.mul;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static real_T nav3v2_xnrm2(int32_T n, const real_T x[16], int32_T ix0)
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      for (k = ix0; k <= ix0 + 1; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau, real_T
  C[16], int32_T ic0, real_T work[4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0) {
    nav3v2_B.lastv = m;
    nav3v2_B.lastc_a = iv0 + m;
    while ((nav3v2_B.lastv > 0) && (C[nav3v2_B.lastc_a - 2] == 0.0)) {
      nav3v2_B.lastv--;
      nav3v2_B.lastc_a--;
    }

    nav3v2_B.lastc_a = n - 1;
    exitg2 = false;
    while ((!exitg2) && (nav3v2_B.lastc_a + 1 > 0)) {
      nav3v2_B.coltop = (nav3v2_B.lastc_a << 2) + ic0;
      nav3v2_B.jy_l = nav3v2_B.coltop;
      do {
        exitg1 = 0;
        if (nav3v2_B.jy_l <= (nav3v2_B.coltop + nav3v2_B.lastv) - 1) {
          if (C[nav3v2_B.jy_l - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3v2_B.jy_l++;
          }
        } else {
          nav3v2_B.lastc_a--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3v2_B.lastv = 0;
    nav3v2_B.lastc_a = -1;
  }

  if (nav3v2_B.lastv > 0) {
    if (nav3v2_B.lastc_a + 1 != 0) {
      nav3v2_B.coltop = 0;
      while (nav3v2_B.coltop <= nav3v2_B.lastc_a) {
        work[nav3v2_B.coltop] = 0.0;
        nav3v2_B.coltop++;
      }

      nav3v2_B.coltop = 0;
      nav3v2_B.jy_l = (nav3v2_B.lastc_a << 2) + ic0;
      nav3v2_B.iac_a = ic0;
      while (nav3v2_B.iac_a <= nav3v2_B.jy_l) {
        nav3v2_B.ix_e = iv0;
        nav3v2_B.c = 0.0;
        nav3v2_B.d_a = (nav3v2_B.iac_a + nav3v2_B.lastv) - 1;
        nav3v2_B.b_ia_i = nav3v2_B.iac_a;
        while (nav3v2_B.b_ia_i <= nav3v2_B.d_a) {
          nav3v2_B.c += C[nav3v2_B.b_ia_i - 1] * C[nav3v2_B.ix_e - 1];
          nav3v2_B.ix_e++;
          nav3v2_B.b_ia_i++;
        }

        work[nav3v2_B.coltop] += nav3v2_B.c;
        nav3v2_B.coltop++;
        nav3v2_B.iac_a += 4;
      }
    }

    if (!(-tau == 0.0)) {
      nav3v2_B.coltop = ic0 - 1;
      nav3v2_B.jy_l = 0;
      nav3v2_B.iac_a = 0;
      while (nav3v2_B.iac_a <= nav3v2_B.lastc_a) {
        if (work[nav3v2_B.jy_l] != 0.0) {
          nav3v2_B.c = work[nav3v2_B.jy_l] * -tau;
          nav3v2_B.ix_e = iv0;
          nav3v2_B.d_a = nav3v2_B.lastv + nav3v2_B.coltop;
          nav3v2_B.b_ia_i = nav3v2_B.coltop;
          while (nav3v2_B.b_ia_i + 1 <= nav3v2_B.d_a) {
            C[nav3v2_B.b_ia_i] += C[nav3v2_B.ix_e - 1] * nav3v2_B.c;
            nav3v2_B.ix_e++;
            nav3v2_B.b_ia_i++;
          }
        }

        nav3v2_B.jy_l++;
        nav3v2_B.coltop += 4;
        nav3v2_B.iac_a++;
      }
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xgehrd(real_T a[16], real_T tau[3])
{
  int32_T exitg1;
  boolean_T exitg2;
  nav3v2_B.work_b[0] = 0.0;
  nav3v2_B.work_b[1] = 0.0;
  nav3v2_B.work_b[2] = 0.0;
  nav3v2_B.work_b[3] = 0.0;
  nav3v2_B.alpha1 = a[1];
  tau[0] = 0.0;
  nav3v2_B.xnorm = nav3v2_xnrm2(2, a, 3);
  if (nav3v2_B.xnorm != 0.0) {
    nav3v2_B.xnorm = nav3v2_rt_hypotd_snf(a[1], nav3v2_B.xnorm);
    if (a[1] >= 0.0) {
      nav3v2_B.xnorm = -nav3v2_B.xnorm;
    }

    if (fabs(nav3v2_B.xnorm) < 1.0020841800044864E-292) {
      nav3v2_B.knt = -1;
      do {
        nav3v2_B.knt++;
        nav3v2_B.lastc = 3;
        while (nav3v2_B.lastc <= 4) {
          a[nav3v2_B.lastc - 1] *= 9.9792015476736E+291;
          nav3v2_B.lastc++;
        }

        nav3v2_B.xnorm *= 9.9792015476736E+291;
        nav3v2_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(nav3v2_B.xnorm) >= 1.0020841800044864E-292));

      nav3v2_B.xnorm = nav3v2_rt_hypotd_snf(nav3v2_B.alpha1, nav3v2_xnrm2(2, a,
        3));
      if (nav3v2_B.alpha1 >= 0.0) {
        nav3v2_B.xnorm = -nav3v2_B.xnorm;
      }

      tau[0] = (nav3v2_B.xnorm - nav3v2_B.alpha1) / nav3v2_B.xnorm;
      nav3v2_B.alpha1 = 1.0 / (nav3v2_B.alpha1 - nav3v2_B.xnorm);
      nav3v2_B.lastc = 3;
      while (nav3v2_B.lastc <= 4) {
        a[nav3v2_B.lastc - 1] *= nav3v2_B.alpha1;
        nav3v2_B.lastc++;
      }

      nav3v2_B.lastc = 0;
      while (nav3v2_B.lastc <= nav3v2_B.knt) {
        nav3v2_B.xnorm *= 1.0020841800044864E-292;
        nav3v2_B.lastc++;
      }

      nav3v2_B.alpha1 = nav3v2_B.xnorm;
    } else {
      tau[0] = (nav3v2_B.xnorm - a[1]) / nav3v2_B.xnorm;
      nav3v2_B.alpha1 = 1.0 / (a[1] - nav3v2_B.xnorm);
      nav3v2_B.knt = 3;
      while (nav3v2_B.knt <= 4) {
        a[nav3v2_B.knt - 1] *= nav3v2_B.alpha1;
        nav3v2_B.knt++;
      }

      nav3v2_B.alpha1 = nav3v2_B.xnorm;
    }
  }

  a[1] = 1.0;
  if (tau[0] != 0.0) {
    nav3v2_B.knt = 2;
    nav3v2_B.lastc = 3;
    while ((nav3v2_B.knt + 1 > 0) && (a[nav3v2_B.lastc] == 0.0)) {
      nav3v2_B.knt--;
      nav3v2_B.lastc--;
    }

    nav3v2_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (nav3v2_B.lastc > 0)) {
      nav3v2_B.rowleft = nav3v2_B.lastc + 4;
      nav3v2_B.jy = nav3v2_B.rowleft;
      do {
        exitg1 = 0;
        if (nav3v2_B.jy <= (nav3v2_B.knt << 2) + nav3v2_B.rowleft) {
          if (a[nav3v2_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3v2_B.jy += 4;
          }
        } else {
          nav3v2_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3v2_B.knt = -1;
    nav3v2_B.lastc = 0;
  }

  if (nav3v2_B.knt + 1 > 0) {
    if (nav3v2_B.lastc != 0) {
      nav3v2_B.rowleft = 0;
      while (nav3v2_B.rowleft <= nav3v2_B.lastc - 1) {
        nav3v2_B.work_b[nav3v2_B.rowleft] = 0.0;
        nav3v2_B.rowleft++;
      }

      nav3v2_B.rowleft = 1;
      nav3v2_B.jy = (nav3v2_B.knt << 2) + 5;
      nav3v2_B.iac = 5;
      while (nav3v2_B.iac <= nav3v2_B.jy) {
        nav3v2_B.b_ix = 0;
        nav3v2_B.g = (nav3v2_B.iac + nav3v2_B.lastc) - 1;
        nav3v2_B.b_ia = nav3v2_B.iac;
        while (nav3v2_B.b_ia <= nav3v2_B.g) {
          nav3v2_B.work_b[nav3v2_B.b_ix] += a[nav3v2_B.b_ia - 1] *
            a[nav3v2_B.rowleft];
          nav3v2_B.b_ix++;
          nav3v2_B.b_ia++;
        }

        nav3v2_B.rowleft++;
        nav3v2_B.iac += 4;
      }
    }

    if (!(-tau[0] == 0.0)) {
      nav3v2_B.rowleft = 4;
      nav3v2_B.jy = 1;
      nav3v2_B.iac = 0;
      while (nav3v2_B.iac <= nav3v2_B.knt) {
        if (a[nav3v2_B.jy] != 0.0) {
          nav3v2_B.xnorm = a[nav3v2_B.jy] * -tau[0];
          nav3v2_B.b_ix = 0;
          nav3v2_B.g = nav3v2_B.lastc + nav3v2_B.rowleft;
          nav3v2_B.b_ia = nav3v2_B.rowleft;
          while (nav3v2_B.b_ia + 1 <= nav3v2_B.g) {
            a[nav3v2_B.b_ia] += nav3v2_B.work_b[nav3v2_B.b_ix] * nav3v2_B.xnorm;
            nav3v2_B.b_ix++;
            nav3v2_B.b_ia++;
          }
        }

        nav3v2_B.jy++;
        nav3v2_B.rowleft += 4;
        nav3v2_B.iac++;
      }
    }
  }

  nav3v2_xzlarf(3, 3, 2, tau[0], a, 6, nav3v2_B.work_b);
  a[1] = nav3v2_B.alpha1;
  nav3v2_B.alpha1 = a[6];
  tau[1] = 0.0;
  nav3v2_B.xnorm = nav3v2_xnrm2(1, a, 8);
  if (nav3v2_B.xnorm != 0.0) {
    nav3v2_B.xnorm = nav3v2_rt_hypotd_snf(a[6], nav3v2_B.xnorm);
    if (a[6] >= 0.0) {
      nav3v2_B.xnorm = -nav3v2_B.xnorm;
    }

    if (fabs(nav3v2_B.xnorm) < 1.0020841800044864E-292) {
      nav3v2_B.knt = -1;
      do {
        nav3v2_B.knt++;
        a[7] *= 9.9792015476736E+291;
        nav3v2_B.xnorm *= 9.9792015476736E+291;
        nav3v2_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(nav3v2_B.xnorm) >= 1.0020841800044864E-292));

      nav3v2_B.xnorm = nav3v2_rt_hypotd_snf(nav3v2_B.alpha1, nav3v2_xnrm2(1, a,
        8));
      if (nav3v2_B.alpha1 >= 0.0) {
        nav3v2_B.xnorm = -nav3v2_B.xnorm;
      }

      tau[1] = (nav3v2_B.xnorm - nav3v2_B.alpha1) / nav3v2_B.xnorm;
      nav3v2_B.alpha1 = 1.0 / (nav3v2_B.alpha1 - nav3v2_B.xnorm);
      a[7] *= nav3v2_B.alpha1;
      nav3v2_B.lastc = 0;
      while (nav3v2_B.lastc <= nav3v2_B.knt) {
        nav3v2_B.xnorm *= 1.0020841800044864E-292;
        nav3v2_B.lastc++;
      }

      nav3v2_B.alpha1 = nav3v2_B.xnorm;
    } else {
      tau[1] = (nav3v2_B.xnorm - a[6]) / nav3v2_B.xnorm;
      a[7] *= 1.0 / (a[6] - nav3v2_B.xnorm);
      nav3v2_B.alpha1 = nav3v2_B.xnorm;
    }
  }

  a[6] = 1.0;
  if (tau[1] != 0.0) {
    nav3v2_B.knt = 1;
    nav3v2_B.lastc = 7;
    while ((nav3v2_B.knt + 1 > 0) && (a[nav3v2_B.lastc] == 0.0)) {
      nav3v2_B.knt--;
      nav3v2_B.lastc--;
    }

    nav3v2_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (nav3v2_B.lastc > 0)) {
      nav3v2_B.rowleft = nav3v2_B.lastc + 8;
      nav3v2_B.jy = nav3v2_B.rowleft;
      do {
        exitg1 = 0;
        if (nav3v2_B.jy <= (nav3v2_B.knt << 2) + nav3v2_B.rowleft) {
          if (a[nav3v2_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3v2_B.jy += 4;
          }
        } else {
          nav3v2_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3v2_B.knt = -1;
    nav3v2_B.lastc = 0;
  }

  if (nav3v2_B.knt + 1 > 0) {
    if (nav3v2_B.lastc != 0) {
      nav3v2_B.rowleft = 0;
      while (nav3v2_B.rowleft <= nav3v2_B.lastc - 1) {
        nav3v2_B.work_b[nav3v2_B.rowleft] = 0.0;
        nav3v2_B.rowleft++;
      }

      nav3v2_B.rowleft = 6;
      nav3v2_B.jy = (nav3v2_B.knt << 2) + 9;
      nav3v2_B.iac = 9;
      while (nav3v2_B.iac <= nav3v2_B.jy) {
        nav3v2_B.b_ix = 0;
        nav3v2_B.g = (nav3v2_B.iac + nav3v2_B.lastc) - 1;
        nav3v2_B.b_ia = nav3v2_B.iac;
        while (nav3v2_B.b_ia <= nav3v2_B.g) {
          nav3v2_B.work_b[nav3v2_B.b_ix] += a[nav3v2_B.b_ia - 1] *
            a[nav3v2_B.rowleft];
          nav3v2_B.b_ix++;
          nav3v2_B.b_ia++;
        }

        nav3v2_B.rowleft++;
        nav3v2_B.iac += 4;
      }
    }

    if (!(-tau[1] == 0.0)) {
      nav3v2_B.rowleft = 8;
      nav3v2_B.jy = 6;
      nav3v2_B.iac = 0;
      while (nav3v2_B.iac <= nav3v2_B.knt) {
        if (a[nav3v2_B.jy] != 0.0) {
          nav3v2_B.xnorm = a[nav3v2_B.jy] * -tau[1];
          nav3v2_B.b_ix = 0;
          nav3v2_B.g = nav3v2_B.lastc + nav3v2_B.rowleft;
          nav3v2_B.b_ia = nav3v2_B.rowleft;
          while (nav3v2_B.b_ia + 1 <= nav3v2_B.g) {
            a[nav3v2_B.b_ia] += nav3v2_B.work_b[nav3v2_B.b_ix] * nav3v2_B.xnorm;
            nav3v2_B.b_ix++;
            nav3v2_B.b_ia++;
          }
        }

        nav3v2_B.jy++;
        nav3v2_B.rowleft += 4;
        nav3v2_B.iac++;
      }
    }
  }

  nav3v2_xzlarf(2, 2, 7, tau[1], a, 11, nav3v2_B.work_b);
  a[6] = nav3v2_B.alpha1;
  nav3v2_B.alpha1 = a[11];
  tau[2] = 0.0;
  nav3v2_B.xnorm = nav3v2_xnrm2(0, a, 12);
  if (nav3v2_B.xnorm != 0.0) {
    nav3v2_B.xnorm = nav3v2_rt_hypotd_snf(a[11], nav3v2_B.xnorm);
    if (a[11] >= 0.0) {
      nav3v2_B.xnorm = -nav3v2_B.xnorm;
    }

    if (fabs(nav3v2_B.xnorm) < 1.0020841800044864E-292) {
      nav3v2_B.knt = -1;
      do {
        nav3v2_B.knt++;
        nav3v2_B.xnorm *= 9.9792015476736E+291;
        nav3v2_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(nav3v2_B.xnorm) >= 1.0020841800044864E-292));

      nav3v2_B.xnorm = nav3v2_rt_hypotd_snf(nav3v2_B.alpha1, nav3v2_xnrm2(0, a,
        12));
      if (nav3v2_B.alpha1 >= 0.0) {
        nav3v2_B.xnorm = -nav3v2_B.xnorm;
      }

      tau[2] = (nav3v2_B.xnorm - nav3v2_B.alpha1) / nav3v2_B.xnorm;
      nav3v2_B.lastc = 0;
      while (nav3v2_B.lastc <= nav3v2_B.knt) {
        nav3v2_B.xnorm *= 1.0020841800044864E-292;
        nav3v2_B.lastc++;
      }

      nav3v2_B.alpha1 = nav3v2_B.xnorm;
    } else {
      tau[2] = (nav3v2_B.xnorm - a[11]) / nav3v2_B.xnorm;
      nav3v2_B.alpha1 = nav3v2_B.xnorm;
    }
  }

  a[11] = 1.0;
  if (tau[2] != 0.0) {
    nav3v2_B.knt = 0;
    nav3v2_B.lastc = 11;
    while ((nav3v2_B.knt + 1 > 0) && (a[nav3v2_B.lastc] == 0.0)) {
      nav3v2_B.knt--;
      nav3v2_B.lastc--;
    }

    nav3v2_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (nav3v2_B.lastc > 0)) {
      nav3v2_B.rowleft = nav3v2_B.lastc + 12;
      nav3v2_B.jy = nav3v2_B.rowleft;
      do {
        exitg1 = 0;
        if (nav3v2_B.jy <= (nav3v2_B.knt << 2) + nav3v2_B.rowleft) {
          if (a[nav3v2_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3v2_B.jy += 4;
          }
        } else {
          nav3v2_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3v2_B.knt = -1;
    nav3v2_B.lastc = 0;
  }

  if (nav3v2_B.knt + 1 > 0) {
    if (nav3v2_B.lastc != 0) {
      nav3v2_B.rowleft = 0;
      while (nav3v2_B.rowleft <= nav3v2_B.lastc - 1) {
        nav3v2_B.work_b[nav3v2_B.rowleft] = 0.0;
        nav3v2_B.rowleft++;
      }

      nav3v2_B.rowleft = 11;
      nav3v2_B.jy = (nav3v2_B.knt << 2) + 13;
      nav3v2_B.iac = 13;
      while (nav3v2_B.iac <= nav3v2_B.jy) {
        nav3v2_B.b_ix = 0;
        nav3v2_B.g = (nav3v2_B.iac + nav3v2_B.lastc) - 1;
        nav3v2_B.b_ia = nav3v2_B.iac;
        while (nav3v2_B.b_ia <= nav3v2_B.g) {
          nav3v2_B.work_b[nav3v2_B.b_ix] += a[nav3v2_B.b_ia - 1] *
            a[nav3v2_B.rowleft];
          nav3v2_B.b_ix++;
          nav3v2_B.b_ia++;
        }

        nav3v2_B.rowleft++;
        nav3v2_B.iac += 4;
      }
    }

    if (!(-tau[2] == 0.0)) {
      nav3v2_B.rowleft = 12;
      nav3v2_B.jy = 11;
      nav3v2_B.iac = 0;
      while (nav3v2_B.iac <= nav3v2_B.knt) {
        if (a[nav3v2_B.jy] != 0.0) {
          nav3v2_B.xnorm = a[nav3v2_B.jy] * -tau[2];
          nav3v2_B.b_ix = 0;
          nav3v2_B.g = nav3v2_B.lastc + nav3v2_B.rowleft;
          nav3v2_B.b_ia = nav3v2_B.rowleft;
          while (nav3v2_B.b_ia + 1 <= nav3v2_B.g) {
            a[nav3v2_B.b_ia] += nav3v2_B.work_b[nav3v2_B.b_ix] * nav3v2_B.xnorm;
            nav3v2_B.b_ix++;
            nav3v2_B.b_ia++;
          }
        }

        nav3v2_B.jy++;
        nav3v2_B.rowleft += 4;
        nav3v2_B.iac++;
      }
    }
  }

  nav3v2_xzlarf(1, 1, 12, tau[2], a, 16, nav3v2_B.work_b);
  a[11] = nav3v2_B.alpha1;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static real_T nav3v2_xnrm2_j(int32_T n, const real_T x[3])
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[1]);
    } else {
      nav3v2_B.scale_jz = 3.3121686421112381E-170;
      nav3v2_B.absxk_f = fabs(x[1]);
      if (nav3v2_B.absxk_f > 3.3121686421112381E-170) {
        y = 1.0;
        nav3v2_B.scale_jz = nav3v2_B.absxk_f;
      } else {
        nav3v2_B.t_a = nav3v2_B.absxk_f / 3.3121686421112381E-170;
        y = nav3v2_B.t_a * nav3v2_B.t_a;
      }

      nav3v2_B.absxk_f = fabs(x[2]);
      if (nav3v2_B.absxk_f > nav3v2_B.scale_jz) {
        nav3v2_B.t_a = nav3v2_B.scale_jz / nav3v2_B.absxk_f;
        y = y * nav3v2_B.t_a * nav3v2_B.t_a + 1.0;
        nav3v2_B.scale_jz = nav3v2_B.absxk_f;
      } else {
        nav3v2_B.t_a = nav3v2_B.absxk_f / nav3v2_B.scale_jz;
        y += nav3v2_B.t_a * nav3v2_B.t_a;
      }

      y = nav3v2_B.scale_jz * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static real_T nav3v2_xzlarfg(int32_T n, real_T *alpha1, real_T x[3])
{
  real_T tau;
  tau = 0.0;
  if (n > 0) {
    nav3v2_B.xnorm_b = nav3v2_xnrm2_j(n - 1, x);
    if (nav3v2_B.xnorm_b != 0.0) {
      nav3v2_B.xnorm_b = nav3v2_rt_hypotd_snf(*alpha1, nav3v2_B.xnorm_b);
      if (*alpha1 >= 0.0) {
        nav3v2_B.xnorm_b = -nav3v2_B.xnorm_b;
      }

      if (fabs(nav3v2_B.xnorm_b) < 1.0020841800044864E-292) {
        nav3v2_B.knt_f = -1;
        do {
          nav3v2_B.knt_f++;
          nav3v2_B.c_k = 1;
          while (nav3v2_B.c_k + 1 <= n) {
            x[nav3v2_B.c_k] *= 9.9792015476736E+291;
            nav3v2_B.c_k++;
          }

          nav3v2_B.xnorm_b *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(nav3v2_B.xnorm_b) >= 1.0020841800044864E-292));

        nav3v2_B.xnorm_b = nav3v2_rt_hypotd_snf(*alpha1, nav3v2_xnrm2_j(n - 1, x));
        if (*alpha1 >= 0.0) {
          nav3v2_B.xnorm_b = -nav3v2_B.xnorm_b;
        }

        tau = (nav3v2_B.xnorm_b - *alpha1) / nav3v2_B.xnorm_b;
        *alpha1 = 1.0 / (*alpha1 - nav3v2_B.xnorm_b);
        nav3v2_B.c_k = 1;
        while (nav3v2_B.c_k + 1 <= n) {
          x[nav3v2_B.c_k] *= *alpha1;
          nav3v2_B.c_k++;
        }

        nav3v2_B.c_k = 0;
        while (nav3v2_B.c_k <= nav3v2_B.knt_f) {
          nav3v2_B.xnorm_b *= 1.0020841800044864E-292;
          nav3v2_B.c_k++;
        }

        *alpha1 = nav3v2_B.xnorm_b;
      } else {
        tau = (nav3v2_B.xnorm_b - *alpha1) / nav3v2_B.xnorm_b;
        *alpha1 = 1.0 / (*alpha1 - nav3v2_B.xnorm_b);
        nav3v2_B.knt_f = 1;
        while (nav3v2_B.knt_f + 1 <= n) {
          x[nav3v2_B.knt_f] *= *alpha1;
          nav3v2_B.knt_f++;
        }

        *alpha1 = nav3v2_B.xnorm_b;
      }
    }
  }

  return tau;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *d, real_T
  *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *sn)
{
  if (*c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (*b == 0.0) {
    *cs = 0.0;
    *sn = 1.0;
    nav3v2_B.bcmax = *d;
    *d = *a;
    *a = nav3v2_B.bcmax;
    *b = -*c;
    *c = 0.0;
  } else {
    nav3v2_B.tau_d = *a - *d;
    if ((nav3v2_B.tau_d == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      nav3v2_B.p = 0.5 * nav3v2_B.tau_d;
      nav3v2_B.bcmis = fabs(*b);
      nav3v2_B.z = fabs(*c);
      nav3v2_B.b_co = rtIsNaN(nav3v2_B.z);
      if ((nav3v2_B.bcmis > nav3v2_B.z) || nav3v2_B.b_co) {
        nav3v2_B.bcmax = nav3v2_B.bcmis;
      } else {
        nav3v2_B.bcmax = nav3v2_B.z;
      }

      if ((nav3v2_B.bcmis < nav3v2_B.z) || nav3v2_B.b_co) {
        nav3v2_B.z = nav3v2_B.bcmis;
      }

      if (!(*b < 0.0)) {
        nav3v2_B.b_c = 1;
      } else {
        nav3v2_B.b_c = -1;
      }

      if (!(*c < 0.0)) {
        nav3v2_B.c_c = 1;
      } else {
        nav3v2_B.c_c = -1;
      }

      nav3v2_B.bcmis = nav3v2_B.z * static_cast<real_T>(nav3v2_B.b_c) *
        static_cast<real_T>(nav3v2_B.c_c);
      nav3v2_B.scale_j = fabs(nav3v2_B.p);
      if ((!(nav3v2_B.scale_j > nav3v2_B.bcmax)) && (!rtIsNaN(nav3v2_B.bcmax)))
      {
        nav3v2_B.scale_j = nav3v2_B.bcmax;
      }

      nav3v2_B.z = nav3v2_B.p / nav3v2_B.scale_j * nav3v2_B.p + nav3v2_B.bcmax /
        nav3v2_B.scale_j * nav3v2_B.bcmis;
      if (nav3v2_B.z >= 8.8817841970012523E-16) {
        if (!(nav3v2_B.p < 0.0)) {
          nav3v2_B.tau_d = sqrt(nav3v2_B.scale_j) * sqrt(nav3v2_B.z);
        } else {
          nav3v2_B.tau_d = -(sqrt(nav3v2_B.scale_j) * sqrt(nav3v2_B.z));
        }

        nav3v2_B.z = nav3v2_B.p + nav3v2_B.tau_d;
        *a = *d + nav3v2_B.z;
        *d -= nav3v2_B.bcmax / nav3v2_B.z * nav3v2_B.bcmis;
        nav3v2_B.tau_d = nav3v2_rt_hypotd_snf(*c, nav3v2_B.z);
        *cs = nav3v2_B.z / nav3v2_B.tau_d;
        *sn = *c / nav3v2_B.tau_d;
        *b -= *c;
        *c = 0.0;
      } else {
        nav3v2_B.bcmax = *b + *c;
        nav3v2_B.tau_d = nav3v2_rt_hypotd_snf(nav3v2_B.bcmax, nav3v2_B.tau_d);
        *cs = sqrt((fabs(nav3v2_B.bcmax) / nav3v2_B.tau_d + 1.0) * 0.5);
        if (!(nav3v2_B.bcmax < 0.0)) {
          nav3v2_B.b_c = 1;
        } else {
          nav3v2_B.b_c = -1;
        }

        *sn = -(nav3v2_B.p / (nav3v2_B.tau_d * *cs)) * static_cast<real_T>
          (nav3v2_B.b_c);
        nav3v2_B.p = *a * *cs + *b * *sn;
        nav3v2_B.tau_d = -*a * *sn + *b * *cs;
        nav3v2_B.bcmax = *c * *cs + *d * *sn;
        nav3v2_B.bcmis = -*c * *sn + *d * *cs;
        *b = nav3v2_B.tau_d * *cs + nav3v2_B.bcmis * *sn;
        *c = -nav3v2_B.p * *sn + nav3v2_B.bcmax * *cs;
        nav3v2_B.bcmax = ((nav3v2_B.p * *cs + nav3v2_B.bcmax * *sn) +
                          (-nav3v2_B.tau_d * *sn + nav3v2_B.bcmis * *cs)) * 0.5;
        *a = nav3v2_B.bcmax;
        *d = nav3v2_B.bcmax;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              nav3v2_B.z = sqrt(fabs(*b));
              nav3v2_B.bcmis = sqrt(fabs(*c));
              if (!(*c < 0.0)) {
                nav3v2_B.p = nav3v2_B.z * nav3v2_B.bcmis;
              } else {
                nav3v2_B.p = -(nav3v2_B.z * nav3v2_B.bcmis);
              }

              nav3v2_B.tau_d = 1.0 / sqrt(fabs(*b + *c));
              *a = nav3v2_B.bcmax + nav3v2_B.p;
              *d = nav3v2_B.bcmax - nav3v2_B.p;
              *b -= *c;
              *c = 0.0;
              nav3v2_B.p = nav3v2_B.z * nav3v2_B.tau_d;
              nav3v2_B.tau_d *= nav3v2_B.bcmis;
              nav3v2_B.bcmax = *cs * nav3v2_B.p - *sn * nav3v2_B.tau_d;
              *sn = *cs * nav3v2_B.tau_d + *sn * nav3v2_B.p;
              *cs = nav3v2_B.bcmax;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            nav3v2_B.bcmax = *cs;
            *cs = -*sn;
            *sn = nav3v2_B.bcmax;
          }
        }
      }
    }
  }

  *rt1r = *a;
  *rt2r = *d;
  if (*c == 0.0) {
    *rt1i = 0.0;
    *rt2i = 0.0;
  } else {
    *rt1i = sqrt(fabs(*b)) * sqrt(fabs(*c));
    *rt2i = -*rt1i;
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xrot(int32_T n, real_T x[16], int32_T ix0, int32_T iy0,
  real_T c, real_T s)
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  int32_T k;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < n; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy += 4;
    ix += 4;
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_xrot_o(int32_T n, real_T x[16], int32_T ix0, int32_T iy0,
  real_T c, real_T s)
{
  int32_T ix;
  int32_T iy;
  real_T temp;
  int32_T k;
  if (n >= 1) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      temp = c * x[ix] + s * x[iy];
      x[iy] = c * x[iy] - s * x[ix];
      x[ix] = temp;
      iy++;
      ix++;
    }
  }
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static int32_T nav3v2_eml_dlahqr(real_T h[16], real_T z[16])
{
  int32_T info;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  nav3v2_B.v[0] = 0.0;
  nav3v2_B.v[1] = 0.0;
  nav3v2_B.v[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  nav3v2_B.i_m = 3;
  exitg1 = false;
  while ((!exitg1) && (nav3v2_B.i_m + 1 >= 1)) {
    nav3v2_B.L = 1;
    nav3v2_B.goto150 = false;
    nav3v2_B.ix = 0;
    exitg2 = false;
    while ((!exitg2) && (nav3v2_B.ix < 301)) {
      nav3v2_B.k_m = nav3v2_B.i_m;
      exitg3 = false;
      while ((!exitg3) && (nav3v2_B.k_m + 1 > nav3v2_B.L)) {
        nav3v2_B.s_tmp = ((nav3v2_B.k_m - 1) << 2) + nav3v2_B.k_m;
        nav3v2_B.htmp2 = fabs(h[nav3v2_B.s_tmp]);
        if (nav3v2_B.htmp2 <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          nav3v2_B.m = (nav3v2_B.k_m << 2) + nav3v2_B.k_m;
          nav3v2_B.nr = nav3v2_B.s_tmp - 1;
          nav3v2_B.tst = fabs(h[nav3v2_B.nr]) + fabs(h[nav3v2_B.m]);
          if (nav3v2_B.tst == 0.0) {
            if (nav3v2_B.k_m - 1 >= 1) {
              nav3v2_B.tst = fabs(h[(((nav3v2_B.k_m - 2) << 2) + nav3v2_B.k_m) -
                                  1]);
            }

            if (nav3v2_B.k_m + 2 <= 4) {
              nav3v2_B.tst += fabs(h[((nav3v2_B.k_m << 2) + nav3v2_B.k_m) + 1]);
            }
          }

          if (nav3v2_B.htmp2 <= 2.2204460492503131E-16 * nav3v2_B.tst) {
            nav3v2_B.htmp1 = fabs(h[nav3v2_B.s_tmp]);
            nav3v2_B.htmp2 = fabs(h[nav3v2_B.m - 1]);
            if (nav3v2_B.htmp1 > nav3v2_B.htmp2) {
              nav3v2_B.tst = nav3v2_B.htmp1;
              nav3v2_B.ba = nav3v2_B.htmp2;
            } else {
              nav3v2_B.tst = nav3v2_B.htmp2;
              nav3v2_B.ba = nav3v2_B.htmp1;
            }

            nav3v2_B.htmp1 = fabs(h[nav3v2_B.m]);
            nav3v2_B.htmp2 = fabs(h[nav3v2_B.nr] - h[nav3v2_B.m]);
            if (nav3v2_B.htmp1 > nav3v2_B.htmp2) {
              nav3v2_B.aa = nav3v2_B.htmp1;
              nav3v2_B.htmp1 = nav3v2_B.htmp2;
            } else {
              nav3v2_B.aa = nav3v2_B.htmp2;
            }

            nav3v2_B.htmp2 = nav3v2_B.aa + nav3v2_B.tst;
            nav3v2_B.htmp1 = nav3v2_B.aa / nav3v2_B.htmp2 * nav3v2_B.htmp1 *
              2.2204460492503131E-16;
            if ((4.0083367200179456E-292 > nav3v2_B.htmp1) || rtIsNaN
                (nav3v2_B.htmp1)) {
              nav3v2_B.htmp1 = 4.0083367200179456E-292;
            }

            if (nav3v2_B.tst / nav3v2_B.htmp2 * nav3v2_B.ba <= nav3v2_B.htmp1) {
              exitg3 = true;
            } else {
              nav3v2_B.k_m--;
            }
          } else {
            nav3v2_B.k_m--;
          }
        }
      }

      nav3v2_B.L = nav3v2_B.k_m + 1;
      if (nav3v2_B.k_m + 1 > 1) {
        h[nav3v2_B.k_m + ((nav3v2_B.k_m - 1) << 2)] = 0.0;
      }

      if (nav3v2_B.k_m + 1 >= nav3v2_B.i_m) {
        nav3v2_B.goto150 = true;
        exitg2 = true;
      } else {
        if (nav3v2_B.ix == 10) {
          nav3v2_B.s_tmp = (nav3v2_B.k_m << 2) + nav3v2_B.k_m;
          nav3v2_B.htmp2 = fabs(h[(((nav3v2_B.k_m + 1) << 2) + nav3v2_B.k_m) + 2])
            + fabs(h[nav3v2_B.s_tmp + 1]);
          nav3v2_B.ba = h[nav3v2_B.s_tmp] + 0.75 * nav3v2_B.htmp2;
          nav3v2_B.h12 = -0.4375 * nav3v2_B.htmp2;
          nav3v2_B.aa = nav3v2_B.htmp2;
          nav3v2_B.tst = nav3v2_B.ba;
        } else if (nav3v2_B.ix == 20) {
          nav3v2_B.htmp2 = fabs(h[(((nav3v2_B.i_m - 2) << 2) + nav3v2_B.i_m) - 1])
            + fabs(h[((nav3v2_B.i_m - 1) << 2) + nav3v2_B.i_m]);
          nav3v2_B.ba = h[(nav3v2_B.i_m << 2) + nav3v2_B.i_m] + 0.75 *
            nav3v2_B.htmp2;
          nav3v2_B.h12 = -0.4375 * nav3v2_B.htmp2;
          nav3v2_B.aa = nav3v2_B.htmp2;
          nav3v2_B.tst = nav3v2_B.ba;
        } else {
          nav3v2_B.ba = h[(((nav3v2_B.i_m - 1) << 2) + nav3v2_B.i_m) - 1];
          nav3v2_B.aa = h[((nav3v2_B.i_m - 1) << 2) + nav3v2_B.i_m];
          nav3v2_B.h12 = h[((nav3v2_B.i_m << 2) + nav3v2_B.i_m) - 1];
          nav3v2_B.tst = h[(nav3v2_B.i_m << 2) + nav3v2_B.i_m];
        }

        nav3v2_B.htmp2 = ((fabs(nav3v2_B.ba) + fabs(nav3v2_B.h12)) + fabs
                          (nav3v2_B.aa)) + fabs(nav3v2_B.tst);
        if (nav3v2_B.htmp2 == 0.0) {
          nav3v2_B.ba = 0.0;
          nav3v2_B.tst = 0.0;
          nav3v2_B.htmp1 = 0.0;
          nav3v2_B.aa = 0.0;
        } else {
          nav3v2_B.ba /= nav3v2_B.htmp2;
          nav3v2_B.aa /= nav3v2_B.htmp2;
          nav3v2_B.h12 /= nav3v2_B.htmp2;
          nav3v2_B.tst /= nav3v2_B.htmp2;
          nav3v2_B.htmp1 = (nav3v2_B.ba + nav3v2_B.tst) / 2.0;
          nav3v2_B.ba = (nav3v2_B.ba - nav3v2_B.htmp1) * (nav3v2_B.tst -
            nav3v2_B.htmp1) - nav3v2_B.h12 * nav3v2_B.aa;
          nav3v2_B.aa = sqrt(fabs(nav3v2_B.ba));
          if (nav3v2_B.ba >= 0.0) {
            nav3v2_B.ba = nav3v2_B.htmp1 * nav3v2_B.htmp2;
            nav3v2_B.htmp1 = nav3v2_B.ba;
            nav3v2_B.tst = nav3v2_B.aa * nav3v2_B.htmp2;
            nav3v2_B.aa = -nav3v2_B.tst;
          } else {
            nav3v2_B.ba = nav3v2_B.htmp1 + nav3v2_B.aa;
            nav3v2_B.htmp1 -= nav3v2_B.aa;
            if (fabs(nav3v2_B.ba - nav3v2_B.tst) <= fabs(nav3v2_B.htmp1 -
                 nav3v2_B.tst)) {
              nav3v2_B.ba *= nav3v2_B.htmp2;
              nav3v2_B.htmp1 = nav3v2_B.ba;
            } else {
              nav3v2_B.htmp1 *= nav3v2_B.htmp2;
              nav3v2_B.ba = nav3v2_B.htmp1;
            }

            nav3v2_B.tst = 0.0;
            nav3v2_B.aa = 0.0;
          }
        }

        nav3v2_B.m = nav3v2_B.i_m - 1;
        exitg3 = false;
        while ((!exitg3) && (nav3v2_B.m >= nav3v2_B.k_m + 1)) {
          nav3v2_B.s_tmp = ((nav3v2_B.m - 1) << 2) + nav3v2_B.m;
          nav3v2_B.nr = nav3v2_B.s_tmp - 1;
          nav3v2_B.h12 = h[nav3v2_B.nr] - nav3v2_B.htmp1;
          nav3v2_B.htmp2 = (fabs(nav3v2_B.h12) + fabs(nav3v2_B.aa)) + fabs
            (h[nav3v2_B.s_tmp]);
          nav3v2_B.h21s = h[nav3v2_B.s_tmp] / nav3v2_B.htmp2;
          nav3v2_B.hoffset = (nav3v2_B.m << 2) + nav3v2_B.m;
          nav3v2_B.v[0] = (nav3v2_B.h12 / nav3v2_B.htmp2 * (h[nav3v2_B.nr] -
            nav3v2_B.ba) + h[nav3v2_B.hoffset - 1] * nav3v2_B.h21s) -
            nav3v2_B.aa / nav3v2_B.htmp2 * nav3v2_B.tst;
          nav3v2_B.v[1] = (((h[nav3v2_B.nr] + h[nav3v2_B.hoffset]) - nav3v2_B.ba)
                           - nav3v2_B.htmp1) * nav3v2_B.h21s;
          nav3v2_B.v[2] = h[nav3v2_B.hoffset + 1] * nav3v2_B.h21s;
          nav3v2_B.htmp2 = (fabs(nav3v2_B.v[0]) + fabs(nav3v2_B.v[1])) + fabs
            (nav3v2_B.v[2]);
          nav3v2_B.v[0] /= nav3v2_B.htmp2;
          nav3v2_B.v[1] /= nav3v2_B.htmp2;
          nav3v2_B.v[2] /= nav3v2_B.htmp2;
          if (nav3v2_B.k_m + 1 == nav3v2_B.m) {
            exitg3 = true;
          } else {
            nav3v2_B.s_tmp = ((nav3v2_B.m - 2) << 2) + nav3v2_B.m;
            if (fabs(h[nav3v2_B.s_tmp - 1]) * (fabs(nav3v2_B.v[1]) + fabs
                 (nav3v2_B.v[2])) <= ((fabs(h[nav3v2_B.s_tmp - 2]) + fabs
                  (h[nav3v2_B.nr])) + fabs(h[nav3v2_B.hoffset])) *
                (2.2204460492503131E-16 * fabs(nav3v2_B.v[0]))) {
              exitg3 = true;
            } else {
              nav3v2_B.m--;
            }
          }
        }

        nav3v2_B.s_tmp = nav3v2_B.m;
        while (nav3v2_B.s_tmp <= nav3v2_B.i_m) {
          nav3v2_B.nr = (nav3v2_B.i_m - nav3v2_B.s_tmp) + 2;
          if (3 < nav3v2_B.nr) {
            nav3v2_B.nr = 3;
          }

          if (nav3v2_B.s_tmp > nav3v2_B.m) {
            nav3v2_B.hoffset = ((nav3v2_B.s_tmp - 2) << 2) + nav3v2_B.s_tmp;
            nav3v2_B.j_j = 0;
            while (nav3v2_B.j_j <= nav3v2_B.nr - 1) {
              nav3v2_B.v[nav3v2_B.j_j] = h[(nav3v2_B.j_j + nav3v2_B.hoffset) - 1];
              nav3v2_B.j_j++;
            }
          }

          nav3v2_B.tst = nav3v2_B.v[0];
          nav3v2_B.b_v[0] = nav3v2_B.v[0];
          nav3v2_B.b_v[1] = nav3v2_B.v[1];
          nav3v2_B.b_v[2] = nav3v2_B.v[2];
          nav3v2_B.htmp2 = nav3v2_xzlarfg(nav3v2_B.nr, &nav3v2_B.tst,
            nav3v2_B.b_v);
          nav3v2_B.v[1] = nav3v2_B.b_v[1];
          nav3v2_B.v[2] = nav3v2_B.b_v[2];
          nav3v2_B.v[0] = nav3v2_B.tst;
          if (nav3v2_B.s_tmp > nav3v2_B.m) {
            h[(nav3v2_B.s_tmp + ((nav3v2_B.s_tmp - 2) << 2)) - 1] = nav3v2_B.tst;
            h[nav3v2_B.s_tmp + ((nav3v2_B.s_tmp - 2) << 2)] = 0.0;
            if (nav3v2_B.s_tmp < nav3v2_B.i_m) {
              h[nav3v2_B.s_tmp + 1] = 0.0;
            }
          } else {
            if (nav3v2_B.m > nav3v2_B.k_m + 1) {
              h[nav3v2_B.s_tmp - 1] *= 1.0 - nav3v2_B.htmp2;
            }
          }

          nav3v2_B.tst = nav3v2_B.b_v[1];
          nav3v2_B.ba = nav3v2_B.htmp2 * nav3v2_B.b_v[1];
          if (nav3v2_B.nr == 3) {
            nav3v2_B.aa = nav3v2_B.b_v[2];
            nav3v2_B.h12 = nav3v2_B.htmp2 * nav3v2_B.b_v[2];
            nav3v2_B.b_j_h = nav3v2_B.s_tmp - 1;
            while (nav3v2_B.b_j_h + 1 < 5) {
              nav3v2_B.nr = (nav3v2_B.b_j_h << 2) + nav3v2_B.s_tmp;
              nav3v2_B.hoffset = nav3v2_B.nr - 1;
              nav3v2_B.j_j = nav3v2_B.nr + 1;
              nav3v2_B.htmp1 = (h[nav3v2_B.hoffset] + h[nav3v2_B.nr] *
                                nav3v2_B.tst) + h[nav3v2_B.j_j] * nav3v2_B.aa;
              h[nav3v2_B.hoffset] -= nav3v2_B.htmp1 * nav3v2_B.htmp2;
              h[nav3v2_B.nr] -= nav3v2_B.htmp1 * nav3v2_B.ba;
              h[nav3v2_B.j_j] -= nav3v2_B.htmp1 * nav3v2_B.h12;
              nav3v2_B.b_j_h++;
            }

            nav3v2_B.nr = nav3v2_B.s_tmp + 3;
            nav3v2_B.b_j_h = nav3v2_B.i_m + 1;
            if (nav3v2_B.nr < nav3v2_B.b_j_h) {
              nav3v2_B.b_j_h = nav3v2_B.nr;
            }

            nav3v2_B.c_j = 0;
            while (nav3v2_B.c_j <= nav3v2_B.b_j_h - 1) {
              nav3v2_B.nr = ((nav3v2_B.s_tmp - 1) << 2) + nav3v2_B.c_j;
              nav3v2_B.hoffset = (nav3v2_B.s_tmp << 2) + nav3v2_B.c_j;
              nav3v2_B.j_j = ((nav3v2_B.s_tmp + 1) << 2) + nav3v2_B.c_j;
              nav3v2_B.htmp1 = (h[nav3v2_B.nr] + h[nav3v2_B.hoffset] *
                                nav3v2_B.tst) + h[nav3v2_B.j_j] * nav3v2_B.aa;
              h[nav3v2_B.nr] -= nav3v2_B.htmp1 * nav3v2_B.htmp2;
              h[nav3v2_B.hoffset] -= nav3v2_B.htmp1 * nav3v2_B.ba;
              h[nav3v2_B.j_j] -= nav3v2_B.htmp1 * nav3v2_B.h12;
              nav3v2_B.c_j++;
            }

            for (nav3v2_B.b_j_h = 0; nav3v2_B.b_j_h < 4; nav3v2_B.b_j_h++) {
              nav3v2_B.nr = ((nav3v2_B.s_tmp - 1) << 2) + nav3v2_B.b_j_h;
              nav3v2_B.hoffset = (nav3v2_B.s_tmp << 2) + nav3v2_B.b_j_h;
              nav3v2_B.j_j = ((nav3v2_B.s_tmp + 1) << 2) + nav3v2_B.b_j_h;
              nav3v2_B.htmp1 = (z[nav3v2_B.nr] + z[nav3v2_B.hoffset] *
                                nav3v2_B.tst) + z[nav3v2_B.j_j] * nav3v2_B.aa;
              z[nav3v2_B.nr] -= nav3v2_B.htmp1 * nav3v2_B.htmp2;
              z[nav3v2_B.hoffset] -= nav3v2_B.htmp1 * nav3v2_B.ba;
              z[nav3v2_B.j_j] -= nav3v2_B.htmp1 * nav3v2_B.h12;
            }
          } else {
            if (nav3v2_B.nr == 2) {
              nav3v2_B.j_j = nav3v2_B.s_tmp - 1;
              while (nav3v2_B.j_j + 1 < 5) {
                nav3v2_B.nr = (nav3v2_B.j_j << 2) + nav3v2_B.s_tmp;
                nav3v2_B.hoffset = nav3v2_B.nr - 1;
                nav3v2_B.htmp1 = h[nav3v2_B.hoffset] + h[nav3v2_B.nr] *
                  nav3v2_B.tst;
                h[nav3v2_B.hoffset] -= nav3v2_B.htmp1 * nav3v2_B.htmp2;
                h[nav3v2_B.nr] -= nav3v2_B.htmp1 * nav3v2_B.ba;
                nav3v2_B.j_j++;
              }

              nav3v2_B.j_j = 0;
              while (nav3v2_B.j_j <= nav3v2_B.i_m) {
                nav3v2_B.nr = ((nav3v2_B.s_tmp - 1) << 2) + nav3v2_B.j_j;
                nav3v2_B.hoffset = (nav3v2_B.s_tmp << 2) + nav3v2_B.j_j;
                nav3v2_B.htmp1 = h[nav3v2_B.nr] + h[nav3v2_B.hoffset] *
                  nav3v2_B.tst;
                h[nav3v2_B.nr] -= nav3v2_B.htmp1 * nav3v2_B.htmp2;
                h[nav3v2_B.hoffset] -= nav3v2_B.htmp1 * nav3v2_B.ba;
                nav3v2_B.j_j++;
              }

              for (nav3v2_B.j_j = 0; nav3v2_B.j_j < 4; nav3v2_B.j_j++) {
                nav3v2_B.nr = ((nav3v2_B.s_tmp - 1) << 2) + nav3v2_B.j_j;
                nav3v2_B.hoffset = (nav3v2_B.s_tmp << 2) + nav3v2_B.j_j;
                nav3v2_B.htmp1 = z[nav3v2_B.nr] + z[nav3v2_B.hoffset] *
                  nav3v2_B.tst;
                z[nav3v2_B.nr] -= nav3v2_B.htmp1 * nav3v2_B.htmp2;
                z[nav3v2_B.hoffset] -= nav3v2_B.htmp1 * nav3v2_B.ba;
              }
            }
          }

          nav3v2_B.s_tmp++;
        }

        nav3v2_B.ix++;
      }
    }

    if (!nav3v2_B.goto150) {
      info = nav3v2_B.i_m + 1;
      exitg1 = true;
    } else {
      if ((nav3v2_B.i_m + 1 != nav3v2_B.L) && (nav3v2_B.L == nav3v2_B.i_m)) {
        nav3v2_B.ix = (nav3v2_B.i_m - 1) << 2;
        nav3v2_B.k_m = nav3v2_B.ix + nav3v2_B.i_m;
        nav3v2_B.m = nav3v2_B.k_m - 1;
        nav3v2_B.ba = h[nav3v2_B.m];
        nav3v2_B.s_tmp = nav3v2_B.i_m << 2;
        nav3v2_B.nr = nav3v2_B.s_tmp + nav3v2_B.i_m;
        nav3v2_B.hoffset = nav3v2_B.nr - 1;
        nav3v2_B.htmp1 = h[nav3v2_B.hoffset];
        nav3v2_B.aa = h[nav3v2_B.k_m];
        nav3v2_B.h12 = h[nav3v2_B.nr];
        nav3v2_xdlanv2(&nav3v2_B.ba, &nav3v2_B.htmp1, &nav3v2_B.aa,
                       &nav3v2_B.h12, &nav3v2_B.h21s, &nav3v2_B.unusedU1,
                       &nav3v2_B.unusedU2, &nav3v2_B.unusedU3, &nav3v2_B.htmp2,
                       &nav3v2_B.tst);
        h[nav3v2_B.m] = nav3v2_B.ba;
        h[nav3v2_B.hoffset] = nav3v2_B.htmp1;
        h[nav3v2_B.k_m] = nav3v2_B.aa;
        h[nav3v2_B.nr] = nav3v2_B.h12;
        if (4 > nav3v2_B.i_m + 1) {
          nav3v2_xrot(3 - nav3v2_B.i_m, h, nav3v2_B.i_m + ((nav3v2_B.i_m + 1) <<
            2), (nav3v2_B.i_m + ((nav3v2_B.i_m + 1) << 2)) + 1, nav3v2_B.htmp2,
                      nav3v2_B.tst);
        }

        nav3v2_xrot_o(nav3v2_B.i_m - 1, h, ((nav3v2_B.i_m - 1) << 2) + 1,
                      (nav3v2_B.i_m << 2) + 1, nav3v2_B.htmp2, nav3v2_B.tst);
        nav3v2_B.ba = nav3v2_B.htmp2 * z[nav3v2_B.ix] + nav3v2_B.tst *
          z[nav3v2_B.s_tmp];
        z[nav3v2_B.s_tmp] = nav3v2_B.htmp2 * z[nav3v2_B.s_tmp] - nav3v2_B.tst *
          z[nav3v2_B.ix];
        z[nav3v2_B.ix] = nav3v2_B.ba;
        nav3v2_B.i_m = nav3v2_B.s_tmp + 1;
        nav3v2_B.ix++;
        nav3v2_B.ba = nav3v2_B.htmp2 * z[nav3v2_B.ix] + nav3v2_B.tst *
          z[nav3v2_B.i_m];
        z[nav3v2_B.i_m] = nav3v2_B.htmp2 * z[nav3v2_B.i_m] - nav3v2_B.tst *
          z[nav3v2_B.ix];
        z[nav3v2_B.ix] = nav3v2_B.ba;
        nav3v2_B.i_m++;
        nav3v2_B.ix++;
        nav3v2_B.ba = nav3v2_B.htmp2 * z[nav3v2_B.ix] + nav3v2_B.tst *
          z[nav3v2_B.i_m];
        z[nav3v2_B.i_m] = nav3v2_B.htmp2 * z[nav3v2_B.i_m] - nav3v2_B.tst *
          z[nav3v2_B.ix];
        z[nav3v2_B.ix] = nav3v2_B.ba;
        nav3v2_B.i_m++;
        nav3v2_B.ix++;
        nav3v2_B.ba = nav3v2_B.htmp2 * z[nav3v2_B.ix] + nav3v2_B.tst *
          z[nav3v2_B.i_m];
        z[nav3v2_B.i_m] = nav3v2_B.htmp2 * z[nav3v2_B.i_m] - nav3v2_B.tst *
          z[nav3v2_B.ix];
        z[nav3v2_B.ix] = nav3v2_B.ba;
      }

      nav3v2_B.i_m = nav3v2_B.L - 2;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S107>/World to Robot Transform' */
static void nav3v2_eig(const real_T A[16], creal_T V[16], creal_T D[4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (nav3v2_anyNonFinite(A)) {
    for (nav3v2_B.b_j = 0; nav3v2_B.b_j < 16; nav3v2_B.b_j++) {
      V[nav3v2_B.b_j].re = (rtNaN);
      V[nav3v2_B.b_j].im = 0.0;
    }

    D[0].re = (rtNaN);
    D[0].im = 0.0;
    D[1].re = (rtNaN);
    D[1].im = 0.0;
    D[2].re = (rtNaN);
    D[2].im = 0.0;
    D[3].re = (rtNaN);
    D[3].im = 0.0;
  } else {
    nav3v2_B.p_f = true;
    nav3v2_B.b_j = 0;
    exitg2 = false;
    while ((!exitg2) && (nav3v2_B.b_j < 4)) {
      nav3v2_B.i_n = 0;
      do {
        exitg1 = 0;
        if (nav3v2_B.i_n <= nav3v2_B.b_j) {
          if (!(A[(nav3v2_B.b_j << 2) + nav3v2_B.i_n] == A[(nav3v2_B.i_n << 2) +
                nav3v2_B.b_j])) {
            nav3v2_B.p_f = false;
            exitg1 = 1;
          } else {
            nav3v2_B.i_n++;
          }
        } else {
          nav3v2_B.b_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (nav3v2_B.p_f) {
      if (nav3v2_anyNonFinite(A)) {
        for (nav3v2_B.b_j = 0; nav3v2_B.b_j < 16; nav3v2_B.b_j++) {
          nav3v2_B.b_V[nav3v2_B.b_j] = (rtNaN);
        }

        nav3v2_B.b_j = 2;
        while (nav3v2_B.b_j < 5) {
          nav3v2_B.b_V[nav3v2_B.b_j - 1] = 0.0;
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_j = 3;
        while (nav3v2_B.b_j < 5) {
          nav3v2_B.b_V[nav3v2_B.b_j + 3] = 0.0;
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_V[11] = 0.0;
        for (nav3v2_B.b_j = 0; nav3v2_B.b_j < 16; nav3v2_B.b_j++) {
          nav3v2_B.b_A[nav3v2_B.b_j] = (rtNaN);
        }
      } else {
        memcpy(&nav3v2_B.b_A[0], &A[0], sizeof(real_T) << 4U);
        nav3v2_xgehrd(nav3v2_B.b_A, nav3v2_B.tau);
        memcpy(&nav3v2_B.b_V[0], &nav3v2_B.b_A[0], sizeof(real_T) << 4U);
        nav3v2_B.b_j = 0;
        while (nav3v2_B.b_j <= 2) {
          nav3v2_B.b_V[nav3v2_B.b_j + 12] = 0.0;
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_j = 0;
        while (nav3v2_B.b_j <= 1) {
          nav3v2_B.b_V[nav3v2_B.b_j + 8] = 0.0;
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_j = 1;
        while (nav3v2_B.b_j + 3 < 5) {
          nav3v2_B.b_V[nav3v2_B.b_j + 10] = nav3v2_B.b_V[nav3v2_B.b_j + 6];
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_V[4] = 0.0;
        nav3v2_B.b_j = 0;
        while (nav3v2_B.b_j + 3 < 5) {
          nav3v2_B.b_V[nav3v2_B.b_j + 6] = nav3v2_B.b_V[nav3v2_B.b_j + 2];
          nav3v2_B.b_j++;
        }

        nav3v2_B.work[0] = 0.0;
        nav3v2_B.b_V[1] = 0.0;
        nav3v2_B.work[1] = 0.0;
        nav3v2_B.b_V[2] = 0.0;
        nav3v2_B.work[2] = 0.0;
        nav3v2_B.b_V[3] = 0.0;
        nav3v2_B.work[3] = 0.0;
        nav3v2_B.b_V[0] = 1.0;
        nav3v2_B.b_V[15] = 1.0 - nav3v2_B.tau[2];
        nav3v2_B.b_j = 0;
        while (nav3v2_B.b_j <= 1) {
          nav3v2_B.b_V[14 - nav3v2_B.b_j] = 0.0;
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_V[10] = 1.0;
        nav3v2_xzlarf(2, 1, 11, nav3v2_B.tau[1], nav3v2_B.b_V, 15, nav3v2_B.work);
        nav3v2_B.b_j = 11;
        while (nav3v2_B.b_j + 1 <= 12) {
          nav3v2_B.b_V[nav3v2_B.b_j] *= -nav3v2_B.tau[1];
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_V[10] = 1.0 - nav3v2_B.tau[1];
        nav3v2_B.b_V[9] = 0.0;
        nav3v2_B.b_V[5] = 1.0;
        nav3v2_xzlarf(3, 2, 6, nav3v2_B.tau[0], nav3v2_B.b_V, 10, nav3v2_B.work);
        nav3v2_B.b_j = 6;
        while (nav3v2_B.b_j + 1 <= 8) {
          nav3v2_B.b_V[nav3v2_B.b_j] *= -nav3v2_B.tau[0];
          nav3v2_B.b_j++;
        }

        nav3v2_B.b_V[5] = 1.0 - nav3v2_B.tau[0];
        nav3v2_eml_dlahqr(nav3v2_B.b_A, nav3v2_B.b_V);
      }

      for (nav3v2_B.b_j = 0; nav3v2_B.b_j < 16; nav3v2_B.b_j++) {
        V[nav3v2_B.b_j].re = nav3v2_B.b_V[nav3v2_B.b_j];
        V[nav3v2_B.b_j].im = 0.0;
      }

      D[0].re = nav3v2_B.b_A[0];
      D[0].im = 0.0;
      D[1].re = nav3v2_B.b_A[5];
      D[1].im = 0.0;
      D[2].re = nav3v2_B.b_A[10];
      D[2].im = 0.0;
      D[3].re = nav3v2_B.b_A[15];
      D[3].im = 0.0;
    } else {
      for (nav3v2_B.b_j = 0; nav3v2_B.b_j < 16; nav3v2_B.b_j++) {
        nav3v2_B.At[nav3v2_B.b_j].re = A[nav3v2_B.b_j];
        nav3v2_B.At[nav3v2_B.b_j].im = 0.0;
      }

      nav3v2_xzggev(nav3v2_B.At, &nav3v2_B.b_j, D, nav3v2_B.beta1, V);
      nav3v2_B.colnorm = 0.0;
      nav3v2_B.scale = 3.3121686421112381E-170;
      nav3v2_B.b_j = 0;
      while (nav3v2_B.b_j + 1 <= 4) {
        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].re);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].im);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.b_j++;
      }

      nav3v2_B.colnorm = nav3v2_B.scale * sqrt(nav3v2_B.colnorm);
      nav3v2_B.b_j = 0;
      while (nav3v2_B.b_j + 1 <= 4) {
        if (V[nav3v2_B.b_j].im == 0.0) {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = 0.0;
        } else if (V[nav3v2_B.b_j].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        } else {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        }

        V[nav3v2_B.b_j].re = nav3v2_B.scale;
        V[nav3v2_B.b_j].im = nav3v2_B.absxk;
        nav3v2_B.b_j++;
      }

      nav3v2_B.colnorm = 0.0;
      nav3v2_B.scale = 3.3121686421112381E-170;
      nav3v2_B.b_j = 4;
      while (nav3v2_B.b_j + 1 <= 8) {
        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].re);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].im);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.b_j++;
      }

      nav3v2_B.colnorm = nav3v2_B.scale * sqrt(nav3v2_B.colnorm);
      nav3v2_B.b_j = 4;
      while (nav3v2_B.b_j + 1 <= 8) {
        if (V[nav3v2_B.b_j].im == 0.0) {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = 0.0;
        } else if (V[nav3v2_B.b_j].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        } else {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        }

        V[nav3v2_B.b_j].re = nav3v2_B.scale;
        V[nav3v2_B.b_j].im = nav3v2_B.absxk;
        nav3v2_B.b_j++;
      }

      nav3v2_B.colnorm = 0.0;
      nav3v2_B.scale = 3.3121686421112381E-170;
      nav3v2_B.b_j = 8;
      while (nav3v2_B.b_j + 1 <= 12) {
        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].re);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].im);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.b_j++;
      }

      nav3v2_B.colnorm = nav3v2_B.scale * sqrt(nav3v2_B.colnorm);
      nav3v2_B.b_j = 8;
      while (nav3v2_B.b_j + 1 <= 12) {
        if (V[nav3v2_B.b_j].im == 0.0) {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = 0.0;
        } else if (V[nav3v2_B.b_j].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        } else {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        }

        V[nav3v2_B.b_j].re = nav3v2_B.scale;
        V[nav3v2_B.b_j].im = nav3v2_B.absxk;
        nav3v2_B.b_j++;
      }

      nav3v2_B.colnorm = 0.0;
      nav3v2_B.scale = 3.3121686421112381E-170;
      nav3v2_B.b_j = 12;
      while (nav3v2_B.b_j + 1 <= 16) {
        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].re);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.absxk = fabs(V[nav3v2_B.b_j].im);
        if (nav3v2_B.absxk > nav3v2_B.scale) {
          nav3v2_B.t = nav3v2_B.scale / nav3v2_B.absxk;
          nav3v2_B.colnorm = nav3v2_B.colnorm * nav3v2_B.t * nav3v2_B.t + 1.0;
          nav3v2_B.scale = nav3v2_B.absxk;
        } else {
          nav3v2_B.t = nav3v2_B.absxk / nav3v2_B.scale;
          nav3v2_B.colnorm += nav3v2_B.t * nav3v2_B.t;
        }

        nav3v2_B.b_j++;
      }

      nav3v2_B.colnorm = nav3v2_B.scale * sqrt(nav3v2_B.colnorm);
      nav3v2_B.b_j = 12;
      while (nav3v2_B.b_j + 1 <= 16) {
        if (V[nav3v2_B.b_j].im == 0.0) {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = 0.0;
        } else if (V[nav3v2_B.b_j].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        } else {
          nav3v2_B.scale = V[nav3v2_B.b_j].re / nav3v2_B.colnorm;
          nav3v2_B.absxk = V[nav3v2_B.b_j].im / nav3v2_B.colnorm;
        }

        V[nav3v2_B.b_j].re = nav3v2_B.scale;
        V[nav3v2_B.b_j].im = nav3v2_B.absxk;
        nav3v2_B.b_j++;
      }

      if (nav3v2_B.beta1[0].im == 0.0) {
        if (D[0].im == 0.0) {
          nav3v2_B.scale = D[0].re / nav3v2_B.beta1[0].re;
          nav3v2_B.absxk = 0.0;
        } else if (D[0].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = D[0].im / nav3v2_B.beta1[0].re;
        } else {
          nav3v2_B.scale = D[0].re / nav3v2_B.beta1[0].re;
          nav3v2_B.absxk = D[0].im / nav3v2_B.beta1[0].re;
        }
      } else if (nav3v2_B.beta1[0].re == 0.0) {
        if (D[0].re == 0.0) {
          nav3v2_B.scale = D[0].im / nav3v2_B.beta1[0].im;
          nav3v2_B.absxk = 0.0;
        } else if (D[0].im == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = -(D[0].re / nav3v2_B.beta1[0].im);
        } else {
          nav3v2_B.scale = D[0].im / nav3v2_B.beta1[0].im;
          nav3v2_B.absxk = -(D[0].re / nav3v2_B.beta1[0].im);
        }
      } else {
        nav3v2_B.colnorm = fabs(nav3v2_B.beta1[0].re);
        nav3v2_B.scale = fabs(nav3v2_B.beta1[0].im);
        if (nav3v2_B.colnorm > nav3v2_B.scale) {
          nav3v2_B.colnorm = nav3v2_B.beta1[0].im / nav3v2_B.beta1[0].re;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[0].im +
            nav3v2_B.beta1[0].re;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[0].im + D[0].re) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (D[0].im - nav3v2_B.colnorm * D[0].re) /
            nav3v2_B.absxk;
        } else if (nav3v2_B.scale == nav3v2_B.colnorm) {
          nav3v2_B.absxk = nav3v2_B.beta1[0].re > 0.0 ? 0.5 : -0.5;
          nav3v2_B.t = nav3v2_B.beta1[0].im > 0.0 ? 0.5 : -0.5;
          nav3v2_B.scale = (D[0].re * nav3v2_B.absxk + D[0].im * nav3v2_B.t) /
            nav3v2_B.colnorm;
          nav3v2_B.absxk = (D[0].im * nav3v2_B.absxk - D[0].re * nav3v2_B.t) /
            nav3v2_B.colnorm;
        } else {
          nav3v2_B.colnorm = nav3v2_B.beta1[0].re / nav3v2_B.beta1[0].im;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[0].re +
            nav3v2_B.beta1[0].im;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[0].re + D[0].im) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (nav3v2_B.colnorm * D[0].im - D[0].re) /
            nav3v2_B.absxk;
        }
      }

      D[0].re = nav3v2_B.scale;
      D[0].im = nav3v2_B.absxk;
      if (nav3v2_B.beta1[1].im == 0.0) {
        if (D[1].im == 0.0) {
          nav3v2_B.scale = D[1].re / nav3v2_B.beta1[1].re;
          nav3v2_B.absxk = 0.0;
        } else if (D[1].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = D[1].im / nav3v2_B.beta1[1].re;
        } else {
          nav3v2_B.scale = D[1].re / nav3v2_B.beta1[1].re;
          nav3v2_B.absxk = D[1].im / nav3v2_B.beta1[1].re;
        }
      } else if (nav3v2_B.beta1[1].re == 0.0) {
        if (D[1].re == 0.0) {
          nav3v2_B.scale = D[1].im / nav3v2_B.beta1[1].im;
          nav3v2_B.absxk = 0.0;
        } else if (D[1].im == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = -(D[1].re / nav3v2_B.beta1[1].im);
        } else {
          nav3v2_B.scale = D[1].im / nav3v2_B.beta1[1].im;
          nav3v2_B.absxk = -(D[1].re / nav3v2_B.beta1[1].im);
        }
      } else {
        nav3v2_B.colnorm = fabs(nav3v2_B.beta1[1].re);
        nav3v2_B.scale = fabs(nav3v2_B.beta1[1].im);
        if (nav3v2_B.colnorm > nav3v2_B.scale) {
          nav3v2_B.colnorm = nav3v2_B.beta1[1].im / nav3v2_B.beta1[1].re;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[1].im +
            nav3v2_B.beta1[1].re;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[1].im + D[1].re) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (D[1].im - nav3v2_B.colnorm * D[1].re) /
            nav3v2_B.absxk;
        } else if (nav3v2_B.scale == nav3v2_B.colnorm) {
          nav3v2_B.absxk = nav3v2_B.beta1[1].re > 0.0 ? 0.5 : -0.5;
          nav3v2_B.t = nav3v2_B.beta1[1].im > 0.0 ? 0.5 : -0.5;
          nav3v2_B.scale = (D[1].re * nav3v2_B.absxk + D[1].im * nav3v2_B.t) /
            nav3v2_B.colnorm;
          nav3v2_B.absxk = (D[1].im * nav3v2_B.absxk - D[1].re * nav3v2_B.t) /
            nav3v2_B.colnorm;
        } else {
          nav3v2_B.colnorm = nav3v2_B.beta1[1].re / nav3v2_B.beta1[1].im;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[1].re +
            nav3v2_B.beta1[1].im;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[1].re + D[1].im) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (nav3v2_B.colnorm * D[1].im - D[1].re) /
            nav3v2_B.absxk;
        }
      }

      D[1].re = nav3v2_B.scale;
      D[1].im = nav3v2_B.absxk;
      if (nav3v2_B.beta1[2].im == 0.0) {
        if (D[2].im == 0.0) {
          nav3v2_B.scale = D[2].re / nav3v2_B.beta1[2].re;
          nav3v2_B.absxk = 0.0;
        } else if (D[2].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = D[2].im / nav3v2_B.beta1[2].re;
        } else {
          nav3v2_B.scale = D[2].re / nav3v2_B.beta1[2].re;
          nav3v2_B.absxk = D[2].im / nav3v2_B.beta1[2].re;
        }
      } else if (nav3v2_B.beta1[2].re == 0.0) {
        if (D[2].re == 0.0) {
          nav3v2_B.scale = D[2].im / nav3v2_B.beta1[2].im;
          nav3v2_B.absxk = 0.0;
        } else if (D[2].im == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = -(D[2].re / nav3v2_B.beta1[2].im);
        } else {
          nav3v2_B.scale = D[2].im / nav3v2_B.beta1[2].im;
          nav3v2_B.absxk = -(D[2].re / nav3v2_B.beta1[2].im);
        }
      } else {
        nav3v2_B.colnorm = fabs(nav3v2_B.beta1[2].re);
        nav3v2_B.scale = fabs(nav3v2_B.beta1[2].im);
        if (nav3v2_B.colnorm > nav3v2_B.scale) {
          nav3v2_B.colnorm = nav3v2_B.beta1[2].im / nav3v2_B.beta1[2].re;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[2].im +
            nav3v2_B.beta1[2].re;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[2].im + D[2].re) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (D[2].im - nav3v2_B.colnorm * D[2].re) /
            nav3v2_B.absxk;
        } else if (nav3v2_B.scale == nav3v2_B.colnorm) {
          nav3v2_B.absxk = nav3v2_B.beta1[2].re > 0.0 ? 0.5 : -0.5;
          nav3v2_B.t = nav3v2_B.beta1[2].im > 0.0 ? 0.5 : -0.5;
          nav3v2_B.scale = (D[2].re * nav3v2_B.absxk + D[2].im * nav3v2_B.t) /
            nav3v2_B.colnorm;
          nav3v2_B.absxk = (D[2].im * nav3v2_B.absxk - D[2].re * nav3v2_B.t) /
            nav3v2_B.colnorm;
        } else {
          nav3v2_B.colnorm = nav3v2_B.beta1[2].re / nav3v2_B.beta1[2].im;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[2].re +
            nav3v2_B.beta1[2].im;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[2].re + D[2].im) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (nav3v2_B.colnorm * D[2].im - D[2].re) /
            nav3v2_B.absxk;
        }
      }

      D[2].re = nav3v2_B.scale;
      D[2].im = nav3v2_B.absxk;
      if (nav3v2_B.beta1[3].im == 0.0) {
        if (D[3].im == 0.0) {
          nav3v2_B.scale = D[3].re / nav3v2_B.beta1[3].re;
          nav3v2_B.absxk = 0.0;
        } else if (D[3].re == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = D[3].im / nav3v2_B.beta1[3].re;
        } else {
          nav3v2_B.scale = D[3].re / nav3v2_B.beta1[3].re;
          nav3v2_B.absxk = D[3].im / nav3v2_B.beta1[3].re;
        }
      } else if (nav3v2_B.beta1[3].re == 0.0) {
        if (D[3].re == 0.0) {
          nav3v2_B.scale = D[3].im / nav3v2_B.beta1[3].im;
          nav3v2_B.absxk = 0.0;
        } else if (D[3].im == 0.0) {
          nav3v2_B.scale = 0.0;
          nav3v2_B.absxk = -(D[3].re / nav3v2_B.beta1[3].im);
        } else {
          nav3v2_B.scale = D[3].im / nav3v2_B.beta1[3].im;
          nav3v2_B.absxk = -(D[3].re / nav3v2_B.beta1[3].im);
        }
      } else {
        nav3v2_B.colnorm = fabs(nav3v2_B.beta1[3].re);
        nav3v2_B.scale = fabs(nav3v2_B.beta1[3].im);
        if (nav3v2_B.colnorm > nav3v2_B.scale) {
          nav3v2_B.colnorm = nav3v2_B.beta1[3].im / nav3v2_B.beta1[3].re;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[3].im +
            nav3v2_B.beta1[3].re;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[3].im + D[3].re) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (D[3].im - nav3v2_B.colnorm * D[3].re) /
            nav3v2_B.absxk;
        } else if (nav3v2_B.scale == nav3v2_B.colnorm) {
          nav3v2_B.absxk = nav3v2_B.beta1[3].re > 0.0 ? 0.5 : -0.5;
          nav3v2_B.t = nav3v2_B.beta1[3].im > 0.0 ? 0.5 : -0.5;
          nav3v2_B.scale = (D[3].re * nav3v2_B.absxk + D[3].im * nav3v2_B.t) /
            nav3v2_B.colnorm;
          nav3v2_B.absxk = (D[3].im * nav3v2_B.absxk - D[3].re * nav3v2_B.t) /
            nav3v2_B.colnorm;
        } else {
          nav3v2_B.colnorm = nav3v2_B.beta1[3].re / nav3v2_B.beta1[3].im;
          nav3v2_B.absxk = nav3v2_B.colnorm * nav3v2_B.beta1[3].re +
            nav3v2_B.beta1[3].im;
          nav3v2_B.scale = (nav3v2_B.colnorm * D[3].re + D[3].im) /
            nav3v2_B.absxk;
          nav3v2_B.absxk = (nav3v2_B.colnorm * D[3].im - D[3].re) /
            nav3v2_B.absxk;
        }
      }

      D[3].re = nav3v2_B.scale;
      D[3].im = nav3v2_B.absxk;
    }
  }
}

static real_T nav3v2_rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      nav3v2_B.u0 = 1;
    } else {
      nav3v2_B.u0 = -1;
    }

    if (u1 > 0.0) {
      nav3v2_B.u1 = 1;
    } else {
      nav3v2_B.u1 = -1;
    }

    y = atan2(static_cast<real_T>(nav3v2_B.u0), static_cast<real_T>(nav3v2_B.u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for Chart: '<S2>/State_Machine' */
static real_T nav3v2_CalculateHeading(const real_T startPose[3], const real_T
  endPose[2])
{
  real_T heading;
  heading = nav3v2_rt_atan2d_snf(endPose[1] - startPose[1], endPose[0] -
    startPose[0]);
  return heading;
}

/* Function for Chart: '<S2>/State_Machine' */
static real_T nav3v2_CalculateReverseHeading(const real_T startPose[3], const
  real_T endPose[2])
{
  real_T heading;
  heading = nav3v2_rt_atan2d_snf(startPose[1] - endPose[1], startPose[0] -
    endPose[0]);
  return heading;
}

/* Function for Chart: '<S2>/State_Machine' */
static real_T nav3v2_norm(const real_T x[2])
{
  real_T y;
  nav3v2_B.scale_n = 3.3121686421112381E-170;
  nav3v2_B.absxk_p = fabs(x[0]);
  if (nav3v2_B.absxk_p > 3.3121686421112381E-170) {
    y = 1.0;
    nav3v2_B.scale_n = nav3v2_B.absxk_p;
  } else {
    nav3v2_B.t_l = nav3v2_B.absxk_p / 3.3121686421112381E-170;
    y = nav3v2_B.t_l * nav3v2_B.t_l;
  }

  nav3v2_B.absxk_p = fabs(x[1]);
  if (nav3v2_B.absxk_p > nav3v2_B.scale_n) {
    nav3v2_B.t_l = nav3v2_B.scale_n / nav3v2_B.absxk_p;
    y = y * nav3v2_B.t_l * nav3v2_B.t_l + 1.0;
    nav3v2_B.scale_n = nav3v2_B.absxk_p;
  } else {
    nav3v2_B.t_l = nav3v2_B.absxk_p / nav3v2_B.scale_n;
    y += nav3v2_B.t_l * nav3v2_B.t_l;
  }

  return nav3v2_B.scale_n * sqrt(y);
}

/* Function for Chart: '<S2>/State_Machine' */
static real_T nav3v2_closestPointOnLine(const real_T pt1[2], real_T pt2[2],
  const real_T refPt[2])
{
  real_T distance;
  boolean_T exitg1;
  nav3v2_B.p_g = false;
  nav3v2_B.b_p = true;
  nav3v2_B.k_c = 0;
  exitg1 = false;
  while ((!exitg1) && (nav3v2_B.k_c < 2)) {
    if (!(pt1[nav3v2_B.k_c] == pt2[nav3v2_B.k_c])) {
      nav3v2_B.b_p = false;
      exitg1 = true;
    } else {
      nav3v2_B.k_c++;
    }
  }

  if (nav3v2_B.b_p) {
    nav3v2_B.p_g = true;
  }

  if (nav3v2_B.p_g) {
    pt2[0] = pt1[0];
    nav3v2_B.refPt[0] = refPt[0] - pt1[0];
    pt2[1] = pt1[1];
    nav3v2_B.refPt[1] = refPt[1] - pt1[1];
    distance = nav3v2_norm(nav3v2_B.refPt);
  } else {
    nav3v2_B.alpha = pt2[0] - pt1[0];
    nav3v2_B.v12 = (pt2[0] - refPt[0]) * nav3v2_B.alpha;
    nav3v2_B.v12_m = nav3v2_B.alpha * nav3v2_B.alpha;
    nav3v2_B.alpha = pt2[1] - pt1[1];
    nav3v2_B.v12 += (pt2[1] - refPt[1]) * nav3v2_B.alpha;
    nav3v2_B.v12_m += nav3v2_B.alpha * nav3v2_B.alpha;
    nav3v2_B.alpha = nav3v2_B.v12 / nav3v2_B.v12_m;
    if (nav3v2_B.alpha > 1.0) {
      pt2[0] = pt1[0];
      pt2[1] = pt1[1];
    } else {
      if (!(nav3v2_B.alpha < 0.0)) {
        pt2[0] = (1.0 - nav3v2_B.alpha) * pt2[0] + nav3v2_B.alpha * pt1[0];
        pt2[1] = (1.0 - nav3v2_B.alpha) * pt2[1] + nav3v2_B.alpha * pt1[1];
      }
    }

    nav3v2_B.refPt[0] = refPt[0] - pt2[0];
    nav3v2_B.refPt[1] = refPt[1] - pt2[1];
    distance = nav3v2_norm(nav3v2_B.refPt);
  }

  return distance;
}

/* Function for Chart: '<S2>/State_Machine' */
static void controllerPurePursuit_stepImpl(controllerPurePursuit_nav3v2_T *obj,
  const real_T curPose[3], real_T *v, real_T *w, real_T lookaheadPoint[2])
{
  boolean_T exitg1;
  for (nav3v2_B.partialTrueCount = 0; nav3v2_B.partialTrueCount < 6;
       nav3v2_B.partialTrueCount++) {
    nav3v2_B.b[nav3v2_B.partialTrueCount] = !rtIsNaN(obj->
      Waypoints[nav3v2_B.partialTrueCount]);
  }

  nav3v2_B.trueCount = 0;
  nav3v2_B.partialTrueCount = 0;
  if (nav3v2_B.b[0] && nav3v2_B.b[3]) {
    nav3v2_B.trueCount = 1;
    nav3v2_B.c_data[0] = 1;
    nav3v2_B.partialTrueCount = 1;
  }

  if (nav3v2_B.b[1] && nav3v2_B.b[4]) {
    nav3v2_B.trueCount++;
    nav3v2_B.c_data[nav3v2_B.partialTrueCount] = 2;
    nav3v2_B.partialTrueCount++;
  }

  if (nav3v2_B.b[2] && nav3v2_B.b[5]) {
    nav3v2_B.trueCount++;
    nav3v2_B.c_data[nav3v2_B.partialTrueCount] = 3;
  }

  for (nav3v2_B.partialTrueCount = 0; nav3v2_B.partialTrueCount <
       nav3v2_B.trueCount; nav3v2_B.partialTrueCount++) {
    nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount] = obj->
      Waypoints[nav3v2_B.c_data[nav3v2_B.partialTrueCount] - 1];
    nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount + nav3v2_B.trueCount] =
      obj->Waypoints[nav3v2_B.c_data[nav3v2_B.partialTrueCount] + 2];
  }

  if (nav3v2_B.trueCount == 0) {
    *v = 0.0;
    *w = 0.0;
    lookaheadPoint[0] = curPose[0];
    lookaheadPoint[1] = curPose[1];
  } else {
    nav3v2_B.searchFlag = false;
    if (obj->ProjectionLineIndex == 0.0) {
      nav3v2_B.searchFlag = true;
      obj->ProjectionPoint[0] = nav3v2_B.waypoints_data[0];
      obj->ProjectionPoint[1] = nav3v2_B.waypoints_data[nav3v2_B.trueCount];
      obj->ProjectionLineIndex = 1.0;
    }

    if (nav3v2_B.trueCount == 1) {
      obj->ProjectionPoint[0] = nav3v2_B.waypoints_data[0];
      obj->ProjectionPoint[1] = nav3v2_B.waypoints_data[nav3v2_B.trueCount];
      nav3v2_B.minDistance = nav3v2_B.waypoints_data[0];
      nav3v2_B.dist = nav3v2_B.waypoints_data[nav3v2_B.trueCount];
    } else {
      nav3v2_B.partialTrueCount = static_cast<int32_T>(obj->ProjectionLineIndex
        + 1.0);
      nav3v2_B.lookaheadStartPt[0] =
        nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount - 1];
      nav3v2_B.lookaheadStartPt[1] = nav3v2_B.waypoints_data
        [(nav3v2_B.partialTrueCount + nav3v2_B.trueCount) - 1];
      nav3v2_B.minDistance = nav3v2_closestPointOnLine(obj->ProjectionPoint,
        nav3v2_B.lookaheadStartPt, &curPose[0]);
      obj->ProjectionPoint[0] = nav3v2_B.lookaheadStartPt[0];
      nav3v2_B.obj[0] = obj->ProjectionPoint[0] -
        nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount - 1];
      obj->ProjectionPoint[1] = nav3v2_B.lookaheadStartPt[1];
      nav3v2_B.obj[1] = obj->ProjectionPoint[1] - nav3v2_B.waypoints_data
        [(nav3v2_B.partialTrueCount + nav3v2_B.trueCount) - 1];
      nav3v2_B.dist = nav3v2_norm(nav3v2_B.obj);
      nav3v2_B.b_dist = obj->ProjectionLineIndex + 1.0;
      nav3v2_B.partialTrueCount = static_cast<int32_T>((1.0 -
        (obj->ProjectionLineIndex + 1.0)) + (static_cast<real_T>
        (nav3v2_B.trueCount) - 1.0));
      nav3v2_B.d_i = 0;
      exitg1 = false;
      while ((!exitg1) && (nav3v2_B.d_i <= nav3v2_B.partialTrueCount - 1)) {
        nav3v2_B.overshootDist = nav3v2_B.b_dist + static_cast<real_T>
          (nav3v2_B.d_i);
        if ((!nav3v2_B.searchFlag) && (nav3v2_B.dist > obj->LookaheadDistance))
        {
          exitg1 = true;
        } else {
          nav3v2_B.c_i_tmp_m = static_cast<int32_T>(nav3v2_B.overshootDist);
          nav3v2_B.c_i_tmp = static_cast<int32_T>(nav3v2_B.overshootDist + 1.0);
          nav3v2_B.obj[0] = nav3v2_B.waypoints_data[nav3v2_B.c_i_tmp_m - 1] -
            nav3v2_B.waypoints_data[nav3v2_B.c_i_tmp - 1];
          nav3v2_B.lookaheadStartPt[0] =
            nav3v2_B.waypoints_data[nav3v2_B.c_i_tmp - 1];
          nav3v2_B.waypoints[0] = nav3v2_B.waypoints_data[nav3v2_B.c_i_tmp_m - 1];
          nav3v2_B.obj[1] = nav3v2_B.waypoints_data[(nav3v2_B.c_i_tmp_m +
            nav3v2_B.trueCount) - 1] - nav3v2_B.waypoints_data[(nav3v2_B.c_i_tmp
            + nav3v2_B.trueCount) - 1];
          nav3v2_B.lookaheadStartPt[1] = nav3v2_B.waypoints_data
            [(nav3v2_B.c_i_tmp + nav3v2_B.trueCount) - 1];
          nav3v2_B.waypoints[1] = nav3v2_B.waypoints_data[(nav3v2_B.c_i_tmp_m +
            nav3v2_B.trueCount) - 1];
          nav3v2_B.dist += nav3v2_norm(nav3v2_B.obj);
          nav3v2_B.lookaheadIdx = nav3v2_closestPointOnLine(nav3v2_B.waypoints,
            nav3v2_B.lookaheadStartPt, &curPose[0]);
          if (nav3v2_B.lookaheadIdx < nav3v2_B.minDistance) {
            nav3v2_B.minDistance = nav3v2_B.lookaheadIdx;
            obj->ProjectionPoint[0] = nav3v2_B.lookaheadStartPt[0];
            obj->ProjectionPoint[1] = nav3v2_B.lookaheadStartPt[1];
            obj->ProjectionLineIndex = nav3v2_B.overshootDist;
          }

          nav3v2_B.d_i++;
        }
      }

      nav3v2_B.partialTrueCount = static_cast<int32_T>(obj->ProjectionLineIndex
        + 1.0);
      nav3v2_B.obj[0] = obj->ProjectionPoint[0] -
        nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount - 1];
      nav3v2_B.obj[1] = obj->ProjectionPoint[1] - nav3v2_B.waypoints_data
        [(nav3v2_B.partialTrueCount + nav3v2_B.trueCount) - 1];
      nav3v2_B.b_dist = nav3v2_norm(nav3v2_B.obj);
      nav3v2_B.partialTrueCount = static_cast<int32_T>(obj->ProjectionLineIndex
        + 1.0);
      nav3v2_B.lookaheadStartPt[0] = obj->ProjectionPoint[0];
      nav3v2_B.minDistance = nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount -
        1];
      nav3v2_B.lookaheadStartPt[1] = obj->ProjectionPoint[1];
      nav3v2_B.dist = nav3v2_B.waypoints_data[(nav3v2_B.partialTrueCount +
        nav3v2_B.trueCount) - 1];
      nav3v2_B.overshootDist = nav3v2_B.b_dist - obj->LookaheadDistance;
      nav3v2_B.lookaheadIdx = obj->ProjectionLineIndex;
      while ((nav3v2_B.overshootDist < 0.0) && (nav3v2_B.lookaheadIdx <
              static_cast<real_T>(nav3v2_B.trueCount) - 1.0)) {
        nav3v2_B.lookaheadIdx++;
        nav3v2_B.partialTrueCount = static_cast<int32_T>(nav3v2_B.lookaheadIdx);
        nav3v2_B.d_i = static_cast<int32_T>(nav3v2_B.lookaheadIdx + 1.0);
        nav3v2_B.lookaheadStartPt[0] =
          nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount - 1];
        nav3v2_B.minDistance = nav3v2_B.waypoints_data[nav3v2_B.d_i - 1];
        nav3v2_B.obj[0] = nav3v2_B.waypoints_data[nav3v2_B.partialTrueCount - 1]
          - nav3v2_B.waypoints_data[nav3v2_B.d_i - 1];
        nav3v2_B.lookaheadStartPt[1] = nav3v2_B.waypoints_data
          [(nav3v2_B.partialTrueCount + nav3v2_B.trueCount) - 1];
        nav3v2_B.dist = nav3v2_B.waypoints_data[(nav3v2_B.d_i +
          nav3v2_B.trueCount) - 1];
        nav3v2_B.obj[1] = nav3v2_B.waypoints_data[(nav3v2_B.partialTrueCount +
          nav3v2_B.trueCount) - 1] - nav3v2_B.waypoints_data[(nav3v2_B.d_i +
          nav3v2_B.trueCount) - 1];
        nav3v2_B.b_dist += nav3v2_norm(nav3v2_B.obj);
        nav3v2_B.overshootDist = nav3v2_B.b_dist - obj->LookaheadDistance;
      }

      nav3v2_B.obj[0] = nav3v2_B.lookaheadStartPt[0] - nav3v2_B.minDistance;
      nav3v2_B.obj[1] = nav3v2_B.lookaheadStartPt[1] - nav3v2_B.dist;
      nav3v2_B.b_dist = nav3v2_B.overshootDist / nav3v2_norm(nav3v2_B.obj);
      if (nav3v2_B.b_dist > 0.0) {
        nav3v2_B.minDistance = (1.0 - nav3v2_B.b_dist) * nav3v2_B.minDistance +
          nav3v2_B.b_dist * nav3v2_B.lookaheadStartPt[0];
        nav3v2_B.dist = (1.0 - nav3v2_B.b_dist) * nav3v2_B.dist +
          nav3v2_B.b_dist * nav3v2_B.lookaheadStartPt[1];
      }
    }

    obj->LookaheadPoint[0] = nav3v2_B.minDistance;
    obj->LookaheadPoint[1] = nav3v2_B.dist;
    nav3v2_B.minDistance = nav3v2_rt_atan2d_snf(obj->LookaheadPoint[1] -
      curPose[1], obj->LookaheadPoint[0] - curPose[0]) - curPose[2];
    if (fabs(nav3v2_B.minDistance) > 3.1415926535897931) {
      if (rtIsNaN(nav3v2_B.minDistance + 3.1415926535897931) || rtIsInf
          (nav3v2_B.minDistance + 3.1415926535897931)) {
        nav3v2_B.dist = (rtNaN);
      } else if (nav3v2_B.minDistance + 3.1415926535897931 == 0.0) {
        nav3v2_B.dist = 0.0;
      } else {
        nav3v2_B.dist = fmod(nav3v2_B.minDistance + 3.1415926535897931,
                             6.2831853071795862);
        nav3v2_B.searchFlag = (nav3v2_B.dist == 0.0);
        if (!nav3v2_B.searchFlag) {
          nav3v2_B.b_dist = fabs((nav3v2_B.minDistance + 3.1415926535897931) /
            6.2831853071795862);
          nav3v2_B.searchFlag = !(fabs(nav3v2_B.b_dist - floor(nav3v2_B.b_dist +
            0.5)) > 2.2204460492503131E-16 * nav3v2_B.b_dist);
        }

        if (nav3v2_B.searchFlag) {
          nav3v2_B.dist = 0.0;
        } else {
          if (nav3v2_B.minDistance + 3.1415926535897931 < 0.0) {
            nav3v2_B.dist += 6.2831853071795862;
          }
        }
      }

      if ((nav3v2_B.dist == 0.0) && (nav3v2_B.minDistance + 3.1415926535897931 >
           0.0)) {
        nav3v2_B.dist = 6.2831853071795862;
      }

      nav3v2_B.minDistance = nav3v2_B.dist - 3.1415926535897931;
    }

    *w = 2.0 * sin(nav3v2_B.minDistance) / obj->LookaheadDistance;
    if (fabs(fabs(nav3v2_B.minDistance) - 3.1415926535897931) <
        1.4901161193847656E-8) {
      if (*w < 0.0) {
        *w = -1.0;
      } else if (*w > 0.0) {
        *w = 1.0;
      } else if (*w == 0.0) {
        *w = 0.0;
      } else {
        *w = (rtNaN);
      }
    }

    if (fabs(*w) > obj->MaxAngularVelocity) {
      if (*w < 0.0) {
        nav3v2_B.minDistance = -1.0;
      } else if (*w > 0.0) {
        nav3v2_B.minDistance = 1.0;
      } else if (*w == 0.0) {
        nav3v2_B.minDistance = 0.0;
      } else {
        nav3v2_B.minDistance = (rtNaN);
      }

      *w = nav3v2_B.minDistance * obj->MaxAngularVelocity;
    }

    *v = obj->DesiredLinearVelocity;
    lookaheadPoint[0] = obj->LookaheadPoint[0];
    lookaheadPoint[1] = obj->LookaheadPoint[1];
    obj->LastPose[0] = curPose[0];
    obj->LastPose[1] = curPose[1];
    obj->LastPose[2] = curPose[2];
  }
}

/* Function for Chart: '<S2>/State_Machine' */
static void nav3v2_purePursuit(real_T lookAhead, real_T maxLinVel, real_T
  maxAngVel, const real_T waypoints[6], const real_T pose[3], real_T
  targetHeading[3])
{
  nav3v2_B.controller.tunablePropertyChanged[0] = false;
  nav3v2_B.controller.tunablePropertyChanged[1] = false;
  nav3v2_B.controller.tunablePropertyChanged[2] = false;
  nav3v2_B.controller.tunablePropertyChanged[3] = false;
  for (nav3v2_B.i2 = 0; nav3v2_B.i2 < 6; nav3v2_B.i2++) {
    nav3v2_B.controller.Waypoints[nav3v2_B.i2] = waypoints[nav3v2_B.i2];
  }

  nav3v2_B.controller.LookaheadDistance = lookAhead;
  nav3v2_B.controller.MaxAngularVelocity = maxAngVel;
  nav3v2_B.controller.DesiredLinearVelocity = maxLinVel;
  nav3v2_B.controller.isInitialized = 1;
  nav3v2_B.controller.TunablePropsChanged = false;
  nav3v2_B.controller.ProjectionPoint[0] = (rtNaN);
  nav3v2_B.controller.LookaheadPoint[0] = 0.0;
  nav3v2_B.controller.ProjectionPoint[1] = (rtNaN);
  nav3v2_B.controller.LookaheadPoint[1] = 0.0;
  nav3v2_B.controller.LastPose[0] = 0.0;
  nav3v2_B.controller.LastPose[1] = 0.0;
  nav3v2_B.controller.LastPose[2] = 0.0;
  nav3v2_B.controller.ProjectionLineIndex = 0.0;
  controllerPurePursuit_stepImpl(&nav3v2_B.controller, pose,
    &nav3v2_B.varargout_1, &nav3v2_B.varargout_2, nav3v2_B.varargout_3);
  targetHeading[0] = nav3v2_rt_atan2d_snf(nav3v2_B.varargout_3[1] - pose[1],
    nav3v2_B.varargout_3[0] - pose[0]);
  targetHeading[1] = nav3v2_B.varargout_3[0];
  targetHeading[2] = nav3v2_B.varargout_3[1];
}

/* Function for Chart: '<S2>/State_Machine' */
static void exit_internal_AUTONOMOUS_CONTRO(void)
{
  switch (nav3v2_DW.is_AUTONOMOUS_CONTROL) {
   case nav3_IN_PreciseRotateToWaypoint:
    nav3v2_B.angularVel = 0.0;
    nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_NO_ACTIVE_CHILD;
    break;

   case na_IN_ReverseRotateToWaypoint_o:
    nav3v2_B.angularVel = 0.0;
    nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_NO_ACTIVE_CHILD;
    break;

   case nav3v2_IN_RotateToWaypoint:
    nav3v2_B.angularVel = 0.0;
    nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_NO_ACTIVE_CHILD;
    break;

   default:
    nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_NO_ACTIVE_CHILD;
    break;
  }
}

/* Function for Chart: '<S2>/State_Machine' */
static void nav3v2_AUTONOMOUS_CONTROL(void)
{
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  if (nav3v2_B.In1_i.Buttons[1] == 1) {
    exit_internal_AUTONOMOUS_CONTRO();
    nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_EMERGENCY;
    nav3v2_B.currentState = nav3v2_DW.EMERGENCY_STATE;
  } else if ((nav3v2_B.In1_i.Buttons[5] == 1) || (nav3v2_DW.finishedMission ==
              1.0)) {
    exit_internal_AUTONOMOUS_CONTRO();
    nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_WAITING;
    nav3v2_B.currentState = nav3v2_DW.WAITING_STATE;
  } else {
    guard1 = false;
    guard2 = false;
    switch (nav3v2_DW.is_AUTONOMOUS_CONTROL) {
     case IN_FinishedReachingAllWaypoin_g:
      break;

     case nav3v2_IN_PreciseMoveToWaypoint:
      nav3v2_B.pose[0] = nav3v2_B.Switch[0];
      nav3v2_B.pose[1] = nav3v2_B.Switch[1];
      nav3v2_B.pose[2] = nav3v2_B.Switch[2];
      nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
      nav3v2_B.goalWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
      nav3v2_B.goalWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
      nav3v2_B.goalTolerance = nav3v2_DW.WAYPOINT_PRECISION;

      /* Outputs for Function Call SubSystem: '<S19>/checkAtGoal' */
      nav3v2_checkAtGoal(nav3v2_B.pose, nav3v2_B.goalWaypoint,
                         nav3v2_B.goalTolerance, &nav3v2_B.checkAtGoal);

      /* End of Outputs for SubSystem: '<S19>/checkAtGoal' */
      if (!nav3v2_B.checkAtGoal.LessThan) {
        nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_ReachedWaypoint_b;
        if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483643) {
          nav3v2_B.currentState = MAX_int32_T;
        } else {
          nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 4;
        }

        nav3v2_B.ledState = nav3v2_DW.LED_WAYPOINT_REACHED_STATE;
      } else {
        nav3v2_B.dv1[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
        nav3v2_B.dv1[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
        nav3v2_DW.currentGoalHeading = nav3v2_CalculateHeading(nav3v2_B.Switch,
          nav3v2_B.dv1);
        nav3v2_B.linearVel = nav3v2_DW.MAX_LINEAR_VEL / 2.0;
        nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
        nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
        nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
        nav3v2_B.desiredHeading = nav3v2_DW.currentGoalHeading;
        nav3v2_B.P = nav3v2_DW.P;
        nav3v2_B.I = nav3v2_DW.I;
        nav3v2_B.D = nav3v2_DW.D;

        /* Outputs for Function Call SubSystem: '<S19>/PID' */
        nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                   nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                   &nav3v2_DW.PID, &nav3v2_P.PID);

        /* End of Outputs for SubSystem: '<S19>/PID' */
        nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
        nav3v2_DW.goalHeading[0] = nav3v2_DW.currentGoalHeading;
        nav3v2_DW.goalHeading[1] = 0.0;
        nav3v2_DW.goalHeading[2] = 0.0;
      }
      break;

     case nav3_IN_PreciseRotateToWaypoint:
      if ((nav3v2_DW.currentGoalHeading - 0.01 < nav3v2_B.Switch[2]) &&
          (nav3v2_B.Switch[2] < nav3v2_DW.currentGoalHeading + 0.01)) {
        nav3v2_B.angularVel = 0.0;
        nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_PreciseMoveToWaypoint;
        if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483644) {
          nav3v2_B.currentState = MAX_int32_T;
        } else {
          nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 3;
        }
      } else {
        nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
        nav3v2_B.dv1[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
        nav3v2_B.dv1[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
        nav3v2_DW.currentGoalHeading = nav3v2_CalculateHeading(nav3v2_B.Switch,
          nav3v2_B.dv1);
        nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
        nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
        nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
        nav3v2_B.desiredHeading = nav3v2_DW.currentGoalHeading;
        nav3v2_B.P = nav3v2_DW.P;
        nav3v2_B.I = nav3v2_DW.I;
        nav3v2_B.D = nav3v2_DW.D;

        /* Outputs for Function Call SubSystem: '<S19>/PID' */
        nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                   nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                   &nav3v2_DW.PID, &nav3v2_P.PID);

        /* End of Outputs for SubSystem: '<S19>/PID' */
        nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
        nav3v2_DW.goalHeading[0] = nav3v2_DW.currentGoalHeading;
        nav3v2_DW.goalHeading[1] = 0.0;
        nav3v2_DW.goalHeading[2] = 0.0;
      }
      break;

     case na_IN_PurePursuitMoveToWaypoint:
      nav3v2_B.pose[0] = nav3v2_B.Switch[0];
      nav3v2_B.pose[1] = nav3v2_B.Switch[1];
      nav3v2_B.pose[2] = nav3v2_B.Switch[2];
      nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
      nav3v2_B.goalWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
      nav3v2_B.goalWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
      nav3v2_B.goalTolerance = nav3v2_DW.LOOKAHEAD_DISTANCE;

      /* Outputs for Function Call SubSystem: '<S19>/checkAtGoal' */
      nav3v2_checkAtGoal(nav3v2_B.pose, nav3v2_B.goalWaypoint,
                         nav3v2_B.goalTolerance, &nav3v2_B.checkAtGoal);

      /* End of Outputs for SubSystem: '<S19>/checkAtGoal' */
      if (!nav3v2_B.checkAtGoal.LessThan) {
        nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_ReachedWaypoint_b;
        if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483643) {
          nav3v2_B.currentState = MAX_int32_T;
        } else {
          nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 4;
        }

        nav3v2_B.ledState = nav3v2_DW.LED_WAYPOINT_REACHED_STATE;
      } else {
        nav3v2_purePursuit(nav3v2_DW.LOOKAHEAD_DISTANCE,
                           nav3v2_DW.MAX_LINEAR_VEL, nav3v2_DW.MAX_ANGULAR_VEL,
                           nav3v2_DW.waypointLine, nav3v2_B.Switch,
                           nav3v2_DW.goalHeading);
        nav3v2_B.linearVel = nav3v2_DW.MAX_LINEAR_VEL;
        nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
        nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
        nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
        nav3v2_B.desiredHeading = nav3v2_DW.goalHeading[0];
        nav3v2_B.P = nav3v2_DW.P;
        nav3v2_B.I = nav3v2_DW.I;
        nav3v2_B.D = nav3v2_DW.D;

        /* Outputs for Function Call SubSystem: '<S19>/PID' */
        nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                   nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                   &nav3v2_DW.PID, &nav3v2_P.PID);

        /* End of Outputs for SubSystem: '<S19>/PID' */
        nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
        nav3v2_B.currentWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
        nav3v2_B.currentWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
      }
      break;

     case nav3v2_IN_ReachedWaypoint_b:
      if (nav3v2_DW.count + 1.0 < nav3v2_DW.last) {
        nav3v2_DW.count++;
        nav3v2_B.ledState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE;
        nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0) - 1;
        if (nav3v2_DW.all_waypoints_config[nav3v2_B.i1] == 2.0) {
          nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3_IN_PreciseRotateToWaypoint;
          if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483646) {
            nav3v2_B.currentState = MAX_int32_T;
          } else {
            nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 1;
          }

          nav3v2_B.linearVel = 0.0;
          nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
          nav3v2_B.currentWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
          nav3v2_B.currentWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
        } else if (nav3v2_DW.all_waypoints_config[nav3v2_B.i1] == 1.0) {
          for (nav3v2_B.i1 = 0; nav3v2_B.i1 < 2; nav3v2_B.i1++) {
            nav3v2_DW.waypointLine[3 * nav3v2_B.i1] = nav3v2_DW.all_waypoints[(5
              * nav3v2_B.i1 + static_cast<int32_T>(nav3v2_DW.count)) - 1];
            nav3v2_DW.waypointLine[3 * nav3v2_B.i1 + 1] =
              nav3v2_DW.all_waypoints[(static_cast<int32_T>(nav3v2_DW.count +
              1.0) + 5 * nav3v2_B.i1) - 1];
            nav3v2_DW.waypointLine[3 * nav3v2_B.i1 + 2] =
              nav3v2_DW.all_waypoints[(static_cast<int32_T>(nav3v2_DW.count +
              2.0) + 5 * nav3v2_B.i1) - 1];
          }

          nav3v2_DW.is_AUTONOMOUS_CONTROL = na_IN_PurePursuitMoveToWaypoint;
          if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483645) {
            nav3v2_B.currentState = MAX_int32_T;
          } else {
            nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 2;
          }
        } else if (nav3v2_DW.all_waypoints_config[nav3v2_B.i1] == 3.0) {
          nav3v2_DW.is_AUTONOMOUS_CONTROL = na_IN_ReverseRotateToWaypoint_o;
          if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483642) {
            nav3v2_B.currentState = MAX_int32_T;
          } else {
            nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 5;
          }

          nav3v2_B.linearVel = 0.0;
          nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
          nav3v2_B.i_nv = static_cast<int32_T>(nav3v2_DW.count + 1.0);
          nav3v2_B.dv1[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
          nav3v2_B.currentWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i_nv -
            1];
          nav3v2_B.dv1[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
          nav3v2_B.currentWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i_nv +
            4];
          nav3v2_DW.currentGoalHeading = nav3v2_CalculateReverseHeading
            (nav3v2_B.Switch, nav3v2_B.dv1);
        } else {
          guard2 = true;
        }
      } else {
        guard2 = true;
      }
      break;

     case nav3_IN_ReverseMoveToWaypoint_a:
      nav3v2_B.pose[0] = nav3v2_B.Switch[0];
      nav3v2_B.pose[1] = nav3v2_B.Switch[1];
      nav3v2_B.pose[2] = nav3v2_B.Switch[2];
      nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
      nav3v2_B.goalWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
      nav3v2_B.goalWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
      nav3v2_B.goalTolerance = nav3v2_DW.WAYPOINT_PRECISION;

      /* Outputs for Function Call SubSystem: '<S19>/checkAtGoal' */
      nav3v2_checkAtGoal(nav3v2_B.pose, nav3v2_B.goalWaypoint,
                         nav3v2_B.goalTolerance, &nav3v2_B.checkAtGoal);

      /* End of Outputs for SubSystem: '<S19>/checkAtGoal' */
      if (!nav3v2_B.checkAtGoal.LessThan) {
        nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_ReachedWaypoint_b;
        if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483643) {
          nav3v2_B.currentState = MAX_int32_T;
        } else {
          nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 4;
        }

        nav3v2_B.ledState = nav3v2_DW.LED_WAYPOINT_REACHED_STATE;
      } else {
        nav3v2_B.dv1[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
        nav3v2_B.dv1[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
        nav3v2_DW.currentGoalHeading = nav3v2_CalculateReverseHeading
          (nav3v2_B.Switch, nav3v2_B.dv1);
        nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
        nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
        nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
        nav3v2_B.desiredHeading = nav3v2_DW.currentGoalHeading;
        nav3v2_B.P = nav3v2_DW.P;
        nav3v2_B.I = nav3v2_DW.I;
        nav3v2_B.D = nav3v2_DW.D;

        /* Outputs for Function Call SubSystem: '<S19>/PID' */
        nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                   nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                   &nav3v2_DW.PID, &nav3v2_P.PID);

        /* End of Outputs for SubSystem: '<S19>/PID' */
        nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
      }
      break;

     case na_IN_ReverseRotateToWaypoint_o:
      if ((nav3v2_DW.currentGoalHeading - 0.01 < nav3v2_B.Switch[2]) &&
          (nav3v2_B.Switch[2] < nav3v2_DW.currentGoalHeading + 0.01)) {
        nav3v2_B.angularVel = 0.0;
        nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3_IN_ReverseMoveToWaypoint_a;
        if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483641) {
          nav3v2_B.currentState = MAX_int32_T;
        } else {
          nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 6;
        }

        nav3v2_B.linearVel = -nav3v2_DW.MAX_LINEAR_VEL / 2.0;
      } else {
        nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
        nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
        nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
        nav3v2_B.desiredHeading = nav3v2_DW.currentGoalHeading;
        nav3v2_B.P = nav3v2_DW.P;
        nav3v2_B.I = nav3v2_DW.I;
        nav3v2_B.D = nav3v2_DW.D;

        /* Outputs for Function Call SubSystem: '<S19>/PID' */
        nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                   nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                   &nav3v2_DW.PID, &nav3v2_P.PID);

        /* End of Outputs for SubSystem: '<S19>/PID' */
        nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
        nav3v2_DW.goalHeading[0] = nav3v2_DW.currentGoalHeading;
        nav3v2_DW.goalHeading[1] = 0.0;
        nav3v2_DW.goalHeading[2] = 0.0;
      }
      break;

     default:
      /* case IN_RotateToWaypoint: */
      if ((nav3v2_DW.currentGoalHeading - 0.01 < nav3v2_B.Switch[2]) &&
          (nav3v2_B.Switch[2] < nav3v2_DW.currentGoalHeading + 0.01)) {
        nav3v2_B.i1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
        nav3v2_B.i_nv = nav3v2_B.i1 - 1;
        if (nav3v2_DW.all_waypoints_config[nav3v2_B.i_nv] == 2.0) {
          nav3v2_B.angularVel = 0.0;
          nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3_IN_PreciseRotateToWaypoint;
          if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483646) {
            nav3v2_B.currentState = MAX_int32_T;
          } else {
            nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 1;
          }

          nav3v2_B.linearVel = 0.0;
          nav3v2_B.currentWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
          nav3v2_B.currentWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
        } else if (nav3v2_DW.all_waypoints_config[nav3v2_B.i_nv] == 1.0) {
          for (nav3v2_B.i_nv = 0; nav3v2_B.i_nv < 2; nav3v2_B.i_nv++) {
            nav3v2_DW.waypointLine[3 * nav3v2_B.i_nv] = nav3v2_DW.all_waypoints
              [(5 * nav3v2_B.i_nv + static_cast<int32_T>(nav3v2_DW.count)) - 1];
            nav3v2_DW.waypointLine[3 * nav3v2_B.i_nv + 1] =
              nav3v2_DW.all_waypoints[(nav3v2_B.i1 + 5 * nav3v2_B.i_nv) - 1];
            nav3v2_DW.waypointLine[3 * nav3v2_B.i_nv + 2] =
              nav3v2_DW.all_waypoints[(static_cast<int32_T>(nav3v2_DW.count +
              2.0) + 5 * nav3v2_B.i_nv) - 1];
          }

          nav3v2_B.angularVel = 0.0;
          nav3v2_DW.is_AUTONOMOUS_CONTROL = na_IN_PurePursuitMoveToWaypoint;
          if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483645) {
            nav3v2_B.currentState = MAX_int32_T;
          } else {
            nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 2;
          }
        } else if (nav3v2_DW.all_waypoints_config[nav3v2_B.i_nv] == 3.0) {
          nav3v2_B.angularVel = 0.0;
          nav3v2_DW.is_AUTONOMOUS_CONTROL = na_IN_ReverseRotateToWaypoint_o;
          if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483642) {
            nav3v2_B.currentState = MAX_int32_T;
          } else {
            nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 5;
          }

          nav3v2_B.linearVel = 0.0;
          nav3v2_B.dv1[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
          nav3v2_B.currentWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.i1 - 1];
          nav3v2_B.dv1[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
          nav3v2_B.currentWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.i1 + 4];
          nav3v2_DW.currentGoalHeading = nav3v2_CalculateReverseHeading
            (nav3v2_B.Switch, nav3v2_B.dv1);
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      break;
    }

    if (guard2) {
      if (nav3v2_DW.count + 1.0 == nav3v2_DW.last) {
        nav3v2_DW.is_AUTONOMOUS_CONTROL = IN_FinishedReachingAllWaypoin_g;
        nav3v2_B.ledState = nav3v2_DW.LED_WAYPOINT_REACHED_STATE;
        nav3v2_DW.finishedMission = 1.0;
      }
    }

    if (guard1) {
      nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
      nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
      nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
      nav3v2_B.desiredHeading = nav3v2_DW.goalHeading[0];
      nav3v2_B.P = nav3v2_DW.P;
      nav3v2_B.I = nav3v2_DW.I;
      nav3v2_B.D = nav3v2_DW.D;

      /* Outputs for Function Call SubSystem: '<S19>/PID' */
      nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading, nav3v2_B.P,
                 nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID, &nav3v2_DW.PID,
                 &nav3v2_P.PID);

      /* End of Outputs for SubSystem: '<S19>/PID' */
      nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
    }
  }
}

static void matlabCodegenHandle_mat_ikxi5sl(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabC_ikx(ros_slros_internal_block_Subs_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void nav3v2_matlabCodegenHa_f0(ros_slros_internal_block_SetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* Model step function */
void nav3v2_step(void)
{
  boolean_T exitg1;

  /* DataStoreWrite: '<S84>/Data Store Write' incorporates:
   *  MATLABSystem: '<S84>/Get Parameter1'
   */
  ParamGet_nav3v2_242.get_parameter(&nav3v2_DW.LOOKAHEAD_DISTANCE);

  /* DataStoreWrite: '<S84>/Data Store Write1' incorporates:
   *  MATLABSystem: '<S84>/Get Parameter2'
   */
  ParamGet_nav3v2_274.get_parameter(&nav3v2_DW.D);

  /* DataStoreWrite: '<S84>/Data Store Write2' incorporates:
   *  MATLABSystem: '<S84>/Get Parameter3'
   */
  ParamGet_nav3v2_275.get_parameter(&nav3v2_DW.I);

  /* DataStoreWrite: '<S84>/Data Store Write3' incorporates:
   *  MATLABSystem: '<S84>/Get Parameter4'
   */
  ParamGet_nav3v2_276.get_parameter(&nav3v2_DW.P);

  /* DataStoreWrite: '<S85>/Data Store Write' incorporates:
   *  MATLABSystem: '<S85>/Get Parameter6'
   */
  ParamGet_nav3v2_541.get_parameter(&nav3v2_DW.WAYPOINT_PRECISION);

  /* DataStoreWrite: '<S85>/Data Store Write1' incorporates:
   *  MATLABSystem: '<S85>/Get Parameter5'
   */
  ParamGet_nav3v2_368.get_parameter(&nav3v2_DW.MAX_LINEAR_VEL);

  /* MATLABSystem: '<S85>/Get Parameter2' */
  ParamGet_nav3v2_326.get_parameter(&nav3v2_B.quat_idx_3);

  /* MATLABSystem: '<S85>/Get Parameter3' */
  ParamGet_nav3v2_327.get_parameter(&nav3v2_B.K12);

  /* MATLABSystem: '<S85>/Get Parameter1' */
  ParamGet_nav3v2_325.get_parameter(&nav3v2_B.Product);

  /* DataStoreWrite: '<S85>/Data Store Write2' incorporates:
   *  MATLABSystem: '<S85>/Get Parameter1'
   *  MATLABSystem: '<S85>/Get Parameter2'
   *  MATLABSystem: '<S85>/Get Parameter3'
   */
  nav3v2_DW.InitialPose[0] = nav3v2_B.quat_idx_3;
  nav3v2_DW.InitialPose[1] = nav3v2_B.K12;
  nav3v2_DW.InitialPose[2] = nav3v2_B.Product;

  /* DataStoreWrite: '<S85>/Data Store Write3' incorporates:
   *  MATLABSystem: '<S85>/Get Parameter4'
   */
  ParamGet_nav3v2_365.get_parameter(&nav3v2_DW.MAX_ANGULAR_VEL);

  /* MATLAB Function: '<S85>/MATLAB Function' incorporates:
   *  MATLABSystem: '<S85>/Get Parameter1'
   */
  nav3v2_B.K34 = sin(nav3v2_B.Product);
  nav3v2_B.quat_idx_2 = cos(nav3v2_B.Product);

  /* MATLABSystem: '<S85>/Get Parameter' */
  ParamGet_nav3v2_237.get_parameter(&nav3v2_B.Product);

  /* DataStoreWrite: '<S87>/Data Store Write' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter4'
   */
  ParamGet_nav3v2_295.get_parameter(&nav3v2_DW.AUTONOMOUS_CONTROL_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write1' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter3'
   */
  ParamGet_nav3v2_294.get_parameter(&nav3v2_DW.EMERGENCY_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write10' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter11'
   */
  ParamGet_nav3v2_628.get_parameter(&nav3v2_DW.LED_MISSION_PAUSED_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write11' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter12'
   */
  ParamGet_nav3v2_629.get_parameter(&nav3v2_DW.LED_STARTING_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write12' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter13'
   */
  ParamGet_nav3v2_636.get_parameter(&nav3v2_DW.LED_WAYPOINT_REACHED_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write2' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter2'
   */
  ParamGet_nav3v2_293.get_parameter(&nav3v2_DW.INITIALISATION_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write3' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter1'
   */
  ParamGet_nav3v2_292.get_parameter(&nav3v2_DW.MANUAL_CONTROL_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write4' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter5'
   */
  ParamGet_nav3v2_303.get_parameter(&nav3v2_DW.REVERSE_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write5' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter6'
   */
  ParamGet_nav3v2_306.get_parameter(&nav3v2_DW.WAITING_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write6' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter10'
   */
  ParamGet_nav3v2_627.get_parameter(&nav3v2_DW.LED_BATTERY_LOW_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write7' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter9'
   */
  ParamGet_nav3v2_626.get_parameter(&nav3v2_DW.LED_EMERGENCY_STOP_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write8' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter8'
   */
  ParamGet_nav3v2_625.get_parameter(&nav3v2_DW.LED_ERROR_STATE);

  /* DataStoreWrite: '<S87>/Data Store Write9' incorporates:
   *  MATLABSystem: '<S87>/Get Parameter7'
   */
  ParamGet_nav3v2_624.get_parameter(&nav3v2_DW.LED_MISSION_COMPLETE_STATE);

  /* MATLABSystem: '<S86>/Get Parameter' */
  ParamGet_nav3v2_340.get_parameter(&nav3v2_B.Product);

  /* MATLABSystem: '<S86>/Get Parameter1' */
  ParamGet_nav3v2_341.get_parameter(&nav3v2_B.Product);

  /* MATLABSystem: '<S86>/Get Parameter2' */
  ParamGet_nav3v2_342.get_parameter(&nav3v2_B.value);

  /* MATLAB Function: '<S86>/MATLAB Function' incorporates:
   *  MATLABSystem: '<S86>/Get Parameter3'
   *  MATLABSystem: '<S86>/Get Parameter4'
   */
  ParamGet_nav3v2_357.get_parameter(&nav3v2_B.Lidar2Robot[7]);
  ParamGet_nav3v2_358.get_parameter(&nav3v2_B.Lidar2Robot[6]);

  /* MATLABSystem: '<S86>/Get Parameter5' */
  ParamGet_nav3v2_359.get_parameter(&nav3v2_B.value_p);

  /* MATLAB Function: '<S86>/MATLAB Function' incorporates:
   *  MATLABSystem: '<S86>/Get Parameter5'
   */
  nav3v2_B.rtb_Lidar2Robot_tmp = sin(nav3v2_B.value_p);
  nav3v2_B.value_p = cos(nav3v2_B.value_p);
  nav3v2_B.Lidar2Robot[0] = nav3v2_B.value_p;
  nav3v2_B.Lidar2Robot[3] = -nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.Lidar2Robot[1] = nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.Lidar2Robot[4] = nav3v2_B.value_p;
  nav3v2_B.Lidar2Robot[2] = 0.0;
  nav3v2_B.Lidar2Robot[5] = 0.0;
  nav3v2_B.Lidar2Robot[8] = 1.0;

  /* MATLABSystem: '<S88>/Get Parameter5' */
  ParamGet_nav3v2_381.get_parameter(&nav3v2_B.value_j);

  /* Outputs for Atomic SubSystem: '<S94>/Live_Subscriber' */
  /* MATLABSystem: '<S108>/SourceBlock' incorporates:
   *  Inport: '<S111>/In1'
   */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_202.getLatestMessage
    (&nav3v2_B.b_varargout_2);

  /* Outputs for Enabled SubSystem: '<S108>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S111>/Enable'
   */
  if (nav3v2_B.b_varargout_1) {
    nav3v2_B.In1 = nav3v2_B.b_varargout_2;
  }

  /* End of MATLABSystem: '<S108>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S108>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S94>/Live_Subscriber' */

  /* MATLAB Function: '<S107>/Quaternion_2_Euler' */
  nav3v2_Quaternion_2_Euler(nav3v2_B.In1.Pose.Pose.Orientation.X,
    nav3v2_B.In1.Pose.Pose.Orientation.Y, nav3v2_B.In1.Pose.Pose.Orientation.Z,
    nav3v2_B.In1.Pose.Pose.Orientation.W, &nav3v2_B.sf_Quaternion_2_Euler_o);

  /* MATLAB Function: '<S107>/World to Robot Transform' incorporates:
   *  MATLAB Function: '<S85>/MATLAB Function'
   *  MATLAB Function: '<S86>/MATLAB Function'
   *  MATLABSystem: '<S85>/Get Parameter2'
   *  MATLABSystem: '<S85>/Get Parameter3'
   *  SignalConversion generated from: '<S110>/ SFunction '
   */
  memcpy(&nav3v2_B.World2RobotOdom[0], &nav3v2_B.Lidar2Robot[0], 9U * sizeof
         (real_T));
  nav3v2_B.r1 = 0;
  nav3v2_B.r2 = 1;
  nav3v2_B.r3 = 2;
  if (fabs(nav3v2_B.rtb_Lidar2Robot_tmp) > fabs(nav3v2_B.value_p)) {
    nav3v2_B.r1 = 1;
    nav3v2_B.r2 = 0;
  }

  nav3v2_B.World2RobotOdom[nav3v2_B.r2] = nav3v2_B.Lidar2Robot[nav3v2_B.r2] /
    nav3v2_B.Lidar2Robot[nav3v2_B.r1];
  nav3v2_B.World2RobotOdom[2] /= nav3v2_B.World2RobotOdom[nav3v2_B.r1];
  nav3v2_B.World2RobotOdom[nav3v2_B.r2 + 3] -=
    nav3v2_B.World2RobotOdom[nav3v2_B.r1 + 3] *
    nav3v2_B.World2RobotOdom[nav3v2_B.r2];
  nav3v2_B.World2RobotOdom[5] -= nav3v2_B.World2RobotOdom[nav3v2_B.r1 + 3] *
    nav3v2_B.World2RobotOdom[2];
  nav3v2_B.World2RobotOdom[nav3v2_B.r2 + 6] -=
    nav3v2_B.World2RobotOdom[nav3v2_B.r1 + 6] *
    nav3v2_B.World2RobotOdom[nav3v2_B.r2];
  nav3v2_B.World2RobotOdom[8] -= nav3v2_B.World2RobotOdom[nav3v2_B.r1 + 6] *
    nav3v2_B.World2RobotOdom[2];
  if (fabs(nav3v2_B.World2RobotOdom[5]) > fabs
      (nav3v2_B.World2RobotOdom[nav3v2_B.r2 + 3])) {
    nav3v2_B.r3 = nav3v2_B.r2;
    nav3v2_B.r2 = 2;
  }

  nav3v2_B.World2RobotOdom[nav3v2_B.r3 + 3] /=
    nav3v2_B.World2RobotOdom[nav3v2_B.r2 + 3];
  nav3v2_B.World2RobotOdom[nav3v2_B.r3 + 6] -=
    nav3v2_B.World2RobotOdom[nav3v2_B.r3 + 3] *
    nav3v2_B.World2RobotOdom[nav3v2_B.r2 + 6];
  nav3v2_B.rtb_Lidar2Robot_tmp = sin(nav3v2_B.sf_Quaternion_2_Euler_o.theta);
  nav3v2_B.value_p = cos(nav3v2_B.sf_Quaternion_2_Euler_o.theta);
  nav3v2_B.W2RTransform_tmp[0] = nav3v2_B.value_p;
  nav3v2_B.W2RTransform_tmp[3] = -nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.W2RTransform_tmp[6] = nav3v2_B.In1.Pose.Pose.Position.X;
  nav3v2_B.W2RTransform_tmp[1] = nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.W2RTransform_tmp[4] = nav3v2_B.value_p;
  nav3v2_B.W2RTransform_tmp[7] = nav3v2_B.In1.Pose.Pose.Position.Y;
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r1] = nav3v2_B.quat_idx_2 /
    nav3v2_B.World2RobotOdom[nav3v2_B.r1];
  nav3v2_B.rtb_Lidar2Robot_tmp = nav3v2_B.World2RobotOdom[nav3v2_B.r1 + 3];
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r2] = -nav3v2_B.K34 -
    nav3v2_B.W2LOTransform[3 * nav3v2_B.r1] * nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.value_p = nav3v2_B.World2RobotOdom[nav3v2_B.r1 + 6];
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r3] = nav3v2_B.quat_idx_3 -
    nav3v2_B.W2LOTransform[3 * nav3v2_B.r1] * nav3v2_B.value_p;
  nav3v2_B.quat_idx_3 = nav3v2_B.World2RobotOdom[nav3v2_B.r2 + 3];
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r2] /= nav3v2_B.quat_idx_3;
  nav3v2_B.W2LOTransform_tmp = nav3v2_B.World2RobotOdom[nav3v2_B.r2 + 6];
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r3] -= nav3v2_B.W2LOTransform[3 *
    nav3v2_B.r2] * nav3v2_B.W2LOTransform_tmp;
  nav3v2_B.W2LOTransform_tmp_c = nav3v2_B.World2RobotOdom[nav3v2_B.r3 + 6];
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r3] /= nav3v2_B.W2LOTransform_tmp_c;
  nav3v2_B.W2LOTransform_tmp_f = nav3v2_B.World2RobotOdom[nav3v2_B.r3 + 3];
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r2] -= nav3v2_B.W2LOTransform[3 *
    nav3v2_B.r3] * nav3v2_B.W2LOTransform_tmp_f;
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r1] -= nav3v2_B.W2LOTransform[3 *
    nav3v2_B.r3] * nav3v2_B.World2RobotOdom[nav3v2_B.r3];
  nav3v2_B.W2LOTransform[3 * nav3v2_B.r1] -= nav3v2_B.W2LOTransform[3 *
    nav3v2_B.r2] * nav3v2_B.World2RobotOdom[nav3v2_B.r2];
  nav3v2_B.W2RTransform_tmp[2] = 0.0;
  nav3v2_B.i = 3 * nav3v2_B.r1 + 1;
  nav3v2_B.W2LOTransform[nav3v2_B.i] = nav3v2_B.K34 /
    nav3v2_B.World2RobotOdom[nav3v2_B.r1];
  nav3v2_B.W2LOTransform_tmp_j = 3 * nav3v2_B.r2 + 1;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] = nav3v2_B.quat_idx_2 -
    nav3v2_B.W2LOTransform[nav3v2_B.i] * nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.W2LOTransform_tmp_o = 3 * nav3v2_B.r3 + 1;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] = nav3v2_B.K12 -
    nav3v2_B.W2LOTransform[nav3v2_B.i] * nav3v2_B.value_p;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] /= nav3v2_B.quat_idx_3;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] *
    nav3v2_B.W2LOTransform_tmp;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] /=
    nav3v2_B.W2LOTransform_tmp_c;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] *
    nav3v2_B.W2LOTransform_tmp_f;
  nav3v2_B.W2LOTransform[nav3v2_B.i] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] *
    nav3v2_B.World2RobotOdom[nav3v2_B.r3];
  nav3v2_B.W2LOTransform[nav3v2_B.i] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] *
    nav3v2_B.World2RobotOdom[nav3v2_B.r2];
  nav3v2_B.W2RTransform_tmp[5] = 0.0;
  nav3v2_B.i = 3 * nav3v2_B.r1 + 2;
  nav3v2_B.W2LOTransform[nav3v2_B.i] = 0.0 /
    nav3v2_B.World2RobotOdom[nav3v2_B.r1];
  nav3v2_B.W2LOTransform_tmp_j = 3 * nav3v2_B.r2 + 2;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] = 0.0 -
    nav3v2_B.W2LOTransform[nav3v2_B.i] * nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.W2LOTransform_tmp_o = 3 * nav3v2_B.r3 + 2;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] = 1.0 -
    nav3v2_B.W2LOTransform[nav3v2_B.i] * nav3v2_B.value_p;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] /= nav3v2_B.quat_idx_3;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] *
    nav3v2_B.W2LOTransform_tmp;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] /=
    nav3v2_B.W2LOTransform_tmp_c;
  nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] *
    nav3v2_B.W2LOTransform_tmp_f;
  nav3v2_B.W2LOTransform[nav3v2_B.i] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_o] *
    nav3v2_B.World2RobotOdom[nav3v2_B.r3];
  nav3v2_B.W2LOTransform[nav3v2_B.i] -=
    nav3v2_B.W2LOTransform[nav3v2_B.W2LOTransform_tmp_j] *
    nav3v2_B.World2RobotOdom[nav3v2_B.r2];
  nav3v2_B.W2RTransform_tmp[8] = 1.0;
  for (nav3v2_B.r1 = 0; nav3v2_B.r1 < 3; nav3v2_B.r1++) {
    for (nav3v2_B.r2 = 0; nav3v2_B.r2 < 3; nav3v2_B.r2++) {
      nav3v2_B.i = nav3v2_B.r1 + 3 * nav3v2_B.r2;
      nav3v2_B.W2LOTransform_c[nav3v2_B.i] = 0.0;
      nav3v2_B.W2LOTransform_c[nav3v2_B.i] += nav3v2_B.W2RTransform_tmp[3 *
        nav3v2_B.r2] * nav3v2_B.W2LOTransform[nav3v2_B.r1];
      nav3v2_B.W2LOTransform_c[nav3v2_B.i] += nav3v2_B.W2RTransform_tmp[3 *
        nav3v2_B.r2 + 1] * nav3v2_B.W2LOTransform[nav3v2_B.r1 + 3];
      nav3v2_B.W2LOTransform_c[nav3v2_B.i] += nav3v2_B.W2RTransform_tmp[3 *
        nav3v2_B.r2 + 2] * nav3v2_B.W2LOTransform[nav3v2_B.r1 + 6];
    }

    for (nav3v2_B.r2 = 0; nav3v2_B.r2 < 3; nav3v2_B.r2++) {
      nav3v2_B.i = nav3v2_B.r1 + 3 * nav3v2_B.r2;
      nav3v2_B.World2RobotOdom[nav3v2_B.i] = 0.0;
      nav3v2_B.World2RobotOdom[nav3v2_B.i] += nav3v2_B.Lidar2Robot[3 *
        nav3v2_B.r2] * nav3v2_B.W2LOTransform_c[nav3v2_B.r1];
      nav3v2_B.World2RobotOdom[nav3v2_B.i] += nav3v2_B.Lidar2Robot[3 *
        nav3v2_B.r2 + 1] * nav3v2_B.W2LOTransform_c[nav3v2_B.r1 + 3];
      nav3v2_B.World2RobotOdom[nav3v2_B.i] += nav3v2_B.Lidar2Robot[3 *
        nav3v2_B.r2 + 2] * nav3v2_B.W2LOTransform_c[nav3v2_B.r1 + 6];
    }
  }

  nav3v2_B.K12 = nav3v2_B.World2RobotOdom[3] + nav3v2_B.World2RobotOdom[1];
  nav3v2_B.K34 = nav3v2_B.World2RobotOdom[1] - nav3v2_B.World2RobotOdom[3];
  nav3v2_B.rotm[0] = ((nav3v2_B.World2RobotOdom[0] - nav3v2_B.World2RobotOdom[4])
                      - 1.0) / 3.0;
  nav3v2_B.rotm[4] = nav3v2_B.K12 / 3.0;
  nav3v2_B.rotm[8] = 0.0;
  nav3v2_B.rotm[12] = 0.0;
  nav3v2_B.rotm[1] = nav3v2_B.K12 / 3.0;
  nav3v2_B.rotm[5] = ((nav3v2_B.World2RobotOdom[4] - nav3v2_B.World2RobotOdom[0])
                      - 1.0) / 3.0;
  nav3v2_B.rotm[9] = 0.0;
  nav3v2_B.rotm[13] = 0.0;
  nav3v2_B.rotm[2] = 0.0;
  nav3v2_B.rotm[6] = 0.0;
  nav3v2_B.rotm[10] = ((1.0 - nav3v2_B.World2RobotOdom[0]) -
                       nav3v2_B.World2RobotOdom[4]) / 3.0;
  nav3v2_B.rotm[14] = nav3v2_B.K34 / 3.0;
  nav3v2_B.rotm[3] = 0.0;
  nav3v2_B.rotm[7] = 0.0;
  nav3v2_B.rotm[11] = nav3v2_B.K34 / 3.0;
  nav3v2_B.rotm[15] = ((nav3v2_B.World2RobotOdom[0] + nav3v2_B.World2RobotOdom[4])
                       + 1.0) / 3.0;
  nav3v2_eig(nav3v2_B.rotm, nav3v2_B.eigVec, nav3v2_B.eigVal);
  nav3v2_B.varargin_1[0] = nav3v2_B.eigVal[0].re;
  nav3v2_B.varargin_1[1] = nav3v2_B.eigVal[1].re;
  nav3v2_B.varargin_1[2] = nav3v2_B.eigVal[2].re;
  nav3v2_B.varargin_1[3] = nav3v2_B.eigVal[3].re;
  if (!rtIsNaN(nav3v2_B.eigVal[0].re)) {
    nav3v2_B.i = 1;
  } else {
    nav3v2_B.i = 0;
    nav3v2_B.r1 = 2;
    exitg1 = false;
    while ((!exitg1) && (nav3v2_B.r1 < 5)) {
      if (!rtIsNaN(nav3v2_B.varargin_1[nav3v2_B.r1 - 1])) {
        nav3v2_B.i = nav3v2_B.r1;
        exitg1 = true;
      } else {
        nav3v2_B.r1++;
      }
    }
  }

  if (nav3v2_B.i != 0) {
    nav3v2_B.K12 = nav3v2_B.varargin_1[nav3v2_B.i - 1];
    nav3v2_B.r1 = nav3v2_B.i - 1;
    while (nav3v2_B.i + 1 < 5) {
      if (nav3v2_B.K12 < nav3v2_B.varargin_1[nav3v2_B.i]) {
        nav3v2_B.K12 = nav3v2_B.varargin_1[nav3v2_B.i];
        nav3v2_B.r1 = nav3v2_B.i;
      }

      nav3v2_B.i++;
    }

    nav3v2_B.i = nav3v2_B.r1;
  }

  nav3v2_B.i <<= 2;
  nav3v2_B.K12 = nav3v2_B.eigVec[nav3v2_B.i + 3].re;
  nav3v2_B.K34 = nav3v2_B.eigVec[nav3v2_B.i].re;
  nav3v2_B.quat_idx_2 = nav3v2_B.eigVec[nav3v2_B.i + 1].re;
  nav3v2_B.quat_idx_3 = nav3v2_B.eigVec[nav3v2_B.i + 2].re;
  if (nav3v2_B.K12 < 0.0) {
    nav3v2_B.K12 = -nav3v2_B.K12;
    nav3v2_B.K34 = -nav3v2_B.eigVec[nav3v2_B.i].re;
    nav3v2_B.quat_idx_2 = -nav3v2_B.quat_idx_2;
    nav3v2_B.quat_idx_3 = -nav3v2_B.quat_idx_3;
  }

  nav3v2_B.rtb_Lidar2Robot_tmp = 1.0 / sqrt(((nav3v2_B.K12 * nav3v2_B.K12 +
    nav3v2_B.K34 * nav3v2_B.K34) + nav3v2_B.quat_idx_2 * nav3v2_B.quat_idx_2) +
    nav3v2_B.quat_idx_3 * nav3v2_B.quat_idx_3);
  nav3v2_B.varargin_1[0] = nav3v2_B.K12 * nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.varargin_1[1] = nav3v2_B.K34 * nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.varargin_1[2] = nav3v2_B.quat_idx_2 * nav3v2_B.rtb_Lidar2Robot_tmp;
  nav3v2_B.varargin_1[3] = nav3v2_B.quat_idx_3 * nav3v2_B.rtb_Lidar2Robot_tmp;

  /* Outputs for Atomic SubSystem: '<S95>/Simulation_Subscriber' */
  /* MATLABSystem: '<S113>/SourceBlock' incorporates:
   *  Inport: '<S115>/In1'
   */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_223.getLatestMessage
    (&nav3v2_B.b_varargout_2_m);

  /* Outputs for Enabled SubSystem: '<S113>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S115>/Enable'
   */
  if (nav3v2_B.b_varargout_1) {
    nav3v2_B.In1_p = nav3v2_B.b_varargout_2_m;
  }

  /* End of MATLABSystem: '<S113>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S113>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S95>/Simulation_Subscriber' */

  /* MATLAB Function: '<S112>/Quaternion_2_Euler' */
  nav3v2_Quaternion_2_Euler(nav3v2_B.In1_p.Pose.Pose.Orientation.X,
    nav3v2_B.In1_p.Pose.Pose.Orientation.Y,
    nav3v2_B.In1_p.Pose.Pose.Orientation.Z,
    nav3v2_B.In1_p.Pose.Pose.Orientation.W, &nav3v2_B.sf_Quaternion_2_Euler_g);

  /* Switch: '<S6>/Switch' incorporates:
   *  MATLAB Function: '<S107>/World to Robot Transform'
   *  MATLABSystem: '<S88>/Get Parameter5'
   */
  if (nav3v2_B.value_j >= nav3v2_P.Switch_Threshold) {
    nav3v2_B.Switch[0] = nav3v2_B.World2RobotOdom[6];
    nav3v2_B.Switch[1] = nav3v2_B.World2RobotOdom[7];
    nav3v2_B.Switch[2] = nav3v2_rt_atan2d_snf((nav3v2_B.varargin_1[1] *
      nav3v2_B.varargin_1[2] + nav3v2_B.varargin_1[0] * nav3v2_B.varargin_1[3]) *
      2.0, ((nav3v2_B.varargin_1[0] * nav3v2_B.varargin_1[0] +
             nav3v2_B.varargin_1[1] * nav3v2_B.varargin_1[1]) -
            nav3v2_B.varargin_1[2] * nav3v2_B.varargin_1[2]) -
      nav3v2_B.varargin_1[3] * nav3v2_B.varargin_1[3]);
  } else {
    nav3v2_B.Switch[0] = nav3v2_B.In1_p.Pose.Pose.Position.X;
    nav3v2_B.Switch[1] = nav3v2_B.In1_p.Pose.Pose.Position.Y;
    nav3v2_B.Switch[2] = nav3v2_B.sf_Quaternion_2_Euler_g.theta;
  }

  /* End of Switch: '<S6>/Switch' */

  /* Outputs for Atomic SubSystem: '<S91>/Live_Subscriber' */
  /* MATLABSystem: '<S96>/SourceBlock' */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_165.getLatestMessage
    (&nav3v2_B.b_varargout_2_i);

  /* Outputs for Enabled SubSystem: '<S96>/Enabled Subsystem' */
  nav3v2_EnabledSubsystem(nav3v2_B.b_varargout_1, &nav3v2_B.b_varargout_2_i,
    &nav3v2_B.EnabledSubsystem_m);

  /* End of Outputs for SubSystem: '<S96>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S91>/Live_Subscriber' */

  /* Outputs for Atomic SubSystem: '<S3>/Subscribe' */
  /* MATLABSystem: '<S76>/SourceBlock' incorporates:
   *  Inport: '<S77>/In1'
   */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_555.getLatestMessage
    (&nav3v2_B.b_varargout_2_k);

  /* Outputs for Enabled SubSystem: '<S76>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S77>/Enable'
   */
  if (nav3v2_B.b_varargout_1) {
    nav3v2_B.In1_i = nav3v2_B.b_varargout_2_k;
  }

  /* End of MATLABSystem: '<S76>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S76>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S3>/Subscribe' */

  /* DataTypeConversion: '<S3>/Data Type Conversion' */
  for (nav3v2_B.i = 0; nav3v2_B.i < 8; nav3v2_B.i++) {
    nav3v2_B.DataTypeConversion[nav3v2_B.i] = nav3v2_B.In1_i.Axes[nav3v2_B.i];
  }

  /* End of DataTypeConversion: '<S3>/Data Type Conversion' */

  /* Outputs for Atomic SubSystem: '<S93>/Live_Subscriber' */
  /* MATLABSystem: '<S105>/SourceBlock' incorporates:
   *  Inport: '<S106>/In1'
   */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_590.getLatestMessage
    (&nav3v2_B.BusAssignment_l);

  /* Outputs for Enabled SubSystem: '<S105>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S106>/Enable'
   */
  if (nav3v2_B.b_varargout_1) {
    nav3v2_B.In1_b = nav3v2_B.BusAssignment_l;
  }

  /* End of MATLABSystem: '<S105>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S105>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S93>/Live_Subscriber' */

  /* Chart: '<S2>/State_Machine' incorporates:
   *  Constant: '<S2>/waypoints'
   *  DataStoreRead: '<S3>/Data Store Read'
   *  Gain: '<S3>/Linear_Accel'
   *  Product: '<S3>/Product'
   *  Sum: '<S3>/Sum'
   */
  if (nav3v2_DW.is_active_c3_nav3v2 == 0U) {
    nav3v2_DW.is_active_c3_nav3v2 = 1U;
    nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_INITIALISATION;
    nav3v2_B.currentState = nav3v2_DW.INITIALISATION_STATE;
  } else {
    switch (nav3v2_DW.is_c3_nav3v2) {
     case nav3v2_IN_AUTONOMOUS_CONTROL:
      nav3v2_AUTONOMOUS_CONTROL();
      break;

     case nav3v2_IN_EMERGENCY:
      if (nav3v2_B.In1_i.Buttons[3] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_WAITING;
        nav3v2_B.currentState = nav3v2_DW.WAITING_STATE;
      } else {
        nav3v2_B.linearVel = 0.0;
        nav3v2_B.angularVel = 0.0;
        nav3v2_B.ledState = nav3v2_DW.LED_EMERGENCY_STOP_STATE;
      }
      break;

     case nav3v2_IN_INITIALISATION:
      nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_WAITING;
      nav3v2_B.currentState = nav3v2_DW.WAITING_STATE;
      break;

     case nav3v2_IN_MANUAL_CONTROL:
      if (nav3v2_B.In1_i.Buttons[1] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_EMERGENCY;
        nav3v2_B.currentState = nav3v2_DW.EMERGENCY_STATE;
      } else if (nav3v2_B.In1_i.Buttons[5] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_WAITING;
        nav3v2_B.currentState = nav3v2_DW.WAITING_STATE;
      } else {
        nav3v2_B.angularVel = nav3v2_B.DataTypeConversion[0];
        nav3v2_B.linearVel = (nav3v2_P.Linear_Accel_Gain *
                              nav3v2_B.DataTypeConversion[4] +
                              nav3v2_B.DataTypeConversion[1]) *
          nav3v2_DW.MAX_LINEAR_VEL;
      }
      break;

     case nav3v2_IN_MOVEBASE:
      if (nav3v2_B.In1_i.Buttons[5] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_WAITING;
        nav3v2_B.currentState = nav3v2_DW.WAITING_STATE;
      } else {
        nav3v2_B.angularVel = nav3v2_B.In1_b.Angular.Z;
        nav3v2_B.linearVel = nav3v2_B.In1_b.Linear.X;
      }
      break;

     case nav3v2_IN_REVERSE:
      if (nav3v2_B.In1_i.Buttons[1] == 1) {
        switch (nav3v2_DW.is_REVERSE) {
         case nav_IN_PreciseRotateToWaypoint1:
          nav3v2_B.angularVel = 0.0;
          nav3v2_DW.is_REVERSE = nav3v2_IN_NO_ACTIVE_CHILD;
          break;

         case nav3_IN_ReverseRotateToWaypoint:
          nav3v2_B.angularVel = 0.0;
          nav3v2_DW.is_REVERSE = nav3v2_IN_NO_ACTIVE_CHILD;
          break;

         default:
          nav3v2_DW.is_REVERSE = nav3v2_IN_NO_ACTIVE_CHILD;
          break;
        }

        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_EMERGENCY;
        nav3v2_B.currentState = nav3v2_DW.EMERGENCY_STATE;
      } else if ((nav3v2_B.In1_i.Buttons[5] == 1) || (nav3v2_DW.finishedMission ==
                  1.0)) {
        switch (nav3v2_DW.is_REVERSE) {
         case nav_IN_PreciseRotateToWaypoint1:
          nav3v2_B.angularVel = 0.0;
          nav3v2_DW.is_REVERSE = nav3v2_IN_NO_ACTIVE_CHILD;
          break;

         case nav3_IN_ReverseRotateToWaypoint:
          nav3v2_B.angularVel = 0.0;
          nav3v2_DW.is_REVERSE = nav3v2_IN_NO_ACTIVE_CHILD;
          break;

         default:
          nav3v2_DW.is_REVERSE = nav3v2_IN_NO_ACTIVE_CHILD;
          break;
        }

        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_WAITING;
        nav3v2_B.currentState = nav3v2_DW.WAITING_STATE;
      } else {
        switch (nav3v2_DW.is_REVERSE) {
         case IN_FinishedReachingAllWaypoints:
          break;

         case nav_IN_PreciseRotateToWaypoint1:
          if ((nav3v2_DW.currentGoalHeading - 0.01 < nav3v2_B.Switch[2]) &&
              (nav3v2_B.Switch[2] < nav3v2_DW.currentGoalHeading + 0.01)) {
            nav3v2_B.angularVel = 0.0;
            nav3v2_DW.is_REVERSE = IN_FinishedReachingAllWaypoints;
            nav3v2_B.ledState = nav3v2_DW.LED_MISSION_COMPLETE_STATE;
            nav3v2_DW.finishedMission = 1.0;
          } else {
            nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
            nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
            nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
            nav3v2_B.desiredHeading = nav3v2_DW.currentGoalHeading;
            nav3v2_B.P = nav3v2_DW.P;
            nav3v2_B.I = nav3v2_DW.I;
            nav3v2_B.D = nav3v2_DW.D;

            /* Outputs for Function Call SubSystem: '<S19>/PID' */
            nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                       nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                       &nav3v2_DW.PID, &nav3v2_P.PID);

            /* End of Outputs for SubSystem: '<S19>/PID' */
            nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
            nav3v2_DW.goalHeading[0] = nav3v2_DW.currentGoalHeading;
            nav3v2_DW.goalHeading[1] = 0.0;
            nav3v2_DW.goalHeading[2] = 0.0;
          }
          break;

         case nav3v2_IN_ReachedWaypoint:
          if (nav3v2_DW.count + 1.0 < nav3v2_DW.last) {
            nav3v2_DW.count++;
            nav3v2_B.ledState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE;
            nav3v2_DW.is_REVERSE = nav3_IN_ReverseRotateToWaypoint;
            if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483642) {
              nav3v2_B.currentState = MAX_int32_T;
            } else {
              nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 5;
            }

            nav3v2_B.linearVel = 0.0;
            nav3v2_B.r1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
            nav3v2_B.r2 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
            nav3v2_B.dv[0] = nav3v2_DW.all_waypoints[nav3v2_B.r1 - 1];
            nav3v2_B.currentWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.r2 -
              1];
            nav3v2_B.dv[1] = nav3v2_DW.all_waypoints[nav3v2_B.r1 + 4];
            nav3v2_B.currentWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.r2 +
              4];
            nav3v2_DW.currentGoalHeading = nav3v2_CalculateReverseHeading
              (nav3v2_B.Switch, nav3v2_B.dv);
          } else {
            if (nav3v2_DW.count + 1.0 == nav3v2_DW.last) {
              nav3v2_DW.is_REVERSE = nav_IN_PreciseRotateToWaypoint1;
              nav3v2_B.linearVel = 0.0;
              nav3v2_DW.currentGoalHeading = 1.5707963267948966;
            }
          }
          break;

         case nav3v2_IN_ReverseMoveToWaypoint:
          nav3v2_B.pose[0] = nav3v2_B.Switch[0];
          nav3v2_B.pose[1] = nav3v2_B.Switch[1];
          nav3v2_B.pose[2] = nav3v2_B.Switch[2];
          nav3v2_B.r1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
          nav3v2_B.goalWaypoint[0] = nav3v2_DW.all_waypoints[nav3v2_B.r1 - 1];
          nav3v2_B.goalWaypoint[1] = nav3v2_DW.all_waypoints[nav3v2_B.r1 + 4];
          nav3v2_B.goalTolerance = nav3v2_DW.WAYPOINT_PRECISION;

          /* Outputs for Function Call SubSystem: '<S19>/checkAtGoal' */
          nav3v2_checkAtGoal(nav3v2_B.pose, nav3v2_B.goalWaypoint,
                             nav3v2_B.goalTolerance, &nav3v2_B.checkAtGoal);

          /* End of Outputs for SubSystem: '<S19>/checkAtGoal' */
          if (!nav3v2_B.checkAtGoal.LessThan) {
            nav3v2_DW.is_REVERSE = nav3v2_IN_ReachedWaypoint;
            if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483643) {
              nav3v2_B.currentState = MAX_int32_T;
            } else {
              nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 4;
            }

            nav3v2_B.ledState = nav3v2_DW.LED_WAYPOINT_REACHED_STATE;
          } else {
            nav3v2_B.dv[0] = nav3v2_DW.all_waypoints[nav3v2_B.r1 - 1];
            nav3v2_B.dv[1] = nav3v2_DW.all_waypoints[nav3v2_B.r1 + 4];
            nav3v2_DW.currentGoalHeading = nav3v2_CalculateReverseHeading
              (nav3v2_B.Switch, nav3v2_B.dv);
            nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
            nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
            nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
            nav3v2_B.desiredHeading = nav3v2_DW.currentGoalHeading;
            nav3v2_B.P = nav3v2_DW.P;
            nav3v2_B.I = nav3v2_DW.I;
            nav3v2_B.D = nav3v2_DW.D;

            /* Outputs for Function Call SubSystem: '<S19>/PID' */
            nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                       nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                       &nav3v2_DW.PID, &nav3v2_P.PID);

            /* End of Outputs for SubSystem: '<S19>/PID' */
            nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
          }
          break;

         default:
          /* case IN_ReverseRotateToWaypoint: */
          if ((nav3v2_DW.currentGoalHeading - 0.01 < nav3v2_B.Switch[2]) &&
              (nav3v2_B.Switch[2] < nav3v2_DW.currentGoalHeading + 0.01)) {
            nav3v2_B.angularVel = 0.0;
            nav3v2_DW.is_REVERSE = nav3v2_IN_ReverseMoveToWaypoint;
            if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483641) {
              nav3v2_B.currentState = MAX_int32_T;
            } else {
              nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 6;
            }

            nav3v2_B.linearVel = -nav3v2_DW.MAX_LINEAR_VEL / 2.0;
          } else {
            nav3v2_B.pose_j[0] = nav3v2_B.Switch[0];
            nav3v2_B.pose_j[1] = nav3v2_B.Switch[1];
            nav3v2_B.pose_j[2] = nav3v2_B.Switch[2];
            nav3v2_B.desiredHeading = nav3v2_DW.currentGoalHeading;
            nav3v2_B.P = nav3v2_DW.P;
            nav3v2_B.I = nav3v2_DW.I;
            nav3v2_B.D = nav3v2_DW.D;

            /* Outputs for Function Call SubSystem: '<S19>/PID' */
            nav3v2_PID(nav3v2_M, nav3v2_B.pose_j, nav3v2_B.desiredHeading,
                       nav3v2_B.P, nav3v2_B.I, nav3v2_B.D, &nav3v2_B.PID,
                       &nav3v2_DW.PID, &nav3v2_P.PID);

            /* End of Outputs for SubSystem: '<S19>/PID' */
            nav3v2_B.angularVel = nav3v2_B.PID.Saturation;
            nav3v2_DW.goalHeading[0] = nav3v2_DW.currentGoalHeading;
            nav3v2_DW.goalHeading[1] = 0.0;
            nav3v2_DW.goalHeading[2] = 0.0;
          }
          break;
        }
      }
      break;

     default:
      /* case IN_WAITING: */
      if (nav3v2_B.In1_i.Buttons[4] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_MANUAL_CONTROL;
        nav3v2_B.currentState = nav3v2_DW.MANUAL_CONTROL_STATE;
        nav3v2_B.ledState = nav3v2_DW.MANUAL_CONTROL_STATE;
      } else if (nav3v2_B.In1_i.Buttons[1] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_EMERGENCY;
        nav3v2_B.currentState = nav3v2_DW.EMERGENCY_STATE;
      } else if (nav3v2_B.In1_i.Buttons[2] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_REVERSE;
        for (nav3v2_B.r1 = 0; nav3v2_B.r1 < 2; nav3v2_B.r1++) {
          nav3v2_B.i = nav3v2_B.r1 << 2;
          nav3v2_B.DataTypeConversion[nav3v2_B.i] =
            nav3v2_P.waypoints_Value[nav3v2_B.i];
          nav3v2_B.r2 = nav3v2_B.i + 1;
          nav3v2_B.DataTypeConversion[nav3v2_B.r2] =
            nav3v2_P.waypoints_Value[nav3v2_B.r2];
          nav3v2_B.r2 = nav3v2_B.i + 2;
          nav3v2_B.DataTypeConversion[nav3v2_B.r2] =
            nav3v2_P.waypoints_Value[nav3v2_B.r2];
          nav3v2_B.i += 3;
          nav3v2_B.DataTypeConversion[nav3v2_B.i] =
            nav3v2_P.waypoints_Value[nav3v2_B.i];
        }

        nav3v2_B.K12 = nav3v2_B.DataTypeConversion[0];
        nav3v2_B.DataTypeConversion[0] = nav3v2_B.DataTypeConversion[3];
        nav3v2_B.DataTypeConversion[3] = nav3v2_B.K12;
        nav3v2_B.K12 = nav3v2_B.DataTypeConversion[1];
        nav3v2_B.DataTypeConversion[1] = nav3v2_B.DataTypeConversion[2];
        nav3v2_B.DataTypeConversion[2] = nav3v2_B.K12;
        nav3v2_B.K12 = nav3v2_B.DataTypeConversion[4];
        nav3v2_B.DataTypeConversion[4] = nav3v2_B.DataTypeConversion[7];
        nav3v2_B.DataTypeConversion[7] = nav3v2_B.K12;
        nav3v2_B.K12 = nav3v2_B.DataTypeConversion[5];
        nav3v2_B.DataTypeConversion[5] = nav3v2_B.DataTypeConversion[6];
        nav3v2_B.DataTypeConversion[6] = nav3v2_B.K12;
        nav3v2_DW.last = 5.0;
        nav3v2_DW.finishedMission = 0.0;
        nav3v2_DW.count = 1.0;
        nav3v2_DW.is_REVERSE = nav3_IN_ReverseRotateToWaypoint;
        if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483642) {
          nav3v2_B.currentState = MAX_int32_T;
        } else {
          nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 5;
        }

        nav3v2_B.linearVel = 0.0;
        nav3v2_B.r1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
        for (nav3v2_B.r2 = 0; nav3v2_B.r2 < 2; nav3v2_B.r2++) {
          nav3v2_B.i = nav3v2_B.r2 << 2;
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r2] =
            nav3v2_B.DataTypeConversion[nav3v2_B.i];
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r2 + 1] =
            nav3v2_B.DataTypeConversion[nav3v2_B.i + 1];
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r2 + 2] =
            nav3v2_B.DataTypeConversion[nav3v2_B.i + 2];
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r2 + 3] =
            nav3v2_B.DataTypeConversion[nav3v2_B.i + 3];
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r2 + 4] = 0.0;
          nav3v2_B.dv[nav3v2_B.r2] = nav3v2_DW.all_waypoints[(5 * nav3v2_B.r2 +
            nav3v2_B.r1) - 1];
          nav3v2_B.currentWaypoint[nav3v2_B.r2] = nav3v2_DW.all_waypoints[(5 *
            nav3v2_B.r2 + nav3v2_B.r1) - 1];
        }

        nav3v2_DW.currentGoalHeading = nav3v2_CalculateReverseHeading
          (nav3v2_B.Switch, nav3v2_B.dv);
      } else if (nav3v2_B.In1_i.Buttons[0] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_AUTONOMOUS_CONTROL;
        nav3v2_B.ledState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE;
        for (nav3v2_B.r1 = 0; nav3v2_B.r1 < 2; nav3v2_B.r1++) {
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r1] = nav3v2_B.Switch[nav3v2_B.r1];
          nav3v2_B.i = nav3v2_B.r1 << 2;
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r1 + 1] =
            nav3v2_P.waypoints_Value[nav3v2_B.i];
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r1 + 2] =
            nav3v2_P.waypoints_Value[nav3v2_B.i + 1];
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r1 + 3] =
            nav3v2_P.waypoints_Value[nav3v2_B.i + 2];
          nav3v2_DW.all_waypoints[5 * nav3v2_B.r1 + 4] =
            nav3v2_P.waypoints_Value[nav3v2_B.i + 3];
        }

        nav3v2_DW.all_waypoints_config[0] = 0.0;
        nav3v2_DW.all_waypoints_config[1] = nav3v2_P.waypoints_Value[8];
        nav3v2_DW.all_waypoints_config[2] = nav3v2_P.waypoints_Value[9];
        nav3v2_DW.all_waypoints_config[3] = nav3v2_P.waypoints_Value[10];
        nav3v2_DW.all_waypoints_config[4] = nav3v2_P.waypoints_Value[11];
        nav3v2_DW.last = 5.0;
        nav3v2_DW.finishedMission = 0.0;
        nav3v2_DW.count = 1.0;
        nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_RotateToWaypoint;
        if (nav3v2_DW.AUTONOMOUS_CONTROL_STATE > 2147483646) {
          nav3v2_B.currentState = MAX_int32_T;
        } else {
          nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE + 1;
        }

        nav3v2_B.linearVel = 0.0;
        nav3v2_B.r1 = static_cast<int32_T>(nav3v2_DW.count + 1.0);
        nav3v2_B.dv[0] = nav3v2_DW.all_waypoints[nav3v2_B.r1 - 1];
        nav3v2_B.dv[1] = nav3v2_DW.all_waypoints[nav3v2_B.r1 + 4];
        nav3v2_DW.currentGoalHeading = nav3v2_CalculateHeading(nav3v2_B.Switch,
          nav3v2_B.dv);
        nav3v2_DW.goalHeading[0] = nav3v2_DW.currentGoalHeading;
        nav3v2_DW.goalHeading[1] = 0.0;
        nav3v2_DW.goalHeading[2] = 0.0;
      } else if (nav3v2_B.In1_i.Buttons[7] == 1) {
        nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_MOVEBASE;
        nav3v2_B.currentState = nav3v2_DW.AUTONOMOUS_CONTROL_STATE;
      } else {
        nav3v2_B.linearVel = 0.0;
        nav3v2_B.angularVel = 0.0;
        nav3v2_B.ledState = nav3v2_DW.LED_MISSION_PAUSED_STATE;
      }
      break;
    }
  }

  /* End of Chart: '<S2>/State_Machine' */

  /* Outputs for Enabled SubSystem: '<S1>/Real_Actuation' incorporates:
   *  EnablePort: '<S9>/Enable'
   */
  /* MATLABSystem: '<S88>/Get Parameter5' */
  if (nav3v2_B.value_j > 0) {
    /* Product: '<S14>/Product' incorporates:
     *  Gain: '<S14>/Gain'
     *  MATLABSystem: '<S86>/Get Parameter1'
     */
    nav3v2_B.Product *= nav3v2_P.Gain_Gain * nav3v2_B.angularVel;

    /* Product: '<S14>/Divide2' incorporates:
     *  MATLABSystem: '<S86>/Get Parameter2'
     */
    nav3v2_B.K12 = nav3v2_B.linearVel / nav3v2_B.value;

    /* BusAssignment: '<S13>/Bus Assignment' incorporates:
     *  Constant: '<S15>/Constant'
     *  MATLABSystem: '<S86>/Get Parameter2'
     *  Product: '<S14>/Divide'
     *  Product: '<S14>/Divide1'
     *  Sum: '<S14>/Add'
     *  Sum: '<S14>/Add1'
     */
    nav3v2_B.BusAssignment_l = nav3v2_P.Constant_Value_n;
    nav3v2_B.BusAssignment_l.Angular.X = 1.0 / nav3v2_B.value * (nav3v2_B.K12 -
      nav3v2_B.Product);
    nav3v2_B.BusAssignment_l.Angular.Y = (nav3v2_B.K12 + nav3v2_B.Product) /
      nav3v2_B.value;

    /* Outputs for Atomic SubSystem: '<S13>/Publish' */
    /* MATLABSystem: '<S16>/SinkBlock' */
    Pub_nav3v2_442.publish(&nav3v2_B.BusAssignment_l);

    /* End of Outputs for SubSystem: '<S13>/Publish' */
  }

  /* End of Outputs for SubSystem: '<S1>/Real_Actuation' */

  /* Outputs for Enabled SubSystem: '<S1>/Simulation_Actuation' incorporates:
   *  EnablePort: '<S10>/Enable'
   */
  /* Logic: '<S1>/NOT' incorporates:
   *  MATLABSystem: '<S88>/Get Parameter5'
   */
  if (nav3v2_B.value_j == 0) {
    /* BusAssignment: '<S10>/Bus Assignment' incorporates:
     *  Constant: '<S17>/Constant'
     */
    nav3v2_B.BusAssignment_l = nav3v2_P.Constant_Value_o;
    nav3v2_B.BusAssignment_l.Linear.X = nav3v2_B.linearVel;
    nav3v2_B.BusAssignment_l.Angular.Z = nav3v2_B.angularVel;

    /* Outputs for Atomic SubSystem: '<S10>/Publish' */
    /* MATLABSystem: '<S18>/SinkBlock' */
    Pub_nav3v2_422.publish(&nav3v2_B.BusAssignment_l);

    /* End of Outputs for SubSystem: '<S10>/Publish' */
  }

  /* End of Logic: '<S1>/NOT' */
  /* End of Outputs for SubSystem: '<S1>/Simulation_Actuation' */

  /* BusAssignment: '<S8>/Bus Assignment2' */
  nav3v2_B.BusAssignment2.Data = nav3v2_B.ledState;

  /* Outputs for Atomic SubSystem: '<S8>/Publish2' */
  /* MATLABSystem: '<S12>/SinkBlock' */
  Pub_nav3v2_615.publish(&nav3v2_B.BusAssignment2);

  /* End of Outputs for SubSystem: '<S8>/Publish2' */

  /* Outputs for Atomic SubSystem: '<S92>/Live_Subscriber' */
  /* MATLABSystem: '<S101>/SourceBlock' incorporates:
   *  Inport: '<S104>/In1'
   */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_187.getLatestMessage
    (&nav3v2_B.b_varargout_2_c);

  /* Outputs for Enabled SubSystem: '<S101>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S104>/Enable'
   */
  if (nav3v2_B.b_varargout_1) {
    nav3v2_B.In1_a = nav3v2_B.b_varargout_2_c;
  }

  /* End of MATLABSystem: '<S101>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S101>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S92>/Live_Subscriber' */

  /* MATLAB Function: '<S100>/Quaternion_2_Euler' */
  nav3v2_Quaternion_2_Euler(nav3v2_B.In1_a.Orientation.X,
    nav3v2_B.In1_a.Orientation.Y, nav3v2_B.In1_a.Orientation.Z,
    nav3v2_B.In1_a.Orientation.W, &nav3v2_B.sf_Quaternion_2_Euler);

  /* Outputs for Atomic SubSystem: '<S91>/Live_Subscriber1' */
  /* MATLABSystem: '<S97>/SourceBlock' */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_166.getLatestMessage
    (&nav3v2_B.b_varargout_2_i);

  /* Outputs for Enabled SubSystem: '<S97>/Enabled Subsystem' */
  nav3v2_EnabledSubsystem(nav3v2_B.b_varargout_1, &nav3v2_B.b_varargout_2_i,
    &nav3v2_B.EnabledSubsystem_d);

  /* End of Outputs for SubSystem: '<S97>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S91>/Live_Subscriber1' */

  /* BusAssignment: '<S4>/Bus Assignment' incorporates:
   *  Constant: '<S78>/Constant'
   *  DataStoreRead: '<S4>/goalHeading'
   */
  nav3v2_B.BusAssignment = nav3v2_P.Constant_Value;
  nav3v2_B.BusAssignment.Data[0] = nav3v2_B.currentWaypoint[0];
  nav3v2_B.BusAssignment.Data[1] = nav3v2_B.currentWaypoint[1];
  nav3v2_B.BusAssignment.Data[2] = nav3v2_DW.goalHeading[0];
  nav3v2_B.BusAssignment.Data[5] = nav3v2_B.Switch[0];
  nav3v2_B.BusAssignment.Data[3] = nav3v2_DW.goalHeading[1];
  nav3v2_B.BusAssignment.Data[6] = nav3v2_B.Switch[1];
  nav3v2_B.BusAssignment.Data[4] = nav3v2_DW.goalHeading[2];
  nav3v2_B.BusAssignment.Data[7] = nav3v2_B.Switch[2];
  nav3v2_B.BusAssignment.Data_SL_Info.CurrentLength = nav3v2_B.ProbeWidth;

  /* Outputs for Atomic SubSystem: '<S4>/Publish' */
  /* MATLABSystem: '<S81>/SinkBlock' */
  Pub_nav3v2_516.publish(&nav3v2_B.BusAssignment);

  /* End of Outputs for SubSystem: '<S4>/Publish' */

  /* BusAssignment: '<S4>/Bus Assignment1' incorporates:
   *  DataTypeConversion: '<S4>/Data Type Conversion2'
   */
  nav3v2_B.BusAssignment1.Data = static_cast<real32_T>(nav3v2_B.angularVel);

  /* Outputs for Atomic SubSystem: '<S4>/Publish1' */
  /* MATLABSystem: '<S82>/SinkBlock' */
  Pub_nav3v2_566.publish(&nav3v2_B.BusAssignment1);

  /* End of Outputs for SubSystem: '<S4>/Publish1' */

  /* BusAssignment: '<S4>/Bus Assignment2' */
  nav3v2_B.BusAssignment2_k.Data = nav3v2_B.currentState;

  /* Outputs for Atomic SubSystem: '<S4>/Publish2' */
  /* MATLABSystem: '<S83>/SinkBlock' */
  Pub_nav3v2_570.publish(&nav3v2_B.BusAssignment2_k);

  /* End of Outputs for SubSystem: '<S4>/Publish2' */

  /* BusAssignment: '<S116>/Bus Assignment' incorporates:
   *  Constant: '<S116>/Constant'
   */
  nav3v2_B.BusAssignment_n.Data = nav3v2_P.Constant_Value_oa;

  /* Outputs for Atomic SubSystem: '<S116>/Publish' */
  /* MATLABSystem: '<S118>/SinkBlock' */
  Pub_nav3v2_494.publish(&nav3v2_B.BusAssignment_n);

  /* End of Outputs for SubSystem: '<S116>/Publish' */

  /* Outputs for Atomic SubSystem: '<S116>/Subscribe' */
  /* MATLABSystem: '<S120>/SourceBlock' incorporates:
   *  Inport: '<S121>/In1'
   */
  nav3v2_B.b_varargout_1 = Sub_nav3v2_499.getLatestMessage
    (&nav3v2_B.BusAssignment_n);

  /* Outputs for Enabled SubSystem: '<S120>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S121>/Enable'
   */
  if (nav3v2_B.b_varargout_1) {
    nav3v2_B.In1_d = nav3v2_B.BusAssignment_n;
  }

  /* End of MATLABSystem: '<S120>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S120>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S116>/Subscribe' */

  /* Outputs for Enabled SubSystem: '<S116>/RESET_IMU' incorporates:
   *  EnablePort: '<S119>/Enable'
   */
  if (nav3v2_B.In1_d.Data > 0) {
    /* MATLABSystem: '<S119>/Set Parameter' incorporates:
     *  Constant: '<S119>/Constant'
     *  Sum: '<S119>/Sum'
     */
    ParamSet_nav3v2_475.set_parameter(nav3v2_P.Constant_Value_c -
      nav3v2_B.sf_Quaternion_2_Euler.theta);
  }

  /* End of Outputs for SubSystem: '<S116>/RESET_IMU' */

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.2, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  nav3v2_M->Timing.clockTick0++;
  if (!nav3v2_M->Timing.clockTick0) {
    nav3v2_M->Timing.clockTickH0++;
  }
}

/* Model initialize function */
void nav3v2_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* block I/O */
  (void) memset((static_cast<void *>(&nav3v2_B)), 0,
                sizeof(B_nav3v2_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&nav3v2_DW), 0,
                sizeof(DW_nav3v2_T));

  {
    char_T tmp[11];
    char_T tmp_0[14];
    char_T tmp_1[13];
    char_T tmp_2[5];
    char_T tmp_3[9];
    int32_T i;
    static const char_T tmp_4[18] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'l', 'o', 'o', 'k', 'a', 'h', 'e', 'a', 'd' };

    static const char_T tmp_5[10] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'D' };

    static const char_T tmp_6[10] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'I' };

    static const char_T tmp_7[10] = { '/', 'c', 'o', 'n', 't', 'r', 'o', 'l',
      '/', 'P' };

    static const char_T tmp_8[29] = { '/', 'e', 'x', 'p', 'e', 'r', 'i', 'm',
      'e', 'n', 't', '/', 'W', 'a', 'y', 'p', 'o', 'i', 'n', 't', 'P', 'r', 'e',
      'c', 'i', 's', 'i', 'o', 'n' };

    static const char_T tmp_9[33] = { '/', 'e', 'x', 'p', 'e', 'r', 'i', 'm',
      'e', 'n', 't', '/', 'M', 'a', 'x', 'i', 'm', 'u', 'm', 'L', 'i', 'n', 'e',
      'a', 'r', 'V', 'e', 'l', 'o', 'c', 'i', 't', 'y' };

    static const char_T tmp_a[24] = { '/', 'e', 'x', 'p', 'e', 'r', 'i', 'm',
      'e', 'n', 't', '/', 'I', 'n', 'i', 't', 'i', 'a', 'l', 'P', 'o', 's', 'e',
      'X' };

    static const char_T tmp_b[24] = { '/', 'e', 'x', 'p', 'e', 'r', 'i', 'm',
      'e', 'n', 't', '/', 'I', 'n', 'i', 't', 'i', 'a', 'l', 'P', 'o', 's', 'e',
      'Y' };

    static const char_T tmp_c[28] = { '/', 'e', 'x', 'p', 'e', 'r', 'i', 'm',
      'e', 'n', 't', '/', 'I', 'n', 'i', 't', 'i', 'a', 'l', 'P', 'o', 's', 'e',
      'T', 'h', 'e', 't', 'a' };

    static const char_T tmp_d[34] = { '/', 'e', 'x', 'p', 'e', 'r', 'i', 'm',
      'e', 'n', 't', '/', 'M', 'a', 'x', 'i', 'm', 'u', 'm', 'A', 'n', 'g', 'u',
      'l', 'a', 'r', 'V', 'e', 'l', 'o', 'c', 'i', 't', 'y' };

    static const char_T tmp_e[29] = { '/', 'e', 'x', 'p', 'e', 'r', 'i', 'm',
      'e', 'n', 't', '/', 'T', 'r', 'u', 'e', 'N', 'o', 'r', 't', 'h', 'T', 'o',
      'G', 'r', 'o', 'u', 'n', 'd' };

    static const char_T tmp_f[17] = { '/', 's', 't', 'a', 't', 'e', '/', 'a',
      'u', 't', 'o', 'n', 'o', 'm', 'o', 'u', 's' };

    static const char_T tmp_g[16] = { '/', 's', 't', 'a', 't', 'e', '/', 'e',
      'm', 'e', 'r', 'g', 'e', 'n', 'c', 'y' };

    static const char_T tmp_h[23] = { '/', 'l', 'e', 'd', 's', 't', 'a', 't',
      'e', '/', 'm', 'i', 's', 's', 'i', 'o', 'n', 'p', 'a', 'u', 's', 'e', 'd'
    };

    static const char_T tmp_i[18] = { '/', 'l', 'e', 'd', 's', 't', 'a', 't',
      'e', '/', 's', 't', 'a', 'r', 't', 'i', 'n', 'g' };

    static const char_T tmp_j[25] = { '/', 'l', 'e', 'd', 's', 't', 'a', 't',
      'e', '/', 'w', 'a', 'y', 'p', 'o', 'i', 'n', 't', 'r', 'e', 'a', 'c', 'h',
      'e', 'd' };

    static const char_T tmp_k[21] = { '/', 's', 't', 'a', 't', 'e', '/', 'i',
      'n', 'i', 't', 'i', 'a', 'l', 'i', 's', 'a', 't', 'i', 'o', 'n' };

    static const char_T tmp_l[13] = { '/', 's', 't', 'a', 't', 'e', '/', 'm',
      'a', 'n', 'u', 'a', 'l' };

    static const char_T tmp_m[14] = { '/', 's', 't', 'a', 't', 'e', '/', 'r',
      'e', 'v', 'e', 'r', 's', 'e' };

    static const char_T tmp_n[14] = { '/', 's', 't', 'a', 't', 'e', '/', 'w',
      'a', 'i', 't', 'i', 'n', 'g' };

    static const char_T tmp_o[20] = { '/', 'l', 'e', 'd', 's', 't', 'a', 't',
      'e', '/', 'b', 'a', 't', 't', 'e', 'r', 'y', 'l', 'o', 'w' };

    static const char_T tmp_p[23] = { '/', 'l', 'e', 'd', 's', 't', 'a', 't',
      'e', '/', 'e', 'm', 'e', 'r', 'g', 'e', 'n', 'c', 'y', 's', 't', 'o', 'p'
    };

    static const char_T tmp_q[15] = { '/', 'l', 'e', 'd', 's', 't', 'a', 't',
      'e', '/', 'e', 'r', 'r', 'o', 'r' };

    static const char_T tmp_r[25] = { '/', 'l', 'e', 'd', 's', 't', 'a', 't',
      'e', '/', 'm', 'i', 's', 's', 'i', 'o', 'n', 'c', 'o', 'm', 'p', 'l', 'e',
      't', 'e' };

    static const char_T tmp_s[18] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'r',
      'o', 'b', 'o', 't', 'L', 'e', 'n', 'g', 't', 'h' };

    static const char_T tmp_t[16] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'w',
      'h', 'e', 'e', 'l', 'B', 'a', 's', 'e' };

    static const char_T tmp_u[18] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'w',
      'h', 'e', 'e', 'l', 'R', 'a', 'd', 'i', 'u', 's' };

    static const char_T tmp_v[19] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'L',
      'i', 'd', 'a', 'r', '2', 'R', 'o', 'b', 'o', 't', 'Y' };

    static const char_T tmp_w[19] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'L',
      'i', 'd', 'a', 'r', '2', 'R', 'o', 'b', 'o', 't', 'X' };

    static const char_T tmp_x[23] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'L',
      'i', 'd', 'a', 'r', '2', 'R', 'o', 'b', 'o', 't', 'T', 'h', 'e', 't', 'a'
    };

    static const char_T tmp_y[12] = { '/', 's', 'y', 's', 't', 'e', 'm', '/',
      'l', 'i', 'v', 'e' };

    static const char_T tmp_z[34] = { '/', 's', 'l', 'a', 'm', 'w', 'a', 'r',
      'e', '_', 'r', 'o', 's', '_', 's', 'd', 'k', '_', 's', 'e', 'r', 'v', 'e',
      'r', '_', 'n', 'o', 'd', 'e', '/', 'o', 'd', 'o', 'm' };

    static const char_T tmp_10[10] = { '/', 'a', 'm', 'c', 'l', '_', 'p', 'o',
      's', 'e' };

    static const char_T tmp_11[19] = { '/', 'a', 'm', 'r', '/', 's', 't', 'a',
      't', 'u', 's', '/', 'v', 'o', 'l', 't', 'a', 'g', 'e' };

    static const char_T tmp_12[12] = { '/', 'a', 'm', 'r', '/', 'c', 'm', 'd',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_13[14] = { '/', 'z', 'l', 'a', 'c', '8', '0', '1',
      '5', 'd', '/', 'r', 'p', 'm' };

    static const char_T tmp_14[12] = { '/', 's', 'i', 'm', '_', 'c', 'm', 'd',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_15[13] = { '/', 'a', 'm', 'r', '/', 'L', 'e', 'd',
      'S', 't', 'a', 't', 'e' };

    static const char_T tmp_16[24] = { '/', 't', 'i', 'n', 'k', 'e', 'r', 'f',
      'o', 'r', 'g', 'e', '_', 'i', 'm', 'u', '_', 'r', 'o', 's', '/', 'i', 'm',
      'u' };

    static const char_T tmp_17[19] = { '/', 'a', 'm', 'r', '/', 's', 't', 'a',
      't', 'u', 's', '/', 'c', 'u', 'r', 'r', 'e', 'n', 't' };

    static const char_T tmp_18[8] = { '/', 'a', 'm', 'r', '/', 'l', 'o', 'g' };

    static const char_T tmp_19[13] = { '/', 'a', 'm', 'r', '/', 'c', 'm', 'd',
      'I', 'n', 'p', 'u', 't' };

    static const char_T tmp_1a[10] = { '/', 'a', 'm', 'r', '/', 'S', 't', 'a',
      't', 'e' };

    static const char_T tmp_1b[20] = { '/', 'a', 'm', 'r', '/', 's', 't', 'a',
      't', 'u', 's', '/', 'r', 'e', 's', 'e', 't', 'i', 'm', 'u' };

    /* Start for MATLABSystem: '<S84>/Get Parameter1' */
    nav3v2_DW.obj_gl.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_fg = true;
    nav3v2_DW.obj_gl.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      nav3v2_B.cv10[i] = tmp_4[i];
    }

    nav3v2_B.cv10[18] = '\x00';
    ParamGet_nav3v2_242.initialize(nav3v2_B.cv10);
    ParamGet_nav3v2_242.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_242.set_initial_value(0.0);
    nav3v2_DW.obj_gl.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S84>/Get Parameter1' */

    /* Start for MATLABSystem: '<S84>/Get Parameter2' */
    nav3v2_DW.obj_j.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_bh = true;
    nav3v2_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_5[i];
    }

    tmp[10] = '\x00';
    ParamGet_nav3v2_274.initialize(tmp);
    ParamGet_nav3v2_274.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_274.set_initial_value(0.0);
    nav3v2_DW.obj_j.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S84>/Get Parameter2' */

    /* Start for MATLABSystem: '<S84>/Get Parameter3' */
    nav3v2_DW.obj_a.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_c5 = true;
    nav3v2_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_6[i];
    }

    tmp[10] = '\x00';
    ParamGet_nav3v2_275.initialize(tmp);
    ParamGet_nav3v2_275.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_275.set_initial_value(0.0);
    nav3v2_DW.obj_a.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S84>/Get Parameter3' */

    /* Start for MATLABSystem: '<S84>/Get Parameter4' */
    nav3v2_DW.obj_b.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_bhy = true;
    nav3v2_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_7[i];
    }

    tmp[10] = '\x00';
    ParamGet_nav3v2_276.initialize(tmp);
    ParamGet_nav3v2_276.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_276.set_initial_value(0.0);
    nav3v2_DW.obj_b.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S84>/Get Parameter4' */

    /* Start for MATLABSystem: '<S85>/Get Parameter6' */
    nav3v2_DW.obj_ek.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_et = true;
    nav3v2_DW.obj_ek.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      nav3v2_B.cv2[i] = tmp_8[i];
    }

    nav3v2_B.cv2[29] = '\x00';
    ParamGet_nav3v2_541.initialize(nav3v2_B.cv2);
    ParamGet_nav3v2_541.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_541.set_initial_value(0.25);
    nav3v2_DW.obj_ek.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S85>/Get Parameter6' */

    /* Start for MATLABSystem: '<S85>/Get Parameter5' */
    nav3v2_DW.obj_lb.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_an = true;
    nav3v2_DW.obj_lb.isInitialized = 1;
    for (i = 0; i < 33; i++) {
      nav3v2_B.cv1[i] = tmp_9[i];
    }

    nav3v2_B.cv1[33] = '\x00';
    ParamGet_nav3v2_368.initialize(nav3v2_B.cv1);
    ParamGet_nav3v2_368.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_368.set_initial_value(0.0);
    nav3v2_DW.obj_lb.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S85>/Get Parameter5' */

    /* Start for MATLABSystem: '<S85>/Get Parameter2' */
    nav3v2_DW.obj_kg.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_gi = true;
    nav3v2_DW.obj_kg.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      nav3v2_B.cv5[i] = tmp_a[i];
    }

    nav3v2_B.cv5[24] = '\x00';
    ParamGet_nav3v2_326.initialize(nav3v2_B.cv5);
    ParamGet_nav3v2_326.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_326.set_initial_value(0.0);
    nav3v2_DW.obj_kg.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S85>/Get Parameter2' */

    /* Start for MATLABSystem: '<S85>/Get Parameter3' */
    nav3v2_DW.obj_ef.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_mn = true;
    nav3v2_DW.obj_ef.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      nav3v2_B.cv5[i] = tmp_b[i];
    }

    nav3v2_B.cv5[24] = '\x00';
    ParamGet_nav3v2_327.initialize(nav3v2_B.cv5);
    ParamGet_nav3v2_327.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_327.set_initial_value(0.0);
    nav3v2_DW.obj_ef.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S85>/Get Parameter3' */

    /* Start for MATLABSystem: '<S85>/Get Parameter1' */
    nav3v2_DW.obj_g.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_ow = true;
    nav3v2_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 28; i++) {
      nav3v2_B.cv3[i] = tmp_c[i];
    }

    nav3v2_B.cv3[28] = '\x00';
    ParamGet_nav3v2_325.initialize(nav3v2_B.cv3);
    ParamGet_nav3v2_325.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_325.set_initial_value(0.0);
    nav3v2_DW.obj_g.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S85>/Get Parameter1' */

    /* Start for MATLABSystem: '<S85>/Get Parameter4' */
    nav3v2_DW.obj_dr.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_oi = true;
    nav3v2_DW.obj_dr.isInitialized = 1;
    for (i = 0; i < 34; i++) {
      nav3v2_B.cv[i] = tmp_d[i];
    }

    nav3v2_B.cv[34] = '\x00';
    ParamGet_nav3v2_365.initialize(nav3v2_B.cv);
    ParamGet_nav3v2_365.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_365.set_initial_value(0.0);
    nav3v2_DW.obj_dr.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S85>/Get Parameter4' */

    /* Start for MATLABSystem: '<S85>/Get Parameter' */
    nav3v2_DW.obj_p.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_lk = true;
    nav3v2_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      nav3v2_B.cv2[i] = tmp_e[i];
    }

    nav3v2_B.cv2[29] = '\x00';
    ParamGet_nav3v2_237.initialize(nav3v2_B.cv2);
    ParamGet_nav3v2_237.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_237.set_initial_value(0.0);
    nav3v2_DW.obj_p.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S85>/Get Parameter' */

    /* Start for MATLABSystem: '<S87>/Get Parameter4' */
    nav3v2_DW.obj_n.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_gn = true;
    nav3v2_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      nav3v2_B.cv11[i] = tmp_f[i];
    }

    nav3v2_B.cv11[17] = '\x00';
    ParamGet_nav3v2_295.initialize(nav3v2_B.cv11);
    ParamGet_nav3v2_295.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_295.set_initial_value(0);
    nav3v2_DW.obj_n.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter4' */

    /* Start for MATLABSystem: '<S87>/Get Parameter3' */
    nav3v2_DW.obj_l.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_ou = true;
    nav3v2_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      nav3v2_B.cv12[i] = tmp_g[i];
    }

    nav3v2_B.cv12[16] = '\x00';
    ParamGet_nav3v2_294.initialize(nav3v2_B.cv12);
    ParamGet_nav3v2_294.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_294.set_initial_value(0);
    nav3v2_DW.obj_l.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter3' */

    /* Start for MATLABSystem: '<S87>/Get Parameter11' */
    nav3v2_DW.obj_l3.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_h = true;
    nav3v2_DW.obj_l3.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      nav3v2_B.cv6[i] = tmp_h[i];
    }

    nav3v2_B.cv6[23] = '\x00';
    ParamGet_nav3v2_628.initialize(nav3v2_B.cv6);
    ParamGet_nav3v2_628.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_628.set_initial_value(3);
    nav3v2_DW.obj_l3.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter11' */

    /* Start for MATLABSystem: '<S87>/Get Parameter12' */
    nav3v2_DW.obj_ls.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_mi = true;
    nav3v2_DW.obj_ls.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      nav3v2_B.cv10[i] = tmp_i[i];
    }

    nav3v2_B.cv10[18] = '\x00';
    ParamGet_nav3v2_629.initialize(nav3v2_B.cv10);
    ParamGet_nav3v2_629.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_629.set_initial_value(7);
    nav3v2_DW.obj_ls.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter12' */

    /* Start for MATLABSystem: '<S87>/Get Parameter13' */
    nav3v2_DW.obj_k.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_b1 = true;
    nav3v2_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      nav3v2_B.cv4[i] = tmp_j[i];
    }

    nav3v2_B.cv4[25] = '\x00';
    ParamGet_nav3v2_636.initialize(nav3v2_B.cv4);
    ParamGet_nav3v2_636.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_636.set_initial_value(1);
    nav3v2_DW.obj_k.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter13' */

    /* Start for MATLABSystem: '<S87>/Get Parameter2' */
    nav3v2_DW.obj_e.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_c = true;
    nav3v2_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 21; i++) {
      nav3v2_B.cv7[i] = tmp_k[i];
    }

    nav3v2_B.cv7[21] = '\x00';
    ParamGet_nav3v2_293.initialize(nav3v2_B.cv7);
    ParamGet_nav3v2_293.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_293.set_initial_value(0);
    nav3v2_DW.obj_e.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter2' */

    /* Start for MATLABSystem: '<S87>/Get Parameter1' */
    nav3v2_DW.obj_f.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_e = true;
    nav3v2_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_l[i];
    }

    tmp_0[13] = '\x00';
    ParamGet_nav3v2_292.initialize(tmp_0);
    ParamGet_nav3v2_292.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_292.set_initial_value(0);
    nav3v2_DW.obj_f.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter1' */

    /* Start for MATLABSystem: '<S87>/Get Parameter5' */
    nav3v2_DW.obj_e0.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_p = true;
    nav3v2_DW.obj_e0.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      nav3v2_B.cv14[i] = tmp_m[i];
    }

    nav3v2_B.cv14[14] = '\x00';
    ParamGet_nav3v2_303.initialize(nav3v2_B.cv14);
    ParamGet_nav3v2_303.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_303.set_initial_value(0);
    nav3v2_DW.obj_e0.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter5' */

    /* Start for MATLABSystem: '<S87>/Get Parameter6' */
    nav3v2_DW.obj_lq.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_ci = true;
    nav3v2_DW.obj_lq.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      nav3v2_B.cv14[i] = tmp_n[i];
    }

    nav3v2_B.cv14[14] = '\x00';
    ParamGet_nav3v2_306.initialize(nav3v2_B.cv14);
    ParamGet_nav3v2_306.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_306.set_initial_value(0);
    nav3v2_DW.obj_lq.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter6' */

    /* Start for MATLABSystem: '<S87>/Get Parameter10' */
    nav3v2_DW.obj_na.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_dh = true;
    nav3v2_DW.obj_na.isInitialized = 1;
    for (i = 0; i < 20; i++) {
      nav3v2_B.cv8[i] = tmp_o[i];
    }

    nav3v2_B.cv8[20] = '\x00';
    ParamGet_nav3v2_627.initialize(nav3v2_B.cv8);
    ParamGet_nav3v2_627.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_627.set_initial_value(6);
    nav3v2_DW.obj_na.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter10' */

    /* Start for MATLABSystem: '<S87>/Get Parameter9' */
    nav3v2_DW.obj_i.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_f = true;
    nav3v2_DW.obj_i.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      nav3v2_B.cv6[i] = tmp_p[i];
    }

    nav3v2_B.cv6[23] = '\x00';
    ParamGet_nav3v2_626.initialize(nav3v2_B.cv6);
    ParamGet_nav3v2_626.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_626.set_initial_value(4);
    nav3v2_DW.obj_i.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter9' */

    /* Start for MATLABSystem: '<S87>/Get Parameter8' */
    nav3v2_DW.obj_i3.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_k = true;
    nav3v2_DW.obj_i3.isInitialized = 1;
    for (i = 0; i < 15; i++) {
      nav3v2_B.cv13[i] = tmp_q[i];
    }

    nav3v2_B.cv13[15] = '\x00';
    ParamGet_nav3v2_625.initialize(nav3v2_B.cv13);
    ParamGet_nav3v2_625.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_625.set_initial_value(5);
    nav3v2_DW.obj_i3.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter8' */

    /* Start for MATLABSystem: '<S87>/Get Parameter7' */
    nav3v2_DW.obj_m.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_a = true;
    nav3v2_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 25; i++) {
      nav3v2_B.cv4[i] = tmp_r[i];
    }

    nav3v2_B.cv4[25] = '\x00';
    ParamGet_nav3v2_624.initialize(nav3v2_B.cv4);
    ParamGet_nav3v2_624.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_624.set_initial_value(0);
    nav3v2_DW.obj_m.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S87>/Get Parameter7' */

    /* Start for MATLABSystem: '<S86>/Get Parameter' */
    nav3v2_DW.obj_mt.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_ot = true;
    nav3v2_DW.obj_mt.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      nav3v2_B.cv10[i] = tmp_s[i];
    }

    nav3v2_B.cv10[18] = '\x00';
    ParamGet_nav3v2_340.initialize(nav3v2_B.cv10);
    ParamGet_nav3v2_340.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_340.set_initial_value(0.0);
    nav3v2_DW.obj_mt.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S86>/Get Parameter' */

    /* Start for MATLABSystem: '<S86>/Get Parameter1' */
    nav3v2_DW.obj_h.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_l5 = true;
    nav3v2_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      nav3v2_B.cv12[i] = tmp_t[i];
    }

    nav3v2_B.cv12[16] = '\x00';
    ParamGet_nav3v2_341.initialize(nav3v2_B.cv12);
    ParamGet_nav3v2_341.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_341.set_initial_value(0.0);
    nav3v2_DW.obj_h.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S86>/Get Parameter1' */

    /* Start for MATLABSystem: '<S86>/Get Parameter2' */
    nav3v2_DW.obj_my.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_i = true;
    nav3v2_DW.obj_my.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      nav3v2_B.cv10[i] = tmp_u[i];
    }

    nav3v2_B.cv10[18] = '\x00';
    ParamGet_nav3v2_342.initialize(nav3v2_B.cv10);
    ParamGet_nav3v2_342.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_342.set_initial_value(0.0);
    nav3v2_DW.obj_my.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S86>/Get Parameter2' */

    /* Start for MATLABSystem: '<S86>/Get Parameter3' */
    nav3v2_DW.obj_d.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_be = true;
    nav3v2_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3v2_B.cv9[i] = tmp_v[i];
    }

    nav3v2_B.cv9[19] = '\x00';
    ParamGet_nav3v2_357.initialize(nav3v2_B.cv9);
    ParamGet_nav3v2_357.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_357.set_initial_value(0.0);
    nav3v2_DW.obj_d.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S86>/Get Parameter3' */

    /* Start for MATLABSystem: '<S86>/Get Parameter4' */
    nav3v2_DW.obj_lm.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_ba = true;
    nav3v2_DW.obj_lm.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3v2_B.cv9[i] = tmp_w[i];
    }

    nav3v2_B.cv9[19] = '\x00';
    ParamGet_nav3v2_358.initialize(nav3v2_B.cv9);
    ParamGet_nav3v2_358.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_358.set_initial_value(0.0);
    nav3v2_DW.obj_lm.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S86>/Get Parameter4' */

    /* Start for MATLABSystem: '<S86>/Get Parameter5' */
    nav3v2_DW.obj_kb.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_k5 = true;
    nav3v2_DW.obj_kb.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      nav3v2_B.cv6[i] = tmp_x[i];
    }

    nav3v2_B.cv6[23] = '\x00';
    ParamGet_nav3v2_359.initialize(nav3v2_B.cv6);
    ParamGet_nav3v2_359.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_359.set_initial_value(0.0);
    nav3v2_DW.obj_kb.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S86>/Get Parameter5' */

    /* Start for MATLABSystem: '<S88>/Get Parameter5' */
    nav3v2_DW.obj.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_oj = true;
    nav3v2_DW.obj.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_y[i];
    }

    tmp_1[12] = '\x00';
    ParamGet_nav3v2_381.initialize(tmp_1);
    ParamGet_nav3v2_381.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3v2_381.set_initial_value(0);
    nav3v2_DW.obj.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S88>/Get Parameter5' */

    /* Start for Atomic SubSystem: '<S94>/Live_Subscriber' */
    /* Start for MATLABSystem: '<S108>/SourceBlock' */
    nav3v2_DW.obj_gt.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_g = true;
    nav3v2_DW.obj_gt.isInitialized = 1;
    for (i = 0; i < 34; i++) {
      nav3v2_B.cv[i] = tmp_z[i];
    }

    nav3v2_B.cv[34] = '\x00';
    Sub_nav3v2_202.createSubscriber(nav3v2_B.cv, 1);
    nav3v2_DW.obj_gt.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S108>/SourceBlock' */
    /* End of Start for SubSystem: '<S94>/Live_Subscriber' */

    /* Start for Atomic SubSystem: '<S95>/Simulation_Subscriber' */
    /* Start for MATLABSystem: '<S113>/SourceBlock' */
    nav3v2_DW.obj_fy.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_b = true;
    nav3v2_DW.obj_fy.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_10[i];
    }

    tmp[10] = '\x00';
    Sub_nav3v2_223.createSubscriber(tmp, 1);
    nav3v2_DW.obj_fy.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S113>/SourceBlock' */
    /* End of Start for SubSystem: '<S95>/Simulation_Subscriber' */

    /* Start for Atomic SubSystem: '<S91>/Live_Subscriber' */
    /* Start for MATLABSystem: '<S96>/SourceBlock' */
    nav3v2_DW.obj_nh.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_o = true;
    nav3v2_DW.obj_nh.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3v2_B.cv9[i] = tmp_11[i];
    }

    nav3v2_B.cv9[19] = '\x00';
    Sub_nav3v2_165.createSubscriber(nav3v2_B.cv9, 1);
    nav3v2_DW.obj_nh.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S96>/SourceBlock' */
    /* End of Start for SubSystem: '<S91>/Live_Subscriber' */

    /* Start for Atomic SubSystem: '<S3>/Subscribe' */
    /* Start for MATLABSystem: '<S76>/SourceBlock' */
    nav3v2_DW.obj_mf.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_pp = true;
    nav3v2_DW.obj_mf.isInitialized = 1;
    tmp_2[0] = '/';
    tmp_2[1] = 'j';
    tmp_2[2] = 'o';
    tmp_2[3] = 'y';
    tmp_2[4] = '\x00';
    Sub_nav3v2_555.createSubscriber(tmp_2, 1);
    nav3v2_DW.obj_mf.isSetupComplete = true;

    /* End of Start for SubSystem: '<S3>/Subscribe' */

    /* Start for Atomic SubSystem: '<S93>/Live_Subscriber' */
    /* Start for MATLABSystem: '<S105>/SourceBlock' */
    nav3v2_DW.obj_do.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_ls = true;
    nav3v2_DW.obj_do.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_12[i];
    }

    tmp_1[12] = '\x00';
    Sub_nav3v2_590.createSubscriber(tmp_1, 1);
    nav3v2_DW.obj_do.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S105>/SourceBlock' */
    /* End of Start for SubSystem: '<S93>/Live_Subscriber' */

    /* Start for Enabled SubSystem: '<S1>/Real_Actuation' */
    /* Start for Atomic SubSystem: '<S13>/Publish' */
    /* Start for MATLABSystem: '<S16>/SinkBlock' */
    nav3v2_DW.obj_ft.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_j = true;
    nav3v2_DW.obj_ft.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      nav3v2_B.cv14[i] = tmp_13[i];
    }

    nav3v2_B.cv14[14] = '\x00';
    Pub_nav3v2_442.createPublisher(nav3v2_B.cv14, 1);
    nav3v2_DW.obj_ft.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S16>/SinkBlock' */
    /* End of Start for SubSystem: '<S13>/Publish' */
    /* End of Start for SubSystem: '<S1>/Real_Actuation' */

    /* Start for Enabled SubSystem: '<S1>/Simulation_Actuation' */
    /* Start for Atomic SubSystem: '<S10>/Publish' */
    /* Start for MATLABSystem: '<S18>/SinkBlock' */
    nav3v2_DW.obj_gv.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_gc = true;
    nav3v2_DW.obj_gv.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_14[i];
    }

    tmp_1[12] = '\x00';
    Pub_nav3v2_422.createPublisher(tmp_1, 1);
    nav3v2_DW.obj_gv.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S18>/SinkBlock' */
    /* End of Start for SubSystem: '<S10>/Publish' */
    /* End of Start for SubSystem: '<S1>/Simulation_Actuation' */

    /* Start for Atomic SubSystem: '<S8>/Publish2' */
    /* Start for MATLABSystem: '<S12>/SinkBlock' */
    nav3v2_DW.obj_nw.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_pw = true;
    nav3v2_DW.obj_nw.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_15[i];
    }

    tmp_0[13] = '\x00';
    Pub_nav3v2_615.createPublisher(tmp_0, 1);
    nav3v2_DW.obj_nw.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S12>/SinkBlock' */
    /* End of Start for SubSystem: '<S8>/Publish2' */

    /* Start for Atomic SubSystem: '<S92>/Live_Subscriber' */
    /* Start for MATLABSystem: '<S101>/SourceBlock' */
    nav3v2_DW.obj_dm.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_d = true;
    nav3v2_DW.obj_dm.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      nav3v2_B.cv5[i] = tmp_16[i];
    }

    nav3v2_B.cv5[24] = '\x00';
    Sub_nav3v2_187.createSubscriber(nav3v2_B.cv5, 1);
    nav3v2_DW.obj_dm.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S101>/SourceBlock' */
    /* End of Start for SubSystem: '<S92>/Live_Subscriber' */

    /* Start for Atomic SubSystem: '<S91>/Live_Subscriber1' */
    /* Start for MATLABSystem: '<S97>/SourceBlock' */
    nav3v2_DW.obj_n4.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_da = true;
    nav3v2_DW.obj_n4.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3v2_B.cv9[i] = tmp_17[i];
    }

    nav3v2_B.cv9[19] = '\x00';
    Sub_nav3v2_166.createSubscriber(nav3v2_B.cv9, 1);
    nav3v2_DW.obj_n4.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S97>/SourceBlock' */
    /* End of Start for SubSystem: '<S91>/Live_Subscriber1' */

    /* Start for Probe: '<S4>/Probe Width' */
    nav3v2_B.ProbeWidth = 8U;

    /* Start for Atomic SubSystem: '<S4>/Publish' */
    /* Start for MATLABSystem: '<S81>/SinkBlock' */
    nav3v2_DW.obj_o.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_gs = true;
    nav3v2_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_3[i] = tmp_18[i];
    }

    tmp_3[8] = '\x00';
    Pub_nav3v2_516.createPublisher(tmp_3, 1);
    nav3v2_DW.obj_o.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S81>/SinkBlock' */
    /* End of Start for SubSystem: '<S4>/Publish' */

    /* Start for Atomic SubSystem: '<S4>/Publish1' */
    /* Start for MATLABSystem: '<S82>/SinkBlock' */
    nav3v2_DW.obj_jr.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_b2 = true;
    nav3v2_DW.obj_jr.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_19[i];
    }

    tmp_0[13] = '\x00';
    Pub_nav3v2_566.createPublisher(tmp_0, 1);
    nav3v2_DW.obj_jr.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S82>/SinkBlock' */
    /* End of Start for SubSystem: '<S4>/Publish1' */

    /* Start for Atomic SubSystem: '<S4>/Publish2' */
    /* Start for MATLABSystem: '<S83>/SinkBlock' */
    nav3v2_DW.obj_c.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_mo = true;
    nav3v2_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_1a[i];
    }

    tmp[10] = '\x00';
    Pub_nav3v2_570.createPublisher(tmp, 1);
    nav3v2_DW.obj_c.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S83>/SinkBlock' */
    /* End of Start for SubSystem: '<S4>/Publish2' */

    /* Start for Atomic SubSystem: '<S116>/Publish' */
    /* Start for MATLABSystem: '<S118>/SinkBlock' */
    nav3v2_DW.obj_jx.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_m = true;
    nav3v2_DW.obj_jx.isInitialized = 1;
    for (i = 0; i < 20; i++) {
      nav3v2_B.cv8[i] = tmp_1b[i];
    }

    nav3v2_B.cv8[20] = '\x00';
    Pub_nav3v2_494.createPublisher(nav3v2_B.cv8, 1);
    nav3v2_DW.obj_jx.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S118>/SinkBlock' */
    /* End of Start for SubSystem: '<S116>/Publish' */

    /* Start for Atomic SubSystem: '<S116>/Subscribe' */
    /* Start for MATLABSystem: '<S120>/SourceBlock' */
    nav3v2_DW.obj_jp.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty = true;
    nav3v2_DW.obj_jp.isInitialized = 1;
    for (i = 0; i < 20; i++) {
      nav3v2_B.cv8[i] = tmp_1b[i];
    }

    nav3v2_B.cv8[20] = '\x00';
    Sub_nav3v2_499.createSubscriber(nav3v2_B.cv8, 1);
    nav3v2_DW.obj_jp.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S120>/SourceBlock' */
    /* End of Start for SubSystem: '<S116>/Subscribe' */

    /* Start for Enabled SubSystem: '<S116>/RESET_IMU' */
    /* Start for MATLABSystem: '<S119>/Set Parameter' */
    nav3v2_DW.obj_cw.matlabCodegenIsDeleted = false;
    nav3v2_DW.objisempty_l = true;
    nav3v2_DW.obj_cw.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      nav3v2_B.cv2[i] = tmp_e[i];
    }

    nav3v2_B.cv2[29] = '\x00';
    ParamSet_nav3v2_475.initialize(nav3v2_B.cv2);
    nav3v2_DW.obj_cw.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S119>/Set Parameter' */
    /* End of Start for SubSystem: '<S116>/RESET_IMU' */

    /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
    nav3v2_DW.AUTONOMOUS_CONTROL_STATE = nav3v2_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory1' */
    nav3v2_DW.EMERGENCY_STATE = nav3v2_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory10' */
    nav3v2_DW.I = nav3v2_P.DataStoreMemory10_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory11' */
    nav3v2_DW.D = nav3v2_P.DataStoreMemory11_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory13' */
    nav3v2_DW.WAYPOINT_PRECISION = nav3v2_P.DataStoreMemory13_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory12' */
    nav3v2_DW.goalHeading[0] = nav3v2_P.DataStoreMemory12_InitialValue[0];

    /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
    nav3v2_DW.InitialPose[0] = nav3v2_P.DataStoreMemory14_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory12' */
    nav3v2_DW.goalHeading[1] = nav3v2_P.DataStoreMemory12_InitialValue[1];

    /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
    nav3v2_DW.InitialPose[1] = nav3v2_P.DataStoreMemory14_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory12' */
    nav3v2_DW.goalHeading[2] = nav3v2_P.DataStoreMemory12_InitialValue[2];

    /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
    nav3v2_DW.InitialPose[2] = nav3v2_P.DataStoreMemory14_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory15' */
    nav3v2_DW.LED_WAYPOINT_REACHED_STATE =
      nav3v2_P.DataStoreMemory15_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory16' */
    nav3v2_DW.LED_MISSION_PAUSED_STATE = nav3v2_P.DataStoreMemory16_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory17' */
    nav3v2_DW.LED_STARTING_STATE = nav3v2_P.DataStoreMemory17_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory18' */
    nav3v2_DW.LED_MISSION_COMPLETE_STATE =
      nav3v2_P.DataStoreMemory18_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory19' */
    nav3v2_DW.LED_ERROR_STATE = nav3v2_P.DataStoreMemory19_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory2' */
    nav3v2_DW.INITIALISATION_STATE = nav3v2_P.DataStoreMemory2_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory20' */
    nav3v2_DW.LED_EMERGENCY_STOP_STATE = nav3v2_P.DataStoreMemory20_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory21' */
    nav3v2_DW.LED_BATTERY_LOW_STATE = nav3v2_P.DataStoreMemory21_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory3' */
    nav3v2_DW.MANUAL_CONTROL_STATE = nav3v2_P.DataStoreMemory3_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory4' */
    nav3v2_DW.REVERSE_STATE = nav3v2_P.DataStoreMemory4_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory5' */
    nav3v2_DW.WAITING_STATE = nav3v2_P.DataStoreMemory5_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory6' */
    nav3v2_DW.LOOKAHEAD_DISTANCE = nav3v2_P.DataStoreMemory6_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory7' */
    nav3v2_DW.MAX_ANGULAR_VEL = nav3v2_P.DataStoreMemory7_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory8' */
    nav3v2_DW.MAX_LINEAR_VEL = nav3v2_P.DataStoreMemory8_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory9' */
    nav3v2_DW.P = nav3v2_P.DataStoreMemory9_InitialValue;
  }

  /* SystemInitialize for Atomic SubSystem: '<S94>/Live_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S108>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S111>/Out1' */
  nav3v2_B.In1 = nav3v2_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S108>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S94>/Live_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S95>/Simulation_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S113>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S115>/Out1' */
  nav3v2_B.In1_p = nav3v2_P.Out1_Y0_m;

  /* End of SystemInitialize for SubSystem: '<S113>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S95>/Simulation_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S91>/Live_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S96>/Enabled Subsystem' */
  nav3v2_EnabledSubsystem_Init(&nav3v2_B.EnabledSubsystem_m,
    &nav3v2_P.EnabledSubsystem_m);

  /* End of SystemInitialize for SubSystem: '<S96>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S91>/Live_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S3>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S76>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S77>/Out1' */
  nav3v2_B.In1_i = nav3v2_P.Out1_Y0_ix;

  /* End of SystemInitialize for SubSystem: '<S76>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S3>/Subscribe' */

  /* SystemInitialize for Atomic SubSystem: '<S93>/Live_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S105>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S106>/Out1' */
  nav3v2_B.In1_b = nav3v2_P.Out1_Y0_o;

  /* End of SystemInitialize for SubSystem: '<S105>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S93>/Live_Subscriber' */
  nav3v2_DW.is_AUTONOMOUS_CONTROL = nav3v2_IN_NO_ACTIVE_CHILD;
  nav3v2_DW.is_REVERSE = nav3v2_IN_NO_ACTIVE_CHILD;
  nav3v2_DW.is_active_c3_nav3v2 = 0U;
  nav3v2_DW.is_c3_nav3v2 = nav3v2_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for Chart: '<S2>/State_Machine' incorporates:
   *  SubSystem: '<S19>/checkAtGoal'
   */
  nav3v2_checkAtGoal_Init(&nav3v2_B.checkAtGoal, &nav3v2_P.checkAtGoal);

  /* SystemInitialize for Chart: '<S2>/State_Machine' incorporates:
   *  SubSystem: '<S19>/PID'
   */
  nav3v2_PID_Init(&nav3v2_B.PID, &nav3v2_DW.PID, &nav3v2_P.PID);

  /* SystemInitialize for Atomic SubSystem: '<S92>/Live_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S101>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S104>/Out1' */
  nav3v2_B.In1_a = nav3v2_P.Out1_Y0_i;

  /* End of SystemInitialize for SubSystem: '<S101>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S92>/Live_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S91>/Live_Subscriber1' */
  /* SystemInitialize for Enabled SubSystem: '<S97>/Enabled Subsystem' */
  nav3v2_EnabledSubsystem_Init(&nav3v2_B.EnabledSubsystem_d,
    &nav3v2_P.EnabledSubsystem_d);

  /* End of SystemInitialize for SubSystem: '<S97>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S91>/Live_Subscriber1' */

  /* SystemInitialize for Atomic SubSystem: '<S116>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S120>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S121>/Out1' */
  nav3v2_B.In1_d = nav3v2_P.Out1_Y0_c;

  /* End of SystemInitialize for SubSystem: '<S120>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S116>/Subscribe' */

  /* Enable for Chart: '<S2>/State_Machine' incorporates:
   *  SubSystem: '<S19>/PID'
   */
  nav3v2_PID_Enable(&nav3v2_DW.PID);
}

/* Model terminate function */
void nav3v2_terminate(void)
{
  /* Terminate for MATLABSystem: '<S84>/Get Parameter1' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_gl);

  /* Terminate for MATLABSystem: '<S84>/Get Parameter2' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_j);

  /* Terminate for MATLABSystem: '<S84>/Get Parameter3' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_a);

  /* Terminate for MATLABSystem: '<S84>/Get Parameter4' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_b);

  /* Terminate for MATLABSystem: '<S85>/Get Parameter6' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_ek);

  /* Terminate for MATLABSystem: '<S85>/Get Parameter5' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_lb);

  /* Terminate for MATLABSystem: '<S85>/Get Parameter2' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_kg);

  /* Terminate for MATLABSystem: '<S85>/Get Parameter3' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_ef);

  /* Terminate for MATLABSystem: '<S85>/Get Parameter1' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_g);

  /* Terminate for MATLABSystem: '<S85>/Get Parameter4' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_dr);

  /* Terminate for MATLABSystem: '<S85>/Get Parameter' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_p);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter4' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_n);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter3' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_l);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter11' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_l3);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter12' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_ls);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter13' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_k);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter2' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_e);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter1' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_f);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter5' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_e0);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter6' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_lq);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter10' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_na);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter9' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_i);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter8' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_i3);

  /* Terminate for MATLABSystem: '<S87>/Get Parameter7' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_m);

  /* Terminate for MATLABSystem: '<S86>/Get Parameter' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_mt);

  /* Terminate for MATLABSystem: '<S86>/Get Parameter1' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_h);

  /* Terminate for MATLABSystem: '<S86>/Get Parameter2' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_my);

  /* Terminate for MATLABSystem: '<S86>/Get Parameter3' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_d);

  /* Terminate for MATLABSystem: '<S86>/Get Parameter4' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_lm);

  /* Terminate for MATLABSystem: '<S86>/Get Parameter5' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj_kb);

  /* Terminate for MATLABSystem: '<S88>/Get Parameter5' */
  matlabCodegenHandle_mat_ikxi5sl(&nav3v2_DW.obj);

  /* Terminate for Atomic SubSystem: '<S94>/Live_Subscriber' */
  /* Terminate for MATLABSystem: '<S108>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_gt);

  /* End of Terminate for SubSystem: '<S94>/Live_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S95>/Simulation_Subscriber' */
  /* Terminate for MATLABSystem: '<S113>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_fy);

  /* End of Terminate for SubSystem: '<S95>/Simulation_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S91>/Live_Subscriber' */
  /* Terminate for MATLABSystem: '<S96>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_nh);

  /* End of Terminate for SubSystem: '<S91>/Live_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S3>/Subscribe' */
  /* Terminate for MATLABSystem: '<S76>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_mf);

  /* End of Terminate for SubSystem: '<S3>/Subscribe' */

  /* Terminate for Atomic SubSystem: '<S93>/Live_Subscriber' */
  /* Terminate for MATLABSystem: '<S105>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_do);

  /* End of Terminate for SubSystem: '<S93>/Live_Subscriber' */

  /* Terminate for Enabled SubSystem: '<S1>/Real_Actuation' */
  /* Terminate for Atomic SubSystem: '<S13>/Publish' */
  /* Terminate for MATLABSystem: '<S16>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3v2_DW.obj_ft);

  /* End of Terminate for SubSystem: '<S13>/Publish' */
  /* End of Terminate for SubSystem: '<S1>/Real_Actuation' */

  /* Terminate for Enabled SubSystem: '<S1>/Simulation_Actuation' */
  /* Terminate for Atomic SubSystem: '<S10>/Publish' */
  /* Terminate for MATLABSystem: '<S18>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3v2_DW.obj_gv);

  /* End of Terminate for SubSystem: '<S10>/Publish' */
  /* End of Terminate for SubSystem: '<S1>/Simulation_Actuation' */

  /* Terminate for Atomic SubSystem: '<S8>/Publish2' */
  /* Terminate for MATLABSystem: '<S12>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3v2_DW.obj_nw);

  /* End of Terminate for SubSystem: '<S8>/Publish2' */

  /* Terminate for Atomic SubSystem: '<S92>/Live_Subscriber' */
  /* Terminate for MATLABSystem: '<S101>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_dm);

  /* End of Terminate for SubSystem: '<S92>/Live_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S91>/Live_Subscriber1' */
  /* Terminate for MATLABSystem: '<S97>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_n4);

  /* End of Terminate for SubSystem: '<S91>/Live_Subscriber1' */

  /* Terminate for Atomic SubSystem: '<S4>/Publish' */
  /* Terminate for MATLABSystem: '<S81>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3v2_DW.obj_o);

  /* End of Terminate for SubSystem: '<S4>/Publish' */

  /* Terminate for Atomic SubSystem: '<S4>/Publish1' */
  /* Terminate for MATLABSystem: '<S82>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3v2_DW.obj_jr);

  /* End of Terminate for SubSystem: '<S4>/Publish1' */

  /* Terminate for Atomic SubSystem: '<S4>/Publish2' */
  /* Terminate for MATLABSystem: '<S83>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3v2_DW.obj_c);

  /* End of Terminate for SubSystem: '<S4>/Publish2' */

  /* Terminate for Atomic SubSystem: '<S116>/Publish' */
  /* Terminate for MATLABSystem: '<S118>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3v2_DW.obj_jx);

  /* End of Terminate for SubSystem: '<S116>/Publish' */

  /* Terminate for Atomic SubSystem: '<S116>/Subscribe' */
  /* Terminate for MATLABSystem: '<S120>/SourceBlock' */
  matlabCodegenHandle_matlabC_ikx(&nav3v2_DW.obj_jp);

  /* End of Terminate for SubSystem: '<S116>/Subscribe' */

  /* Terminate for Enabled SubSystem: '<S116>/RESET_IMU' */
  /* Terminate for MATLABSystem: '<S119>/Set Parameter' */
  nav3v2_matlabCodegenHa_f0(&nav3v2_DW.obj_cw);

  /* End of Terminate for SubSystem: '<S116>/RESET_IMU' */
}
