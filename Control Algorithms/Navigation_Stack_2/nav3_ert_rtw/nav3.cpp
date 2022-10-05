/*
 * nav3.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "nav3".
 *
 * Model version              : 1.40
 * Simulink Coder version : 9.3 (R2020a) 18-Nov-2019
 * C++ source code generated on : Tue Oct  4 14:45:45 2022
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "nav3.h"
#include "nav3_private.h"

/* Named constants for Chart: '<S2>/State_Machine' */
const uint8_T na_IN_PurePursuitMoveToWaypoint = 3U;
const uint8_T nav3_IN_AUTONOMOUS_CONTROL = 1U;
const uint8_T nav3_IN_EMERGENCY = 2U;
const uint8_T nav3_IN_INITIALISATION = 3U;
const uint8_T nav3_IN_MANUAL_CONTROL = 4U;
const uint8_T nav3_IN_MoveToWaypoint = 1U;
const uint8_T nav3_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T nav3_IN_PreciseMoveToWaypoint = 2U;
const uint8_T nav3_IN_PreciseMoveToWaypoint_f = 1U;
const uint8_T nav3_IN_PreciseRotateToWaypoint = 2U;
const uint8_T nav3_IN_REVERSE = 5U;
const uint8_T nav3_IN_ReachedWaypoint = 3U;
const uint8_T nav3_IN_ReachedWaypoint_b = 4U;
const uint8_T nav3_IN_RotateToWaypoint = 4U;
const uint8_T nav3_IN_RotateToWaypoint_i = 5U;
const uint8_T nav3_IN_WAITING = 6U;

/* Block signals (default storage) */
B_nav3_T nav3_B;

/* Block states (default storage) */
DW_nav3_T nav3_DW;

/* Real-time model */
RT_MODEL_nav3_T nav3_M_ = RT_MODEL_nav3_T();
RT_MODEL_nav3_T *const nav3_M = &nav3_M_;

/* Forward declaration for local functions */
static boolean_T nav3_anyNonFinite(const real_T x[16]);
static real_T nav3_rt_hypotd_snf(real_T u0, real_T u1);
static real_T nav3_xzlangeM(const creal_T x[16]);
static void nav3_xzlascl(real_T cfrom, real_T cto, creal_T A[16]);
static real_T nav3_xzlanhs(const creal_T A[16], int32_T ilo, int32_T ihi);
static void nav3_xzlartg_g(const creal_T f, const creal_T g, real_T *cs, creal_T
  *sn);
static void nav3_xzlartg(const creal_T f, const creal_T g, real_T *cs, creal_T
  *sn, creal_T *r);
static void nav3_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi, creal_T Z[16],
  int32_T *info, creal_T alpha1[4], creal_T beta1[4]);
static void nav3_xztgevc(const creal_T A[16], creal_T V[16]);
static void nav3_xzggev(creal_T A[16], int32_T *info, creal_T alpha1[4], creal_T
  beta1[4], creal_T V[16]);
static real_T nav3_xnrm2(int32_T n, const real_T x[16], int32_T ix0);
static void nav3_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau, real_T C
  [16], int32_T ic0, real_T work[4]);
static void nav3_xgehrd(real_T a[16], real_T tau[3]);
static real_T nav3_xnrm2_j(int32_T n, const real_T x[3]);
static real_T nav3_xzlarfg(int32_T n, real_T *alpha1, real_T x[3]);
static void nav3_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *d, real_T
  *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *sn);
static void nav3_xrot(int32_T n, real_T x[16], int32_T ix0, int32_T iy0, real_T
                      c, real_T s);
static void nav3_xrot_o(int32_T n, real_T x[16], int32_T ix0, int32_T iy0,
  real_T c, real_T s);
static int32_T nav3_eml_dlahqr(real_T h[16], real_T z[16]);
static void nav3_eig(const real_T A[16], creal_T V[16], creal_T D[4]);
static real_T nav3_rt_atan2d_snf(real_T u0, real_T u1);
static real_T nav3_CalculateHeading(const real_T startPose[3], const real_T
  endPose[2]);
static real_T nav3_norm(const real_T x[2]);
static real_T nav3_closestPointOnLine(const real_T pt1[2], real_T pt2[2], const
  real_T refPt[2]);
static void nav3_purePursuit_b(real_T lookAhead, const real_T waypoints[6],
  const real_T pose[3], real_T targetHeading[3]);
static real_T nav3_mod(real_T x);
static void nav3_flip(real_T x[18]);
static void nav3_AUTONOMOUS_CONTROL(void);
static void nav3_purePursuit(real_T lookAhead, const real_T waypoints[4], const
  real_T pose[3], real_T targetHeading[3]);
static void matlabCodegenHandle_matlab_kgkc(ros_slros_internal_block_GetP_T *obj);
static void matlabCodegenHandle_matlabCo_kg(ros_slros_internal_block_Subs_T *obj);
static void matlabCodegenHandle_matlabCodeg(ros_slros_internal_block_Publ_T *obj);
static void nav3_matlabCodegenHa_en(ros_slros_internal_block_SetP_T *obj);
int32_T div_nzp_s32(int32_T numerator, int32_T denominator)
{
  uint32_T tempAbsQuotient;
  tempAbsQuotient = (numerator < 0 ? ~static_cast<uint32_T>(numerator) + 1U :
                     static_cast<uint32_T>(numerator)) / (denominator < 0 ? ~
    static_cast<uint32_T>(denominator) + 1U : static_cast<uint32_T>(denominator));
  return (numerator < 0) != (denominator < 0) ? -static_cast<int32_T>
    (tempAbsQuotient) : static_cast<int32_T>(tempAbsQuotient);
}

/* System initialize for function-call system: '<S16>/checkAtGoal' */
void nav3_checkAtGoal_Init(B_checkAtGoal_nav3_T *localB, P_checkAtGoal_nav3_T
  *localP)
{
  /* SystemInitialize for Outport: '<S18>/atGoal' */
  localB->LessThan = localP->atGoal_Y0;
}

/* Output and update for function-call system: '<S16>/checkAtGoal' */
void nav3_checkAtGoal(const real_T rtu_pose[3], const real_T rtu_goalWaypoint[2],
                      real_T rtu_goalTolerance, B_checkAtGoal_nav3_T *localB)
{
  real_T rtb_Subtract_0;
  real_T rtb_Subtract;

  /* Sum: '<S18>/Subtract' */
  rtb_Subtract = rtu_pose[0] - rtu_goalWaypoint[0];

  /* DotProduct: '<S18>/Dot Product' */
  rtb_Subtract_0 = rtb_Subtract * rtb_Subtract;

  /* Sum: '<S18>/Subtract' */
  rtb_Subtract = rtu_pose[1] - rtu_goalWaypoint[1];

  /* DotProduct: '<S18>/Dot Product' */
  rtb_Subtract_0 += rtb_Subtract * rtb_Subtract;

  /* RelationalOperator: '<S18>/Less Than' incorporates:
   *  DotProduct: '<S18>/Dot Product'
   *  Sqrt: '<S18>/Sqrt'
   */
  localB->LessThan = (sqrt(rtb_Subtract_0) >= rtu_goalTolerance);
}

/*
 * Output and update for atomic system:
 *    '<S17>/Normalise_Goal_Heading'
 *    '<S17>/Normalise_Heading'
 */
void nav3_Normalise_Goal_Heading(real_T rtu_headingAngle,
  B_Normalise_Goal_Heading_nav3_T *localB)
{
  localB->bearingAngle = rtu_headingAngle;
  if (rtu_headingAngle < 0.0) {
    localB->bearingAngle = (3.1415926535897931 - fabs(rtu_headingAngle)) +
      3.1415926535897931;
  }
}

/* System initialize for function-call system: '<S16>/PID' */
void nav3_PID_Init(B_PID_nav3_T *localB, DW_PID_nav3_T *localDW, P_PID_nav3_T
                   *localP)
{
  /* InitializeConditions for DiscreteIntegrator: '<S56>/Integrator' */
  localDW->Integrator_DSTATE = localP->DiscreteVaryingPID_InitialCondi;
  localDW->Integrator_PREV_U = 0.0;

  /* InitializeConditions for Delay: '<S49>/UD' */
  localDW->UD_DSTATE = localP->DiscreteVaryingPID_Differentiat;

  /* SystemInitialize for Outport: '<S17>/angVel' */
  localB->Saturation = localP->angVel_Y0;
}

/* Enable for function-call system: '<S16>/PID' */
void nav3_PID_Enable(DW_PID_nav3_T *localDW)
{
  localDW->PID_RESET_ELAPS_T = true;

  /* Enable for DiscreteIntegrator: '<S56>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE = 1U;
}

/* Output and update for function-call system: '<S16>/PID' */
void nav3_PID(RT_MODEL_nav3_T * const nav3_M, const real_T rtu_pose[3], real_T
              rtu_desiredHeading, real_T rtu_P, real_T rtu_I, real_T rtu_D,
              B_PID_nav3_T *localB, DW_PID_nav3_T *localDW, P_PID_nav3_T *localP)
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
    PID_ELAPS_T_tmp = nav3_M->Timing.clockTick0;
    localDW->PID_ELAPS_T[0] = PID_ELAPS_T_tmp - localDW->PID_PREV_T[0];
    elapseT_H = nav3_M->Timing.clockTickH0 - localDW->PID_PREV_T[1];
    if (localDW->PID_PREV_T[0] > PID_ELAPS_T_tmp) {
      elapseT_H--;
    }

    localDW->PID_ELAPS_T[1] = elapseT_H;
  }

  localDW->PID_PREV_T[0] = nav3_M->Timing.clockTick0;
  localDW->PID_PREV_T[1] = nav3_M->Timing.clockTickH0;
  localDW->PID_RESET_ELAPS_T = false;

  /* MATLAB Function: '<S17>/Normalise_Goal_Heading' */
  nav3_Normalise_Goal_Heading(rtu_desiredHeading,
    &localB->sf_Normalise_Goal_Heading);

  /* MATLAB Function: '<S17>/Normalise_Heading' */
  nav3_Normalise_Goal_Heading(rtu_pose[2], &localB->sf_Normalise_Heading);

  /* MATLAB Function: '<S17>/Calculate_Error' */
  rtb_error = localB->sf_Normalise_Goal_Heading.bearingAngle -
    localB->sf_Normalise_Heading.bearingAngle;
  if (rtb_error < -3.1415926535897931) {
    rtb_error += 6.2831853071795862;
  } else {
    if (rtb_error > 3.1415926535897931) {
      rtb_error -= 6.2831853071795862;
    }
  }

  /* End of MATLAB Function: '<S17>/Calculate_Error' */

  /* DiscreteIntegrator: '<S56>/Integrator' */
  if (localDW->Integrator_SYSTEM_ENABLE == 0) {
    localDW->Integrator_DSTATE += localP->Integrator_gainval * static_cast<
      real_T>(localDW->PID_ELAPS_T[0]) * localDW->Integrator_PREV_U;
  }

  /* End of DiscreteIntegrator: '<S56>/Integrator' */

  /* SampleTimeMath: '<S51>/Tsamp' incorporates:
   *  Product: '<S48>/DProd Out'
   *
   * About '<S51>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Tsamp = rtb_error * rtu_D / ((static_cast<real_T>(localDW->PID_ELAPS_T[0])
    * 0.2 + static_cast<real_T>(localDW->PID_ELAPS_T[1]) * 8.589934592E+8) *
    localP->Tsamp_WtEt);

  /* Sum: '<S65>/Sum' incorporates:
   *  Delay: '<S49>/UD'
   *  Product: '<S61>/PProd Out'
   *  Sum: '<S49>/Diff'
   */
  localB->Saturation = (rtb_error * rtu_P + localDW->Integrator_DSTATE) +
    (rtb_Tsamp - localDW->UD_DSTATE);

  /* DeadZone: '<S47>/DeadZone' */
  if (localB->Saturation > localP->DiscreteVaryingPID_UpperSaturat) {
    localDW->Integrator_PREV_U = localB->Saturation -
      localP->DiscreteVaryingPID_UpperSaturat;
  } else if (localB->Saturation >= localP->DiscreteVaryingPID_LowerSaturat) {
    localDW->Integrator_PREV_U = 0.0;
  } else {
    localDW->Integrator_PREV_U = localB->Saturation -
      localP->DiscreteVaryingPID_LowerSaturat;
  }

  /* End of DeadZone: '<S47>/DeadZone' */

  /* RelationalOperator: '<S47>/NotEqual' incorporates:
   *  Gain: '<S47>/ZeroGain'
   */
  rtb_NotEqual = (localP->ZeroGain_Gain * localB->Saturation !=
                  localDW->Integrator_PREV_U);

  /* Signum: '<S47>/SignPreSat' */
  if (localDW->Integrator_PREV_U < 0.0) {
    localDW->Integrator_PREV_U = -1.0;
  } else if (localDW->Integrator_PREV_U > 0.0) {
    localDW->Integrator_PREV_U = 1.0;
  } else if (localDW->Integrator_PREV_U == 0.0) {
    localDW->Integrator_PREV_U = 0.0;
  } else {
    localDW->Integrator_PREV_U = (rtNaN);
  }

  /* End of Signum: '<S47>/SignPreSat' */

  /* DataTypeConversion: '<S47>/DataTypeConv1' */
  if (rtIsNaN(localDW->Integrator_PREV_U)) {
    tmp = 0.0;
  } else {
    tmp = fmod(localDW->Integrator_PREV_U, 256.0);
  }

  /* Product: '<S53>/IProd Out' */
  localDW->Integrator_PREV_U = rtb_error * rtu_I;

  /* Saturate: '<S63>/Saturation' */
  if (localB->Saturation > localP->DiscreteVaryingPID_UpperSaturat) {
    localB->Saturation = localP->DiscreteVaryingPID_UpperSaturat;
  } else {
    if (localB->Saturation < localP->DiscreteVaryingPID_LowerSaturat) {
      localB->Saturation = localP->DiscreteVaryingPID_LowerSaturat;
    }
  }

  /* End of Saturate: '<S63>/Saturation' */

  /* Update for DiscreteIntegrator: '<S56>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE = 0U;

  /* Signum: '<S47>/SignPreIntegrator' */
  if (localDW->Integrator_PREV_U < 0.0) {
    /* DataTypeConversion: '<S47>/DataTypeConv2' */
    rtb_error = -1.0;
  } else if (localDW->Integrator_PREV_U > 0.0) {
    /* DataTypeConversion: '<S47>/DataTypeConv2' */
    rtb_error = 1.0;
  } else if (localDW->Integrator_PREV_U == 0.0) {
    /* DataTypeConversion: '<S47>/DataTypeConv2' */
    rtb_error = 0.0;
  } else {
    /* DataTypeConversion: '<S47>/DataTypeConv2' */
    rtb_error = (rtNaN);
  }

  /* End of Signum: '<S47>/SignPreIntegrator' */

  /* DataTypeConversion: '<S47>/DataTypeConv2' */
  if (rtIsNaN(rtb_error)) {
    rtb_error = 0.0;
  } else {
    rtb_error = fmod(rtb_error, 256.0);
  }

  /* Switch: '<S47>/Switch' incorporates:
   *  DataTypeConversion: '<S47>/DataTypeConv1'
   *  DataTypeConversion: '<S47>/DataTypeConv2'
   *  Logic: '<S47>/AND3'
   *  RelationalOperator: '<S47>/Equal1'
   */
  if (rtb_NotEqual && (static_cast<int8_T>(tmp < 0.0 ? static_cast<int32_T>(
         static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-tmp)))) :
        static_cast<int32_T>(static_cast<int8_T>(static_cast<uint8_T>(tmp)))) ==
                       (rtb_error < 0.0 ? static_cast<int32_T>
                        (static_cast<int8_T>(-static_cast<int8_T>
          (static_cast<uint8_T>(-rtb_error)))) : static_cast<int32_T>(
         static_cast<int8_T>(static_cast<uint8_T>(rtb_error)))))) {
    /* Update for DiscreteIntegrator: '<S56>/Integrator' incorporates:
     *  Constant: '<S47>/Constant1'
     */
    localDW->Integrator_PREV_U = localP->Constant1_Value;
  }

  /* End of Switch: '<S47>/Switch' */

  /* Update for Delay: '<S49>/UD' */
  localDW->UD_DSTATE = rtb_Tsamp;
}

/*
 * System initialize for enable system:
 *    '<S88>/Enabled Subsystem'
 *    '<S89>/Enabled Subsystem'
 */
void nav3_EnabledSubsystem_Init(B_EnabledSubsystem_nav3_T *localB,
  P_EnabledSubsystem_nav3_T *localP)
{
  /* SystemInitialize for Outport: '<S90>/Out1' */
  localB->In1 = localP->Out1_Y0;
}

/*
 * Output and update for enable system:
 *    '<S88>/Enabled Subsystem'
 *    '<S89>/Enabled Subsystem'
 */
void nav3_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_nav3_std_msgs_Float32 *rtu_In1, B_EnabledSubsystem_nav3_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S88>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S90>/Enable'
   */
  if (rtu_Enable) {
    /* Inport: '<S90>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S88>/Enabled Subsystem' */
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
 *    '<S92>/Quaternion_2_Euler'
 *    '<S97>/Quaternion_2_Euler'
 *    '<S102>/Quaternion_2_Euler'
 */
void nav3_Quaternion_2_Euler(real_T rtu_x, real_T rtu_y, real_T rtu_z, real_T
  rtu_w, B_Quaternion_2_Euler_nav3_T *localB)
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

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static boolean_T nav3_anyNonFinite(const real_T x[16])
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

static real_T nav3_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  nav3_B.a = fabs(u0);
  y = fabs(u1);
  if (nav3_B.a < y) {
    nav3_B.a /= y;
    y *= sqrt(nav3_B.a * nav3_B.a + 1.0);
  } else if (nav3_B.a > y) {
    y /= nav3_B.a;
    y = sqrt(y * y + 1.0) * nav3_B.a;
  } else {
    if (!rtIsNaN(y)) {
      y = nav3_B.a * 1.4142135623730951;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static real_T nav3_xzlangeM(const creal_T x[16])
{
  real_T y;
  real_T absxk;
  int32_T k;
  boolean_T exitg1;
  y = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 16)) {
    absxk = nav3_rt_hypotd_snf(x[k].re, x[k].im);
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

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xzlascl(real_T cfrom, real_T cto, creal_T A[16])
{
  nav3_B.cfromc = cfrom;
  nav3_B.ctoc = cto;
  nav3_B.notdone = true;
  while (nav3_B.notdone) {
    nav3_B.cfrom1 = nav3_B.cfromc * 2.0041683600089728E-292;
    nav3_B.cto1 = nav3_B.ctoc / 4.9896007738368E+291;
    if ((fabs(nav3_B.cfrom1) > fabs(nav3_B.ctoc)) && (nav3_B.ctoc != 0.0)) {
      nav3_B.mul_j = 2.0041683600089728E-292;
      nav3_B.cfromc = nav3_B.cfrom1;
    } else if (fabs(nav3_B.cto1) > fabs(nav3_B.cfromc)) {
      nav3_B.mul_j = 4.9896007738368E+291;
      nav3_B.ctoc = nav3_B.cto1;
    } else {
      nav3_B.mul_j = nav3_B.ctoc / nav3_B.cfromc;
      nav3_B.notdone = false;
    }

    for (nav3_B.i1 = 0; nav3_B.i1 < 16; nav3_B.i1++) {
      A[nav3_B.i1].re *= nav3_B.mul_j;
      A[nav3_B.i1].im *= nav3_B.mul_j;
    }
  }
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static real_T nav3_xzlanhs(const creal_T A[16], int32_T ilo, int32_T ihi)
{
  real_T f;
  f = 0.0;
  if (ilo <= ihi) {
    nav3_B.scale_j = 0.0;
    nav3_B.sumsq = 0.0;
    nav3_B.firstNonZero = true;
    nav3_B.j_l = ilo;
    while (nav3_B.j_l <= ihi) {
      nav3_B.b_m = nav3_B.j_l + 1;
      if (ihi < nav3_B.j_l + 1) {
        nav3_B.b_m = ihi;
      }

      nav3_B.i_mj = ilo;
      while (nav3_B.i_mj <= nav3_B.b_m) {
        nav3_B.reAij_tmp = (((nav3_B.j_l - 1) << 2) + nav3_B.i_mj) - 1;
        if (A[nav3_B.reAij_tmp].re != 0.0) {
          nav3_B.temp1 = fabs(A[nav3_B.reAij_tmp].re);
          if (nav3_B.firstNonZero) {
            nav3_B.sumsq = 1.0;
            nav3_B.scale_j = nav3_B.temp1;
            nav3_B.firstNonZero = false;
          } else if (nav3_B.scale_j < nav3_B.temp1) {
            nav3_B.temp2 = nav3_B.scale_j / nav3_B.temp1;
            nav3_B.sumsq = nav3_B.sumsq * nav3_B.temp2 * nav3_B.temp2 + 1.0;
            nav3_B.scale_j = nav3_B.temp1;
          } else {
            nav3_B.temp2 = nav3_B.temp1 / nav3_B.scale_j;
            nav3_B.sumsq += nav3_B.temp2 * nav3_B.temp2;
          }
        }

        if (A[nav3_B.reAij_tmp].im != 0.0) {
          nav3_B.temp1 = fabs(A[nav3_B.reAij_tmp].im);
          if (nav3_B.firstNonZero) {
            nav3_B.sumsq = 1.0;
            nav3_B.scale_j = nav3_B.temp1;
            nav3_B.firstNonZero = false;
          } else if (nav3_B.scale_j < nav3_B.temp1) {
            nav3_B.temp2 = nav3_B.scale_j / nav3_B.temp1;
            nav3_B.sumsq = nav3_B.sumsq * nav3_B.temp2 * nav3_B.temp2 + 1.0;
            nav3_B.scale_j = nav3_B.temp1;
          } else {
            nav3_B.temp2 = nav3_B.temp1 / nav3_B.scale_j;
            nav3_B.sumsq += nav3_B.temp2 * nav3_B.temp2;
          }
        }

        nav3_B.i_mj++;
      }

      nav3_B.j_l++;
    }

    f = nav3_B.scale_j * sqrt(nav3_B.sumsq);
  }

  return f;
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xzlartg_g(const creal_T f, const creal_T g, real_T *cs, creal_T
  *sn)
{
  boolean_T guard1 = false;
  nav3_B.f2s_b = fabs(f.re);
  nav3_B.di_d = fabs(f.im);
  nav3_B.scale_ln = nav3_B.f2s_b;
  if (nav3_B.di_d > nav3_B.f2s_b) {
    nav3_B.scale_ln = nav3_B.di_d;
  }

  nav3_B.gs_re_f = fabs(g.re);
  nav3_B.gs_im_a = fabs(g.im);
  if (nav3_B.gs_im_a > nav3_B.gs_re_f) {
    nav3_B.gs_re_f = nav3_B.gs_im_a;
  }

  if (nav3_B.gs_re_f > nav3_B.scale_ln) {
    nav3_B.scale_ln = nav3_B.gs_re_f;
  }

  nav3_B.fs_re_b = f.re;
  nav3_B.fs_im_j = f.im;
  nav3_B.gs_re_f = g.re;
  nav3_B.gs_im_a = g.im;
  guard1 = false;
  if (nav3_B.scale_ln >= 7.4428285367870146E+137) {
    do {
      nav3_B.fs_re_b *= 1.3435752215134178E-138;
      nav3_B.fs_im_j *= 1.3435752215134178E-138;
      nav3_B.gs_re_f *= 1.3435752215134178E-138;
      nav3_B.gs_im_a *= 1.3435752215134178E-138;
      nav3_B.scale_ln *= 1.3435752215134178E-138;
    } while (!(nav3_B.scale_ln < 7.4428285367870146E+137));

    guard1 = true;
  } else if (nav3_B.scale_ln <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
    } else {
      do {
        nav3_B.fs_re_b *= 7.4428285367870146E+137;
        nav3_B.fs_im_j *= 7.4428285367870146E+137;
        nav3_B.gs_re_f *= 7.4428285367870146E+137;
        nav3_B.gs_im_a *= 7.4428285367870146E+137;
        nav3_B.scale_ln *= 7.4428285367870146E+137;
      } while (!(nav3_B.scale_ln > 1.3435752215134178E-138));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    nav3_B.scale_ln = nav3_B.fs_re_b * nav3_B.fs_re_b + nav3_B.fs_im_j *
      nav3_B.fs_im_j;
    nav3_B.g2_h = nav3_B.gs_re_f * nav3_B.gs_re_f + nav3_B.gs_im_a *
      nav3_B.gs_im_a;
    nav3_B.x_e = nav3_B.g2_h;
    if (1.0 > nav3_B.g2_h) {
      nav3_B.x_e = 1.0;
    }

    if (nav3_B.scale_ln <= nav3_B.x_e * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        nav3_B.f2s_b = nav3_rt_hypotd_snf(nav3_B.gs_re_f, nav3_B.gs_im_a);
        sn->re = nav3_B.gs_re_f / nav3_B.f2s_b;
        sn->im = -nav3_B.gs_im_a / nav3_B.f2s_b;
      } else {
        nav3_B.scale_ln = sqrt(nav3_B.g2_h);
        *cs = nav3_rt_hypotd_snf(nav3_B.fs_re_b, nav3_B.fs_im_j) /
          nav3_B.scale_ln;
        if (nav3_B.di_d > nav3_B.f2s_b) {
          nav3_B.f2s_b = nav3_B.di_d;
        }

        if (nav3_B.f2s_b > 1.0) {
          nav3_B.f2s_b = nav3_rt_hypotd_snf(f.re, f.im);
          nav3_B.fs_re_b = f.re / nav3_B.f2s_b;
          nav3_B.fs_im_j = f.im / nav3_B.f2s_b;
        } else {
          nav3_B.fs_re_b = 7.4428285367870146E+137 * f.re;
          nav3_B.di_d = 7.4428285367870146E+137 * f.im;
          nav3_B.f2s_b = nav3_rt_hypotd_snf(nav3_B.fs_re_b, nav3_B.di_d);
          nav3_B.fs_re_b /= nav3_B.f2s_b;
          nav3_B.fs_im_j = nav3_B.di_d / nav3_B.f2s_b;
        }

        nav3_B.gs_re_f /= nav3_B.scale_ln;
        nav3_B.gs_im_a = -nav3_B.gs_im_a / nav3_B.scale_ln;
        sn->re = nav3_B.fs_re_b * nav3_B.gs_re_f - nav3_B.fs_im_j *
          nav3_B.gs_im_a;
        sn->im = nav3_B.fs_re_b * nav3_B.gs_im_a + nav3_B.fs_im_j *
          nav3_B.gs_re_f;
      }
    } else {
      nav3_B.f2s_b = sqrt(nav3_B.g2_h / nav3_B.scale_ln + 1.0);
      nav3_B.fs_re_b *= nav3_B.f2s_b;
      nav3_B.fs_im_j *= nav3_B.f2s_b;
      *cs = 1.0 / nav3_B.f2s_b;
      nav3_B.f2s_b = nav3_B.scale_ln + nav3_B.g2_h;
      nav3_B.fs_re_b /= nav3_B.f2s_b;
      nav3_B.fs_im_j /= nav3_B.f2s_b;
      sn->re = nav3_B.fs_re_b * nav3_B.gs_re_f - nav3_B.fs_im_j *
        -nav3_B.gs_im_a;
      sn->im = nav3_B.fs_re_b * -nav3_B.gs_im_a + nav3_B.fs_im_j *
        nav3_B.gs_re_f;
    }
  }
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xzlartg(const creal_T f, const creal_T g, real_T *cs, creal_T
  *sn, creal_T *r)
{
  boolean_T guard1 = false;
  nav3_B.f2s = fabs(f.re);
  nav3_B.di = fabs(f.im);
  nav3_B.scale_l = nav3_B.f2s;
  if (nav3_B.di > nav3_B.f2s) {
    nav3_B.scale_l = nav3_B.di;
  }

  nav3_B.gs_re = fabs(g.re);
  nav3_B.gs_im = fabs(g.im);
  if (nav3_B.gs_im > nav3_B.gs_re) {
    nav3_B.gs_re = nav3_B.gs_im;
  }

  if (nav3_B.gs_re > nav3_B.scale_l) {
    nav3_B.scale_l = nav3_B.gs_re;
  }

  nav3_B.fs_re = f.re;
  nav3_B.fs_im = f.im;
  nav3_B.gs_re = g.re;
  nav3_B.gs_im = g.im;
  nav3_B.count = -1;
  nav3_B.rescaledir = 0;
  guard1 = false;
  if (nav3_B.scale_l >= 7.4428285367870146E+137) {
    do {
      nav3_B.count++;
      nav3_B.fs_re *= 1.3435752215134178E-138;
      nav3_B.fs_im *= 1.3435752215134178E-138;
      nav3_B.gs_re *= 1.3435752215134178E-138;
      nav3_B.gs_im *= 1.3435752215134178E-138;
      nav3_B.scale_l *= 1.3435752215134178E-138;
    } while (!(nav3_B.scale_l < 7.4428285367870146E+137));

    nav3_B.rescaledir = 1;
    guard1 = true;
  } else if (nav3_B.scale_l <= 1.3435752215134178E-138) {
    if ((g.re == 0.0) && (g.im == 0.0)) {
      *cs = 1.0;
      sn->re = 0.0;
      sn->im = 0.0;
      *r = f;
    } else {
      do {
        nav3_B.count++;
        nav3_B.fs_re *= 7.4428285367870146E+137;
        nav3_B.fs_im *= 7.4428285367870146E+137;
        nav3_B.gs_re *= 7.4428285367870146E+137;
        nav3_B.gs_im *= 7.4428285367870146E+137;
        nav3_B.scale_l *= 7.4428285367870146E+137;
      } while (!(nav3_B.scale_l > 1.3435752215134178E-138));

      nav3_B.rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    nav3_B.scale_l = nav3_B.fs_re * nav3_B.fs_re + nav3_B.fs_im * nav3_B.fs_im;
    nav3_B.g2 = nav3_B.gs_re * nav3_B.gs_re + nav3_B.gs_im * nav3_B.gs_im;
    nav3_B.x = nav3_B.g2;
    if (1.0 > nav3_B.g2) {
      nav3_B.x = 1.0;
    }

    if (nav3_B.scale_l <= nav3_B.x * 2.0041683600089728E-292) {
      if ((f.re == 0.0) && (f.im == 0.0)) {
        *cs = 0.0;
        r->re = nav3_rt_hypotd_snf(g.re, g.im);
        r->im = 0.0;
        nav3_B.f2s = nav3_rt_hypotd_snf(nav3_B.gs_re, nav3_B.gs_im);
        sn->re = nav3_B.gs_re / nav3_B.f2s;
        sn->im = -nav3_B.gs_im / nav3_B.f2s;
      } else {
        nav3_B.scale_l = sqrt(nav3_B.g2);
        *cs = nav3_rt_hypotd_snf(nav3_B.fs_re, nav3_B.fs_im) / nav3_B.scale_l;
        if (nav3_B.di > nav3_B.f2s) {
          nav3_B.f2s = nav3_B.di;
        }

        if (nav3_B.f2s > 1.0) {
          nav3_B.f2s = nav3_rt_hypotd_snf(f.re, f.im);
          nav3_B.fs_re = f.re / nav3_B.f2s;
          nav3_B.fs_im = f.im / nav3_B.f2s;
        } else {
          nav3_B.fs_re = 7.4428285367870146E+137 * f.re;
          nav3_B.di = 7.4428285367870146E+137 * f.im;
          nav3_B.f2s = nav3_rt_hypotd_snf(nav3_B.fs_re, nav3_B.di);
          nav3_B.fs_re /= nav3_B.f2s;
          nav3_B.fs_im = nav3_B.di / nav3_B.f2s;
        }

        nav3_B.gs_re /= nav3_B.scale_l;
        nav3_B.gs_im = -nav3_B.gs_im / nav3_B.scale_l;
        sn->re = nav3_B.fs_re * nav3_B.gs_re - nav3_B.fs_im * nav3_B.gs_im;
        sn->im = nav3_B.fs_re * nav3_B.gs_im + nav3_B.fs_im * nav3_B.gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      nav3_B.f2s = sqrt(nav3_B.g2 / nav3_B.scale_l + 1.0);
      r->re = nav3_B.f2s * nav3_B.fs_re;
      r->im = nav3_B.f2s * nav3_B.fs_im;
      *cs = 1.0 / nav3_B.f2s;
      nav3_B.f2s = nav3_B.scale_l + nav3_B.g2;
      nav3_B.fs_re = r->re / nav3_B.f2s;
      nav3_B.f2s = r->im / nav3_B.f2s;
      sn->re = nav3_B.fs_re * nav3_B.gs_re - nav3_B.f2s * -nav3_B.gs_im;
      sn->im = nav3_B.fs_re * -nav3_B.gs_im + nav3_B.f2s * nav3_B.gs_re;
      if (nav3_B.rescaledir > 0) {
        nav3_B.rescaledir = 0;
        while (nav3_B.rescaledir <= nav3_B.count) {
          r->re *= 7.4428285367870146E+137;
          r->im *= 7.4428285367870146E+137;
          nav3_B.rescaledir++;
        }
      } else {
        if (nav3_B.rescaledir < 0) {
          nav3_B.rescaledir = 0;
          while (nav3_B.rescaledir <= nav3_B.count) {
            r->re *= 1.3435752215134178E-138;
            r->im *= 1.3435752215134178E-138;
            nav3_B.rescaledir++;
          }
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xzhgeqz(creal_T A[16], int32_T ilo, int32_T ihi, creal_T Z[16],
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
  nav3_B.eshift_re = 0.0;
  nav3_B.eshift_im = 0.0;
  nav3_B.ctemp.re = 0.0;
  nav3_B.ctemp.im = 0.0;
  nav3_B.anorm = nav3_xzlanhs(A, ilo, ihi);
  nav3_B.shift_re = 2.2204460492503131E-16 * nav3_B.anorm;
  nav3_B.b_atol = 2.2250738585072014E-308;
  if (nav3_B.shift_re > 2.2250738585072014E-308) {
    nav3_B.b_atol = nav3_B.shift_re;
  }

  nav3_B.shift_re = 2.2250738585072014E-308;
  if (nav3_B.anorm > 2.2250738585072014E-308) {
    nav3_B.shift_re = nav3_B.anorm;
  }

  nav3_B.anorm = 1.0 / nav3_B.shift_re;
  nav3_B.failed = true;
  nav3_B.ilast = ihi;
  while (nav3_B.ilast + 1 < 5) {
    alpha1[nav3_B.ilast] = A[(nav3_B.ilast << 2) + nav3_B.ilast];
    nav3_B.ilast++;
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    nav3_B.ifirst = ilo;
    nav3_B.istart = ilo;
    nav3_B.ilast = ihi - 1;
    nav3_B.ilastm1 = ihi - 2;
    nav3_B.iiter = 0;
    nav3_B.goto60 = false;
    nav3_B.goto70 = false;
    nav3_B.goto90 = false;
    nav3_B.jiter = 0;
    do {
      exitg1 = 0;
      if (nav3_B.jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        if (nav3_B.ilast + 1 == ilo) {
          nav3_B.goto60 = true;
        } else {
          nav3_B.jp1 = (nav3_B.ilastm1 << 2) + nav3_B.ilast;
          if (fabs(A[nav3_B.jp1].re) + fabs(A[nav3_B.jp1].im) <= nav3_B.b_atol)
          {
            A[nav3_B.jp1].re = 0.0;
            A[nav3_B.jp1].im = 0.0;
            nav3_B.goto60 = true;
          } else {
            nav3_B.j = nav3_B.ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (nav3_B.j + 1 >= ilo)) {
              if (nav3_B.j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                nav3_B.jp1 = ((nav3_B.j - 1) << 2) + nav3_B.j;
                if (fabs(A[nav3_B.jp1].re) + fabs(A[nav3_B.jp1].im) <=
                    nav3_B.b_atol) {
                  A[nav3_B.jp1].re = 0.0;
                  A[nav3_B.jp1].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  nav3_B.j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              nav3_B.ifirst = nav3_B.j + 1;
              nav3_B.goto70 = true;
            }
          }
        }

        if ((!nav3_B.goto60) && (!nav3_B.goto70)) {
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
          for (nav3_B.jp1 = 0; nav3_B.jp1 < 16; nav3_B.jp1++) {
            Z[nav3_B.jp1].re = (rtNaN);
            Z[nav3_B.jp1].im = 0.0;
          }

          *info = 1;
          exitg1 = 1;
        } else if (nav3_B.goto60) {
          nav3_B.goto60 = false;
          alpha1[nav3_B.ilast] = A[(nav3_B.ilast << 2) + nav3_B.ilast];
          nav3_B.ilast = nav3_B.ilastm1;
          nav3_B.ilastm1--;
          if (nav3_B.ilast + 1 < ilo) {
            nav3_B.failed = false;
            guard2 = true;
            exitg1 = 1;
          } else {
            nav3_B.iiter = 0;
            nav3_B.eshift_re = 0.0;
            nav3_B.eshift_im = 0.0;
            nav3_B.jiter++;
          }
        } else {
          if (nav3_B.goto70) {
            nav3_B.goto70 = false;
            nav3_B.iiter++;
            if (nav3_B.iiter - div_nzp_s32(nav3_B.iiter, 10) * 10 != 0) {
              nav3_B.j = (nav3_B.ilastm1 << 2) + nav3_B.ilastm1;
              nav3_B.ar = A[nav3_B.j].re * nav3_B.anorm;
              nav3_B.ai = A[nav3_B.j].im * nav3_B.anorm;
              if (nav3_B.ai == 0.0) {
                nav3_B.shift_re = nav3_B.ar / 0.5;
                nav3_B.shift_im = 0.0;
              } else if (nav3_B.ar == 0.0) {
                nav3_B.shift_re = 0.0;
                nav3_B.shift_im = nav3_B.ai / 0.5;
              } else {
                nav3_B.shift_re = nav3_B.ar / 0.5;
                nav3_B.shift_im = nav3_B.ai / 0.5;
              }

              nav3_B.j = (nav3_B.ilast << 2) + nav3_B.ilast;
              nav3_B.ar = A[nav3_B.j].re * nav3_B.anorm;
              nav3_B.ai = A[nav3_B.j].im * nav3_B.anorm;
              if (nav3_B.ai == 0.0) {
                nav3_B.ad22.re = nav3_B.ar / 0.5;
                nav3_B.ad22.im = 0.0;
              } else if (nav3_B.ar == 0.0) {
                nav3_B.ad22.re = 0.0;
                nav3_B.ad22.im = nav3_B.ai / 0.5;
              } else {
                nav3_B.ad22.re = nav3_B.ar / 0.5;
                nav3_B.ad22.im = nav3_B.ai / 0.5;
              }

              nav3_B.t1_re = (nav3_B.shift_re + nav3_B.ad22.re) * 0.5;
              nav3_B.t1_im = (nav3_B.shift_im + nav3_B.ad22.im) * 0.5;
              nav3_B.j = (nav3_B.ilast << 2) + nav3_B.ilastm1;
              nav3_B.ar = A[nav3_B.j].re * nav3_B.anorm;
              nav3_B.ai = A[nav3_B.j].im * nav3_B.anorm;
              if (nav3_B.ai == 0.0) {
                nav3_B.absxr = nav3_B.ar / 0.5;
                nav3_B.absxi = 0.0;
              } else if (nav3_B.ar == 0.0) {
                nav3_B.absxr = 0.0;
                nav3_B.absxi = nav3_B.ai / 0.5;
              } else {
                nav3_B.absxr = nav3_B.ar / 0.5;
                nav3_B.absxi = nav3_B.ai / 0.5;
              }

              nav3_B.j = (nav3_B.ilastm1 << 2) + nav3_B.ilast;
              nav3_B.ar = A[nav3_B.j].re * nav3_B.anorm;
              nav3_B.ai = A[nav3_B.j].im * nav3_B.anorm;
              if (nav3_B.ai == 0.0) {
                nav3_B.ar /= 0.5;
                nav3_B.ai = 0.0;
              } else if (nav3_B.ar == 0.0) {
                nav3_B.ar = 0.0;
                nav3_B.ai /= 0.5;
              } else {
                nav3_B.ar /= 0.5;
                nav3_B.ai /= 0.5;
              }

              nav3_B.shift_im_p = nav3_B.shift_re * nav3_B.ad22.im +
                nav3_B.shift_im * nav3_B.ad22.re;
              nav3_B.shift_re = ((nav3_B.t1_re * nav3_B.t1_re - nav3_B.t1_im *
                                  nav3_B.t1_im) + (nav3_B.absxr * nav3_B.ar -
                nav3_B.absxi * nav3_B.ai)) - (nav3_B.shift_re * nav3_B.ad22.re -
                nav3_B.shift_im * nav3_B.ad22.im);
              nav3_B.shift_im = nav3_B.t1_re * nav3_B.t1_im;
              nav3_B.shift_im = ((nav3_B.shift_im + nav3_B.shift_im) +
                                 (nav3_B.absxr * nav3_B.ai + nav3_B.absxi *
                                  nav3_B.ar)) - nav3_B.shift_im_p;
              if (nav3_B.shift_im == 0.0) {
                if (nav3_B.shift_re < 0.0) {
                  nav3_B.absxr = 0.0;
                  nav3_B.absxi = sqrt(-nav3_B.shift_re);
                } else {
                  nav3_B.absxr = sqrt(nav3_B.shift_re);
                  nav3_B.absxi = 0.0;
                }
              } else if (nav3_B.shift_re == 0.0) {
                if (nav3_B.shift_im < 0.0) {
                  nav3_B.absxr = sqrt(-nav3_B.shift_im / 2.0);
                  nav3_B.absxi = -nav3_B.absxr;
                } else {
                  nav3_B.absxr = sqrt(nav3_B.shift_im / 2.0);
                  nav3_B.absxi = nav3_B.absxr;
                }
              } else if (rtIsNaN(nav3_B.shift_re)) {
                nav3_B.absxr = nav3_B.shift_re;
                nav3_B.absxi = nav3_B.shift_re;
              } else if (rtIsNaN(nav3_B.shift_im)) {
                nav3_B.absxr = nav3_B.shift_im;
                nav3_B.absxi = nav3_B.shift_im;
              } else if (rtIsInf(nav3_B.shift_im)) {
                nav3_B.absxr = fabs(nav3_B.shift_im);
                nav3_B.absxi = nav3_B.shift_im;
              } else if (rtIsInf(nav3_B.shift_re)) {
                if (nav3_B.shift_re < 0.0) {
                  nav3_B.absxr = 0.0;
                  nav3_B.absxi = nav3_B.shift_im * -nav3_B.shift_re;
                } else {
                  nav3_B.absxr = nav3_B.shift_re;
                  nav3_B.absxi = 0.0;
                }
              } else {
                nav3_B.absxr = fabs(nav3_B.shift_re);
                nav3_B.absxi = fabs(nav3_B.shift_im);
                if ((nav3_B.absxr > 4.4942328371557893E+307) || (nav3_B.absxi >
                     4.4942328371557893E+307)) {
                  nav3_B.absxr *= 0.5;
                  nav3_B.absxi *= 0.5;
                  nav3_B.absxi = nav3_rt_hypotd_snf(nav3_B.absxr, nav3_B.absxi);
                  if (nav3_B.absxi > nav3_B.absxr) {
                    nav3_B.absxr = sqrt(nav3_B.absxr / nav3_B.absxi + 1.0) *
                      sqrt(nav3_B.absxi);
                  } else {
                    nav3_B.absxr = sqrt(nav3_B.absxi) * 1.4142135623730951;
                  }
                } else {
                  nav3_B.absxr = sqrt((nav3_rt_hypotd_snf(nav3_B.absxr,
                    nav3_B.absxi) + nav3_B.absxr) * 0.5);
                }

                if (nav3_B.shift_re > 0.0) {
                  nav3_B.absxi = nav3_B.shift_im / nav3_B.absxr * 0.5;
                } else {
                  if (nav3_B.shift_im < 0.0) {
                    nav3_B.absxi = -nav3_B.absxr;
                  } else {
                    nav3_B.absxi = nav3_B.absxr;
                  }

                  nav3_B.absxr = nav3_B.shift_im / nav3_B.absxi * 0.5;
                }
              }

              if ((nav3_B.t1_re - nav3_B.ad22.re) * nav3_B.absxr + (nav3_B.t1_im
                   - nav3_B.ad22.im) * nav3_B.absxi <= 0.0) {
                nav3_B.shift_re = nav3_B.t1_re + nav3_B.absxr;
                nav3_B.shift_im = nav3_B.t1_im + nav3_B.absxi;
              } else {
                nav3_B.shift_re = nav3_B.t1_re - nav3_B.absxr;
                nav3_B.shift_im = nav3_B.t1_im - nav3_B.absxi;
              }
            } else {
              nav3_B.j = (nav3_B.ilastm1 << 2) + nav3_B.ilast;
              nav3_B.ar = A[nav3_B.j].re * nav3_B.anorm;
              nav3_B.ai = A[nav3_B.j].im * nav3_B.anorm;
              if (nav3_B.ai == 0.0) {
                nav3_B.absxr = nav3_B.ar / 0.5;
                nav3_B.absxi = 0.0;
              } else if (nav3_B.ar == 0.0) {
                nav3_B.absxr = 0.0;
                nav3_B.absxi = nav3_B.ai / 0.5;
              } else {
                nav3_B.absxr = nav3_B.ar / 0.5;
                nav3_B.absxi = nav3_B.ai / 0.5;
              }

              nav3_B.eshift_re += nav3_B.absxr;
              nav3_B.eshift_im += nav3_B.absxi;
              nav3_B.shift_re = nav3_B.eshift_re;
              nav3_B.shift_im = nav3_B.eshift_im;
            }

            nav3_B.j = nav3_B.ilastm1;
            nav3_B.jp1 = nav3_B.ilastm1 + 1;
            exitg2 = false;
            while ((!exitg2) && (nav3_B.j + 1 > nav3_B.ifirst)) {
              nav3_B.istart = nav3_B.j + 1;
              nav3_B.ctemp_tmp_tmp = nav3_B.j << 2;
              nav3_B.ctemp_tmp = nav3_B.ctemp_tmp_tmp + nav3_B.j;
              nav3_B.ctemp.re = A[nav3_B.ctemp_tmp].re * nav3_B.anorm -
                nav3_B.shift_re * 0.5;
              nav3_B.ctemp.im = A[nav3_B.ctemp_tmp].im * nav3_B.anorm -
                nav3_B.shift_im * 0.5;
              nav3_B.t1_re = fabs(nav3_B.ctemp.re) + fabs(nav3_B.ctemp.im);
              nav3_B.jp1 += nav3_B.ctemp_tmp_tmp;
              nav3_B.t1_im = (fabs(A[nav3_B.jp1].re) + fabs(A[nav3_B.jp1].im)) *
                nav3_B.anorm;
              nav3_B.absxr = nav3_B.t1_re;
              if (nav3_B.t1_im > nav3_B.t1_re) {
                nav3_B.absxr = nav3_B.t1_im;
              }

              if ((nav3_B.absxr < 1.0) && (nav3_B.absxr != 0.0)) {
                nav3_B.t1_re /= nav3_B.absxr;
                nav3_B.t1_im /= nav3_B.absxr;
              }

              nav3_B.jp1 = ((nav3_B.j - 1) << 2) + nav3_B.j;
              if ((fabs(A[nav3_B.jp1].re) + fabs(A[nav3_B.jp1].im)) *
                  nav3_B.t1_im <= nav3_B.t1_re * nav3_B.b_atol) {
                nav3_B.goto90 = true;
                exitg2 = true;
              } else {
                nav3_B.jp1 = nav3_B.j;
                nav3_B.j--;
              }
            }

            if (!nav3_B.goto90) {
              nav3_B.istart = nav3_B.ifirst;
              nav3_B.ctemp_tmp = (((nav3_B.ifirst - 1) << 2) + nav3_B.ifirst) -
                1;
              nav3_B.ctemp.re = A[nav3_B.ctemp_tmp].re * nav3_B.anorm -
                nav3_B.shift_re * 0.5;
              nav3_B.ctemp.im = A[nav3_B.ctemp_tmp].im * nav3_B.anorm -
                nav3_B.shift_im * 0.5;
            }

            nav3_B.goto90 = false;
            nav3_B.j = ((nav3_B.istart - 1) << 2) + nav3_B.istart;
            nav3_B.ascale.re = A[nav3_B.j].re * nav3_B.anorm;
            nav3_B.ascale.im = A[nav3_B.j].im * nav3_B.anorm;
            nav3_xzlartg_g(nav3_B.ctemp, nav3_B.ascale, &nav3_B.t1_re,
                           &nav3_B.ad22);
            nav3_B.j = nav3_B.istart;
            nav3_B.jp1 = nav3_B.istart - 2;
            while (nav3_B.j < nav3_B.ilast + 1) {
              if (nav3_B.j > nav3_B.istart) {
                nav3_xzlartg(A[(nav3_B.j + (nav3_B.jp1 << 2)) - 1], A[nav3_B.j +
                             (nav3_B.jp1 << 2)], &nav3_B.t1_re, &nav3_B.ad22,
                             &A[(nav3_B.j + (nav3_B.jp1 << 2)) - 1]);
                nav3_B.jp1 = nav3_B.j + (nav3_B.jp1 << 2);
                A[nav3_B.jp1].re = 0.0;
                A[nav3_B.jp1].im = 0.0;
              }

              nav3_B.ctemp_tmp = nav3_B.j - 1;
              while (nav3_B.ctemp_tmp + 1 < 5) {
                nav3_B.jp1 = (nav3_B.ctemp_tmp << 2) + nav3_B.j;
                nav3_B.ctemp_tmp_tmp = nav3_B.jp1 - 1;
                nav3_B.shift_re = A[nav3_B.ctemp_tmp_tmp].re * nav3_B.t1_re +
                  (A[nav3_B.jp1].re * nav3_B.ad22.re - A[nav3_B.jp1].im *
                   nav3_B.ad22.im);
                nav3_B.shift_im = A[nav3_B.ctemp_tmp_tmp].im * nav3_B.t1_re +
                  (A[nav3_B.jp1].im * nav3_B.ad22.re + A[nav3_B.jp1].re *
                   nav3_B.ad22.im);
                nav3_B.t1_im = A[nav3_B.ctemp_tmp_tmp].im;
                nav3_B.absxr = A[nav3_B.ctemp_tmp_tmp].re;
                A[nav3_B.jp1].re = A[nav3_B.jp1].re * nav3_B.t1_re -
                  (A[nav3_B.ctemp_tmp_tmp].re * nav3_B.ad22.re +
                   A[nav3_B.ctemp_tmp_tmp].im * nav3_B.ad22.im);
                A[nav3_B.jp1].im = A[nav3_B.jp1].im * nav3_B.t1_re -
                  (nav3_B.ad22.re * nav3_B.t1_im - nav3_B.ad22.im * nav3_B.absxr);
                A[nav3_B.ctemp_tmp_tmp].re = nav3_B.shift_re;
                A[nav3_B.ctemp_tmp_tmp].im = nav3_B.shift_im;
                nav3_B.ctemp_tmp++;
              }

              nav3_B.ad22.re = -nav3_B.ad22.re;
              nav3_B.ad22.im = -nav3_B.ad22.im;
              nav3_B.ctemp_tmp = nav3_B.j;
              if (nav3_B.ilast + 1 < nav3_B.j + 2) {
                nav3_B.ctemp_tmp = nav3_B.ilast - 1;
              }

              nav3_B.i_h = 0;
              while (nav3_B.i_h + 1 <= nav3_B.ctemp_tmp + 2) {
                nav3_B.jp1 = ((nav3_B.j - 1) << 2) + nav3_B.i_h;
                nav3_B.ctemp_tmp_tmp = (nav3_B.j << 2) + nav3_B.i_h;
                nav3_B.shift_re = (A[nav3_B.jp1].re * nav3_B.ad22.re -
                                   A[nav3_B.jp1].im * nav3_B.ad22.im) +
                  A[nav3_B.ctemp_tmp_tmp].re * nav3_B.t1_re;
                nav3_B.shift_im = (A[nav3_B.jp1].im * nav3_B.ad22.re +
                                   A[nav3_B.jp1].re * nav3_B.ad22.im) +
                  A[nav3_B.ctemp_tmp_tmp].im * nav3_B.t1_re;
                nav3_B.t1_im = A[nav3_B.ctemp_tmp_tmp].im;
                nav3_B.absxr = A[nav3_B.ctemp_tmp_tmp].re;
                A[nav3_B.jp1].re = A[nav3_B.jp1].re * nav3_B.t1_re -
                  (A[nav3_B.ctemp_tmp_tmp].re * nav3_B.ad22.re +
                   A[nav3_B.ctemp_tmp_tmp].im * nav3_B.ad22.im);
                A[nav3_B.jp1].im = A[nav3_B.jp1].im * nav3_B.t1_re -
                  (nav3_B.ad22.re * nav3_B.t1_im - nav3_B.ad22.im * nav3_B.absxr);
                A[nav3_B.ctemp_tmp_tmp].re = nav3_B.shift_re;
                A[nav3_B.ctemp_tmp_tmp].im = nav3_B.shift_im;
                nav3_B.i_h++;
              }

              nav3_B.jp1 = (nav3_B.j - 1) << 2;
              nav3_B.ctemp_tmp_tmp = nav3_B.j << 2;
              nav3_B.shift_re = (Z[nav3_B.jp1].re * nav3_B.ad22.re -
                                 Z[nav3_B.jp1].im * nav3_B.ad22.im) +
                Z[nav3_B.ctemp_tmp_tmp].re * nav3_B.t1_re;
              nav3_B.shift_im = (Z[nav3_B.jp1].im * nav3_B.ad22.re +
                                 Z[nav3_B.jp1].re * nav3_B.ad22.im) +
                Z[nav3_B.ctemp_tmp_tmp].im * nav3_B.t1_re;
              nav3_B.t1_im = Z[nav3_B.ctemp_tmp_tmp].im;
              nav3_B.absxr = Z[nav3_B.ctemp_tmp_tmp].re;
              Z[nav3_B.jp1].re = Z[nav3_B.jp1].re * nav3_B.t1_re -
                (Z[nav3_B.ctemp_tmp_tmp].re * nav3_B.ad22.re +
                 Z[nav3_B.ctemp_tmp_tmp].im * nav3_B.ad22.im);
              Z[nav3_B.jp1].im = Z[nav3_B.jp1].im * nav3_B.t1_re -
                (nav3_B.ad22.re * nav3_B.t1_im - nav3_B.ad22.im * nav3_B.absxr);
              Z[nav3_B.ctemp_tmp_tmp].re = nav3_B.shift_re;
              Z[nav3_B.ctemp_tmp_tmp].im = nav3_B.shift_im;
              nav3_B.ctemp_tmp = nav3_B.jp1 + 1;
              nav3_B.i_h = nav3_B.ctemp_tmp_tmp + 1;
              nav3_B.shift_re = (Z[nav3_B.ctemp_tmp].re * nav3_B.ad22.re -
                                 Z[nav3_B.ctemp_tmp].im * nav3_B.ad22.im) +
                Z[nav3_B.i_h].re * nav3_B.t1_re;
              nav3_B.shift_im = (Z[nav3_B.ctemp_tmp].im * nav3_B.ad22.re +
                                 Z[nav3_B.ctemp_tmp].re * nav3_B.ad22.im) +
                Z[nav3_B.i_h].im * nav3_B.t1_re;
              nav3_B.t1_im = Z[nav3_B.i_h].im;
              nav3_B.absxr = Z[nav3_B.i_h].re;
              Z[nav3_B.ctemp_tmp].re = Z[nav3_B.ctemp_tmp].re * nav3_B.t1_re -
                (Z[nav3_B.i_h].re * nav3_B.ad22.re + Z[nav3_B.i_h].im *
                 nav3_B.ad22.im);
              Z[nav3_B.ctemp_tmp].im = Z[nav3_B.ctemp_tmp].im * nav3_B.t1_re -
                (nav3_B.ad22.re * nav3_B.t1_im - nav3_B.ad22.im * nav3_B.absxr);
              Z[nav3_B.i_h].re = nav3_B.shift_re;
              Z[nav3_B.i_h].im = nav3_B.shift_im;
              nav3_B.ctemp_tmp = nav3_B.jp1 + 2;
              nav3_B.i_h = nav3_B.ctemp_tmp_tmp + 2;
              nav3_B.shift_re = (Z[nav3_B.ctemp_tmp].re * nav3_B.ad22.re -
                                 Z[nav3_B.ctemp_tmp].im * nav3_B.ad22.im) +
                Z[nav3_B.i_h].re * nav3_B.t1_re;
              nav3_B.shift_im = (Z[nav3_B.ctemp_tmp].im * nav3_B.ad22.re +
                                 Z[nav3_B.ctemp_tmp].re * nav3_B.ad22.im) +
                Z[nav3_B.i_h].im * nav3_B.t1_re;
              nav3_B.t1_im = Z[nav3_B.i_h].im;
              nav3_B.absxr = Z[nav3_B.i_h].re;
              Z[nav3_B.ctemp_tmp].re = Z[nav3_B.ctemp_tmp].re * nav3_B.t1_re -
                (Z[nav3_B.i_h].re * nav3_B.ad22.re + Z[nav3_B.i_h].im *
                 nav3_B.ad22.im);
              Z[nav3_B.ctemp_tmp].im = Z[nav3_B.ctemp_tmp].im * nav3_B.t1_re -
                (nav3_B.ad22.re * nav3_B.t1_im - nav3_B.ad22.im * nav3_B.absxr);
              Z[nav3_B.i_h].re = nav3_B.shift_re;
              Z[nav3_B.i_h].im = nav3_B.shift_im;
              nav3_B.jp1 += 3;
              nav3_B.ctemp_tmp_tmp += 3;
              nav3_B.shift_re = (Z[nav3_B.jp1].re * nav3_B.ad22.re -
                                 Z[nav3_B.jp1].im * nav3_B.ad22.im) +
                Z[nav3_B.ctemp_tmp_tmp].re * nav3_B.t1_re;
              nav3_B.shift_im = (Z[nav3_B.jp1].im * nav3_B.ad22.re +
                                 Z[nav3_B.jp1].re * nav3_B.ad22.im) +
                Z[nav3_B.ctemp_tmp_tmp].im * nav3_B.t1_re;
              nav3_B.t1_im = Z[nav3_B.ctemp_tmp_tmp].im;
              nav3_B.absxr = Z[nav3_B.ctemp_tmp_tmp].re;
              Z[nav3_B.jp1].re = Z[nav3_B.jp1].re * nav3_B.t1_re -
                (Z[nav3_B.ctemp_tmp_tmp].re * nav3_B.ad22.re +
                 Z[nav3_B.ctemp_tmp_tmp].im * nav3_B.ad22.im);
              Z[nav3_B.jp1].im = Z[nav3_B.jp1].im * nav3_B.t1_re -
                (nav3_B.ad22.re * nav3_B.t1_im - nav3_B.ad22.im * nav3_B.absxr);
              Z[nav3_B.ctemp_tmp_tmp].re = nav3_B.shift_re;
              Z[nav3_B.ctemp_tmp_tmp].im = nav3_B.shift_im;
              nav3_B.jp1 = nav3_B.j - 1;
              nav3_B.j++;
            }
          }

          nav3_B.jiter++;
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
    if (nav3_B.failed) {
      *info = nav3_B.ilast + 1;
      nav3_B.ifirst = 0;
      while (nav3_B.ifirst <= nav3_B.ilast) {
        alpha1[nav3_B.ifirst].re = (rtNaN);
        alpha1[nav3_B.ifirst].im = 0.0;
        beta1[nav3_B.ifirst].re = (rtNaN);
        beta1[nav3_B.ifirst].im = 0.0;
        nav3_B.ifirst++;
      }

      for (nav3_B.jp1 = 0; nav3_B.jp1 < 16; nav3_B.jp1++) {
        Z[nav3_B.jp1].re = (rtNaN);
        Z[nav3_B.jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    nav3_B.ilast = 0;
    while (nav3_B.ilast <= ilo - 2) {
      alpha1[nav3_B.ilast] = A[(nav3_B.ilast << 2) + nav3_B.ilast];
      nav3_B.ilast++;
    }
  }
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xztgevc(const creal_T A[16], creal_T V[16])
{
  nav3_B.rworka[0] = 0.0;
  nav3_B.rworka[2] = 0.0;
  nav3_B.rworka[3] = 0.0;
  nav3_B.anorm_o = fabs(A[0].re) + fabs(A[0].im);
  nav3_B.rworka[1] = fabs(A[4].re) + fabs(A[4].im);
  nav3_B.ascale_b = (fabs(A[5].re) + fabs(A[5].im)) + nav3_B.rworka[1];
  if (nav3_B.ascale_b > nav3_B.anorm_o) {
    nav3_B.anorm_o = nav3_B.ascale_b;
  }

  nav3_B.i_l = 0;
  while (nav3_B.i_l <= 1) {
    nav3_B.rworka[2] += fabs(A[nav3_B.i_l + 8].re) + fabs(A[nav3_B.i_l + 8].im);
    nav3_B.i_l++;
  }

  nav3_B.ascale_b = (fabs(A[10].re) + fabs(A[10].im)) + nav3_B.rworka[2];
  if (nav3_B.ascale_b > nav3_B.anorm_o) {
    nav3_B.anorm_o = nav3_B.ascale_b;
  }

  nav3_B.i_l = 0;
  while (nav3_B.i_l <= 2) {
    nav3_B.rworka[3] += fabs(A[nav3_B.i_l + 12].re) + fabs(A[nav3_B.i_l + 12].im);
    nav3_B.i_l++;
  }

  nav3_B.ascale_b = (fabs(A[15].re) + fabs(A[15].im)) + nav3_B.rworka[3];
  if (nav3_B.ascale_b > nav3_B.anorm_o) {
    nav3_B.anorm_o = nav3_B.ascale_b;
  }

  nav3_B.ascale_b = nav3_B.anorm_o;
  if (2.2250738585072014E-308 > nav3_B.anorm_o) {
    nav3_B.ascale_b = 2.2250738585072014E-308;
  }

  nav3_B.ascale_b = 1.0 / nav3_B.ascale_b;
  for (nav3_B.i_l = 0; nav3_B.i_l < 4; nav3_B.i_l++) {
    nav3_B.c_x_tmp_tmp = (3 - nav3_B.i_l) << 2;
    nav3_B.c_x_tmp = (nav3_B.c_x_tmp_tmp - nav3_B.i_l) + 3;
    nav3_B.salpha_re = (fabs(A[nav3_B.c_x_tmp].re) + fabs(A[nav3_B.c_x_tmp].im))
      * nav3_B.ascale_b;
    if (1.0 > nav3_B.salpha_re) {
      nav3_B.salpha_re = 1.0;
    }

    nav3_B.temp = 1.0 / nav3_B.salpha_re;
    nav3_B.salpha_re = A[nav3_B.c_x_tmp].re * nav3_B.temp * nav3_B.ascale_b;
    nav3_B.salpha_im = A[nav3_B.c_x_tmp].im * nav3_B.temp * nav3_B.ascale_b;
    nav3_B.acoeff = nav3_B.temp * nav3_B.ascale_b;
    nav3_B.lscalea = ((nav3_B.temp >= 2.2250738585072014E-308) && (nav3_B.acoeff
      < 4.0083367200179456E-292));
    nav3_B.dmin = fabs(nav3_B.salpha_re) + fabs(nav3_B.salpha_im);
    if ((nav3_B.dmin >= 2.2250738585072014E-308) && (nav3_B.dmin <
         4.0083367200179456E-292)) {
      nav3_B.lscaleb = true;
    } else {
      nav3_B.lscaleb = false;
    }

    nav3_B.scale_n = 1.0;
    if (nav3_B.lscalea) {
      nav3_B.scale_n = nav3_B.anorm_o;
      if (2.4948003869184E+291 < nav3_B.anorm_o) {
        nav3_B.scale_n = 2.4948003869184E+291;
      }

      nav3_B.scale_n *= 4.0083367200179456E-292 / nav3_B.temp;
    }

    if (nav3_B.lscaleb) {
      nav3_B.work2_idx_2_im = 4.0083367200179456E-292 / nav3_B.dmin;
      if (nav3_B.work2_idx_2_im > nav3_B.scale_n) {
        nav3_B.scale_n = nav3_B.work2_idx_2_im;
      }
    }

    if (nav3_B.lscalea || nav3_B.lscaleb) {
      nav3_B.work2_idx_2_im = nav3_B.acoeff;
      if (1.0 > nav3_B.acoeff) {
        nav3_B.work2_idx_2_im = 1.0;
      }

      if (nav3_B.dmin > nav3_B.work2_idx_2_im) {
        nav3_B.work2_idx_2_im = nav3_B.dmin;
      }

      nav3_B.dmin = 1.0 / (2.2250738585072014E-308 * nav3_B.work2_idx_2_im);
      if (nav3_B.dmin < nav3_B.scale_n) {
        nav3_B.scale_n = nav3_B.dmin;
      }

      if (nav3_B.lscalea) {
        nav3_B.acoeff = nav3_B.scale_n * nav3_B.temp * nav3_B.ascale_b;
      } else {
        nav3_B.acoeff *= nav3_B.scale_n;
      }

      nav3_B.salpha_re *= nav3_B.scale_n;
      nav3_B.salpha_im *= nav3_B.scale_n;
    }

    memset(&nav3_B.work1[0], 0, sizeof(creal_T) << 2U);
    nav3_B.work1[3 - nav3_B.i_l].re = 1.0;
    nav3_B.work1[3 - nav3_B.i_l].im = 0.0;
    nav3_B.dmin = 2.2204460492503131E-16 * nav3_B.acoeff * nav3_B.anorm_o;
    nav3_B.temp = (fabs(nav3_B.salpha_re) + fabs(nav3_B.salpha_im)) *
      2.2204460492503131E-16;
    if (nav3_B.temp > nav3_B.dmin) {
      nav3_B.dmin = nav3_B.temp;
    }

    if (2.2250738585072014E-308 > nav3_B.dmin) {
      nav3_B.dmin = 2.2250738585072014E-308;
    }

    nav3_B.c_x_tmp = 0;
    while (nav3_B.c_x_tmp <= 2 - nav3_B.i_l) {
      nav3_B.d_re_tmp = nav3_B.c_x_tmp_tmp + nav3_B.c_x_tmp;
      nav3_B.work1[nav3_B.c_x_tmp].re = A[nav3_B.d_re_tmp].re * nav3_B.acoeff;
      nav3_B.work1[nav3_B.c_x_tmp].im = A[nav3_B.d_re_tmp].im * nav3_B.acoeff;
      nav3_B.c_x_tmp++;
    }

    nav3_B.work1[3 - nav3_B.i_l].re = 1.0;
    nav3_B.work1[3 - nav3_B.i_l].im = 0.0;
    nav3_B.c_x_tmp = static_cast<int32_T>(((-1.0 - ((-static_cast<real_T>
      (nav3_B.i_l) + 4.0) - 1.0)) + 1.0) / -1.0);
    nav3_B.c_j_o = 0;
    while (nav3_B.c_j_o <= nav3_B.c_x_tmp - 1) {
      nav3_B.work2_idx_1_re_tmp = 2 - (nav3_B.i_l + nav3_B.c_j_o);
      nav3_B.d_re_tmp_tmp = nav3_B.work2_idx_1_re_tmp << 2;
      nav3_B.d_re_tmp = nav3_B.d_re_tmp_tmp + nav3_B.work2_idx_1_re_tmp;
      nav3_B.work2_idx_3_re = A[nav3_B.d_re_tmp].re * nav3_B.acoeff -
        nav3_B.salpha_re;
      nav3_B.scale_n = A[nav3_B.d_re_tmp].im * nav3_B.acoeff - nav3_B.salpha_im;
      if (fabs(nav3_B.work2_idx_3_re) + fabs(nav3_B.scale_n) <= nav3_B.dmin) {
        nav3_B.work2_idx_3_re = nav3_B.dmin;
        nav3_B.scale_n = 0.0;
      }

      nav3_B.work2_idx_2_im = fabs(nav3_B.work2_idx_3_re);
      nav3_B.work2_idx_3_im = fabs(nav3_B.scale_n);
      nav3_B.temp = nav3_B.work2_idx_2_im + nav3_B.work2_idx_3_im;
      if (nav3_B.temp < 1.0) {
        nav3_B.f_y = fabs(nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re) + fabs
          (nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im);
        if (nav3_B.f_y >= nav3_B.temp * 1.1235582092889474E+307) {
          nav3_B.temp = 1.0 / nav3_B.f_y;
          nav3_B.d_re_tmp = 0;
          while (nav3_B.d_re_tmp <= 3 - nav3_B.i_l) {
            nav3_B.work1[nav3_B.d_re_tmp].re *= nav3_B.temp;
            nav3_B.work1[nav3_B.d_re_tmp].im *= nav3_B.temp;
            nav3_B.d_re_tmp++;
          }
        }
      }

      if (nav3_B.scale_n == 0.0) {
        if (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im == 0.0) {
          nav3_B.temp = -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re /
            nav3_B.work2_idx_3_re;
          nav3_B.scale_n = 0.0;
        } else if (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re == 0.0) {
          nav3_B.temp = 0.0;
          nav3_B.scale_n = -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im /
            nav3_B.work2_idx_3_re;
        } else {
          nav3_B.temp = -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re /
            nav3_B.work2_idx_3_re;
          nav3_B.scale_n = -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im /
            nav3_B.work2_idx_3_re;
        }
      } else if (nav3_B.work2_idx_3_re == 0.0) {
        if (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re == 0.0) {
          nav3_B.temp = -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im /
            nav3_B.scale_n;
          nav3_B.scale_n = 0.0;
        } else if (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im == 0.0) {
          nav3_B.temp = 0.0;
          nav3_B.scale_n = -(-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re /
                             nav3_B.scale_n);
        } else {
          nav3_B.temp = -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im /
            nav3_B.scale_n;
          nav3_B.scale_n = -(-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re /
                             nav3_B.scale_n);
        }
      } else if (nav3_B.work2_idx_2_im > nav3_B.work2_idx_3_im) {
        nav3_B.work2_idx_2_im = nav3_B.scale_n / nav3_B.work2_idx_3_re;
        nav3_B.scale_n = nav3_B.work2_idx_2_im * nav3_B.scale_n +
          nav3_B.work2_idx_3_re;
        nav3_B.temp = (nav3_B.work2_idx_2_im *
                       -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im +
                       -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re) /
          nav3_B.scale_n;
        nav3_B.scale_n = (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im -
                          nav3_B.work2_idx_2_im *
                          -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re) /
          nav3_B.scale_n;
      } else if (nav3_B.work2_idx_3_im == nav3_B.work2_idx_2_im) {
        nav3_B.work2_idx_3_re = nav3_B.work2_idx_3_re > 0.0 ? 0.5 : -0.5;
        nav3_B.scale_n = nav3_B.scale_n > 0.0 ? 0.5 : -0.5;
        nav3_B.temp = (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re *
                       nav3_B.work2_idx_3_re +
                       -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im *
                       nav3_B.scale_n) / nav3_B.work2_idx_2_im;
        nav3_B.scale_n = (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im *
                          nav3_B.work2_idx_3_re -
                          -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re *
                          nav3_B.scale_n) / nav3_B.work2_idx_2_im;
      } else {
        nav3_B.work2_idx_2_im = nav3_B.work2_idx_3_re / nav3_B.scale_n;
        nav3_B.scale_n += nav3_B.work2_idx_2_im * nav3_B.work2_idx_3_re;
        nav3_B.temp = (nav3_B.work2_idx_2_im *
                       -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re +
                       -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im) /
          nav3_B.scale_n;
        nav3_B.scale_n = (nav3_B.work2_idx_2_im *
                          -nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im -
                          (-nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re)) /
          nav3_B.scale_n;
      }

      nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re = nav3_B.temp;
      nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im = nav3_B.scale_n;
      if (nav3_B.work2_idx_1_re_tmp + 1 > 1) {
        if (fabs(nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re) + fabs
            (nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im) > 1.0) {
          nav3_B.temp = 1.0 / (fabs(nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re)
                               + fabs(nav3_B.work1[nav3_B.work2_idx_1_re_tmp].im));
          if (nav3_B.acoeff * nav3_B.rworka[nav3_B.work2_idx_1_re_tmp] >=
              1.1235582092889474E+307 * nav3_B.temp) {
            nav3_B.d_re_tmp = 0;
            while (nav3_B.d_re_tmp <= 3 - nav3_B.i_l) {
              nav3_B.work1[nav3_B.d_re_tmp].re *= nav3_B.temp;
              nav3_B.work1[nav3_B.d_re_tmp].im *= nav3_B.temp;
              nav3_B.d_re_tmp++;
            }
          }
        }

        nav3_B.work2_idx_3_re = nav3_B.acoeff *
          nav3_B.work1[nav3_B.work2_idx_1_re_tmp].re;
        nav3_B.scale_n = nav3_B.acoeff * nav3_B.work1[nav3_B.work2_idx_1_re_tmp]
          .im;
        nav3_B.e_jr = 0;
        while (nav3_B.e_jr <= nav3_B.work2_idx_1_re_tmp - 1) {
          nav3_B.d_re_tmp = nav3_B.d_re_tmp_tmp + nav3_B.e_jr;
          nav3_B.work1[nav3_B.e_jr].re += A[nav3_B.d_re_tmp].re *
            nav3_B.work2_idx_3_re - A[nav3_B.d_re_tmp].im * nav3_B.scale_n;
          nav3_B.work1[nav3_B.e_jr].im += A[nav3_B.d_re_tmp].im *
            nav3_B.work2_idx_3_re + A[nav3_B.d_re_tmp].re * nav3_B.scale_n;
          nav3_B.e_jr++;
        }
      }

      nav3_B.c_j_o++;
    }

    nav3_B.salpha_re = 0.0;
    nav3_B.salpha_im = 0.0;
    nav3_B.acoeff = 0.0;
    nav3_B.dmin = 0.0;
    nav3_B.scale_n = 0.0;
    nav3_B.work2_idx_2_im = 0.0;
    nav3_B.work2_idx_3_re = 0.0;
    nav3_B.work2_idx_3_im = 0.0;
    nav3_B.c_x_tmp = 0;
    while (nav3_B.c_x_tmp <= 3 - nav3_B.i_l) {
      nav3_B.c_j_o = nav3_B.c_x_tmp << 2;
      nav3_B.salpha_re += V[nav3_B.c_j_o].re * nav3_B.work1[nav3_B.c_x_tmp].re -
        V[nav3_B.c_j_o].im * nav3_B.work1[nav3_B.c_x_tmp].im;
      nav3_B.salpha_im += V[nav3_B.c_j_o].re * nav3_B.work1[nav3_B.c_x_tmp].im +
        V[nav3_B.c_j_o].im * nav3_B.work1[nav3_B.c_x_tmp].re;
      nav3_B.work2_idx_1_re_tmp = nav3_B.c_j_o + 1;
      nav3_B.acoeff += V[nav3_B.work2_idx_1_re_tmp].re *
        nav3_B.work1[nav3_B.c_x_tmp].re - V[nav3_B.work2_idx_1_re_tmp].im *
        nav3_B.work1[nav3_B.c_x_tmp].im;
      nav3_B.dmin += V[nav3_B.work2_idx_1_re_tmp].re *
        nav3_B.work1[nav3_B.c_x_tmp].im + V[nav3_B.work2_idx_1_re_tmp].im *
        nav3_B.work1[nav3_B.c_x_tmp].re;
      nav3_B.work2_idx_1_re_tmp = nav3_B.c_j_o + 2;
      nav3_B.scale_n += V[nav3_B.work2_idx_1_re_tmp].re *
        nav3_B.work1[nav3_B.c_x_tmp].re - V[nav3_B.work2_idx_1_re_tmp].im *
        nav3_B.work1[nav3_B.c_x_tmp].im;
      nav3_B.work2_idx_2_im += V[nav3_B.work2_idx_1_re_tmp].re *
        nav3_B.work1[nav3_B.c_x_tmp].im + V[nav3_B.work2_idx_1_re_tmp].im *
        nav3_B.work1[nav3_B.c_x_tmp].re;
      nav3_B.c_j_o += 3;
      nav3_B.work2_idx_3_re += V[nav3_B.c_j_o].re * nav3_B.work1[nav3_B.c_x_tmp]
        .re - V[nav3_B.c_j_o].im * nav3_B.work1[nav3_B.c_x_tmp].im;
      nav3_B.work2_idx_3_im += V[nav3_B.c_j_o].re * nav3_B.work1[nav3_B.c_x_tmp]
        .im + V[nav3_B.c_j_o].im * nav3_B.work1[nav3_B.c_x_tmp].re;
      nav3_B.c_x_tmp++;
    }

    nav3_B.temp = fabs(nav3_B.salpha_re) + fabs(nav3_B.salpha_im);
    nav3_B.f_y = fabs(nav3_B.acoeff) + fabs(nav3_B.dmin);
    if (nav3_B.f_y > nav3_B.temp) {
      nav3_B.temp = nav3_B.f_y;
    }

    nav3_B.f_y = fabs(nav3_B.scale_n) + fabs(nav3_B.work2_idx_2_im);
    if (nav3_B.f_y > nav3_B.temp) {
      nav3_B.temp = nav3_B.f_y;
    }

    nav3_B.f_y = fabs(nav3_B.work2_idx_3_re) + fabs(nav3_B.work2_idx_3_im);
    if (nav3_B.f_y > nav3_B.temp) {
      nav3_B.temp = nav3_B.f_y;
    }

    if (nav3_B.temp > 2.2250738585072014E-308) {
      nav3_B.temp = 1.0 / nav3_B.temp;
      V[nav3_B.c_x_tmp_tmp].re = nav3_B.temp * nav3_B.salpha_re;
      V[nav3_B.c_x_tmp_tmp].im = nav3_B.temp * nav3_B.salpha_im;
      nav3_B.d_re_tmp = ((3 - nav3_B.i_l) << 2) + 1;
      V[nav3_B.d_re_tmp].re = nav3_B.temp * nav3_B.acoeff;
      V[nav3_B.d_re_tmp].im = nav3_B.temp * nav3_B.dmin;
      nav3_B.d_re_tmp = ((3 - nav3_B.i_l) << 2) + 2;
      V[nav3_B.d_re_tmp].re = nav3_B.temp * nav3_B.scale_n;
      V[nav3_B.d_re_tmp].im = nav3_B.temp * nav3_B.work2_idx_2_im;
      nav3_B.d_re_tmp = ((3 - nav3_B.i_l) << 2) + 3;
      V[nav3_B.d_re_tmp].re = nav3_B.temp * nav3_B.work2_idx_3_re;
      V[nav3_B.d_re_tmp].im = nav3_B.temp * nav3_B.work2_idx_3_im;
    } else {
      V[nav3_B.c_x_tmp_tmp].re = 0.0;
      V[nav3_B.c_x_tmp_tmp].im = 0.0;
      nav3_B.d_re_tmp = nav3_B.c_x_tmp_tmp + 1;
      V[nav3_B.d_re_tmp].re = 0.0;
      V[nav3_B.d_re_tmp].im = 0.0;
      nav3_B.d_re_tmp = nav3_B.c_x_tmp_tmp + 2;
      V[nav3_B.d_re_tmp].re = 0.0;
      V[nav3_B.d_re_tmp].im = 0.0;
      nav3_B.d_re_tmp = nav3_B.c_x_tmp_tmp + 3;
      V[nav3_B.d_re_tmp].re = 0.0;
      V[nav3_B.d_re_tmp].im = 0.0;
    }
  }
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xzggev(creal_T A[16], int32_T *info, creal_T alpha1[4], creal_T
  beta1[4], creal_T V[16])
{
  int32_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  *info = 0;
  nav3_B.anrm = nav3_xzlangeM(A);
  if (rtIsInf(nav3_B.anrm) || rtIsNaN(nav3_B.anrm)) {
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
    for (nav3_B.k = 0; nav3_B.k < 16; nav3_B.k++) {
      V[nav3_B.k].re = (rtNaN);
      V[nav3_B.k].im = 0.0;
    }
  } else {
    nav3_B.ilascl = false;
    nav3_B.anrmto = nav3_B.anrm;
    if ((nav3_B.anrm > 0.0) && (nav3_B.anrm < 6.7178761075670888E-139)) {
      nav3_B.anrmto = 6.7178761075670888E-139;
      nav3_B.ilascl = true;
      nav3_xzlascl(nav3_B.anrm, nav3_B.anrmto, A);
    } else {
      if (nav3_B.anrm > 1.4885657073574029E+138) {
        nav3_B.anrmto = 1.4885657073574029E+138;
        nav3_B.ilascl = true;
        nav3_xzlascl(nav3_B.anrm, nav3_B.anrmto, A);
      }
    }

    nav3_B.rscale[0] = 1;
    nav3_B.rscale[1] = 1;
    nav3_B.rscale[2] = 1;
    nav3_B.rscale[3] = 1;
    nav3_B.c_i = 0;
    nav3_B.ihi = 4;
    do {
      exitg2 = 0;
      nav3_B.i_j = 0;
      nav3_B.jcol = 0;
      nav3_B.found = false;
      nav3_B.ii = nav3_B.ihi;
      exitg3 = false;
      while ((!exitg3) && (nav3_B.ii > 0)) {
        nav3_B.nzcount = 0;
        nav3_B.i_j = nav3_B.ii;
        nav3_B.jcol = nav3_B.ihi;
        nav3_B.jj = 0;
        exitg4 = false;
        while ((!exitg4) && (nav3_B.jj <= nav3_B.ihi - 1)) {
          nav3_B.k = ((nav3_B.jj << 2) + nav3_B.ii) - 1;
          if ((A[nav3_B.k].re != 0.0) || (A[nav3_B.k].im != 0.0) || (nav3_B.jj +
               1 == nav3_B.ii)) {
            if (nav3_B.nzcount == 0) {
              nav3_B.jcol = nav3_B.jj + 1;
              nav3_B.nzcount = 1;
              nav3_B.jj++;
            } else {
              nav3_B.nzcount = 2;
              exitg4 = true;
            }
          } else {
            nav3_B.jj++;
          }
        }

        if (nav3_B.nzcount < 2) {
          nav3_B.found = true;
          exitg3 = true;
        } else {
          nav3_B.ii--;
        }
      }

      if (!nav3_B.found) {
        exitg2 = 2;
      } else {
        if (nav3_B.i_j != nav3_B.ihi) {
          nav3_B.atmp_re = A[nav3_B.i_j - 1].re;
          nav3_B.atmp_im = A[nav3_B.i_j - 1].im;
          A[nav3_B.i_j - 1] = A[nav3_B.ihi - 1];
          A[nav3_B.ihi - 1].re = nav3_B.atmp_re;
          A[nav3_B.ihi - 1].im = nav3_B.atmp_im;
          nav3_B.atmp_re = A[nav3_B.i_j + 3].re;
          nav3_B.atmp_im = A[nav3_B.i_j + 3].im;
          A[nav3_B.i_j + 3] = A[nav3_B.ihi + 3];
          A[nav3_B.ihi + 3].re = nav3_B.atmp_re;
          A[nav3_B.ihi + 3].im = nav3_B.atmp_im;
          nav3_B.atmp_re = A[nav3_B.i_j + 7].re;
          nav3_B.atmp_im = A[nav3_B.i_j + 7].im;
          A[nav3_B.i_j + 7] = A[nav3_B.ihi + 7];
          A[nav3_B.ihi + 7].re = nav3_B.atmp_re;
          A[nav3_B.ihi + 7].im = nav3_B.atmp_im;
          nav3_B.atmp_re = A[nav3_B.i_j + 11].re;
          nav3_B.atmp_im = A[nav3_B.i_j + 11].im;
          A[nav3_B.i_j + 11] = A[nav3_B.ihi + 11];
          A[nav3_B.ihi + 11].re = nav3_B.atmp_re;
          A[nav3_B.ihi + 11].im = nav3_B.atmp_im;
        }

        if (nav3_B.jcol != nav3_B.ihi) {
          nav3_B.ii = 0;
          while (nav3_B.ii <= nav3_B.ihi - 1) {
            nav3_B.i_j = ((nav3_B.jcol - 1) << 2) + nav3_B.ii;
            nav3_B.atmp_re = A[nav3_B.i_j].re;
            nav3_B.atmp_im = A[nav3_B.i_j].im;
            nav3_B.k = ((nav3_B.ihi - 1) << 2) + nav3_B.ii;
            A[nav3_B.i_j] = A[nav3_B.k];
            A[nav3_B.k].re = nav3_B.atmp_re;
            A[nav3_B.k].im = nav3_B.atmp_im;
            nav3_B.ii++;
          }
        }

        nav3_B.rscale[nav3_B.ihi - 1] = nav3_B.jcol;
        nav3_B.ihi--;
        if (nav3_B.ihi == 1) {
          nav3_B.rscale[0] = 1;
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);

    if (exitg2 == 1) {
    } else {
      do {
        exitg1 = 0;
        nav3_B.ii = 0;
        nav3_B.jcol = 0;
        nav3_B.found = false;
        nav3_B.i_j = nav3_B.c_i + 1;
        exitg3 = false;
        while ((!exitg3) && (nav3_B.i_j <= nav3_B.ihi)) {
          nav3_B.nzcount = 0;
          nav3_B.ii = nav3_B.ihi;
          nav3_B.jcol = nav3_B.i_j;
          nav3_B.jj = nav3_B.c_i + 1;
          exitg4 = false;
          while ((!exitg4) && (nav3_B.jj <= nav3_B.ihi)) {
            nav3_B.k = (((nav3_B.i_j - 1) << 2) + nav3_B.jj) - 1;
            if ((A[nav3_B.k].re != 0.0) || (A[nav3_B.k].im != 0.0) || (nav3_B.jj
                 == nav3_B.i_j)) {
              if (nav3_B.nzcount == 0) {
                nav3_B.ii = nav3_B.jj;
                nav3_B.nzcount = 1;
                nav3_B.jj++;
              } else {
                nav3_B.nzcount = 2;
                exitg4 = true;
              }
            } else {
              nav3_B.jj++;
            }
          }

          if (nav3_B.nzcount < 2) {
            nav3_B.found = true;
            exitg3 = true;
          } else {
            nav3_B.i_j++;
          }
        }

        if (!nav3_B.found) {
          exitg1 = 1;
        } else {
          if (nav3_B.c_i + 1 != nav3_B.ii) {
            nav3_B.nzcount = nav3_B.c_i;
            while (nav3_B.nzcount + 1 < 5) {
              nav3_B.k = nav3_B.nzcount << 2;
              nav3_B.i_j = (nav3_B.k + nav3_B.ii) - 1;
              nav3_B.atmp_re = A[nav3_B.i_j].re;
              nav3_B.atmp_im = A[nav3_B.i_j].im;
              nav3_B.k += nav3_B.c_i;
              A[nav3_B.i_j] = A[nav3_B.k];
              A[nav3_B.k].re = nav3_B.atmp_re;
              A[nav3_B.k].im = nav3_B.atmp_im;
              nav3_B.nzcount++;
            }
          }

          if (nav3_B.c_i + 1 != nav3_B.jcol) {
            nav3_B.ii = 0;
            while (nav3_B.ii <= nav3_B.ihi - 1) {
              nav3_B.i_j = ((nav3_B.jcol - 1) << 2) + nav3_B.ii;
              nav3_B.atmp_re = A[nav3_B.i_j].re;
              nav3_B.atmp_im = A[nav3_B.i_j].im;
              nav3_B.k = (nav3_B.c_i << 2) + nav3_B.ii;
              A[nav3_B.i_j] = A[nav3_B.k];
              A[nav3_B.k].re = nav3_B.atmp_re;
              A[nav3_B.k].im = nav3_B.atmp_im;
              nav3_B.ii++;
            }
          }

          nav3_B.rscale[nav3_B.c_i] = nav3_B.jcol;
          nav3_B.c_i++;
          if (nav3_B.c_i + 1 == nav3_B.ihi) {
            nav3_B.rscale[nav3_B.c_i] = nav3_B.c_i + 1;
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }

    for (nav3_B.k = 0; nav3_B.k < 16; nav3_B.k++) {
      nav3_B.b_I[nav3_B.k] = 0;
    }

    nav3_B.b_I[0] = 1;
    nav3_B.b_I[5] = 1;
    nav3_B.b_I[10] = 1;
    nav3_B.b_I[15] = 1;
    for (nav3_B.k = 0; nav3_B.k < 16; nav3_B.k++) {
      V[nav3_B.k].re = nav3_B.b_I[nav3_B.k];
      V[nav3_B.k].im = 0.0;
    }

    if (nav3_B.ihi >= nav3_B.c_i + 3) {
      nav3_B.jcol = nav3_B.c_i;
      while (nav3_B.jcol + 1 < nav3_B.ihi - 1) {
        nav3_B.ii = nav3_B.ihi - 1;
        while (nav3_B.ii + 1 > nav3_B.jcol + 2) {
          nav3_xzlartg(A[(nav3_B.ii + (nav3_B.jcol << 2)) - 1], A[nav3_B.ii +
                       (nav3_B.jcol << 2)], &nav3_B.mul, &nav3_B.s, &A
                       [(nav3_B.ii + (nav3_B.jcol << 2)) - 1]);
          nav3_B.k = nav3_B.ii + (nav3_B.jcol << 2);
          A[nav3_B.k].re = 0.0;
          A[nav3_B.k].im = 0.0;
          nav3_B.nzcount = nav3_B.jcol + 1;
          while (nav3_B.nzcount + 1 < 5) {
            nav3_B.i_j = (nav3_B.nzcount << 2) + nav3_B.ii;
            nav3_B.k = nav3_B.i_j - 1;
            nav3_B.atmp_re = A[nav3_B.k].re * nav3_B.mul + (A[nav3_B.i_j].re *
              nav3_B.s.re - A[nav3_B.i_j].im * nav3_B.s.im);
            nav3_B.atmp_im = A[nav3_B.k].im * nav3_B.mul + (A[nav3_B.i_j].im *
              nav3_B.s.re + A[nav3_B.i_j].re * nav3_B.s.im);
            nav3_B.d1 = A[nav3_B.k].im;
            nav3_B.d = A[nav3_B.k].re;
            A[nav3_B.i_j].re = A[nav3_B.i_j].re * nav3_B.mul - (A[nav3_B.k].re *
              nav3_B.s.re + A[nav3_B.k].im * nav3_B.s.im);
            A[nav3_B.i_j].im = A[nav3_B.i_j].im * nav3_B.mul - (nav3_B.s.re *
              nav3_B.d1 - nav3_B.s.im * nav3_B.d);
            A[nav3_B.k].re = nav3_B.atmp_re;
            A[nav3_B.k].im = nav3_B.atmp_im;
            nav3_B.nzcount++;
          }

          nav3_B.s.re = -nav3_B.s.re;
          nav3_B.s.im = -nav3_B.s.im;
          nav3_B.nzcount = 0;
          while (nav3_B.nzcount + 1 <= nav3_B.ihi) {
            nav3_B.i_j = ((nav3_B.ii - 1) << 2) + nav3_B.nzcount;
            nav3_B.k = (nav3_B.ii << 2) + nav3_B.nzcount;
            nav3_B.atmp_re = (A[nav3_B.i_j].re * nav3_B.s.re - A[nav3_B.i_j].im *
                              nav3_B.s.im) + A[nav3_B.k].re * nav3_B.mul;
            nav3_B.atmp_im = (A[nav3_B.i_j].im * nav3_B.s.re + A[nav3_B.i_j].re *
                              nav3_B.s.im) + A[nav3_B.k].im * nav3_B.mul;
            nav3_B.d1 = A[nav3_B.k].im;
            nav3_B.d = A[nav3_B.k].re;
            A[nav3_B.i_j].re = A[nav3_B.i_j].re * nav3_B.mul - (A[nav3_B.k].re *
              nav3_B.s.re + A[nav3_B.k].im * nav3_B.s.im);
            A[nav3_B.i_j].im = A[nav3_B.i_j].im * nav3_B.mul - (nav3_B.s.re *
              nav3_B.d1 - nav3_B.s.im * nav3_B.d);
            A[nav3_B.k].re = nav3_B.atmp_re;
            A[nav3_B.k].im = nav3_B.atmp_im;
            nav3_B.nzcount++;
          }

          nav3_B.i_j = (nav3_B.ii - 1) << 2;
          nav3_B.k = nav3_B.ii << 2;
          nav3_B.atmp_re = (V[nav3_B.i_j].re * nav3_B.s.re - V[nav3_B.i_j].im *
                            nav3_B.s.im) + V[nav3_B.k].re * nav3_B.mul;
          nav3_B.atmp_im = (V[nav3_B.i_j].im * nav3_B.s.re + V[nav3_B.i_j].re *
                            nav3_B.s.im) + V[nav3_B.k].im * nav3_B.mul;
          nav3_B.d1 = V[nav3_B.k].re;
          V[nav3_B.i_j].re = V[nav3_B.i_j].re * nav3_B.mul - (V[nav3_B.k].re *
            nav3_B.s.re + V[nav3_B.k].im * nav3_B.s.im);
          V[nav3_B.i_j].im = V[nav3_B.i_j].im * nav3_B.mul - (V[nav3_B.k].im *
            nav3_B.s.re - nav3_B.s.im * nav3_B.d1);
          V[nav3_B.k].re = nav3_B.atmp_re;
          V[nav3_B.k].im = nav3_B.atmp_im;
          nav3_B.nzcount = nav3_B.i_j + 1;
          nav3_B.jj = nav3_B.k + 1;
          nav3_B.atmp_re = (V[nav3_B.nzcount].re * nav3_B.s.re -
                            V[nav3_B.nzcount].im * nav3_B.s.im) + V[nav3_B.jj].
            re * nav3_B.mul;
          nav3_B.atmp_im = (V[nav3_B.nzcount].im * nav3_B.s.re +
                            V[nav3_B.nzcount].re * nav3_B.s.im) + V[nav3_B.jj].
            im * nav3_B.mul;
          nav3_B.d1 = V[nav3_B.jj].re;
          V[nav3_B.nzcount].re = V[nav3_B.nzcount].re * nav3_B.mul -
            (V[nav3_B.jj].re * nav3_B.s.re + V[nav3_B.jj].im * nav3_B.s.im);
          V[nav3_B.nzcount].im = V[nav3_B.nzcount].im * nav3_B.mul -
            (V[nav3_B.jj].im * nav3_B.s.re - nav3_B.s.im * nav3_B.d1);
          V[nav3_B.jj].re = nav3_B.atmp_re;
          V[nav3_B.jj].im = nav3_B.atmp_im;
          nav3_B.nzcount = nav3_B.i_j + 2;
          nav3_B.jj = nav3_B.k + 2;
          nav3_B.atmp_re = (V[nav3_B.nzcount].re * nav3_B.s.re -
                            V[nav3_B.nzcount].im * nav3_B.s.im) + V[nav3_B.jj].
            re * nav3_B.mul;
          nav3_B.atmp_im = (V[nav3_B.nzcount].im * nav3_B.s.re +
                            V[nav3_B.nzcount].re * nav3_B.s.im) + V[nav3_B.jj].
            im * nav3_B.mul;
          nav3_B.d1 = V[nav3_B.jj].re;
          V[nav3_B.nzcount].re = V[nav3_B.nzcount].re * nav3_B.mul -
            (V[nav3_B.jj].re * nav3_B.s.re + V[nav3_B.jj].im * nav3_B.s.im);
          V[nav3_B.nzcount].im = V[nav3_B.nzcount].im * nav3_B.mul -
            (V[nav3_B.jj].im * nav3_B.s.re - nav3_B.s.im * nav3_B.d1);
          V[nav3_B.jj].re = nav3_B.atmp_re;
          V[nav3_B.jj].im = nav3_B.atmp_im;
          nav3_B.i_j += 3;
          nav3_B.k += 3;
          nav3_B.atmp_re = (V[nav3_B.i_j].re * nav3_B.s.re - V[nav3_B.i_j].im *
                            nav3_B.s.im) + V[nav3_B.k].re * nav3_B.mul;
          nav3_B.atmp_im = (V[nav3_B.i_j].im * nav3_B.s.re + V[nav3_B.i_j].re *
                            nav3_B.s.im) + V[nav3_B.k].im * nav3_B.mul;
          nav3_B.d1 = V[nav3_B.k].re;
          V[nav3_B.i_j].re = V[nav3_B.i_j].re * nav3_B.mul - (V[nav3_B.k].re *
            nav3_B.s.re + V[nav3_B.k].im * nav3_B.s.im);
          V[nav3_B.i_j].im = V[nav3_B.i_j].im * nav3_B.mul - (V[nav3_B.k].im *
            nav3_B.s.re - nav3_B.s.im * nav3_B.d1);
          V[nav3_B.k].re = nav3_B.atmp_re;
          V[nav3_B.k].im = nav3_B.atmp_im;
          nav3_B.ii--;
        }

        nav3_B.jcol++;
      }
    }

    nav3_xzhgeqz(A, nav3_B.c_i + 1, nav3_B.ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      nav3_xztgevc(A, V);
      if (nav3_B.c_i + 1 > 1) {
        nav3_B.c_i--;
        while (nav3_B.c_i + 1 >= 1) {
          nav3_B.k = nav3_B.rscale[nav3_B.c_i] - 1;
          if (nav3_B.c_i + 1 != nav3_B.rscale[nav3_B.c_i]) {
            nav3_B.atmp_re = V[nav3_B.c_i].re;
            nav3_B.atmp_im = V[nav3_B.c_i].im;
            V[nav3_B.c_i] = V[nav3_B.k];
            V[nav3_B.k].re = nav3_B.atmp_re;
            V[nav3_B.k].im = nav3_B.atmp_im;
            nav3_B.atmp_re = V[nav3_B.c_i + 4].re;
            nav3_B.atmp_im = V[nav3_B.c_i + 4].im;
            V[nav3_B.c_i + 4] = V[nav3_B.k + 4];
            V[nav3_B.k + 4].re = nav3_B.atmp_re;
            V[nav3_B.k + 4].im = nav3_B.atmp_im;
            nav3_B.atmp_re = V[nav3_B.c_i + 8].re;
            nav3_B.atmp_im = V[nav3_B.c_i + 8].im;
            V[nav3_B.c_i + 8] = V[nav3_B.k + 8];
            V[nav3_B.k + 8].re = nav3_B.atmp_re;
            V[nav3_B.k + 8].im = nav3_B.atmp_im;
            nav3_B.atmp_re = V[nav3_B.c_i + 12].re;
            nav3_B.atmp_im = V[nav3_B.c_i + 12].im;
            V[nav3_B.c_i + 12] = V[nav3_B.k + 12];
            V[nav3_B.k + 12].re = nav3_B.atmp_re;
            V[nav3_B.k + 12].im = nav3_B.atmp_im;
          }

          nav3_B.c_i--;
        }
      }

      if (nav3_B.ihi < 4) {
        while (nav3_B.ihi + 1 < 5) {
          nav3_B.k = nav3_B.rscale[nav3_B.ihi] - 1;
          if (nav3_B.ihi + 1 != nav3_B.rscale[nav3_B.ihi]) {
            nav3_B.atmp_re = V[nav3_B.ihi].re;
            nav3_B.atmp_im = V[nav3_B.ihi].im;
            V[nav3_B.ihi] = V[nav3_B.k];
            V[nav3_B.k].re = nav3_B.atmp_re;
            V[nav3_B.k].im = nav3_B.atmp_im;
            nav3_B.atmp_re = V[nav3_B.ihi + 4].re;
            nav3_B.atmp_im = V[nav3_B.ihi + 4].im;
            V[nav3_B.ihi + 4] = V[nav3_B.k + 4];
            V[nav3_B.k + 4].re = nav3_B.atmp_re;
            V[nav3_B.k + 4].im = nav3_B.atmp_im;
            nav3_B.atmp_re = V[nav3_B.ihi + 8].re;
            nav3_B.atmp_im = V[nav3_B.ihi + 8].im;
            V[nav3_B.ihi + 8] = V[nav3_B.k + 8];
            V[nav3_B.k + 8].re = nav3_B.atmp_re;
            V[nav3_B.k + 8].im = nav3_B.atmp_im;
            nav3_B.atmp_re = V[nav3_B.ihi + 12].re;
            nav3_B.atmp_im = V[nav3_B.ihi + 12].im;
            V[nav3_B.ihi + 12] = V[nav3_B.k + 12];
            V[nav3_B.k + 12].re = nav3_B.atmp_re;
            V[nav3_B.k + 12].im = nav3_B.atmp_im;
          }

          nav3_B.ihi++;
        }
      }

      for (nav3_B.ihi = 0; nav3_B.ihi < 4; nav3_B.ihi++) {
        nav3_B.c_i = nav3_B.ihi << 2;
        nav3_B.atmp_re = fabs(V[nav3_B.c_i].re) + fabs(V[nav3_B.c_i].im);
        nav3_B.k = nav3_B.c_i + 1;
        nav3_B.atmp_im = fabs(V[nav3_B.k].re) + fabs(V[nav3_B.k].im);
        if (nav3_B.atmp_im > nav3_B.atmp_re) {
          nav3_B.atmp_re = nav3_B.atmp_im;
        }

        nav3_B.i_j = nav3_B.c_i + 2;
        nav3_B.atmp_im = fabs(V[nav3_B.i_j].re) + fabs(V[nav3_B.i_j].im);
        if (nav3_B.atmp_im > nav3_B.atmp_re) {
          nav3_B.atmp_re = nav3_B.atmp_im;
        }

        nav3_B.jcol = nav3_B.c_i + 3;
        nav3_B.atmp_im = fabs(V[nav3_B.jcol].re) + fabs(V[nav3_B.jcol].im);
        if (nav3_B.atmp_im > nav3_B.atmp_re) {
          nav3_B.atmp_re = nav3_B.atmp_im;
        }

        if (nav3_B.atmp_re >= 6.7178761075670888E-139) {
          nav3_B.atmp_re = 1.0 / nav3_B.atmp_re;
          V[nav3_B.c_i].re *= nav3_B.atmp_re;
          V[nav3_B.c_i].im *= nav3_B.atmp_re;
          V[nav3_B.k].re *= nav3_B.atmp_re;
          V[nav3_B.k].im *= nav3_B.atmp_re;
          V[nav3_B.i_j].re *= nav3_B.atmp_re;
          V[nav3_B.i_j].im *= nav3_B.atmp_re;
          V[nav3_B.jcol].re *= nav3_B.atmp_re;
          V[nav3_B.jcol].im *= nav3_B.atmp_re;
        }
      }

      if (nav3_B.ilascl) {
        nav3_B.ilascl = true;
        while (nav3_B.ilascl) {
          nav3_B.atmp_re = nav3_B.anrmto * 2.0041683600089728E-292;
          nav3_B.atmp_im = nav3_B.anrm / 4.9896007738368E+291;
          if ((fabs(nav3_B.atmp_re) > fabs(nav3_B.anrm)) && (nav3_B.anrm != 0.0))
          {
            nav3_B.mul = 2.0041683600089728E-292;
            nav3_B.anrmto = nav3_B.atmp_re;
          } else if (fabs(nav3_B.atmp_im) > fabs(nav3_B.anrmto)) {
            nav3_B.mul = 4.9896007738368E+291;
            nav3_B.anrm = nav3_B.atmp_im;
          } else {
            nav3_B.mul = nav3_B.anrm / nav3_B.anrmto;
            nav3_B.ilascl = false;
          }

          alpha1[0].re *= nav3_B.mul;
          alpha1[0].im *= nav3_B.mul;
          alpha1[1].re *= nav3_B.mul;
          alpha1[1].im *= nav3_B.mul;
          alpha1[2].re *= nav3_B.mul;
          alpha1[2].im *= nav3_B.mul;
          alpha1[3].re *= nav3_B.mul;
          alpha1[3].im *= nav3_B.mul;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static real_T nav3_xnrm2(int32_T n, const real_T x[16], int32_T ix0)
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

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau, real_T C
  [16], int32_T ic0, real_T work[4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0) {
    nav3_B.lastv = m;
    nav3_B.lastc_o = iv0 + m;
    while ((nav3_B.lastv > 0) && (C[nav3_B.lastc_o - 2] == 0.0)) {
      nav3_B.lastv--;
      nav3_B.lastc_o--;
    }

    nav3_B.lastc_o = n - 1;
    exitg2 = false;
    while ((!exitg2) && (nav3_B.lastc_o + 1 > 0)) {
      nav3_B.coltop = (nav3_B.lastc_o << 2) + ic0;
      nav3_B.jy_g = nav3_B.coltop;
      do {
        exitg1 = 0;
        if (nav3_B.jy_g <= (nav3_B.coltop + nav3_B.lastv) - 1) {
          if (C[nav3_B.jy_g - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3_B.jy_g++;
          }
        } else {
          nav3_B.lastc_o--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3_B.lastv = 0;
    nav3_B.lastc_o = -1;
  }

  if (nav3_B.lastv > 0) {
    if (nav3_B.lastc_o + 1 != 0) {
      nav3_B.coltop = 0;
      while (nav3_B.coltop <= nav3_B.lastc_o) {
        work[nav3_B.coltop] = 0.0;
        nav3_B.coltop++;
      }

      nav3_B.coltop = 0;
      nav3_B.jy_g = (nav3_B.lastc_o << 2) + ic0;
      nav3_B.iac_f = ic0;
      while (nav3_B.iac_f <= nav3_B.jy_g) {
        nav3_B.ix_i = iv0;
        nav3_B.c = 0.0;
        nav3_B.d_i = (nav3_B.iac_f + nav3_B.lastv) - 1;
        nav3_B.b_ia_f = nav3_B.iac_f;
        while (nav3_B.b_ia_f <= nav3_B.d_i) {
          nav3_B.c += C[nav3_B.b_ia_f - 1] * C[nav3_B.ix_i - 1];
          nav3_B.ix_i++;
          nav3_B.b_ia_f++;
        }

        work[nav3_B.coltop] += nav3_B.c;
        nav3_B.coltop++;
        nav3_B.iac_f += 4;
      }
    }

    if (!(-tau == 0.0)) {
      nav3_B.coltop = ic0 - 1;
      nav3_B.jy_g = 0;
      nav3_B.iac_f = 0;
      while (nav3_B.iac_f <= nav3_B.lastc_o) {
        if (work[nav3_B.jy_g] != 0.0) {
          nav3_B.c = work[nav3_B.jy_g] * -tau;
          nav3_B.ix_i = iv0;
          nav3_B.d_i = nav3_B.lastv + nav3_B.coltop;
          nav3_B.b_ia_f = nav3_B.coltop;
          while (nav3_B.b_ia_f + 1 <= nav3_B.d_i) {
            C[nav3_B.b_ia_f] += C[nav3_B.ix_i - 1] * nav3_B.c;
            nav3_B.ix_i++;
            nav3_B.b_ia_f++;
          }
        }

        nav3_B.jy_g++;
        nav3_B.coltop += 4;
        nav3_B.iac_f++;
      }
    }
  }
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xgehrd(real_T a[16], real_T tau[3])
{
  int32_T exitg1;
  boolean_T exitg2;
  nav3_B.work_p[0] = 0.0;
  nav3_B.work_p[1] = 0.0;
  nav3_B.work_p[2] = 0.0;
  nav3_B.work_p[3] = 0.0;
  nav3_B.alpha1 = a[1];
  tau[0] = 0.0;
  nav3_B.xnorm = nav3_xnrm2(2, a, 3);
  if (nav3_B.xnorm != 0.0) {
    nav3_B.xnorm = nav3_rt_hypotd_snf(a[1], nav3_B.xnorm);
    if (a[1] >= 0.0) {
      nav3_B.xnorm = -nav3_B.xnorm;
    }

    if (fabs(nav3_B.xnorm) < 1.0020841800044864E-292) {
      nav3_B.knt = -1;
      do {
        nav3_B.knt++;
        nav3_B.lastc = 3;
        while (nav3_B.lastc <= 4) {
          a[nav3_B.lastc - 1] *= 9.9792015476736E+291;
          nav3_B.lastc++;
        }

        nav3_B.xnorm *= 9.9792015476736E+291;
        nav3_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(nav3_B.xnorm) >= 1.0020841800044864E-292));

      nav3_B.xnorm = nav3_rt_hypotd_snf(nav3_B.alpha1, nav3_xnrm2(2, a, 3));
      if (nav3_B.alpha1 >= 0.0) {
        nav3_B.xnorm = -nav3_B.xnorm;
      }

      tau[0] = (nav3_B.xnorm - nav3_B.alpha1) / nav3_B.xnorm;
      nav3_B.alpha1 = 1.0 / (nav3_B.alpha1 - nav3_B.xnorm);
      nav3_B.lastc = 3;
      while (nav3_B.lastc <= 4) {
        a[nav3_B.lastc - 1] *= nav3_B.alpha1;
        nav3_B.lastc++;
      }

      nav3_B.lastc = 0;
      while (nav3_B.lastc <= nav3_B.knt) {
        nav3_B.xnorm *= 1.0020841800044864E-292;
        nav3_B.lastc++;
      }

      nav3_B.alpha1 = nav3_B.xnorm;
    } else {
      tau[0] = (nav3_B.xnorm - a[1]) / nav3_B.xnorm;
      nav3_B.alpha1 = 1.0 / (a[1] - nav3_B.xnorm);
      nav3_B.knt = 3;
      while (nav3_B.knt <= 4) {
        a[nav3_B.knt - 1] *= nav3_B.alpha1;
        nav3_B.knt++;
      }

      nav3_B.alpha1 = nav3_B.xnorm;
    }
  }

  a[1] = 1.0;
  if (tau[0] != 0.0) {
    nav3_B.knt = 2;
    nav3_B.lastc = 3;
    while ((nav3_B.knt + 1 > 0) && (a[nav3_B.lastc] == 0.0)) {
      nav3_B.knt--;
      nav3_B.lastc--;
    }

    nav3_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (nav3_B.lastc > 0)) {
      nav3_B.rowleft = nav3_B.lastc + 4;
      nav3_B.jy = nav3_B.rowleft;
      do {
        exitg1 = 0;
        if (nav3_B.jy <= (nav3_B.knt << 2) + nav3_B.rowleft) {
          if (a[nav3_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3_B.jy += 4;
          }
        } else {
          nav3_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3_B.knt = -1;
    nav3_B.lastc = 0;
  }

  if (nav3_B.knt + 1 > 0) {
    if (nav3_B.lastc != 0) {
      nav3_B.rowleft = 0;
      while (nav3_B.rowleft <= nav3_B.lastc - 1) {
        nav3_B.work_p[nav3_B.rowleft] = 0.0;
        nav3_B.rowleft++;
      }

      nav3_B.rowleft = 1;
      nav3_B.jy = (nav3_B.knt << 2) + 5;
      nav3_B.iac = 5;
      while (nav3_B.iac <= nav3_B.jy) {
        nav3_B.b_ix = 0;
        nav3_B.g = (nav3_B.iac + nav3_B.lastc) - 1;
        nav3_B.b_ia = nav3_B.iac;
        while (nav3_B.b_ia <= nav3_B.g) {
          nav3_B.work_p[nav3_B.b_ix] += a[nav3_B.b_ia - 1] * a[nav3_B.rowleft];
          nav3_B.b_ix++;
          nav3_B.b_ia++;
        }

        nav3_B.rowleft++;
        nav3_B.iac += 4;
      }
    }

    if (!(-tau[0] == 0.0)) {
      nav3_B.rowleft = 4;
      nav3_B.jy = 1;
      nav3_B.iac = 0;
      while (nav3_B.iac <= nav3_B.knt) {
        if (a[nav3_B.jy] != 0.0) {
          nav3_B.xnorm = a[nav3_B.jy] * -tau[0];
          nav3_B.b_ix = 0;
          nav3_B.g = nav3_B.lastc + nav3_B.rowleft;
          nav3_B.b_ia = nav3_B.rowleft;
          while (nav3_B.b_ia + 1 <= nav3_B.g) {
            a[nav3_B.b_ia] += nav3_B.work_p[nav3_B.b_ix] * nav3_B.xnorm;
            nav3_B.b_ix++;
            nav3_B.b_ia++;
          }
        }

        nav3_B.jy++;
        nav3_B.rowleft += 4;
        nav3_B.iac++;
      }
    }
  }

  nav3_xzlarf(3, 3, 2, tau[0], a, 6, nav3_B.work_p);
  a[1] = nav3_B.alpha1;
  nav3_B.alpha1 = a[6];
  tau[1] = 0.0;
  nav3_B.xnorm = nav3_xnrm2(1, a, 8);
  if (nav3_B.xnorm != 0.0) {
    nav3_B.xnorm = nav3_rt_hypotd_snf(a[6], nav3_B.xnorm);
    if (a[6] >= 0.0) {
      nav3_B.xnorm = -nav3_B.xnorm;
    }

    if (fabs(nav3_B.xnorm) < 1.0020841800044864E-292) {
      nav3_B.knt = -1;
      do {
        nav3_B.knt++;
        a[7] *= 9.9792015476736E+291;
        nav3_B.xnorm *= 9.9792015476736E+291;
        nav3_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(nav3_B.xnorm) >= 1.0020841800044864E-292));

      nav3_B.xnorm = nav3_rt_hypotd_snf(nav3_B.alpha1, nav3_xnrm2(1, a, 8));
      if (nav3_B.alpha1 >= 0.0) {
        nav3_B.xnorm = -nav3_B.xnorm;
      }

      tau[1] = (nav3_B.xnorm - nav3_B.alpha1) / nav3_B.xnorm;
      nav3_B.alpha1 = 1.0 / (nav3_B.alpha1 - nav3_B.xnorm);
      a[7] *= nav3_B.alpha1;
      nav3_B.lastc = 0;
      while (nav3_B.lastc <= nav3_B.knt) {
        nav3_B.xnorm *= 1.0020841800044864E-292;
        nav3_B.lastc++;
      }

      nav3_B.alpha1 = nav3_B.xnorm;
    } else {
      tau[1] = (nav3_B.xnorm - a[6]) / nav3_B.xnorm;
      a[7] *= 1.0 / (a[6] - nav3_B.xnorm);
      nav3_B.alpha1 = nav3_B.xnorm;
    }
  }

  a[6] = 1.0;
  if (tau[1] != 0.0) {
    nav3_B.knt = 1;
    nav3_B.lastc = 7;
    while ((nav3_B.knt + 1 > 0) && (a[nav3_B.lastc] == 0.0)) {
      nav3_B.knt--;
      nav3_B.lastc--;
    }

    nav3_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (nav3_B.lastc > 0)) {
      nav3_B.rowleft = nav3_B.lastc + 8;
      nav3_B.jy = nav3_B.rowleft;
      do {
        exitg1 = 0;
        if (nav3_B.jy <= (nav3_B.knt << 2) + nav3_B.rowleft) {
          if (a[nav3_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3_B.jy += 4;
          }
        } else {
          nav3_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3_B.knt = -1;
    nav3_B.lastc = 0;
  }

  if (nav3_B.knt + 1 > 0) {
    if (nav3_B.lastc != 0) {
      nav3_B.rowleft = 0;
      while (nav3_B.rowleft <= nav3_B.lastc - 1) {
        nav3_B.work_p[nav3_B.rowleft] = 0.0;
        nav3_B.rowleft++;
      }

      nav3_B.rowleft = 6;
      nav3_B.jy = (nav3_B.knt << 2) + 9;
      nav3_B.iac = 9;
      while (nav3_B.iac <= nav3_B.jy) {
        nav3_B.b_ix = 0;
        nav3_B.g = (nav3_B.iac + nav3_B.lastc) - 1;
        nav3_B.b_ia = nav3_B.iac;
        while (nav3_B.b_ia <= nav3_B.g) {
          nav3_B.work_p[nav3_B.b_ix] += a[nav3_B.b_ia - 1] * a[nav3_B.rowleft];
          nav3_B.b_ix++;
          nav3_B.b_ia++;
        }

        nav3_B.rowleft++;
        nav3_B.iac += 4;
      }
    }

    if (!(-tau[1] == 0.0)) {
      nav3_B.rowleft = 8;
      nav3_B.jy = 6;
      nav3_B.iac = 0;
      while (nav3_B.iac <= nav3_B.knt) {
        if (a[nav3_B.jy] != 0.0) {
          nav3_B.xnorm = a[nav3_B.jy] * -tau[1];
          nav3_B.b_ix = 0;
          nav3_B.g = nav3_B.lastc + nav3_B.rowleft;
          nav3_B.b_ia = nav3_B.rowleft;
          while (nav3_B.b_ia + 1 <= nav3_B.g) {
            a[nav3_B.b_ia] += nav3_B.work_p[nav3_B.b_ix] * nav3_B.xnorm;
            nav3_B.b_ix++;
            nav3_B.b_ia++;
          }
        }

        nav3_B.jy++;
        nav3_B.rowleft += 4;
        nav3_B.iac++;
      }
    }
  }

  nav3_xzlarf(2, 2, 7, tau[1], a, 11, nav3_B.work_p);
  a[6] = nav3_B.alpha1;
  nav3_B.alpha1 = a[11];
  tau[2] = 0.0;
  nav3_B.xnorm = nav3_xnrm2(0, a, 12);
  if (nav3_B.xnorm != 0.0) {
    nav3_B.xnorm = nav3_rt_hypotd_snf(a[11], nav3_B.xnorm);
    if (a[11] >= 0.0) {
      nav3_B.xnorm = -nav3_B.xnorm;
    }

    if (fabs(nav3_B.xnorm) < 1.0020841800044864E-292) {
      nav3_B.knt = -1;
      do {
        nav3_B.knt++;
        nav3_B.xnorm *= 9.9792015476736E+291;
        nav3_B.alpha1 *= 9.9792015476736E+291;
      } while (!(fabs(nav3_B.xnorm) >= 1.0020841800044864E-292));

      nav3_B.xnorm = nav3_rt_hypotd_snf(nav3_B.alpha1, nav3_xnrm2(0, a, 12));
      if (nav3_B.alpha1 >= 0.0) {
        nav3_B.xnorm = -nav3_B.xnorm;
      }

      tau[2] = (nav3_B.xnorm - nav3_B.alpha1) / nav3_B.xnorm;
      nav3_B.lastc = 0;
      while (nav3_B.lastc <= nav3_B.knt) {
        nav3_B.xnorm *= 1.0020841800044864E-292;
        nav3_B.lastc++;
      }

      nav3_B.alpha1 = nav3_B.xnorm;
    } else {
      tau[2] = (nav3_B.xnorm - a[11]) / nav3_B.xnorm;
      nav3_B.alpha1 = nav3_B.xnorm;
    }
  }

  a[11] = 1.0;
  if (tau[2] != 0.0) {
    nav3_B.knt = 0;
    nav3_B.lastc = 11;
    while ((nav3_B.knt + 1 > 0) && (a[nav3_B.lastc] == 0.0)) {
      nav3_B.knt--;
      nav3_B.lastc--;
    }

    nav3_B.lastc = 4;
    exitg2 = false;
    while ((!exitg2) && (nav3_B.lastc > 0)) {
      nav3_B.rowleft = nav3_B.lastc + 12;
      nav3_B.jy = nav3_B.rowleft;
      do {
        exitg1 = 0;
        if (nav3_B.jy <= (nav3_B.knt << 2) + nav3_B.rowleft) {
          if (a[nav3_B.jy - 1] != 0.0) {
            exitg1 = 1;
          } else {
            nav3_B.jy += 4;
          }
        } else {
          nav3_B.lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    nav3_B.knt = -1;
    nav3_B.lastc = 0;
  }

  if (nav3_B.knt + 1 > 0) {
    if (nav3_B.lastc != 0) {
      nav3_B.rowleft = 0;
      while (nav3_B.rowleft <= nav3_B.lastc - 1) {
        nav3_B.work_p[nav3_B.rowleft] = 0.0;
        nav3_B.rowleft++;
      }

      nav3_B.rowleft = 11;
      nav3_B.jy = (nav3_B.knt << 2) + 13;
      nav3_B.iac = 13;
      while (nav3_B.iac <= nav3_B.jy) {
        nav3_B.b_ix = 0;
        nav3_B.g = (nav3_B.iac + nav3_B.lastc) - 1;
        nav3_B.b_ia = nav3_B.iac;
        while (nav3_B.b_ia <= nav3_B.g) {
          nav3_B.work_p[nav3_B.b_ix] += a[nav3_B.b_ia - 1] * a[nav3_B.rowleft];
          nav3_B.b_ix++;
          nav3_B.b_ia++;
        }

        nav3_B.rowleft++;
        nav3_B.iac += 4;
      }
    }

    if (!(-tau[2] == 0.0)) {
      nav3_B.rowleft = 12;
      nav3_B.jy = 11;
      nav3_B.iac = 0;
      while (nav3_B.iac <= nav3_B.knt) {
        if (a[nav3_B.jy] != 0.0) {
          nav3_B.xnorm = a[nav3_B.jy] * -tau[2];
          nav3_B.b_ix = 0;
          nav3_B.g = nav3_B.lastc + nav3_B.rowleft;
          nav3_B.b_ia = nav3_B.rowleft;
          while (nav3_B.b_ia + 1 <= nav3_B.g) {
            a[nav3_B.b_ia] += nav3_B.work_p[nav3_B.b_ix] * nav3_B.xnorm;
            nav3_B.b_ix++;
            nav3_B.b_ia++;
          }
        }

        nav3_B.jy++;
        nav3_B.rowleft += 4;
        nav3_B.iac++;
      }
    }
  }

  nav3_xzlarf(1, 1, 12, tau[2], a, 16, nav3_B.work_p);
  a[11] = nav3_B.alpha1;
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static real_T nav3_xnrm2_j(int32_T n, const real_T x[3])
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[1]);
    } else {
      nav3_B.scale_ny = 3.3121686421112381E-170;
      nav3_B.absxk_i = fabs(x[1]);
      if (nav3_B.absxk_i > 3.3121686421112381E-170) {
        y = 1.0;
        nav3_B.scale_ny = nav3_B.absxk_i;
      } else {
        nav3_B.t_o = nav3_B.absxk_i / 3.3121686421112381E-170;
        y = nav3_B.t_o * nav3_B.t_o;
      }

      nav3_B.absxk_i = fabs(x[2]);
      if (nav3_B.absxk_i > nav3_B.scale_ny) {
        nav3_B.t_o = nav3_B.scale_ny / nav3_B.absxk_i;
        y = y * nav3_B.t_o * nav3_B.t_o + 1.0;
        nav3_B.scale_ny = nav3_B.absxk_i;
      } else {
        nav3_B.t_o = nav3_B.absxk_i / nav3_B.scale_ny;
        y += nav3_B.t_o * nav3_B.t_o;
      }

      y = nav3_B.scale_ny * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static real_T nav3_xzlarfg(int32_T n, real_T *alpha1, real_T x[3])
{
  real_T tau;
  tau = 0.0;
  if (n > 0) {
    nav3_B.xnorm_o = nav3_xnrm2_j(n - 1, x);
    if (nav3_B.xnorm_o != 0.0) {
      nav3_B.xnorm_o = nav3_rt_hypotd_snf(*alpha1, nav3_B.xnorm_o);
      if (*alpha1 >= 0.0) {
        nav3_B.xnorm_o = -nav3_B.xnorm_o;
      }

      if (fabs(nav3_B.xnorm_o) < 1.0020841800044864E-292) {
        nav3_B.knt_c = -1;
        do {
          nav3_B.knt_c++;
          nav3_B.c_k = 1;
          while (nav3_B.c_k + 1 <= n) {
            x[nav3_B.c_k] *= 9.9792015476736E+291;
            nav3_B.c_k++;
          }

          nav3_B.xnorm_o *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(nav3_B.xnorm_o) >= 1.0020841800044864E-292));

        nav3_B.xnorm_o = nav3_rt_hypotd_snf(*alpha1, nav3_xnrm2_j(n - 1, x));
        if (*alpha1 >= 0.0) {
          nav3_B.xnorm_o = -nav3_B.xnorm_o;
        }

        tau = (nav3_B.xnorm_o - *alpha1) / nav3_B.xnorm_o;
        *alpha1 = 1.0 / (*alpha1 - nav3_B.xnorm_o);
        nav3_B.c_k = 1;
        while (nav3_B.c_k + 1 <= n) {
          x[nav3_B.c_k] *= *alpha1;
          nav3_B.c_k++;
        }

        nav3_B.c_k = 0;
        while (nav3_B.c_k <= nav3_B.knt_c) {
          nav3_B.xnorm_o *= 1.0020841800044864E-292;
          nav3_B.c_k++;
        }

        *alpha1 = nav3_B.xnorm_o;
      } else {
        tau = (nav3_B.xnorm_o - *alpha1) / nav3_B.xnorm_o;
        *alpha1 = 1.0 / (*alpha1 - nav3_B.xnorm_o);
        nav3_B.knt_c = 1;
        while (nav3_B.knt_c + 1 <= n) {
          x[nav3_B.knt_c] *= *alpha1;
          nav3_B.knt_c++;
        }

        *alpha1 = nav3_B.xnorm_o;
      }
    }
  }

  return tau;
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xdlanv2(real_T *a, real_T *b, real_T *c, real_T *d, real_T
  *rt1r, real_T *rt1i, real_T *rt2r, real_T *rt2i, real_T *cs, real_T *sn)
{
  if (*c == 0.0) {
    *cs = 1.0;
    *sn = 0.0;
  } else if (*b == 0.0) {
    *cs = 0.0;
    *sn = 1.0;
    nav3_B.bcmax = *d;
    *d = *a;
    *a = nav3_B.bcmax;
    *b = -*c;
    *c = 0.0;
  } else {
    nav3_B.tau_l = *a - *d;
    if ((nav3_B.tau_l == 0.0) && ((*b < 0.0) != (*c < 0.0))) {
      *cs = 1.0;
      *sn = 0.0;
    } else {
      nav3_B.p = 0.5 * nav3_B.tau_l;
      nav3_B.bcmis = fabs(*b);
      nav3_B.z = fabs(*c);
      nav3_B.b_h = rtIsNaN(nav3_B.z);
      if ((nav3_B.bcmis > nav3_B.z) || nav3_B.b_h) {
        nav3_B.bcmax = nav3_B.bcmis;
      } else {
        nav3_B.bcmax = nav3_B.z;
      }

      if ((nav3_B.bcmis < nav3_B.z) || nav3_B.b_h) {
        nav3_B.z = nav3_B.bcmis;
      }

      if (!(*b < 0.0)) {
        nav3_B.b_a = 1;
      } else {
        nav3_B.b_a = -1;
      }

      if (!(*c < 0.0)) {
        nav3_B.c_if = 1;
      } else {
        nav3_B.c_if = -1;
      }

      nav3_B.bcmis = nav3_B.z * static_cast<real_T>(nav3_B.b_a) * static_cast<
        real_T>(nav3_B.c_if);
      nav3_B.scale_d = fabs(nav3_B.p);
      if ((!(nav3_B.scale_d > nav3_B.bcmax)) && (!rtIsNaN(nav3_B.bcmax))) {
        nav3_B.scale_d = nav3_B.bcmax;
      }

      nav3_B.z = nav3_B.p / nav3_B.scale_d * nav3_B.p + nav3_B.bcmax /
        nav3_B.scale_d * nav3_B.bcmis;
      if (nav3_B.z >= 8.8817841970012523E-16) {
        if (!(nav3_B.p < 0.0)) {
          nav3_B.tau_l = sqrt(nav3_B.scale_d) * sqrt(nav3_B.z);
        } else {
          nav3_B.tau_l = -(sqrt(nav3_B.scale_d) * sqrt(nav3_B.z));
        }

        nav3_B.z = nav3_B.p + nav3_B.tau_l;
        *a = *d + nav3_B.z;
        *d -= nav3_B.bcmax / nav3_B.z * nav3_B.bcmis;
        nav3_B.tau_l = nav3_rt_hypotd_snf(*c, nav3_B.z);
        *cs = nav3_B.z / nav3_B.tau_l;
        *sn = *c / nav3_B.tau_l;
        *b -= *c;
        *c = 0.0;
      } else {
        nav3_B.bcmax = *b + *c;
        nav3_B.tau_l = nav3_rt_hypotd_snf(nav3_B.bcmax, nav3_B.tau_l);
        *cs = sqrt((fabs(nav3_B.bcmax) / nav3_B.tau_l + 1.0) * 0.5);
        if (!(nav3_B.bcmax < 0.0)) {
          nav3_B.b_a = 1;
        } else {
          nav3_B.b_a = -1;
        }

        *sn = -(nav3_B.p / (nav3_B.tau_l * *cs)) * static_cast<real_T>
          (nav3_B.b_a);
        nav3_B.p = *a * *cs + *b * *sn;
        nav3_B.tau_l = -*a * *sn + *b * *cs;
        nav3_B.bcmax = *c * *cs + *d * *sn;
        nav3_B.bcmis = -*c * *sn + *d * *cs;
        *b = nav3_B.tau_l * *cs + nav3_B.bcmis * *sn;
        *c = -nav3_B.p * *sn + nav3_B.bcmax * *cs;
        nav3_B.bcmax = ((nav3_B.p * *cs + nav3_B.bcmax * *sn) + (-nav3_B.tau_l *
          *sn + nav3_B.bcmis * *cs)) * 0.5;
        *a = nav3_B.bcmax;
        *d = nav3_B.bcmax;
        if (*c != 0.0) {
          if (*b != 0.0) {
            if ((*b < 0.0) == (*c < 0.0)) {
              nav3_B.z = sqrt(fabs(*b));
              nav3_B.bcmis = sqrt(fabs(*c));
              if (!(*c < 0.0)) {
                nav3_B.p = nav3_B.z * nav3_B.bcmis;
              } else {
                nav3_B.p = -(nav3_B.z * nav3_B.bcmis);
              }

              nav3_B.tau_l = 1.0 / sqrt(fabs(*b + *c));
              *a = nav3_B.bcmax + nav3_B.p;
              *d = nav3_B.bcmax - nav3_B.p;
              *b -= *c;
              *c = 0.0;
              nav3_B.p = nav3_B.z * nav3_B.tau_l;
              nav3_B.tau_l *= nav3_B.bcmis;
              nav3_B.bcmax = *cs * nav3_B.p - *sn * nav3_B.tau_l;
              *sn = *cs * nav3_B.tau_l + *sn * nav3_B.p;
              *cs = nav3_B.bcmax;
            }
          } else {
            *b = -*c;
            *c = 0.0;
            nav3_B.bcmax = *cs;
            *cs = -*sn;
            *sn = nav3_B.bcmax;
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

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xrot(int32_T n, real_T x[16], int32_T ix0, int32_T iy0, real_T
                      c, real_T s)
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

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_xrot_o(int32_T n, real_T x[16], int32_T ix0, int32_T iy0,
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

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static int32_T nav3_eml_dlahqr(real_T h[16], real_T z[16])
{
  int32_T info;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  nav3_B.v[0] = 0.0;
  nav3_B.v[1] = 0.0;
  nav3_B.v[2] = 0.0;
  h[2] = 0.0;
  h[3] = 0.0;
  h[7] = 0.0;
  nav3_B.i_p = 3;
  exitg1 = false;
  while ((!exitg1) && (nav3_B.i_p + 1 >= 1)) {
    nav3_B.L = 1;
    nav3_B.goto150 = false;
    nav3_B.ix = 0;
    exitg2 = false;
    while ((!exitg2) && (nav3_B.ix < 301)) {
      nav3_B.k_a = nav3_B.i_p;
      exitg3 = false;
      while ((!exitg3) && (nav3_B.k_a + 1 > nav3_B.L)) {
        nav3_B.s_tmp = ((nav3_B.k_a - 1) << 2) + nav3_B.k_a;
        nav3_B.htmp2 = fabs(h[nav3_B.s_tmp]);
        if (nav3_B.htmp2 <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          nav3_B.m = (nav3_B.k_a << 2) + nav3_B.k_a;
          nav3_B.nr = nav3_B.s_tmp - 1;
          nav3_B.tst = fabs(h[nav3_B.nr]) + fabs(h[nav3_B.m]);
          if (nav3_B.tst == 0.0) {
            if (nav3_B.k_a - 1 >= 1) {
              nav3_B.tst = fabs(h[(((nav3_B.k_a - 2) << 2) + nav3_B.k_a) - 1]);
            }

            if (nav3_B.k_a + 2 <= 4) {
              nav3_B.tst += fabs(h[((nav3_B.k_a << 2) + nav3_B.k_a) + 1]);
            }
          }

          if (nav3_B.htmp2 <= 2.2204460492503131E-16 * nav3_B.tst) {
            nav3_B.htmp1 = fabs(h[nav3_B.s_tmp]);
            nav3_B.htmp2 = fabs(h[nav3_B.m - 1]);
            if (nav3_B.htmp1 > nav3_B.htmp2) {
              nav3_B.tst = nav3_B.htmp1;
              nav3_B.ba = nav3_B.htmp2;
            } else {
              nav3_B.tst = nav3_B.htmp2;
              nav3_B.ba = nav3_B.htmp1;
            }

            nav3_B.htmp1 = fabs(h[nav3_B.m]);
            nav3_B.htmp2 = fabs(h[nav3_B.nr] - h[nav3_B.m]);
            if (nav3_B.htmp1 > nav3_B.htmp2) {
              nav3_B.aa = nav3_B.htmp1;
              nav3_B.htmp1 = nav3_B.htmp2;
            } else {
              nav3_B.aa = nav3_B.htmp2;
            }

            nav3_B.htmp2 = nav3_B.aa + nav3_B.tst;
            nav3_B.htmp1 = nav3_B.aa / nav3_B.htmp2 * nav3_B.htmp1 *
              2.2204460492503131E-16;
            if ((4.0083367200179456E-292 > nav3_B.htmp1) || rtIsNaN(nav3_B.htmp1))
            {
              nav3_B.htmp1 = 4.0083367200179456E-292;
            }

            if (nav3_B.tst / nav3_B.htmp2 * nav3_B.ba <= nav3_B.htmp1) {
              exitg3 = true;
            } else {
              nav3_B.k_a--;
            }
          } else {
            nav3_B.k_a--;
          }
        }
      }

      nav3_B.L = nav3_B.k_a + 1;
      if (nav3_B.k_a + 1 > 1) {
        h[nav3_B.k_a + ((nav3_B.k_a - 1) << 2)] = 0.0;
      }

      if (nav3_B.k_a + 1 >= nav3_B.i_p) {
        nav3_B.goto150 = true;
        exitg2 = true;
      } else {
        if (nav3_B.ix == 10) {
          nav3_B.s_tmp = (nav3_B.k_a << 2) + nav3_B.k_a;
          nav3_B.htmp2 = fabs(h[(((nav3_B.k_a + 1) << 2) + nav3_B.k_a) + 2]) +
            fabs(h[nav3_B.s_tmp + 1]);
          nav3_B.ba = h[nav3_B.s_tmp] + 0.75 * nav3_B.htmp2;
          nav3_B.h12 = -0.4375 * nav3_B.htmp2;
          nav3_B.aa = nav3_B.htmp2;
          nav3_B.tst = nav3_B.ba;
        } else if (nav3_B.ix == 20) {
          nav3_B.htmp2 = fabs(h[(((nav3_B.i_p - 2) << 2) + nav3_B.i_p) - 1]) +
            fabs(h[((nav3_B.i_p - 1) << 2) + nav3_B.i_p]);
          nav3_B.ba = h[(nav3_B.i_p << 2) + nav3_B.i_p] + 0.75 * nav3_B.htmp2;
          nav3_B.h12 = -0.4375 * nav3_B.htmp2;
          nav3_B.aa = nav3_B.htmp2;
          nav3_B.tst = nav3_B.ba;
        } else {
          nav3_B.ba = h[(((nav3_B.i_p - 1) << 2) + nav3_B.i_p) - 1];
          nav3_B.aa = h[((nav3_B.i_p - 1) << 2) + nav3_B.i_p];
          nav3_B.h12 = h[((nav3_B.i_p << 2) + nav3_B.i_p) - 1];
          nav3_B.tst = h[(nav3_B.i_p << 2) + nav3_B.i_p];
        }

        nav3_B.htmp2 = ((fabs(nav3_B.ba) + fabs(nav3_B.h12)) + fabs(nav3_B.aa))
          + fabs(nav3_B.tst);
        if (nav3_B.htmp2 == 0.0) {
          nav3_B.ba = 0.0;
          nav3_B.tst = 0.0;
          nav3_B.htmp1 = 0.0;
          nav3_B.aa = 0.0;
        } else {
          nav3_B.ba /= nav3_B.htmp2;
          nav3_B.aa /= nav3_B.htmp2;
          nav3_B.h12 /= nav3_B.htmp2;
          nav3_B.tst /= nav3_B.htmp2;
          nav3_B.htmp1 = (nav3_B.ba + nav3_B.tst) / 2.0;
          nav3_B.ba = (nav3_B.ba - nav3_B.htmp1) * (nav3_B.tst - nav3_B.htmp1) -
            nav3_B.h12 * nav3_B.aa;
          nav3_B.aa = sqrt(fabs(nav3_B.ba));
          if (nav3_B.ba >= 0.0) {
            nav3_B.ba = nav3_B.htmp1 * nav3_B.htmp2;
            nav3_B.htmp1 = nav3_B.ba;
            nav3_B.tst = nav3_B.aa * nav3_B.htmp2;
            nav3_B.aa = -nav3_B.tst;
          } else {
            nav3_B.ba = nav3_B.htmp1 + nav3_B.aa;
            nav3_B.htmp1 -= nav3_B.aa;
            if (fabs(nav3_B.ba - nav3_B.tst) <= fabs(nav3_B.htmp1 - nav3_B.tst))
            {
              nav3_B.ba *= nav3_B.htmp2;
              nav3_B.htmp1 = nav3_B.ba;
            } else {
              nav3_B.htmp1 *= nav3_B.htmp2;
              nav3_B.ba = nav3_B.htmp1;
            }

            nav3_B.tst = 0.0;
            nav3_B.aa = 0.0;
          }
        }

        nav3_B.m = nav3_B.i_p - 1;
        exitg3 = false;
        while ((!exitg3) && (nav3_B.m >= nav3_B.k_a + 1)) {
          nav3_B.s_tmp = ((nav3_B.m - 1) << 2) + nav3_B.m;
          nav3_B.nr = nav3_B.s_tmp - 1;
          nav3_B.h12 = h[nav3_B.nr] - nav3_B.htmp1;
          nav3_B.htmp2 = (fabs(nav3_B.h12) + fabs(nav3_B.aa)) + fabs
            (h[nav3_B.s_tmp]);
          nav3_B.h21s = h[nav3_B.s_tmp] / nav3_B.htmp2;
          nav3_B.hoffset = (nav3_B.m << 2) + nav3_B.m;
          nav3_B.v[0] = (nav3_B.h12 / nav3_B.htmp2 * (h[nav3_B.nr] - nav3_B.ba)
                         + h[nav3_B.hoffset - 1] * nav3_B.h21s) - nav3_B.aa /
            nav3_B.htmp2 * nav3_B.tst;
          nav3_B.v[1] = (((h[nav3_B.nr] + h[nav3_B.hoffset]) - nav3_B.ba) -
                         nav3_B.htmp1) * nav3_B.h21s;
          nav3_B.v[2] = h[nav3_B.hoffset + 1] * nav3_B.h21s;
          nav3_B.htmp2 = (fabs(nav3_B.v[0]) + fabs(nav3_B.v[1])) + fabs
            (nav3_B.v[2]);
          nav3_B.v[0] /= nav3_B.htmp2;
          nav3_B.v[1] /= nav3_B.htmp2;
          nav3_B.v[2] /= nav3_B.htmp2;
          if (nav3_B.k_a + 1 == nav3_B.m) {
            exitg3 = true;
          } else {
            nav3_B.s_tmp = ((nav3_B.m - 2) << 2) + nav3_B.m;
            if (fabs(h[nav3_B.s_tmp - 1]) * (fabs(nav3_B.v[1]) + fabs(nav3_B.v[2]))
                <= ((fabs(h[nav3_B.s_tmp - 2]) + fabs(h[nav3_B.nr])) + fabs
                    (h[nav3_B.hoffset])) * (2.2204460492503131E-16 * fabs
                 (nav3_B.v[0]))) {
              exitg3 = true;
            } else {
              nav3_B.m--;
            }
          }
        }

        nav3_B.s_tmp = nav3_B.m;
        while (nav3_B.s_tmp <= nav3_B.i_p) {
          nav3_B.nr = (nav3_B.i_p - nav3_B.s_tmp) + 2;
          if (3 < nav3_B.nr) {
            nav3_B.nr = 3;
          }

          if (nav3_B.s_tmp > nav3_B.m) {
            nav3_B.hoffset = ((nav3_B.s_tmp - 2) << 2) + nav3_B.s_tmp;
            nav3_B.j_e = 0;
            while (nav3_B.j_e <= nav3_B.nr - 1) {
              nav3_B.v[nav3_B.j_e] = h[(nav3_B.j_e + nav3_B.hoffset) - 1];
              nav3_B.j_e++;
            }
          }

          nav3_B.tst = nav3_B.v[0];
          nav3_B.b_v[0] = nav3_B.v[0];
          nav3_B.b_v[1] = nav3_B.v[1];
          nav3_B.b_v[2] = nav3_B.v[2];
          nav3_B.htmp2 = nav3_xzlarfg(nav3_B.nr, &nav3_B.tst, nav3_B.b_v);
          nav3_B.v[1] = nav3_B.b_v[1];
          nav3_B.v[2] = nav3_B.b_v[2];
          nav3_B.v[0] = nav3_B.tst;
          if (nav3_B.s_tmp > nav3_B.m) {
            h[(nav3_B.s_tmp + ((nav3_B.s_tmp - 2) << 2)) - 1] = nav3_B.tst;
            h[nav3_B.s_tmp + ((nav3_B.s_tmp - 2) << 2)] = 0.0;
            if (nav3_B.s_tmp < nav3_B.i_p) {
              h[nav3_B.s_tmp + 1] = 0.0;
            }
          } else {
            if (nav3_B.m > nav3_B.k_a + 1) {
              h[nav3_B.s_tmp - 1] *= 1.0 - nav3_B.htmp2;
            }
          }

          nav3_B.tst = nav3_B.b_v[1];
          nav3_B.ba = nav3_B.htmp2 * nav3_B.b_v[1];
          if (nav3_B.nr == 3) {
            nav3_B.aa = nav3_B.b_v[2];
            nav3_B.h12 = nav3_B.htmp2 * nav3_B.b_v[2];
            nav3_B.b_j_a = nav3_B.s_tmp - 1;
            while (nav3_B.b_j_a + 1 < 5) {
              nav3_B.nr = (nav3_B.b_j_a << 2) + nav3_B.s_tmp;
              nav3_B.hoffset = nav3_B.nr - 1;
              nav3_B.j_e = nav3_B.nr + 1;
              nav3_B.htmp1 = (h[nav3_B.hoffset] + h[nav3_B.nr] * nav3_B.tst) +
                h[nav3_B.j_e] * nav3_B.aa;
              h[nav3_B.hoffset] -= nav3_B.htmp1 * nav3_B.htmp2;
              h[nav3_B.nr] -= nav3_B.htmp1 * nav3_B.ba;
              h[nav3_B.j_e] -= nav3_B.htmp1 * nav3_B.h12;
              nav3_B.b_j_a++;
            }

            nav3_B.nr = nav3_B.s_tmp + 3;
            nav3_B.b_j_a = nav3_B.i_p + 1;
            if (nav3_B.nr < nav3_B.b_j_a) {
              nav3_B.b_j_a = nav3_B.nr;
            }

            nav3_B.c_j = 0;
            while (nav3_B.c_j <= nav3_B.b_j_a - 1) {
              nav3_B.nr = ((nav3_B.s_tmp - 1) << 2) + nav3_B.c_j;
              nav3_B.hoffset = (nav3_B.s_tmp << 2) + nav3_B.c_j;
              nav3_B.j_e = ((nav3_B.s_tmp + 1) << 2) + nav3_B.c_j;
              nav3_B.htmp1 = (h[nav3_B.nr] + h[nav3_B.hoffset] * nav3_B.tst) +
                h[nav3_B.j_e] * nav3_B.aa;
              h[nav3_B.nr] -= nav3_B.htmp1 * nav3_B.htmp2;
              h[nav3_B.hoffset] -= nav3_B.htmp1 * nav3_B.ba;
              h[nav3_B.j_e] -= nav3_B.htmp1 * nav3_B.h12;
              nav3_B.c_j++;
            }

            for (nav3_B.b_j_a = 0; nav3_B.b_j_a < 4; nav3_B.b_j_a++) {
              nav3_B.nr = ((nav3_B.s_tmp - 1) << 2) + nav3_B.b_j_a;
              nav3_B.hoffset = (nav3_B.s_tmp << 2) + nav3_B.b_j_a;
              nav3_B.j_e = ((nav3_B.s_tmp + 1) << 2) + nav3_B.b_j_a;
              nav3_B.htmp1 = (z[nav3_B.nr] + z[nav3_B.hoffset] * nav3_B.tst) +
                z[nav3_B.j_e] * nav3_B.aa;
              z[nav3_B.nr] -= nav3_B.htmp1 * nav3_B.htmp2;
              z[nav3_B.hoffset] -= nav3_B.htmp1 * nav3_B.ba;
              z[nav3_B.j_e] -= nav3_B.htmp1 * nav3_B.h12;
            }
          } else {
            if (nav3_B.nr == 2) {
              nav3_B.j_e = nav3_B.s_tmp - 1;
              while (nav3_B.j_e + 1 < 5) {
                nav3_B.nr = (nav3_B.j_e << 2) + nav3_B.s_tmp;
                nav3_B.hoffset = nav3_B.nr - 1;
                nav3_B.htmp1 = h[nav3_B.hoffset] + h[nav3_B.nr] * nav3_B.tst;
                h[nav3_B.hoffset] -= nav3_B.htmp1 * nav3_B.htmp2;
                h[nav3_B.nr] -= nav3_B.htmp1 * nav3_B.ba;
                nav3_B.j_e++;
              }

              nav3_B.j_e = 0;
              while (nav3_B.j_e <= nav3_B.i_p) {
                nav3_B.nr = ((nav3_B.s_tmp - 1) << 2) + nav3_B.j_e;
                nav3_B.hoffset = (nav3_B.s_tmp << 2) + nav3_B.j_e;
                nav3_B.htmp1 = h[nav3_B.nr] + h[nav3_B.hoffset] * nav3_B.tst;
                h[nav3_B.nr] -= nav3_B.htmp1 * nav3_B.htmp2;
                h[nav3_B.hoffset] -= nav3_B.htmp1 * nav3_B.ba;
                nav3_B.j_e++;
              }

              for (nav3_B.j_e = 0; nav3_B.j_e < 4; nav3_B.j_e++) {
                nav3_B.nr = ((nav3_B.s_tmp - 1) << 2) + nav3_B.j_e;
                nav3_B.hoffset = (nav3_B.s_tmp << 2) + nav3_B.j_e;
                nav3_B.htmp1 = z[nav3_B.nr] + z[nav3_B.hoffset] * nav3_B.tst;
                z[nav3_B.nr] -= nav3_B.htmp1 * nav3_B.htmp2;
                z[nav3_B.hoffset] -= nav3_B.htmp1 * nav3_B.ba;
              }
            }
          }

          nav3_B.s_tmp++;
        }

        nav3_B.ix++;
      }
    }

    if (!nav3_B.goto150) {
      info = nav3_B.i_p + 1;
      exitg1 = true;
    } else {
      if ((nav3_B.i_p + 1 != nav3_B.L) && (nav3_B.L == nav3_B.i_p)) {
        nav3_B.ix = (nav3_B.i_p - 1) << 2;
        nav3_B.k_a = nav3_B.ix + nav3_B.i_p;
        nav3_B.m = nav3_B.k_a - 1;
        nav3_B.ba = h[nav3_B.m];
        nav3_B.s_tmp = nav3_B.i_p << 2;
        nav3_B.nr = nav3_B.s_tmp + nav3_B.i_p;
        nav3_B.hoffset = nav3_B.nr - 1;
        nav3_B.htmp1 = h[nav3_B.hoffset];
        nav3_B.aa = h[nav3_B.k_a];
        nav3_B.h12 = h[nav3_B.nr];
        nav3_xdlanv2(&nav3_B.ba, &nav3_B.htmp1, &nav3_B.aa, &nav3_B.h12,
                     &nav3_B.h21s, &nav3_B.unusedU1, &nav3_B.unusedU2,
                     &nav3_B.unusedU3, &nav3_B.htmp2, &nav3_B.tst);
        h[nav3_B.m] = nav3_B.ba;
        h[nav3_B.hoffset] = nav3_B.htmp1;
        h[nav3_B.k_a] = nav3_B.aa;
        h[nav3_B.nr] = nav3_B.h12;
        if (4 > nav3_B.i_p + 1) {
          nav3_xrot(3 - nav3_B.i_p, h, nav3_B.i_p + ((nav3_B.i_p + 1) << 2),
                    (nav3_B.i_p + ((nav3_B.i_p + 1) << 2)) + 1, nav3_B.htmp2,
                    nav3_B.tst);
        }

        nav3_xrot_o(nav3_B.i_p - 1, h, ((nav3_B.i_p - 1) << 2) + 1, (nav3_B.i_p <<
          2) + 1, nav3_B.htmp2, nav3_B.tst);
        nav3_B.ba = nav3_B.htmp2 * z[nav3_B.ix] + nav3_B.tst * z[nav3_B.s_tmp];
        z[nav3_B.s_tmp] = nav3_B.htmp2 * z[nav3_B.s_tmp] - nav3_B.tst *
          z[nav3_B.ix];
        z[nav3_B.ix] = nav3_B.ba;
        nav3_B.i_p = nav3_B.s_tmp + 1;
        nav3_B.ix++;
        nav3_B.ba = nav3_B.htmp2 * z[nav3_B.ix] + nav3_B.tst * z[nav3_B.i_p];
        z[nav3_B.i_p] = nav3_B.htmp2 * z[nav3_B.i_p] - nav3_B.tst * z[nav3_B.ix];
        z[nav3_B.ix] = nav3_B.ba;
        nav3_B.i_p++;
        nav3_B.ix++;
        nav3_B.ba = nav3_B.htmp2 * z[nav3_B.ix] + nav3_B.tst * z[nav3_B.i_p];
        z[nav3_B.i_p] = nav3_B.htmp2 * z[nav3_B.i_p] - nav3_B.tst * z[nav3_B.ix];
        z[nav3_B.ix] = nav3_B.ba;
        nav3_B.i_p++;
        nav3_B.ix++;
        nav3_B.ba = nav3_B.htmp2 * z[nav3_B.ix] + nav3_B.tst * z[nav3_B.i_p];
        z[nav3_B.i_p] = nav3_B.htmp2 * z[nav3_B.i_p] - nav3_B.tst * z[nav3_B.ix];
        z[nav3_B.ix] = nav3_B.ba;
      }

      nav3_B.i_p = nav3_B.L - 2;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S97>/World to Robot Transform' */
static void nav3_eig(const real_T A[16], creal_T V[16], creal_T D[4])
{
  int32_T exitg1;
  boolean_T exitg2;
  if (nav3_anyNonFinite(A)) {
    for (nav3_B.b_j = 0; nav3_B.b_j < 16; nav3_B.b_j++) {
      V[nav3_B.b_j].re = (rtNaN);
      V[nav3_B.b_j].im = 0.0;
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
    nav3_B.p_e = true;
    nav3_B.b_j = 0;
    exitg2 = false;
    while ((!exitg2) && (nav3_B.b_j < 4)) {
      nav3_B.i_m = 0;
      do {
        exitg1 = 0;
        if (nav3_B.i_m <= nav3_B.b_j) {
          if (!(A[(nav3_B.b_j << 2) + nav3_B.i_m] == A[(nav3_B.i_m << 2) +
                nav3_B.b_j])) {
            nav3_B.p_e = false;
            exitg1 = 1;
          } else {
            nav3_B.i_m++;
          }
        } else {
          nav3_B.b_j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (nav3_B.p_e) {
      if (nav3_anyNonFinite(A)) {
        for (nav3_B.b_j = 0; nav3_B.b_j < 16; nav3_B.b_j++) {
          nav3_B.b_V[nav3_B.b_j] = (rtNaN);
        }

        nav3_B.b_j = 2;
        while (nav3_B.b_j < 5) {
          nav3_B.b_V[nav3_B.b_j - 1] = 0.0;
          nav3_B.b_j++;
        }

        nav3_B.b_j = 3;
        while (nav3_B.b_j < 5) {
          nav3_B.b_V[nav3_B.b_j + 3] = 0.0;
          nav3_B.b_j++;
        }

        nav3_B.b_V[11] = 0.0;
        for (nav3_B.b_j = 0; nav3_B.b_j < 16; nav3_B.b_j++) {
          nav3_B.b_A[nav3_B.b_j] = (rtNaN);
        }
      } else {
        memcpy(&nav3_B.b_A[0], &A[0], sizeof(real_T) << 4U);
        nav3_xgehrd(nav3_B.b_A, nav3_B.tau);
        memcpy(&nav3_B.b_V[0], &nav3_B.b_A[0], sizeof(real_T) << 4U);
        nav3_B.b_j = 0;
        while (nav3_B.b_j <= 2) {
          nav3_B.b_V[nav3_B.b_j + 12] = 0.0;
          nav3_B.b_j++;
        }

        nav3_B.b_j = 0;
        while (nav3_B.b_j <= 1) {
          nav3_B.b_V[nav3_B.b_j + 8] = 0.0;
          nav3_B.b_j++;
        }

        nav3_B.b_j = 1;
        while (nav3_B.b_j + 3 < 5) {
          nav3_B.b_V[nav3_B.b_j + 10] = nav3_B.b_V[nav3_B.b_j + 6];
          nav3_B.b_j++;
        }

        nav3_B.b_V[4] = 0.0;
        nav3_B.b_j = 0;
        while (nav3_B.b_j + 3 < 5) {
          nav3_B.b_V[nav3_B.b_j + 6] = nav3_B.b_V[nav3_B.b_j + 2];
          nav3_B.b_j++;
        }

        nav3_B.work[0] = 0.0;
        nav3_B.b_V[1] = 0.0;
        nav3_B.work[1] = 0.0;
        nav3_B.b_V[2] = 0.0;
        nav3_B.work[2] = 0.0;
        nav3_B.b_V[3] = 0.0;
        nav3_B.work[3] = 0.0;
        nav3_B.b_V[0] = 1.0;
        nav3_B.b_V[15] = 1.0 - nav3_B.tau[2];
        nav3_B.b_j = 0;
        while (nav3_B.b_j <= 1) {
          nav3_B.b_V[14 - nav3_B.b_j] = 0.0;
          nav3_B.b_j++;
        }

        nav3_B.b_V[10] = 1.0;
        nav3_xzlarf(2, 1, 11, nav3_B.tau[1], nav3_B.b_V, 15, nav3_B.work);
        nav3_B.b_j = 11;
        while (nav3_B.b_j + 1 <= 12) {
          nav3_B.b_V[nav3_B.b_j] *= -nav3_B.tau[1];
          nav3_B.b_j++;
        }

        nav3_B.b_V[10] = 1.0 - nav3_B.tau[1];
        nav3_B.b_V[9] = 0.0;
        nav3_B.b_V[5] = 1.0;
        nav3_xzlarf(3, 2, 6, nav3_B.tau[0], nav3_B.b_V, 10, nav3_B.work);
        nav3_B.b_j = 6;
        while (nav3_B.b_j + 1 <= 8) {
          nav3_B.b_V[nav3_B.b_j] *= -nav3_B.tau[0];
          nav3_B.b_j++;
        }

        nav3_B.b_V[5] = 1.0 - nav3_B.tau[0];
        nav3_eml_dlahqr(nav3_B.b_A, nav3_B.b_V);
      }

      for (nav3_B.b_j = 0; nav3_B.b_j < 16; nav3_B.b_j++) {
        V[nav3_B.b_j].re = nav3_B.b_V[nav3_B.b_j];
        V[nav3_B.b_j].im = 0.0;
      }

      D[0].re = nav3_B.b_A[0];
      D[0].im = 0.0;
      D[1].re = nav3_B.b_A[5];
      D[1].im = 0.0;
      D[2].re = nav3_B.b_A[10];
      D[2].im = 0.0;
      D[3].re = nav3_B.b_A[15];
      D[3].im = 0.0;
    } else {
      for (nav3_B.b_j = 0; nav3_B.b_j < 16; nav3_B.b_j++) {
        nav3_B.At[nav3_B.b_j].re = A[nav3_B.b_j];
        nav3_B.At[nav3_B.b_j].im = 0.0;
      }

      nav3_xzggev(nav3_B.At, &nav3_B.b_j, D, nav3_B.beta1, V);
      nav3_B.colnorm = 0.0;
      nav3_B.scale = 3.3121686421112381E-170;
      nav3_B.b_j = 0;
      while (nav3_B.b_j + 1 <= 4) {
        nav3_B.absxk = fabs(V[nav3_B.b_j].re);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.absxk = fabs(V[nav3_B.b_j].im);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.b_j++;
      }

      nav3_B.colnorm = nav3_B.scale * sqrt(nav3_B.colnorm);
      nav3_B.b_j = 0;
      while (nav3_B.b_j + 1 <= 4) {
        if (V[nav3_B.b_j].im == 0.0) {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = 0.0;
        } else if (V[nav3_B.b_j].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        } else {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        }

        V[nav3_B.b_j].re = nav3_B.scale;
        V[nav3_B.b_j].im = nav3_B.absxk;
        nav3_B.b_j++;
      }

      nav3_B.colnorm = 0.0;
      nav3_B.scale = 3.3121686421112381E-170;
      nav3_B.b_j = 4;
      while (nav3_B.b_j + 1 <= 8) {
        nav3_B.absxk = fabs(V[nav3_B.b_j].re);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.absxk = fabs(V[nav3_B.b_j].im);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.b_j++;
      }

      nav3_B.colnorm = nav3_B.scale * sqrt(nav3_B.colnorm);
      nav3_B.b_j = 4;
      while (nav3_B.b_j + 1 <= 8) {
        if (V[nav3_B.b_j].im == 0.0) {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = 0.0;
        } else if (V[nav3_B.b_j].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        } else {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        }

        V[nav3_B.b_j].re = nav3_B.scale;
        V[nav3_B.b_j].im = nav3_B.absxk;
        nav3_B.b_j++;
      }

      nav3_B.colnorm = 0.0;
      nav3_B.scale = 3.3121686421112381E-170;
      nav3_B.b_j = 8;
      while (nav3_B.b_j + 1 <= 12) {
        nav3_B.absxk = fabs(V[nav3_B.b_j].re);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.absxk = fabs(V[nav3_B.b_j].im);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.b_j++;
      }

      nav3_B.colnorm = nav3_B.scale * sqrt(nav3_B.colnorm);
      nav3_B.b_j = 8;
      while (nav3_B.b_j + 1 <= 12) {
        if (V[nav3_B.b_j].im == 0.0) {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = 0.0;
        } else if (V[nav3_B.b_j].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        } else {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        }

        V[nav3_B.b_j].re = nav3_B.scale;
        V[nav3_B.b_j].im = nav3_B.absxk;
        nav3_B.b_j++;
      }

      nav3_B.colnorm = 0.0;
      nav3_B.scale = 3.3121686421112381E-170;
      nav3_B.b_j = 12;
      while (nav3_B.b_j + 1 <= 16) {
        nav3_B.absxk = fabs(V[nav3_B.b_j].re);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.absxk = fabs(V[nav3_B.b_j].im);
        if (nav3_B.absxk > nav3_B.scale) {
          nav3_B.t = nav3_B.scale / nav3_B.absxk;
          nav3_B.colnorm = nav3_B.colnorm * nav3_B.t * nav3_B.t + 1.0;
          nav3_B.scale = nav3_B.absxk;
        } else {
          nav3_B.t = nav3_B.absxk / nav3_B.scale;
          nav3_B.colnorm += nav3_B.t * nav3_B.t;
        }

        nav3_B.b_j++;
      }

      nav3_B.colnorm = nav3_B.scale * sqrt(nav3_B.colnorm);
      nav3_B.b_j = 12;
      while (nav3_B.b_j + 1 <= 16) {
        if (V[nav3_B.b_j].im == 0.0) {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = 0.0;
        } else if (V[nav3_B.b_j].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        } else {
          nav3_B.scale = V[nav3_B.b_j].re / nav3_B.colnorm;
          nav3_B.absxk = V[nav3_B.b_j].im / nav3_B.colnorm;
        }

        V[nav3_B.b_j].re = nav3_B.scale;
        V[nav3_B.b_j].im = nav3_B.absxk;
        nav3_B.b_j++;
      }

      if (nav3_B.beta1[0].im == 0.0) {
        if (D[0].im == 0.0) {
          nav3_B.scale = D[0].re / nav3_B.beta1[0].re;
          nav3_B.absxk = 0.0;
        } else if (D[0].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = D[0].im / nav3_B.beta1[0].re;
        } else {
          nav3_B.scale = D[0].re / nav3_B.beta1[0].re;
          nav3_B.absxk = D[0].im / nav3_B.beta1[0].re;
        }
      } else if (nav3_B.beta1[0].re == 0.0) {
        if (D[0].re == 0.0) {
          nav3_B.scale = D[0].im / nav3_B.beta1[0].im;
          nav3_B.absxk = 0.0;
        } else if (D[0].im == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = -(D[0].re / nav3_B.beta1[0].im);
        } else {
          nav3_B.scale = D[0].im / nav3_B.beta1[0].im;
          nav3_B.absxk = -(D[0].re / nav3_B.beta1[0].im);
        }
      } else {
        nav3_B.colnorm = fabs(nav3_B.beta1[0].re);
        nav3_B.scale = fabs(nav3_B.beta1[0].im);
        if (nav3_B.colnorm > nav3_B.scale) {
          nav3_B.colnorm = nav3_B.beta1[0].im / nav3_B.beta1[0].re;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[0].im + nav3_B.beta1[0].
            re;
          nav3_B.scale = (nav3_B.colnorm * D[0].im + D[0].re) / nav3_B.absxk;
          nav3_B.absxk = (D[0].im - nav3_B.colnorm * D[0].re) / nav3_B.absxk;
        } else if (nav3_B.scale == nav3_B.colnorm) {
          nav3_B.absxk = nav3_B.beta1[0].re > 0.0 ? 0.5 : -0.5;
          nav3_B.t = nav3_B.beta1[0].im > 0.0 ? 0.5 : -0.5;
          nav3_B.scale = (D[0].re * nav3_B.absxk + D[0].im * nav3_B.t) /
            nav3_B.colnorm;
          nav3_B.absxk = (D[0].im * nav3_B.absxk - D[0].re * nav3_B.t) /
            nav3_B.colnorm;
        } else {
          nav3_B.colnorm = nav3_B.beta1[0].re / nav3_B.beta1[0].im;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[0].re + nav3_B.beta1[0].
            im;
          nav3_B.scale = (nav3_B.colnorm * D[0].re + D[0].im) / nav3_B.absxk;
          nav3_B.absxk = (nav3_B.colnorm * D[0].im - D[0].re) / nav3_B.absxk;
        }
      }

      D[0].re = nav3_B.scale;
      D[0].im = nav3_B.absxk;
      if (nav3_B.beta1[1].im == 0.0) {
        if (D[1].im == 0.0) {
          nav3_B.scale = D[1].re / nav3_B.beta1[1].re;
          nav3_B.absxk = 0.0;
        } else if (D[1].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = D[1].im / nav3_B.beta1[1].re;
        } else {
          nav3_B.scale = D[1].re / nav3_B.beta1[1].re;
          nav3_B.absxk = D[1].im / nav3_B.beta1[1].re;
        }
      } else if (nav3_B.beta1[1].re == 0.0) {
        if (D[1].re == 0.0) {
          nav3_B.scale = D[1].im / nav3_B.beta1[1].im;
          nav3_B.absxk = 0.0;
        } else if (D[1].im == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = -(D[1].re / nav3_B.beta1[1].im);
        } else {
          nav3_B.scale = D[1].im / nav3_B.beta1[1].im;
          nav3_B.absxk = -(D[1].re / nav3_B.beta1[1].im);
        }
      } else {
        nav3_B.colnorm = fabs(nav3_B.beta1[1].re);
        nav3_B.scale = fabs(nav3_B.beta1[1].im);
        if (nav3_B.colnorm > nav3_B.scale) {
          nav3_B.colnorm = nav3_B.beta1[1].im / nav3_B.beta1[1].re;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[1].im + nav3_B.beta1[1].
            re;
          nav3_B.scale = (nav3_B.colnorm * D[1].im + D[1].re) / nav3_B.absxk;
          nav3_B.absxk = (D[1].im - nav3_B.colnorm * D[1].re) / nav3_B.absxk;
        } else if (nav3_B.scale == nav3_B.colnorm) {
          nav3_B.absxk = nav3_B.beta1[1].re > 0.0 ? 0.5 : -0.5;
          nav3_B.t = nav3_B.beta1[1].im > 0.0 ? 0.5 : -0.5;
          nav3_B.scale = (D[1].re * nav3_B.absxk + D[1].im * nav3_B.t) /
            nav3_B.colnorm;
          nav3_B.absxk = (D[1].im * nav3_B.absxk - D[1].re * nav3_B.t) /
            nav3_B.colnorm;
        } else {
          nav3_B.colnorm = nav3_B.beta1[1].re / nav3_B.beta1[1].im;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[1].re + nav3_B.beta1[1].
            im;
          nav3_B.scale = (nav3_B.colnorm * D[1].re + D[1].im) / nav3_B.absxk;
          nav3_B.absxk = (nav3_B.colnorm * D[1].im - D[1].re) / nav3_B.absxk;
        }
      }

      D[1].re = nav3_B.scale;
      D[1].im = nav3_B.absxk;
      if (nav3_B.beta1[2].im == 0.0) {
        if (D[2].im == 0.0) {
          nav3_B.scale = D[2].re / nav3_B.beta1[2].re;
          nav3_B.absxk = 0.0;
        } else if (D[2].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = D[2].im / nav3_B.beta1[2].re;
        } else {
          nav3_B.scale = D[2].re / nav3_B.beta1[2].re;
          nav3_B.absxk = D[2].im / nav3_B.beta1[2].re;
        }
      } else if (nav3_B.beta1[2].re == 0.0) {
        if (D[2].re == 0.0) {
          nav3_B.scale = D[2].im / nav3_B.beta1[2].im;
          nav3_B.absxk = 0.0;
        } else if (D[2].im == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = -(D[2].re / nav3_B.beta1[2].im);
        } else {
          nav3_B.scale = D[2].im / nav3_B.beta1[2].im;
          nav3_B.absxk = -(D[2].re / nav3_B.beta1[2].im);
        }
      } else {
        nav3_B.colnorm = fabs(nav3_B.beta1[2].re);
        nav3_B.scale = fabs(nav3_B.beta1[2].im);
        if (nav3_B.colnorm > nav3_B.scale) {
          nav3_B.colnorm = nav3_B.beta1[2].im / nav3_B.beta1[2].re;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[2].im + nav3_B.beta1[2].
            re;
          nav3_B.scale = (nav3_B.colnorm * D[2].im + D[2].re) / nav3_B.absxk;
          nav3_B.absxk = (D[2].im - nav3_B.colnorm * D[2].re) / nav3_B.absxk;
        } else if (nav3_B.scale == nav3_B.colnorm) {
          nav3_B.absxk = nav3_B.beta1[2].re > 0.0 ? 0.5 : -0.5;
          nav3_B.t = nav3_B.beta1[2].im > 0.0 ? 0.5 : -0.5;
          nav3_B.scale = (D[2].re * nav3_B.absxk + D[2].im * nav3_B.t) /
            nav3_B.colnorm;
          nav3_B.absxk = (D[2].im * nav3_B.absxk - D[2].re * nav3_B.t) /
            nav3_B.colnorm;
        } else {
          nav3_B.colnorm = nav3_B.beta1[2].re / nav3_B.beta1[2].im;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[2].re + nav3_B.beta1[2].
            im;
          nav3_B.scale = (nav3_B.colnorm * D[2].re + D[2].im) / nav3_B.absxk;
          nav3_B.absxk = (nav3_B.colnorm * D[2].im - D[2].re) / nav3_B.absxk;
        }
      }

      D[2].re = nav3_B.scale;
      D[2].im = nav3_B.absxk;
      if (nav3_B.beta1[3].im == 0.0) {
        if (D[3].im == 0.0) {
          nav3_B.scale = D[3].re / nav3_B.beta1[3].re;
          nav3_B.absxk = 0.0;
        } else if (D[3].re == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = D[3].im / nav3_B.beta1[3].re;
        } else {
          nav3_B.scale = D[3].re / nav3_B.beta1[3].re;
          nav3_B.absxk = D[3].im / nav3_B.beta1[3].re;
        }
      } else if (nav3_B.beta1[3].re == 0.0) {
        if (D[3].re == 0.0) {
          nav3_B.scale = D[3].im / nav3_B.beta1[3].im;
          nav3_B.absxk = 0.0;
        } else if (D[3].im == 0.0) {
          nav3_B.scale = 0.0;
          nav3_B.absxk = -(D[3].re / nav3_B.beta1[3].im);
        } else {
          nav3_B.scale = D[3].im / nav3_B.beta1[3].im;
          nav3_B.absxk = -(D[3].re / nav3_B.beta1[3].im);
        }
      } else {
        nav3_B.colnorm = fabs(nav3_B.beta1[3].re);
        nav3_B.scale = fabs(nav3_B.beta1[3].im);
        if (nav3_B.colnorm > nav3_B.scale) {
          nav3_B.colnorm = nav3_B.beta1[3].im / nav3_B.beta1[3].re;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[3].im + nav3_B.beta1[3].
            re;
          nav3_B.scale = (nav3_B.colnorm * D[3].im + D[3].re) / nav3_B.absxk;
          nav3_B.absxk = (D[3].im - nav3_B.colnorm * D[3].re) / nav3_B.absxk;
        } else if (nav3_B.scale == nav3_B.colnorm) {
          nav3_B.absxk = nav3_B.beta1[3].re > 0.0 ? 0.5 : -0.5;
          nav3_B.t = nav3_B.beta1[3].im > 0.0 ? 0.5 : -0.5;
          nav3_B.scale = (D[3].re * nav3_B.absxk + D[3].im * nav3_B.t) /
            nav3_B.colnorm;
          nav3_B.absxk = (D[3].im * nav3_B.absxk - D[3].re * nav3_B.t) /
            nav3_B.colnorm;
        } else {
          nav3_B.colnorm = nav3_B.beta1[3].re / nav3_B.beta1[3].im;
          nav3_B.absxk = nav3_B.colnorm * nav3_B.beta1[3].re + nav3_B.beta1[3].
            im;
          nav3_B.scale = (nav3_B.colnorm * D[3].re + D[3].im) / nav3_B.absxk;
          nav3_B.absxk = (nav3_B.colnorm * D[3].im - D[3].re) / nav3_B.absxk;
        }
      }

      D[3].re = nav3_B.scale;
      D[3].im = nav3_B.absxk;
    }
  }
}

static real_T nav3_rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      nav3_B.u0 = 1;
    } else {
      nav3_B.u0 = -1;
    }

    if (u1 > 0.0) {
      nav3_B.u1 = 1;
    } else {
      nav3_B.u1 = -1;
    }

    y = atan2(static_cast<real_T>(nav3_B.u0), static_cast<real_T>(nav3_B.u1));
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
static real_T nav3_CalculateHeading(const real_T startPose[3], const real_T
  endPose[2])
{
  real_T heading;
  heading = nav3_rt_atan2d_snf(endPose[1] - startPose[1], endPose[0] -
    startPose[0]);
  return heading;
}

/* Function for Chart: '<S2>/State_Machine' */
static real_T nav3_norm(const real_T x[2])
{
  real_T y;
  nav3_B.scale_g = 3.3121686421112381E-170;
  nav3_B.absxk_l = fabs(x[0]);
  if (nav3_B.absxk_l > 3.3121686421112381E-170) {
    y = 1.0;
    nav3_B.scale_g = nav3_B.absxk_l;
  } else {
    nav3_B.t_d = nav3_B.absxk_l / 3.3121686421112381E-170;
    y = nav3_B.t_d * nav3_B.t_d;
  }

  nav3_B.absxk_l = fabs(x[1]);
  if (nav3_B.absxk_l > nav3_B.scale_g) {
    nav3_B.t_d = nav3_B.scale_g / nav3_B.absxk_l;
    y = y * nav3_B.t_d * nav3_B.t_d + 1.0;
    nav3_B.scale_g = nav3_B.absxk_l;
  } else {
    nav3_B.t_d = nav3_B.absxk_l / nav3_B.scale_g;
    y += nav3_B.t_d * nav3_B.t_d;
  }

  return nav3_B.scale_g * sqrt(y);
}

/* Function for Chart: '<S2>/State_Machine' */
static real_T nav3_closestPointOnLine(const real_T pt1[2], real_T pt2[2], const
  real_T refPt[2])
{
  real_T distance;
  boolean_T exitg1;
  nav3_B.p_o = false;
  nav3_B.b_p = true;
  nav3_B.k_p = 0;
  exitg1 = false;
  while ((!exitg1) && (nav3_B.k_p < 2)) {
    if (!(pt1[nav3_B.k_p] == pt2[nav3_B.k_p])) {
      nav3_B.b_p = false;
      exitg1 = true;
    } else {
      nav3_B.k_p++;
    }
  }

  if (nav3_B.b_p) {
    nav3_B.p_o = true;
  }

  if (nav3_B.p_o) {
    pt2[0] = pt1[0];
    nav3_B.refPt[0] = refPt[0] - pt1[0];
    pt2[1] = pt1[1];
    nav3_B.refPt[1] = refPt[1] - pt1[1];
    distance = nav3_norm(nav3_B.refPt);
  } else {
    nav3_B.alpha_j = pt2[0] - pt1[0];
    nav3_B.v12 = (pt2[0] - refPt[0]) * nav3_B.alpha_j;
    nav3_B.v12_d = nav3_B.alpha_j * nav3_B.alpha_j;
    nav3_B.alpha_j = pt2[1] - pt1[1];
    nav3_B.v12 += (pt2[1] - refPt[1]) * nav3_B.alpha_j;
    nav3_B.v12_d += nav3_B.alpha_j * nav3_B.alpha_j;
    nav3_B.alpha_j = nav3_B.v12 / nav3_B.v12_d;
    if (nav3_B.alpha_j > 1.0) {
      pt2[0] = pt1[0];
      pt2[1] = pt1[1];
    } else {
      if (!(nav3_B.alpha_j < 0.0)) {
        pt2[0] = (1.0 - nav3_B.alpha_j) * pt2[0] + nav3_B.alpha_j * pt1[0];
        pt2[1] = (1.0 - nav3_B.alpha_j) * pt2[1] + nav3_B.alpha_j * pt1[1];
      }
    }

    nav3_B.refPt[0] = refPt[0] - pt2[0];
    nav3_B.refPt[1] = refPt[1] - pt2[1];
    distance = nav3_norm(nav3_B.refPt);
  }

  return distance;
}

/* Function for Chart: '<S2>/State_Machine' */
static void nav3_purePursuit_b(real_T lookAhead, const real_T waypoints[6],
  const real_T pose[3], real_T targetHeading[3])
{
  for (nav3_B.lookaheadIdx = 0; nav3_B.lookaheadIdx < 6; nav3_B.lookaheadIdx++)
  {
    nav3_B.b_n[nav3_B.lookaheadIdx] = !rtIsNaN(waypoints[nav3_B.lookaheadIdx]);
  }

  nav3_B.trueCount = 0;
  nav3_B.lookaheadIdx = 0;
  if (nav3_B.b_n[0] && nav3_B.b_n[3]) {
    nav3_B.trueCount = 1;
    nav3_B.c_data[0] = 1;
    nav3_B.lookaheadIdx = 1;
  }

  if (nav3_B.b_n[1] && nav3_B.b_n[4]) {
    nav3_B.trueCount++;
    nav3_B.c_data[nav3_B.lookaheadIdx] = 2;
    nav3_B.lookaheadIdx++;
  }

  if (nav3_B.b_n[2] && nav3_B.b_n[5]) {
    nav3_B.trueCount++;
    nav3_B.c_data[nav3_B.lookaheadIdx] = 3;
  }

  for (nav3_B.lookaheadIdx = 0; nav3_B.lookaheadIdx < nav3_B.trueCount;
       nav3_B.lookaheadIdx++) {
    nav3_B.b_waypoints_data[nav3_B.lookaheadIdx] =
      waypoints[nav3_B.c_data[nav3_B.lookaheadIdx] - 1];
    nav3_B.b_waypoints_data[nav3_B.lookaheadIdx + nav3_B.trueCount] =
      waypoints[nav3_B.c_data[nav3_B.lookaheadIdx] + 2];
  }

  if (nav3_B.trueCount == 0) {
    nav3_B.lookaheadEndPt[0] = pose[0];
    nav3_B.lookaheadEndPt[1] = pose[1];
  } else {
    nav3_B.lookaheadIdx = 1;
    if (nav3_B.trueCount == 1) {
      nav3_B.lookaheadEndPt[0] = nav3_B.b_waypoints_data[0];
      nav3_B.lookaheadEndPt[1] = nav3_B.b_waypoints_data[nav3_B.trueCount];
    } else {
      nav3_B.lookaheadEndPt[0] = nav3_B.b_waypoints_data[1];
      nav3_B.b_waypoints[0] = nav3_B.b_waypoints_data[0];
      nav3_B.alpha = nav3_B.b_waypoints_data[nav3_B.trueCount + 1];
      nav3_B.lookaheadEndPt[1] = nav3_B.alpha;
      nav3_B.b_waypoints[1] = nav3_B.b_waypoints_data[nav3_B.trueCount];
      nav3_B.minDistance = nav3_closestPointOnLine(nav3_B.b_waypoints,
        nav3_B.lookaheadEndPt, &pose[0]);
      if (0 <= nav3_B.trueCount - 3) {
        nav3_B.b_waypoints_f[0] = nav3_B.b_waypoints_data[1];
        nav3_B.b_waypoints_f[1] = nav3_B.alpha;
      }

      nav3_B.c_i_c = 0;
      while (nav3_B.c_i_c <= nav3_B.trueCount - 3) {
        nav3_B.lookaheadStartPt[0] = nav3_B.b_waypoints_data[2];
        nav3_B.lookaheadStartPt[1] = nav3_B.b_waypoints_data[nav3_B.trueCount +
          2];
        nav3_B.overshootDist = nav3_closestPointOnLine(nav3_B.b_waypoints_f,
          nav3_B.lookaheadStartPt, &pose[0]);
        if (nav3_B.overshootDist < nav3_B.minDistance) {
          nav3_B.minDistance = nav3_B.overshootDist;
          nav3_B.lookaheadEndPt[0] = nav3_B.lookaheadStartPt[0];
          nav3_B.lookaheadEndPt[1] = nav3_B.lookaheadStartPt[1];
          nav3_B.lookaheadIdx = 2;
        }

        nav3_B.c_i_c = 1;
      }

      nav3_B.lookaheadStartPt[0] = nav3_B.lookaheadEndPt[0] -
        nav3_B.b_waypoints_data[nav3_B.lookaheadIdx];
      nav3_B.c_i_c = nav3_B.lookaheadIdx + nav3_B.trueCount;
      nav3_B.lookaheadStartPt[1] = nav3_B.lookaheadEndPt[1] -
        nav3_B.b_waypoints_data[nav3_B.c_i_c];
      nav3_B.minDistance = nav3_norm(nav3_B.lookaheadStartPt);
      nav3_B.lookaheadStartPt[0] = nav3_B.lookaheadEndPt[0];
      nav3_B.lookaheadEndPt[0] = nav3_B.b_waypoints_data[nav3_B.lookaheadIdx];
      nav3_B.lookaheadStartPt[1] = nav3_B.lookaheadEndPt[1];
      nav3_B.lookaheadEndPt[1] = nav3_B.b_waypoints_data[nav3_B.c_i_c];
      nav3_B.overshootDist = nav3_B.minDistance - lookAhead;
      while ((nav3_B.overshootDist < 0.0) && (nav3_B.lookaheadIdx <
              nav3_B.trueCount - 1)) {
        nav3_B.lookaheadIdx = 2;
        nav3_B.lookaheadStartPt[0] = nav3_B.b_waypoints_data[1];
        nav3_B.lookaheadEndPt[0] = nav3_B.b_waypoints_data[2];
        nav3_B.b_waypoints[0] = nav3_B.b_waypoints_data[1] -
          nav3_B.b_waypoints_data[2];
        nav3_B.lookaheadStartPt[1] = nav3_B.alpha;
        nav3_B.overshootDist = nav3_B.b_waypoints_data[nav3_B.trueCount + 2];
        nav3_B.lookaheadEndPt[1] = nav3_B.overshootDist;
        nav3_B.b_waypoints[1] = nav3_B.alpha - nav3_B.overshootDist;
        nav3_B.minDistance += nav3_norm(nav3_B.b_waypoints);
        nav3_B.overshootDist = nav3_B.minDistance - lookAhead;
      }

      nav3_B.b_waypoints[0] = nav3_B.lookaheadStartPt[0] -
        nav3_B.lookaheadEndPt[0];
      nav3_B.b_waypoints[1] = nav3_B.lookaheadStartPt[1] -
        nav3_B.lookaheadEndPt[1];
      nav3_B.alpha = nav3_B.overshootDist / nav3_norm(nav3_B.b_waypoints);
      if (nav3_B.alpha > 0.0) {
        nav3_B.lookaheadEndPt[0] = (1.0 - nav3_B.alpha) * nav3_B.lookaheadEndPt
          [0] + nav3_B.alpha * nav3_B.lookaheadStartPt[0];
        nav3_B.lookaheadEndPt[1] = (1.0 - nav3_B.alpha) * nav3_B.lookaheadEndPt
          [1] + nav3_B.alpha * nav3_B.lookaheadStartPt[1];
      }
    }
  }

  targetHeading[0] = nav3_rt_atan2d_snf(nav3_B.lookaheadEndPt[1] - pose[1],
    nav3_B.lookaheadEndPt[0] - pose[0]);
  targetHeading[1] = nav3_B.lookaheadEndPt[0];
  targetHeading[2] = nav3_B.lookaheadEndPt[1];
}

/* Function for Chart: '<S2>/State_Machine' */
static real_T nav3_mod(real_T x)
{
  real_T r;
  if (rtIsNaN(x) || rtIsInf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    r = fmod(x, 2.0);
    if (r == 0.0) {
      r = 0.0;
    } else {
      if (x < 0.0) {
        r += 2.0;
      }
    }
  }

  return r;
}

/* Function for Chart: '<S2>/State_Machine' */
static void nav3_flip(real_T x[18])
{
  for (nav3_B.j_o = 0; nav3_B.j_o < 2; nav3_B.j_o++) {
    nav3_B.offset = nav3_B.j_o * 9;
    nav3_B.tmp = x[nav3_B.offset];
    x[nav3_B.offset] = x[nav3_B.offset + 8];
    x[nav3_B.offset + 8] = nav3_B.tmp;
    nav3_B.tmp = x[nav3_B.offset + 1];
    x[nav3_B.offset + 1] = x[nav3_B.offset + 7];
    x[nav3_B.offset + 7] = nav3_B.tmp;
    nav3_B.tmp = x[nav3_B.offset + 2];
    x[nav3_B.offset + 2] = x[nav3_B.offset + 6];
    x[nav3_B.offset + 6] = nav3_B.tmp;
    nav3_B.tmp = x[nav3_B.offset + 3];
    x[nav3_B.offset + 3] = x[nav3_B.offset + 5];
    x[nav3_B.offset + 5] = nav3_B.tmp;
  }
}

/* Function for Chart: '<S2>/State_Machine' */
static void nav3_AUTONOMOUS_CONTROL(void)
{
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  if (nav3_B.In1_i.Buttons[1] == 1) {
    switch (nav3_DW.is_AUTONOMOUS_CONTROL) {
     case nav3_IN_PreciseRotateToWaypoint:
      nav3_B.angularVel = 0.0;
      nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_NO_ACTIVE_CHILD;
      break;

     case nav3_IN_RotateToWaypoint_i:
      nav3_B.angularVel = 0.0;
      nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_NO_ACTIVE_CHILD;
      break;

     default:
      nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_NO_ACTIVE_CHILD;
      break;
    }

    nav3_DW.is_c3_nav3 = nav3_IN_EMERGENCY;
    nav3_B.currentState = nav3_DW.EMERGENCY_STATE;
  } else if ((nav3_B.In1_i.Buttons[5] == 1) || (nav3_DW.finishedMission == 1.0))
  {
    switch (nav3_DW.is_AUTONOMOUS_CONTROL) {
     case nav3_IN_PreciseRotateToWaypoint:
      nav3_B.angularVel = 0.0;
      nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_NO_ACTIVE_CHILD;
      break;

     case nav3_IN_RotateToWaypoint_i:
      nav3_B.angularVel = 0.0;
      nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_NO_ACTIVE_CHILD;
      break;

     default:
      nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_NO_ACTIVE_CHILD;
      break;
    }

    nav3_DW.is_c3_nav3 = nav3_IN_WAITING;
    nav3_B.currentState = nav3_DW.WAITING_STATE;
  } else {
    guard1 = false;
    guard2 = false;
    guard3 = false;
    switch (nav3_DW.is_AUTONOMOUS_CONTROL) {
     case nav3_IN_PreciseMoveToWaypoint_f:
      nav3_B.pose[0] = nav3_B.Switch[0];
      nav3_B.pose[1] = nav3_B.Switch[1];
      nav3_B.pose[2] = nav3_B.Switch[2];
      nav3_B.i_c = static_cast<int32_T>(nav3_DW.count + 1.0);
      nav3_B.goalWaypoint[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
      nav3_B.goalWaypoint[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
      nav3_B.goalTolerance = nav3_DW.WAYPOINT_PRECISION;

      /* Outputs for Function Call SubSystem: '<S16>/checkAtGoal' */
      nav3_checkAtGoal(nav3_B.pose, nav3_B.goalWaypoint, nav3_B.goalTolerance,
                       &nav3_B.checkAtGoal);

      /* End of Outputs for SubSystem: '<S16>/checkAtGoal' */
      if (!nav3_B.checkAtGoal.LessThan) {
        nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_ReachedWaypoint_b;
        if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483643) {
          nav3_B.currentState = MAX_int32_T;
        } else {
          nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 4;
        }
      } else {
        nav3_B.i_c = static_cast<int32_T>(nav3_DW.count + 1.0);
        nav3_B.dv1[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
        nav3_B.dv1[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
        nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
          nav3_B.dv1);
        nav3_B.pose_f[0] = nav3_B.Switch[0];
        nav3_B.pose_f[1] = nav3_B.Switch[1];
        nav3_B.pose_f[2] = nav3_B.Switch[2];
        nav3_B.desiredHeading = nav3_DW.currentGoalHeading;
        nav3_B.P = nav3_DW.P;
        nav3_B.I = nav3_DW.I;
        nav3_B.D = nav3_DW.D;

        /* Outputs for Function Call SubSystem: '<S16>/PID' */
        nav3_PID(nav3_M, nav3_B.pose_f, nav3_B.desiredHeading, nav3_B.P,
                 nav3_B.I, nav3_B.D, &nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

        /* End of Outputs for SubSystem: '<S16>/PID' */
        nav3_B.angularVel = nav3_B.PID.Saturation;
      }
      break;

     case nav3_IN_PreciseRotateToWaypoint:
      if ((nav3_DW.currentGoalHeading - 0.01 < nav3_B.Switch[2]) &&
          (nav3_B.Switch[2] < nav3_DW.currentGoalHeading + 0.01)) {
        nav3_B.angularVel = 0.0;
        nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_PreciseMoveToWaypoint_f;
        if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483644) {
          nav3_B.currentState = MAX_int32_T;
        } else {
          nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 3;
        }

        nav3_B.linearVel = nav3_DW.MAX_LINEAR_VEL / 2.0;
      } else {
        nav3_B.pose_f[0] = nav3_B.Switch[0];
        nav3_B.pose_f[1] = nav3_B.Switch[1];
        nav3_B.pose_f[2] = nav3_B.Switch[2];
        nav3_B.desiredHeading = nav3_DW.currentGoalHeading;
        nav3_B.P = nav3_DW.P;
        nav3_B.I = nav3_DW.I;
        nav3_B.D = nav3_DW.D;

        /* Outputs for Function Call SubSystem: '<S16>/PID' */
        nav3_PID(nav3_M, nav3_B.pose_f, nav3_B.desiredHeading, nav3_B.P,
                 nav3_B.I, nav3_B.D, &nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

        /* End of Outputs for SubSystem: '<S16>/PID' */
        nav3_B.angularVel = nav3_B.PID.Saturation;
      }
      break;

     case na_IN_PurePursuitMoveToWaypoint:
      nav3_B.pose[0] = nav3_B.Switch[0];
      nav3_B.pose[1] = nav3_B.Switch[1];
      nav3_B.pose[2] = nav3_B.Switch[2];
      nav3_B.i_c = static_cast<int32_T>(nav3_DW.count + 1.0);
      nav3_B.goalWaypoint[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
      nav3_B.goalWaypoint[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
      nav3_B.goalTolerance = nav3_DW.LOOKAHEAD_DISTANCE;

      /* Outputs for Function Call SubSystem: '<S16>/checkAtGoal' */
      nav3_checkAtGoal(nav3_B.pose, nav3_B.goalWaypoint, nav3_B.goalTolerance,
                       &nav3_B.checkAtGoal);

      /* End of Outputs for SubSystem: '<S16>/checkAtGoal' */
      if (!nav3_B.checkAtGoal.LessThan) {
        nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_ReachedWaypoint_b;
        if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483643) {
          nav3_B.currentState = MAX_int32_T;
        } else {
          nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 4;
        }
      } else {
        nav3_purePursuit_b(nav3_DW.LOOKAHEAD_DISTANCE, nav3_DW.waypointLine,
                           nav3_B.Switch, nav3_DW.goalHeading);
        nav3_B.linearVel = nav3_DW.MAX_LINEAR_VEL;
        nav3_B.pose_f[0] = nav3_B.Switch[0];
        nav3_B.pose_f[1] = nav3_B.Switch[1];
        nav3_B.pose_f[2] = nav3_B.Switch[2];
        nav3_B.desiredHeading = nav3_DW.goalHeading[0];
        nav3_B.P = nav3_DW.P;
        nav3_B.I = nav3_DW.I;
        nav3_B.D = nav3_DW.D;

        /* Outputs for Function Call SubSystem: '<S16>/PID' */
        nav3_PID(nav3_M, nav3_B.pose_f, nav3_B.desiredHeading, nav3_B.P,
                 nav3_B.I, nav3_B.D, &nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

        /* End of Outputs for SubSystem: '<S16>/PID' */
        nav3_B.angularVel = nav3_B.PID.Saturation;
        nav3_B.i_c = static_cast<int32_T>(nav3_DW.count + 1.0);
        nav3_B.currentWaypoint[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
        nav3_B.currentWaypoint[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
      }
      break;

     case nav3_IN_ReachedWaypoint_b:
      if (nav3_DW.count + 1.0 < nav3_DW.last) {
        nav3_DW.count++;
        if (nav3_DW.count + 2.0 > nav3_DW.last) {
          nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_PreciseRotateToWaypoint;
          if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483646) {
            nav3_B.currentState = MAX_int32_T;
          } else {
            nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 1;
          }

          nav3_B.linearVel = 0.0;
          nav3_B.i_c = static_cast<int32_T>(nav3_DW.count + 1.0);
          nav3_DW.waypointLine[0] = nav3_B.Switch[0];
          nav3_B.waypointLine_tmp = static_cast<int32_T>(nav3_DW.count);
          nav3_DW.waypointLine[1] =
            nav3_DW.all_waypoints[nav3_B.waypointLine_tmp - 1];
          nav3_DW.waypointLine[2] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
          nav3_B.dv1[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
          nav3_B.currentWaypoint[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
          nav3_DW.waypointLine[3] = nav3_B.Switch[1];
          nav3_DW.waypointLine[4] =
            nav3_DW.all_waypoints[nav3_B.waypointLine_tmp + 9];
          nav3_DW.waypointLine[5] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
          nav3_B.dv1[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
          nav3_B.currentWaypoint[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
          nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
            nav3_B.dv1);
        } else if (nav3_DW.count + 2.0 <= nav3_DW.last) {
          for (nav3_B.i_c = 0; nav3_B.i_c < 2; nav3_B.i_c++) {
            nav3_DW.waypointLine[3 * nav3_B.i_c] = nav3_DW.all_waypoints[(10 *
              nav3_B.i_c + static_cast<int32_T>(nav3_DW.count)) - 1];
            nav3_DW.waypointLine[3 * nav3_B.i_c + 1] = nav3_DW.all_waypoints[(
              static_cast<int32_T>(nav3_DW.count + 1.0) + 10 * nav3_B.i_c) - 1];
            nav3_DW.waypointLine[3 * nav3_B.i_c + 2] = nav3_DW.all_waypoints[(
              static_cast<int32_T>(nav3_DW.count + 2.0) + 10 * nav3_B.i_c) - 1];
          }

          nav3_DW.is_AUTONOMOUS_CONTROL = na_IN_PurePursuitMoveToWaypoint;
          if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483645) {
            nav3_B.currentState = MAX_int32_T;
          } else {
            nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 2;
          }
        } else {
          guard3 = true;
        }
      } else {
        guard3 = true;
      }
      break;

     default:
      /* case IN_RotateToWaypoint: */
      if ((nav3_DW.currentGoalHeading - 0.01 < nav3_B.Switch[2]) &&
          (nav3_B.Switch[2] < nav3_DW.currentGoalHeading + 0.01)) {
        if (nav3_DW.count + 2.0 > nav3_DW.last) {
          nav3_B.angularVel = 0.0;
          nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_PreciseRotateToWaypoint;
          if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483646) {
            nav3_B.currentState = MAX_int32_T;
          } else {
            nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 1;
          }

          nav3_B.linearVel = 0.0;
          nav3_B.i_c = static_cast<int32_T>(nav3_DW.count + 1.0);
          nav3_DW.waypointLine[0] = nav3_B.Switch[0];
          nav3_DW.waypointLine[1] = nav3_DW.all_waypoints[static_cast<int32_T>
            (nav3_DW.count) - 1];
          nav3_DW.waypointLine[2] = nav3_DW.all_waypoints[static_cast<int32_T>
            (nav3_DW.count + 1.0) - 1];
          nav3_B.dv1[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
          nav3_B.currentWaypoint[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
          nav3_DW.waypointLine[3] = nav3_B.Switch[1];
          nav3_DW.waypointLine[4] = nav3_DW.all_waypoints[static_cast<int32_T>
            (nav3_DW.count) + 9];
          nav3_DW.waypointLine[5] = nav3_DW.all_waypoints[static_cast<int32_T>
            (nav3_DW.count + 1.0) + 9];
          nav3_B.dv1[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
          nav3_B.currentWaypoint[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
          nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
            nav3_B.dv1);
        } else if (nav3_DW.count + 2.0 <= nav3_DW.last) {
          for (nav3_B.i_c = 0; nav3_B.i_c < 2; nav3_B.i_c++) {
            nav3_DW.waypointLine[3 * nav3_B.i_c] = nav3_DW.all_waypoints[(10 *
              nav3_B.i_c + static_cast<int32_T>(nav3_DW.count)) - 1];
            nav3_DW.waypointLine[3 * nav3_B.i_c + 1] = nav3_DW.all_waypoints[(
              static_cast<int32_T>(nav3_DW.count + 1.0) + 10 * nav3_B.i_c) - 1];
            nav3_DW.waypointLine[3 * nav3_B.i_c + 2] = nav3_DW.all_waypoints[(
              static_cast<int32_T>(nav3_DW.count + 2.0) + 10 * nav3_B.i_c) - 1];
          }

          nav3_B.angularVel = 0.0;
          nav3_DW.is_AUTONOMOUS_CONTROL = na_IN_PurePursuitMoveToWaypoint;
          if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483645) {
            nav3_B.currentState = MAX_int32_T;
          } else {
            nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 2;
          }
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      break;
    }

    if (guard3) {
      if (nav3_DW.count + 1.0 == nav3_DW.last) {
        nav3_DW.finishedMissionCount++;
        nav3_DW.count = 1.0;
        nav3_B.d2 = nav3_mod(nav3_DW.finishedMissionCount);
        if (nav3_B.d2 == 1.0) {
          /* Constant: '<S2>/waypoints' */
          memcpy(&nav3_B.b_c[0], &nav3_P.waypoints_Value[0], 18U * sizeof(real_T));
          nav3_flip(nav3_B.b_c);
          for (nav3_B.i_c = 0; nav3_B.i_c < 2; nav3_B.i_c++) {
            memcpy(&nav3_DW.all_waypoints[nav3_B.i_c * 10],
                   &nav3_B.b_c[nav3_B.i_c * 9], 9U * sizeof(real_T));
            nav3_DW.all_waypoints[10 * nav3_B.i_c + 9] =
              nav3_B.Switch[nav3_B.i_c];
          }

          guard2 = true;
        } else {
          if (nav3_B.d2 == 0.0) {
            for (nav3_B.i_c = 0; nav3_B.i_c < 2; nav3_B.i_c++) {
              nav3_DW.all_waypoints[10 * nav3_B.i_c] = nav3_B.Switch[nav3_B.i_c];

              /* Constant: '<S2>/waypoints' */
              memcpy(&nav3_DW.all_waypoints[nav3_B.i_c * 10 + 1],
                     &nav3_P.waypoints_Value[nav3_B.i_c * 9], 9U * sizeof(real_T));
            }

            guard2 = true;
          }
        }
      }
    }

    if (guard2) {
      nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_RotateToWaypoint_i;
      if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483646) {
        nav3_B.currentState = MAX_int32_T;
      } else {
        nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 1;
      }

      nav3_B.linearVel = 0.0;
      nav3_B.i_c = static_cast<int32_T>(nav3_DW.count + 1.0);
      nav3_B.dv1[0] = nav3_DW.all_waypoints[nav3_B.i_c - 1];
      nav3_B.dv1[1] = nav3_DW.all_waypoints[nav3_B.i_c + 9];
      nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
        nav3_B.dv1);
      nav3_DW.goalHeading[0] = nav3_DW.currentGoalHeading;
      nav3_DW.goalHeading[1] = 0.0;
      nav3_DW.goalHeading[2] = 0.0;
    }

    if (guard1) {
      nav3_B.pose_f[0] = nav3_B.Switch[0];
      nav3_B.pose_f[1] = nav3_B.Switch[1];
      nav3_B.pose_f[2] = nav3_B.Switch[2];
      nav3_B.desiredHeading = nav3_DW.goalHeading[0];
      nav3_B.P = nav3_DW.P;
      nav3_B.I = nav3_DW.I;
      nav3_B.D = nav3_DW.D;

      /* Outputs for Function Call SubSystem: '<S16>/PID' */
      nav3_PID(nav3_M, nav3_B.pose_f, nav3_B.desiredHeading, nav3_B.P, nav3_B.I,
               nav3_B.D, &nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

      /* End of Outputs for SubSystem: '<S16>/PID' */
      nav3_B.angularVel = nav3_B.PID.Saturation;
    }
  }
}

/* Function for Chart: '<S2>/State_Machine' */
static void nav3_purePursuit(real_T lookAhead, const real_T waypoints[4], const
  real_T pose[3], real_T targetHeading[3])
{
  nav3_B.trueCount_c = 0;
  nav3_B.partialTrueCount = 0;
  if ((!rtIsNaN(waypoints[0])) && (!rtIsNaN(waypoints[2]))) {
    nav3_B.trueCount_c = 1;
    nav3_B.c_data_p[0] = 1;
    nav3_B.partialTrueCount = 1;
  }

  if ((!rtIsNaN(waypoints[1])) && (!rtIsNaN(waypoints[3]))) {
    nav3_B.trueCount_c++;
    nav3_B.c_data_p[nav3_B.partialTrueCount] = 2;
  }

  for (nav3_B.partialTrueCount = 0; nav3_B.partialTrueCount < nav3_B.trueCount_c;
       nav3_B.partialTrueCount++) {
    nav3_B.b_waypoints_data_c[nav3_B.partialTrueCount] =
      waypoints[nav3_B.c_data_p[nav3_B.partialTrueCount] - 1];
    nav3_B.b_waypoints_data_c[nav3_B.partialTrueCount + nav3_B.trueCount_c] =
      waypoints[nav3_B.c_data_p[nav3_B.partialTrueCount] + 1];
  }

  if (nav3_B.trueCount_c == 0) {
    nav3_B.lookaheadEndPt_idx_0 = pose[0];
    nav3_B.lookaheadEndPt_idx_1 = pose[1];
  } else if (nav3_B.trueCount_c == 1) {
    nav3_B.lookaheadEndPt_idx_0 = nav3_B.b_waypoints_data_c[0];
    nav3_B.lookaheadEndPt_idx_1 = nav3_B.b_waypoints_data_c[nav3_B.trueCount_c];
  } else {
    nav3_B.controller_ProjectionPoint[0] = nav3_B.b_waypoints_data_c[1];
    nav3_B.b_waypoints_g[0] = nav3_B.b_waypoints_data_c[0];
    nav3_B.lookaheadEndPt_idx_1 = nav3_B.b_waypoints_data_c[nav3_B.trueCount_c +
      1];
    nav3_B.controller_ProjectionPoint[1] = nav3_B.lookaheadEndPt_idx_1;
    nav3_B.b_waypoints_g[1] = nav3_B.b_waypoints_data_c[nav3_B.trueCount_c];
    nav3_closestPointOnLine(nav3_B.b_waypoints_g,
      nav3_B.controller_ProjectionPoint, &pose[0]);
    nav3_B.lookaheadEndPt_idx_0 = nav3_B.b_waypoints_data_c[1];
    nav3_B.b_waypoints_g[0] = nav3_B.controller_ProjectionPoint[0] -
      nav3_B.b_waypoints_data_c[1];
    nav3_B.b_waypoints_g[1] = nav3_B.controller_ProjectionPoint[1] -
      nav3_B.lookaheadEndPt_idx_1;
    nav3_B.alpha_b = nav3_norm(nav3_B.b_waypoints_g);
    nav3_B.alpha_b = (nav3_B.alpha_b - lookAhead) / nav3_B.alpha_b;
    if (nav3_B.alpha_b > 0.0) {
      nav3_B.lookaheadEndPt_idx_0 = (1.0 - nav3_B.alpha_b) *
        nav3_B.b_waypoints_data_c[1] + nav3_B.alpha_b *
        nav3_B.controller_ProjectionPoint[0];
      nav3_B.lookaheadEndPt_idx_1 = (1.0 - nav3_B.alpha_b) *
        nav3_B.lookaheadEndPt_idx_1 + nav3_B.alpha_b *
        nav3_B.controller_ProjectionPoint[1];
    }
  }

  targetHeading[0] = nav3_rt_atan2d_snf(nav3_B.lookaheadEndPt_idx_1 - pose[1],
    nav3_B.lookaheadEndPt_idx_0 - pose[0]);
  targetHeading[1] = nav3_B.lookaheadEndPt_idx_0;
  targetHeading[2] = nav3_B.lookaheadEndPt_idx_1;
}

static void matlabCodegenHandle_matlab_kgkc(ros_slros_internal_block_GetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

static void matlabCodegenHandle_matlabCo_kg(ros_slros_internal_block_Subs_T *obj)
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

static void nav3_matlabCodegenHa_en(ros_slros_internal_block_SetP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
  }
}

/* Model step function */
void nav3_step(void)
{
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T exitg1;

  /* DataStoreWrite: '<S77>/Data Store Write' incorporates:
   *  MATLABSystem: '<S77>/Get Parameter1'
   */
  ParamGet_nav3_242.get_parameter(&nav3_DW.LOOKAHEAD_DISTANCE);

  /* DataStoreWrite: '<S77>/Data Store Write1' incorporates:
   *  MATLABSystem: '<S77>/Get Parameter2'
   */
  ParamGet_nav3_274.get_parameter(&nav3_DW.D);

  /* DataStoreWrite: '<S77>/Data Store Write2' incorporates:
   *  MATLABSystem: '<S77>/Get Parameter3'
   */
  ParamGet_nav3_275.get_parameter(&nav3_DW.I);

  /* DataStoreWrite: '<S77>/Data Store Write3' incorporates:
   *  MATLABSystem: '<S77>/Get Parameter4'
   */
  ParamGet_nav3_276.get_parameter(&nav3_DW.P);

  /* DataStoreWrite: '<S78>/Data Store Write' incorporates:
   *  MATLABSystem: '<S78>/Get Parameter6'
   */
  ParamGet_nav3_541.get_parameter(&nav3_DW.WAYPOINT_PRECISION);

  /* DataStoreWrite: '<S78>/Data Store Write1' incorporates:
   *  MATLABSystem: '<S78>/Get Parameter5'
   */
  ParamGet_nav3_368.get_parameter(&nav3_DW.MAX_LINEAR_VEL);

  /* MATLABSystem: '<S78>/Get Parameter2' */
  ParamGet_nav3_326.get_parameter(&nav3_B.quat_idx_3);

  /* MATLABSystem: '<S78>/Get Parameter3' */
  ParamGet_nav3_327.get_parameter(&nav3_B.K12);

  /* MATLABSystem: '<S78>/Get Parameter1' */
  ParamGet_nav3_325.get_parameter(&nav3_B.Product);

  /* DataStoreWrite: '<S78>/Data Store Write2' incorporates:
   *  MATLABSystem: '<S78>/Get Parameter1'
   *  MATLABSystem: '<S78>/Get Parameter2'
   *  MATLABSystem: '<S78>/Get Parameter3'
   */
  nav3_DW.InitialPose[0] = nav3_B.quat_idx_3;
  nav3_DW.InitialPose[1] = nav3_B.K12;
  nav3_DW.InitialPose[2] = nav3_B.Product;

  /* DataStoreWrite: '<S78>/Data Store Write3' incorporates:
   *  MATLABSystem: '<S78>/Get Parameter4'
   */
  ParamGet_nav3_365.get_parameter(&nav3_DW.MAX_ANGULAR_VEL);

  /* MATLAB Function: '<S78>/MATLAB Function' incorporates:
   *  MATLABSystem: '<S78>/Get Parameter1'
   */
  nav3_B.K34 = sin(nav3_B.Product);
  nav3_B.quat_idx_2 = cos(nav3_B.Product);

  /* MATLABSystem: '<S78>/Get Parameter' */
  ParamGet_nav3_237.get_parameter(&nav3_B.Product);

  /* DataStoreWrite: '<S80>/Data Store Write' incorporates:
   *  MATLABSystem: '<S80>/Get Parameter4'
   */
  ParamGet_nav3_295.get_parameter(&nav3_DW.AUTONOMOUS_CONTROL_STATE);

  /* DataStoreWrite: '<S80>/Data Store Write1' incorporates:
   *  MATLABSystem: '<S80>/Get Parameter3'
   */
  ParamGet_nav3_294.get_parameter(&nav3_DW.EMERGENCY_STATE);

  /* DataStoreWrite: '<S80>/Data Store Write2' incorporates:
   *  MATLABSystem: '<S80>/Get Parameter2'
   */
  ParamGet_nav3_293.get_parameter(&nav3_DW.INITIALISATION_STATE);

  /* DataStoreWrite: '<S80>/Data Store Write3' incorporates:
   *  MATLABSystem: '<S80>/Get Parameter1'
   */
  ParamGet_nav3_292.get_parameter(&nav3_DW.MANUAL_CONTROL_STATE);

  /* DataStoreWrite: '<S80>/Data Store Write4' incorporates:
   *  MATLABSystem: '<S80>/Get Parameter5'
   */
  ParamGet_nav3_303.get_parameter(&nav3_DW.REVERSE_STATE);

  /* DataStoreWrite: '<S80>/Data Store Write5' incorporates:
   *  MATLABSystem: '<S80>/Get Parameter6'
   */
  ParamGet_nav3_306.get_parameter(&nav3_DW.WAITING_STATE);

  /* MATLABSystem: '<S79>/Get Parameter' */
  ParamGet_nav3_340.get_parameter(&nav3_B.Product);

  /* MATLABSystem: '<S79>/Get Parameter1' */
  ParamGet_nav3_341.get_parameter(&nav3_B.Product);

  /* MATLABSystem: '<S79>/Get Parameter2' */
  ParamGet_nav3_342.get_parameter(&nav3_B.value);

  /* MATLAB Function: '<S79>/MATLAB Function' incorporates:
   *  MATLABSystem: '<S79>/Get Parameter3'
   *  MATLABSystem: '<S79>/Get Parameter4'
   */
  ParamGet_nav3_357.get_parameter(&nav3_B.Lidar2Robot[7]);
  ParamGet_nav3_358.get_parameter(&nav3_B.Lidar2Robot[6]);

  /* MATLABSystem: '<S79>/Get Parameter5' */
  ParamGet_nav3_359.get_parameter(&nav3_B.value_g);

  /* MATLAB Function: '<S79>/MATLAB Function' incorporates:
   *  MATLABSystem: '<S79>/Get Parameter5'
   */
  nav3_B.rtb_Lidar2Robot_tmp = sin(nav3_B.value_g);
  nav3_B.value_g = cos(nav3_B.value_g);
  nav3_B.Lidar2Robot[0] = nav3_B.value_g;
  nav3_B.Lidar2Robot[3] = -nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.Lidar2Robot[1] = nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.Lidar2Robot[4] = nav3_B.value_g;
  nav3_B.Lidar2Robot[2] = 0.0;
  nav3_B.Lidar2Robot[5] = 0.0;
  nav3_B.Lidar2Robot[8] = 1.0;

  /* MATLABSystem: '<S81>/Get Parameter5' */
  ParamGet_nav3_381.get_parameter(&nav3_B.value_m);

  /* Outputs for Atomic SubSystem: '<S86>/Live_Subscriber' */
  /* MATLABSystem: '<S98>/SourceBlock' incorporates:
   *  Inport: '<S101>/In1'
   */
  nav3_B.b_varargout_1 = Sub_nav3_202.getLatestMessage(&nav3_B.b_varargout_2);

  /* Outputs for Enabled SubSystem: '<S98>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S101>/Enable'
   */
  if (nav3_B.b_varargout_1) {
    nav3_B.In1 = nav3_B.b_varargout_2;
  }

  /* End of MATLABSystem: '<S98>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S98>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S86>/Live_Subscriber' */

  /* MATLAB Function: '<S97>/Quaternion_2_Euler' */
  nav3_Quaternion_2_Euler(nav3_B.In1.Pose.Pose.Orientation.X,
    nav3_B.In1.Pose.Pose.Orientation.Y, nav3_B.In1.Pose.Pose.Orientation.Z,
    nav3_B.In1.Pose.Pose.Orientation.W, &nav3_B.sf_Quaternion_2_Euler_o);

  /* MATLAB Function: '<S97>/World to Robot Transform' incorporates:
   *  MATLAB Function: '<S78>/MATLAB Function'
   *  MATLAB Function: '<S79>/MATLAB Function'
   *  MATLABSystem: '<S78>/Get Parameter2'
   *  MATLABSystem: '<S78>/Get Parameter3'
   *  SignalConversion generated from: '<S100>/ SFunction '
   */
  memcpy(&nav3_B.World2RobotOdom[0], &nav3_B.Lidar2Robot[0], 9U * sizeof(real_T));
  nav3_B.r1 = 0;
  nav3_B.r2 = 1;
  nav3_B.r3 = 2;
  if (fabs(nav3_B.rtb_Lidar2Robot_tmp) > fabs(nav3_B.value_g)) {
    nav3_B.r1 = 1;
    nav3_B.r2 = 0;
  }

  nav3_B.World2RobotOdom[nav3_B.r2] = nav3_B.Lidar2Robot[nav3_B.r2] /
    nav3_B.Lidar2Robot[nav3_B.r1];
  nav3_B.World2RobotOdom[2] /= nav3_B.World2RobotOdom[nav3_B.r1];
  nav3_B.World2RobotOdom[nav3_B.r2 + 3] -= nav3_B.World2RobotOdom[nav3_B.r1 + 3]
    * nav3_B.World2RobotOdom[nav3_B.r2];
  nav3_B.World2RobotOdom[5] -= nav3_B.World2RobotOdom[nav3_B.r1 + 3] *
    nav3_B.World2RobotOdom[2];
  nav3_B.World2RobotOdom[nav3_B.r2 + 6] -= nav3_B.World2RobotOdom[nav3_B.r1 + 6]
    * nav3_B.World2RobotOdom[nav3_B.r2];
  nav3_B.World2RobotOdom[8] -= nav3_B.World2RobotOdom[nav3_B.r1 + 6] *
    nav3_B.World2RobotOdom[2];
  if (fabs(nav3_B.World2RobotOdom[5]) > fabs(nav3_B.World2RobotOdom[nav3_B.r2 +
       3])) {
    nav3_B.r3 = nav3_B.r2;
    nav3_B.r2 = 2;
  }

  nav3_B.World2RobotOdom[nav3_B.r3 + 3] /= nav3_B.World2RobotOdom[nav3_B.r2 + 3];
  nav3_B.World2RobotOdom[nav3_B.r3 + 6] -= nav3_B.World2RobotOdom[nav3_B.r3 + 3]
    * nav3_B.World2RobotOdom[nav3_B.r2 + 6];
  nav3_B.rtb_Lidar2Robot_tmp = sin(nav3_B.sf_Quaternion_2_Euler_o.theta);
  nav3_B.value_g = cos(nav3_B.sf_Quaternion_2_Euler_o.theta);
  nav3_B.W2RTransform_tmp[0] = nav3_B.value_g;
  nav3_B.W2RTransform_tmp[3] = -nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.W2RTransform_tmp[6] = nav3_B.In1.Pose.Pose.Position.X;
  nav3_B.W2RTransform_tmp[1] = nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.W2RTransform_tmp[4] = nav3_B.value_g;
  nav3_B.W2RTransform_tmp[7] = nav3_B.In1.Pose.Pose.Position.Y;
  nav3_B.W2LOTransform[3 * nav3_B.r1] = nav3_B.quat_idx_2 /
    nav3_B.World2RobotOdom[nav3_B.r1];
  nav3_B.rtb_Lidar2Robot_tmp = nav3_B.World2RobotOdom[nav3_B.r1 + 3];
  nav3_B.W2LOTransform[3 * nav3_B.r2] = -nav3_B.K34 - nav3_B.W2LOTransform[3 *
    nav3_B.r1] * nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.value_g = nav3_B.World2RobotOdom[nav3_B.r1 + 6];
  nav3_B.W2LOTransform[3 * nav3_B.r3] = nav3_B.quat_idx_3 -
    nav3_B.W2LOTransform[3 * nav3_B.r1] * nav3_B.value_g;
  nav3_B.quat_idx_3 = nav3_B.World2RobotOdom[nav3_B.r2 + 3];
  nav3_B.W2LOTransform[3 * nav3_B.r2] /= nav3_B.quat_idx_3;
  nav3_B.W2LOTransform_tmp = nav3_B.World2RobotOdom[nav3_B.r2 + 6];
  nav3_B.W2LOTransform[3 * nav3_B.r3] -= nav3_B.W2LOTransform[3 * nav3_B.r2] *
    nav3_B.W2LOTransform_tmp;
  nav3_B.W2LOTransform_tmp_m = nav3_B.World2RobotOdom[nav3_B.r3 + 6];
  nav3_B.W2LOTransform[3 * nav3_B.r3] /= nav3_B.W2LOTransform_tmp_m;
  nav3_B.W2LOTransform_tmp_n = nav3_B.World2RobotOdom[nav3_B.r3 + 3];
  nav3_B.W2LOTransform[3 * nav3_B.r2] -= nav3_B.W2LOTransform[3 * nav3_B.r3] *
    nav3_B.W2LOTransform_tmp_n;
  nav3_B.W2LOTransform[3 * nav3_B.r1] -= nav3_B.W2LOTransform[3 * nav3_B.r3] *
    nav3_B.World2RobotOdom[nav3_B.r3];
  nav3_B.W2LOTransform[3 * nav3_B.r1] -= nav3_B.W2LOTransform[3 * nav3_B.r2] *
    nav3_B.World2RobotOdom[nav3_B.r2];
  nav3_B.W2RTransform_tmp[2] = 0.0;
  nav3_B.i = 3 * nav3_B.r1 + 1;
  nav3_B.W2LOTransform[nav3_B.i] = nav3_B.K34 / nav3_B.World2RobotOdom[nav3_B.r1];
  nav3_B.W2LOTransform_tmp_c = 3 * nav3_B.r2 + 1;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] = nav3_B.quat_idx_2 -
    nav3_B.W2LOTransform[nav3_B.i] * nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.W2LOTransform_tmp_md = 3 * nav3_B.r3 + 1;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] = nav3_B.K12 -
    nav3_B.W2LOTransform[nav3_B.i] * nav3_B.value_g;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] /= nav3_B.quat_idx_3;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] * nav3_B.W2LOTransform_tmp;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] /=
    nav3_B.W2LOTransform_tmp_m;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] *
    nav3_B.W2LOTransform_tmp_n;
  nav3_B.W2LOTransform[nav3_B.i] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] *
    nav3_B.World2RobotOdom[nav3_B.r3];
  nav3_B.W2LOTransform[nav3_B.i] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] *
    nav3_B.World2RobotOdom[nav3_B.r2];
  nav3_B.W2RTransform_tmp[5] = 0.0;
  nav3_B.i = 3 * nav3_B.r1 + 2;
  nav3_B.W2LOTransform[nav3_B.i] = 0.0 / nav3_B.World2RobotOdom[nav3_B.r1];
  nav3_B.W2LOTransform_tmp_c = 3 * nav3_B.r2 + 2;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] = 0.0 -
    nav3_B.W2LOTransform[nav3_B.i] * nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.W2LOTransform_tmp_md = 3 * nav3_B.r3 + 2;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] = 1.0 -
    nav3_B.W2LOTransform[nav3_B.i] * nav3_B.value_g;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] /= nav3_B.quat_idx_3;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] * nav3_B.W2LOTransform_tmp;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] /=
    nav3_B.W2LOTransform_tmp_m;
  nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] *
    nav3_B.W2LOTransform_tmp_n;
  nav3_B.W2LOTransform[nav3_B.i] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_md] *
    nav3_B.World2RobotOdom[nav3_B.r3];
  nav3_B.W2LOTransform[nav3_B.i] -=
    nav3_B.W2LOTransform[nav3_B.W2LOTransform_tmp_c] *
    nav3_B.World2RobotOdom[nav3_B.r2];
  nav3_B.W2RTransform_tmp[8] = 1.0;
  for (nav3_B.r1 = 0; nav3_B.r1 < 3; nav3_B.r1++) {
    for (nav3_B.r2 = 0; nav3_B.r2 < 3; nav3_B.r2++) {
      nav3_B.i = nav3_B.r1 + 3 * nav3_B.r2;
      nav3_B.W2LOTransform_b[nav3_B.i] = 0.0;
      nav3_B.W2LOTransform_b[nav3_B.i] += nav3_B.W2RTransform_tmp[3 * nav3_B.r2]
        * nav3_B.W2LOTransform[nav3_B.r1];
      nav3_B.W2LOTransform_b[nav3_B.i] += nav3_B.W2RTransform_tmp[3 * nav3_B.r2
        + 1] * nav3_B.W2LOTransform[nav3_B.r1 + 3];
      nav3_B.W2LOTransform_b[nav3_B.i] += nav3_B.W2RTransform_tmp[3 * nav3_B.r2
        + 2] * nav3_B.W2LOTransform[nav3_B.r1 + 6];
    }

    for (nav3_B.r2 = 0; nav3_B.r2 < 3; nav3_B.r2++) {
      nav3_B.i = nav3_B.r1 + 3 * nav3_B.r2;
      nav3_B.World2RobotOdom[nav3_B.i] = 0.0;
      nav3_B.World2RobotOdom[nav3_B.i] += nav3_B.Lidar2Robot[3 * nav3_B.r2] *
        nav3_B.W2LOTransform_b[nav3_B.r1];
      nav3_B.World2RobotOdom[nav3_B.i] += nav3_B.Lidar2Robot[3 * nav3_B.r2 + 1] *
        nav3_B.W2LOTransform_b[nav3_B.r1 + 3];
      nav3_B.World2RobotOdom[nav3_B.i] += nav3_B.Lidar2Robot[3 * nav3_B.r2 + 2] *
        nav3_B.W2LOTransform_b[nav3_B.r1 + 6];
    }
  }

  nav3_B.K12 = nav3_B.World2RobotOdom[3] + nav3_B.World2RobotOdom[1];
  nav3_B.K34 = nav3_B.World2RobotOdom[1] - nav3_B.World2RobotOdom[3];
  nav3_B.rotm[0] = ((nav3_B.World2RobotOdom[0] - nav3_B.World2RobotOdom[4]) -
                    1.0) / 3.0;
  nav3_B.rotm[4] = nav3_B.K12 / 3.0;
  nav3_B.rotm[8] = 0.0;
  nav3_B.rotm[12] = 0.0;
  nav3_B.rotm[1] = nav3_B.K12 / 3.0;
  nav3_B.rotm[5] = ((nav3_B.World2RobotOdom[4] - nav3_B.World2RobotOdom[0]) -
                    1.0) / 3.0;
  nav3_B.rotm[9] = 0.0;
  nav3_B.rotm[13] = 0.0;
  nav3_B.rotm[2] = 0.0;
  nav3_B.rotm[6] = 0.0;
  nav3_B.rotm[10] = ((1.0 - nav3_B.World2RobotOdom[0]) - nav3_B.World2RobotOdom
                     [4]) / 3.0;
  nav3_B.rotm[14] = nav3_B.K34 / 3.0;
  nav3_B.rotm[3] = 0.0;
  nav3_B.rotm[7] = 0.0;
  nav3_B.rotm[11] = nav3_B.K34 / 3.0;
  nav3_B.rotm[15] = ((nav3_B.World2RobotOdom[0] + nav3_B.World2RobotOdom[4]) +
                     1.0) / 3.0;
  nav3_eig(nav3_B.rotm, nav3_B.eigVec, nav3_B.eigVal);
  nav3_B.varargin_1[0] = nav3_B.eigVal[0].re;
  nav3_B.varargin_1[1] = nav3_B.eigVal[1].re;
  nav3_B.varargin_1[2] = nav3_B.eigVal[2].re;
  nav3_B.varargin_1[3] = nav3_B.eigVal[3].re;
  if (!rtIsNaN(nav3_B.eigVal[0].re)) {
    nav3_B.i = 1;
  } else {
    nav3_B.i = 0;
    nav3_B.r1 = 2;
    exitg1 = false;
    while ((!exitg1) && (nav3_B.r1 < 5)) {
      if (!rtIsNaN(nav3_B.varargin_1[nav3_B.r1 - 1])) {
        nav3_B.i = nav3_B.r1;
        exitg1 = true;
      } else {
        nav3_B.r1++;
      }
    }
  }

  if (nav3_B.i != 0) {
    nav3_B.K12 = nav3_B.varargin_1[nav3_B.i - 1];
    nav3_B.r1 = nav3_B.i - 1;
    while (nav3_B.i + 1 < 5) {
      if (nav3_B.K12 < nav3_B.varargin_1[nav3_B.i]) {
        nav3_B.K12 = nav3_B.varargin_1[nav3_B.i];
        nav3_B.r1 = nav3_B.i;
      }

      nav3_B.i++;
    }

    nav3_B.i = nav3_B.r1;
  }

  nav3_B.i <<= 2;
  nav3_B.K12 = nav3_B.eigVec[nav3_B.i + 3].re;
  nav3_B.K34 = nav3_B.eigVec[nav3_B.i].re;
  nav3_B.quat_idx_2 = nav3_B.eigVec[nav3_B.i + 1].re;
  nav3_B.quat_idx_3 = nav3_B.eigVec[nav3_B.i + 2].re;
  if (nav3_B.K12 < 0.0) {
    nav3_B.K12 = -nav3_B.K12;
    nav3_B.K34 = -nav3_B.eigVec[nav3_B.i].re;
    nav3_B.quat_idx_2 = -nav3_B.quat_idx_2;
    nav3_B.quat_idx_3 = -nav3_B.quat_idx_3;
  }

  nav3_B.rtb_Lidar2Robot_tmp = 1.0 / sqrt(((nav3_B.K12 * nav3_B.K12 + nav3_B.K34
    * nav3_B.K34) + nav3_B.quat_idx_2 * nav3_B.quat_idx_2) + nav3_B.quat_idx_3 *
    nav3_B.quat_idx_3);
  nav3_B.varargin_1[0] = nav3_B.K12 * nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.varargin_1[1] = nav3_B.K34 * nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.varargin_1[2] = nav3_B.quat_idx_2 * nav3_B.rtb_Lidar2Robot_tmp;
  nav3_B.varargin_1[3] = nav3_B.quat_idx_3 * nav3_B.rtb_Lidar2Robot_tmp;

  /* Outputs for Atomic SubSystem: '<S87>/Simulation_Subscriber' */
  /* MATLABSystem: '<S103>/SourceBlock' incorporates:
   *  Inport: '<S105>/In1'
   */
  nav3_B.b_varargout_1 = Sub_nav3_223.getLatestMessage(&nav3_B.b_varargout_2_m);

  /* Outputs for Enabled SubSystem: '<S103>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S105>/Enable'
   */
  if (nav3_B.b_varargout_1) {
    nav3_B.In1_p = nav3_B.b_varargout_2_m;
  }

  /* End of MATLABSystem: '<S103>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S103>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S87>/Simulation_Subscriber' */

  /* MATLAB Function: '<S102>/Quaternion_2_Euler' */
  nav3_Quaternion_2_Euler(nav3_B.In1_p.Pose.Pose.Orientation.X,
    nav3_B.In1_p.Pose.Pose.Orientation.Y, nav3_B.In1_p.Pose.Pose.Orientation.Z,
    nav3_B.In1_p.Pose.Pose.Orientation.W, &nav3_B.sf_Quaternion_2_Euler_g);

  /* Switch: '<S6>/Switch' incorporates:
   *  MATLAB Function: '<S97>/World to Robot Transform'
   *  MATLABSystem: '<S81>/Get Parameter5'
   */
  if (nav3_B.value_m >= nav3_P.Switch_Threshold) {
    nav3_B.Switch[0] = nav3_B.World2RobotOdom[6];
    nav3_B.Switch[1] = nav3_B.World2RobotOdom[7];
    nav3_B.Switch[2] = nav3_rt_atan2d_snf((nav3_B.varargin_1[1] *
      nav3_B.varargin_1[2] + nav3_B.varargin_1[0] * nav3_B.varargin_1[3]) * 2.0,
      ((nav3_B.varargin_1[0] * nav3_B.varargin_1[0] + nav3_B.varargin_1[1] *
        nav3_B.varargin_1[1]) - nav3_B.varargin_1[2] * nav3_B.varargin_1[2]) -
      nav3_B.varargin_1[3] * nav3_B.varargin_1[3]);
  } else {
    nav3_B.Switch[0] = nav3_B.In1_p.Pose.Pose.Position.X;
    nav3_B.Switch[1] = nav3_B.In1_p.Pose.Pose.Position.Y;
    nav3_B.Switch[2] = nav3_B.sf_Quaternion_2_Euler_g.theta;
  }

  /* End of Switch: '<S6>/Switch' */

  /* Outputs for Atomic SubSystem: '<S84>/Live_Subscriber' */
  /* MATLABSystem: '<S88>/SourceBlock' */
  nav3_B.b_varargout_1 = Sub_nav3_165.getLatestMessage(&nav3_B.b_varargout_2_f);

  /* Outputs for Enabled SubSystem: '<S88>/Enabled Subsystem' */
  nav3_EnabledSubsystem(nav3_B.b_varargout_1, &nav3_B.b_varargout_2_f,
                        &nav3_B.EnabledSubsystem_m);

  /* End of Outputs for SubSystem: '<S88>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S84>/Live_Subscriber' */

  /* Outputs for Atomic SubSystem: '<S3>/Subscribe' */
  /* MATLABSystem: '<S73>/SourceBlock' incorporates:
   *  Inport: '<S74>/In1'
   */
  nav3_B.b_varargout_1 = Sub_nav3_555.getLatestMessage(&nav3_B.b_varargout_2_k);

  /* Outputs for Enabled SubSystem: '<S73>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S74>/Enable'
   */
  if (nav3_B.b_varargout_1) {
    nav3_B.In1_i = nav3_B.b_varargout_2_k;
  }

  /* End of MATLABSystem: '<S73>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S73>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S3>/Subscribe' */

  /* DataTypeConversion: '<S3>/Data Type Conversion' */
  for (nav3_B.i = 0; nav3_B.i < 8; nav3_B.i++) {
    nav3_B.DataTypeConversion[nav3_B.i] = nav3_B.In1_i.Axes[nav3_B.i];
  }

  /* End of DataTypeConversion: '<S3>/Data Type Conversion' */

  /* Chart: '<S2>/State_Machine' incorporates:
   *  Constant: '<S2>/waypoints'
   *  Gain: '<S3>/Linear_Accel'
   *  Sum: '<S3>/Sum'
   */
  if (nav3_DW.is_active_c3_nav3 == 0U) {
    nav3_DW.is_active_c3_nav3 = 1U;
    nav3_DW.is_c3_nav3 = nav3_IN_INITIALISATION;
    nav3_B.currentState = nav3_DW.INITIALISATION_STATE;
  } else {
    guard1 = false;
    guard2 = false;
    switch (nav3_DW.is_c3_nav3) {
     case nav3_IN_AUTONOMOUS_CONTROL:
      nav3_AUTONOMOUS_CONTROL();
      break;

     case nav3_IN_EMERGENCY:
      if (nav3_B.In1_i.Buttons[3] == 1) {
        nav3_DW.is_c3_nav3 = nav3_IN_WAITING;
        nav3_B.currentState = nav3_DW.WAITING_STATE;
      } else {
        nav3_B.linearVel = 0.0;
        nav3_B.angularVel = 0.0;
      }
      break;

     case nav3_IN_INITIALISATION:
      nav3_DW.is_c3_nav3 = nav3_IN_WAITING;
      nav3_B.currentState = nav3_DW.WAITING_STATE;
      break;

     case nav3_IN_MANUAL_CONTROL:
      if (nav3_B.In1_i.Buttons[1] == 1) {
        nav3_DW.is_c3_nav3 = nav3_IN_EMERGENCY;
        nav3_B.currentState = nav3_DW.EMERGENCY_STATE;
      } else if (nav3_B.In1_i.Buttons[5] == 1) {
        nav3_DW.is_c3_nav3 = nav3_IN_WAITING;
        nav3_B.currentState = nav3_DW.WAITING_STATE;
      } else {
        nav3_B.angularVel = nav3_B.DataTypeConversion[0];
        nav3_B.linearVel = nav3_P.Linear_Accel_Gain * nav3_B.DataTypeConversion
          [4] + nav3_B.DataTypeConversion[1];
      }
      break;

     case nav3_IN_REVERSE:
      if (nav3_B.In1_i.Buttons[1] == 1) {
        if (nav3_DW.is_REVERSE == nav3_IN_RotateToWaypoint) {
          nav3_B.angularVel = 0.0;
          nav3_DW.is_REVERSE = nav3_IN_NO_ACTIVE_CHILD;
        } else {
          nav3_DW.is_REVERSE = nav3_IN_NO_ACTIVE_CHILD;
        }

        nav3_DW.is_c3_nav3 = nav3_IN_EMERGENCY;
        nav3_B.currentState = nav3_DW.EMERGENCY_STATE;
      } else if (nav3_B.In1_i.Buttons[5] == 1) {
        if (nav3_DW.is_REVERSE == nav3_IN_RotateToWaypoint) {
          nav3_B.angularVel = 0.0;
          nav3_DW.is_REVERSE = nav3_IN_NO_ACTIVE_CHILD;
        } else {
          nav3_DW.is_REVERSE = nav3_IN_NO_ACTIVE_CHILD;
        }

        nav3_DW.is_c3_nav3 = nav3_IN_WAITING;
        nav3_B.currentState = nav3_DW.WAITING_STATE;
      } else {
        switch (nav3_DW.is_REVERSE) {
         case nav3_IN_MoveToWaypoint:
          nav3_B.pose[0] = nav3_B.Switch[0];
          nav3_B.pose[1] = nav3_B.Switch[1];
          nav3_B.pose[2] = nav3_B.Switch[2];
          nav3_B.r1 = static_cast<int32_T>(nav3_DW.count + 1.0);
          nav3_B.goalWaypoint[0] = nav3_DW.all_waypoints[nav3_B.r1 - 1];
          nav3_B.goalWaypoint[1] = nav3_DW.all_waypoints[nav3_B.r1 + 9];
          nav3_B.goalTolerance = nav3_DW.LOOKAHEAD_DISTANCE;

          /* Outputs for Function Call SubSystem: '<S16>/checkAtGoal' */
          nav3_checkAtGoal(nav3_B.pose, nav3_B.goalWaypoint,
                           nav3_B.goalTolerance, &nav3_B.checkAtGoal);

          /* End of Outputs for SubSystem: '<S16>/checkAtGoal' */
          if (!nav3_B.checkAtGoal.LessThan) {
            nav3_DW.is_REVERSE = nav3_IN_PreciseMoveToWaypoint;
            if (nav3_DW.REVERSE_STATE > 2147483643) {
              nav3_B.currentState = MAX_int32_T;
            } else {
              nav3_B.currentState = nav3_DW.REVERSE_STATE + 4;
            }

            nav3_B.linearVel = nav3_DW.MAX_LINEAR_VEL / 2.0;
          } else {
            nav3_purePursuit(nav3_DW.LOOKAHEAD_DISTANCE, nav3_DW.waypointLinePID,
                             nav3_B.Switch, nav3_DW.goalHeading);
            nav3_B.pose_f[0] = nav3_B.Switch[0];
            nav3_B.pose_f[1] = nav3_B.Switch[1];
            nav3_B.pose_f[2] = nav3_B.Switch[2];
            nav3_B.desiredHeading = nav3_DW.goalHeading[0];
            nav3_B.P = nav3_DW.P;
            nav3_B.I = nav3_DW.I;
            nav3_B.D = nav3_DW.D;

            /* Outputs for Function Call SubSystem: '<S16>/PID' */
            nav3_PID(nav3_M, nav3_B.pose_f, nav3_B.desiredHeading, nav3_B.P,
                     nav3_B.I, nav3_B.D, &nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

            /* End of Outputs for SubSystem: '<S16>/PID' */
            nav3_B.angularVel = nav3_B.PID.Saturation;
          }
          break;

         case nav3_IN_PreciseMoveToWaypoint:
          nav3_B.pose[0] = nav3_B.Switch[0];
          nav3_B.pose[1] = nav3_B.Switch[1];
          nav3_B.pose[2] = nav3_B.Switch[2];
          nav3_B.r1 = static_cast<int32_T>(nav3_DW.count + 1.0);
          nav3_B.goalWaypoint[0] = nav3_DW.all_waypoints[nav3_B.r1 - 1];
          nav3_B.goalWaypoint[1] = nav3_DW.all_waypoints[nav3_B.r1 + 9];
          nav3_B.goalTolerance = nav3_DW.WAYPOINT_PRECISION;

          /* Outputs for Function Call SubSystem: '<S16>/checkAtGoal' */
          nav3_checkAtGoal(nav3_B.pose, nav3_B.goalWaypoint,
                           nav3_B.goalTolerance, &nav3_B.checkAtGoal);

          /* End of Outputs for SubSystem: '<S16>/checkAtGoal' */
          if (!nav3_B.checkAtGoal.LessThan) {
            nav3_DW.is_REVERSE = nav3_IN_ReachedWaypoint;
            if (nav3_DW.REVERSE_STATE > 2147483642) {
              nav3_B.currentState = MAX_int32_T;
            } else {
              nav3_B.currentState = nav3_DW.REVERSE_STATE + 5;
            }
          } else {
            if (nav3_DW.REVERSE_STATE > 2147483643) {
              nav3_B.currentState = MAX_int32_T;
            } else {
              nav3_B.currentState = nav3_DW.REVERSE_STATE + 4;
            }

            nav3_B.linearVel = nav3_DW.MAX_LINEAR_VEL / 2.0;
            nav3_B.r1 = static_cast<int32_T>(nav3_DW.count + 1.0);
            nav3_B.dv[0] = nav3_DW.all_waypoints[nav3_B.r1 - 1];
            nav3_B.dv[1] = nav3_DW.all_waypoints[nav3_B.r1 + 9];
            nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
              nav3_B.dv);
            nav3_DW.goalHeading[0] = nav3_DW.currentGoalHeading;
            nav3_DW.goalHeading[1] = 0.0;
            nav3_DW.goalHeading[2] = 0.0;
            nav3_B.pose_f[0] = nav3_B.Switch[0];
            nav3_B.pose_f[1] = nav3_B.Switch[1];
            nav3_B.pose_f[2] = nav3_B.Switch[2];
            nav3_B.desiredHeading = nav3_DW.goalHeading[0];
            nav3_B.P = nav3_DW.P;
            nav3_B.I = nav3_DW.I;
            nav3_B.D = nav3_DW.D;

            /* Outputs for Function Call SubSystem: '<S16>/PID' */
            nav3_PID(nav3_M, nav3_B.pose_f, nav3_B.desiredHeading, nav3_B.P,
                     nav3_B.I, nav3_B.D, &nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

            /* End of Outputs for SubSystem: '<S16>/PID' */
            nav3_B.angularVel = nav3_B.PID.Saturation;
          }
          break;

         case nav3_IN_ReachedWaypoint:
          if (nav3_DW.count + 1.0 == nav3_DW.last) {
            nav3_DW.finishedMissionCount++;
            nav3_DW.count = 1.0;
            nav3_B.K12 = nav3_mod(nav3_DW.finishedMissionCount);
            if (nav3_B.K12 == 1.0) {
              memcpy(&nav3_B.b[0], &nav3_P.waypoints_Value[0], 18U * sizeof
                     (real_T));
              nav3_flip(nav3_B.b);
              for (nav3_B.r1 = 0; nav3_B.r1 < 2; nav3_B.r1++) {
                memcpy(&nav3_DW.all_waypoints[nav3_B.r1 * 10],
                       &nav3_B.b[nav3_B.r1 * 9], 9U * sizeof(real_T));
                nav3_DW.all_waypoints[10 * nav3_B.r1 + 9] =
                  nav3_B.Switch[nav3_B.r1];
              }

              guard1 = true;
            } else if (nav3_B.K12 == 0.0) {
              for (nav3_B.r1 = 0; nav3_B.r1 < 2; nav3_B.r1++) {
                nav3_DW.all_waypoints[10 * nav3_B.r1] = nav3_B.Switch[nav3_B.r1];
                memcpy(&nav3_DW.all_waypoints[nav3_B.r1 * 10 + 1],
                       &nav3_P.waypoints_Value[nav3_B.r1 * 9], 9U * sizeof
                       (real_T));
              }

              guard1 = true;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
          break;

         default:
          /* case IN_RotateToWaypoint: */
          if ((nav3_DW.currentGoalHeading - 0.01 < nav3_B.Switch[2]) &&
              (nav3_B.Switch[2] < nav3_DW.currentGoalHeading + 0.01)) {
            nav3_B.angularVel = 0.0;
            nav3_DW.is_REVERSE = nav3_IN_MoveToWaypoint;
            if (nav3_DW.REVERSE_STATE > 2147483644) {
              nav3_B.currentState = MAX_int32_T;
            } else {
              nav3_B.currentState = nav3_DW.REVERSE_STATE + 3;
            }

            nav3_B.linearVel = nav3_DW.MAX_LINEAR_VEL / 2.0;
          } else {
            nav3_B.pose_f[0] = nav3_B.Switch[0];
            nav3_B.pose_f[1] = nav3_B.Switch[1];
            nav3_B.pose_f[2] = nav3_B.Switch[2];
            nav3_B.desiredHeading = nav3_DW.goalHeading[0];
            nav3_B.P = nav3_DW.P;
            nav3_B.I = nav3_DW.I;
            nav3_B.D = nav3_DW.D;

            /* Outputs for Function Call SubSystem: '<S16>/PID' */
            nav3_PID(nav3_M, nav3_B.pose_f, nav3_B.desiredHeading, nav3_B.P,
                     nav3_B.I, nav3_B.D, &nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

            /* End of Outputs for SubSystem: '<S16>/PID' */
            nav3_B.angularVel = nav3_B.PID.Saturation;
          }
          break;
        }
      }
      break;

     default:
      /* case IN_WAITING: */
      if (nav3_B.In1_i.Buttons[4] == 1) {
        nav3_DW.is_c3_nav3 = nav3_IN_MANUAL_CONTROL;
        nav3_B.currentState = nav3_DW.MANUAL_CONTROL_STATE;
      } else if (nav3_B.In1_i.Buttons[1] == 1) {
        nav3_DW.is_c3_nav3 = nav3_IN_EMERGENCY;
        nav3_B.currentState = nav3_DW.EMERGENCY_STATE;
      } else if (nav3_B.In1_i.Buttons[2] == 1) {
        nav3_DW.is_c3_nav3 = nav3_IN_REVERSE;
        for (nav3_B.r1 = 0; nav3_B.r1 < 2; nav3_B.r1++) {
          nav3_DW.all_waypoints[10 * nav3_B.r1] = nav3_B.Switch[nav3_B.r1];
          memcpy(&nav3_DW.all_waypoints[nav3_B.r1 * 10 + 1],
                 &nav3_P.waypoints_Value[nav3_B.r1 * 9], 9U * sizeof(real_T));
        }

        nav3_DW.last = 10.0;
        nav3_DW.finishedMissionCount = 0.0;
        nav3_DW.finishedMission = 0.0;
        nav3_DW.count = 1.0;
        nav3_DW.is_REVERSE = nav3_IN_RotateToWaypoint;
        if (nav3_DW.REVERSE_STATE > 2147483646) {
          nav3_B.currentState = MAX_int32_T;
        } else {
          nav3_B.currentState = nav3_DW.REVERSE_STATE + 1;
        }

        nav3_B.linearVel = 0.0;
        nav3_B.r1 = static_cast<int32_T>(nav3_DW.count + 1.0);
        nav3_DW.waypointLinePID[0] = nav3_DW.all_waypoints[static_cast<int32_T>
          (nav3_DW.count) - 1];
        nav3_DW.waypointLinePID[1] = nav3_DW.all_waypoints[static_cast<int32_T>
          (nav3_DW.count + 1.0) - 1];
        nav3_B.dv[0] = nav3_DW.all_waypoints[nav3_B.r1 - 1];
        nav3_DW.waypointLinePID[2] = nav3_DW.all_waypoints[static_cast<int32_T>
          (nav3_DW.count) + 9];
        nav3_DW.waypointLinePID[3] = nav3_DW.all_waypoints[static_cast<int32_T>
          (nav3_DW.count + 1.0) + 9];
        nav3_B.dv[1] = nav3_DW.all_waypoints[nav3_B.r1 + 9];
        nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
          nav3_B.dv);
        nav3_DW.goalHeading[0] = nav3_DW.currentGoalHeading;
        nav3_DW.goalHeading[1] = 0.0;
        nav3_DW.goalHeading[2] = 0.0;
      } else if (nav3_B.In1_i.Buttons[0] == 1) {
        nav3_DW.is_c3_nav3 = nav3_IN_AUTONOMOUS_CONTROL;
        nav3_DW.last = 10.0;
        nav3_DW.finishedMissionCount = 0.0;
        nav3_DW.finishedMission = 0.0;
        nav3_DW.count = 1.0;
        nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_RotateToWaypoint_i;
        if (nav3_DW.AUTONOMOUS_CONTROL_STATE > 2147483646) {
          nav3_B.currentState = MAX_int32_T;
        } else {
          nav3_B.currentState = nav3_DW.AUTONOMOUS_CONTROL_STATE + 1;
        }

        nav3_B.linearVel = 0.0;
        nav3_B.r1 = static_cast<int32_T>(nav3_DW.count + 1.0);
        for (nav3_B.r2 = 0; nav3_B.r2 < 2; nav3_B.r2++) {
          nav3_DW.all_waypoints[10 * nav3_B.r2] = nav3_B.Switch[nav3_B.r2];
          memcpy(&nav3_DW.all_waypoints[nav3_B.r2 * 10 + 1],
                 &nav3_P.waypoints_Value[nav3_B.r2 * 9], 9U * sizeof(real_T));
          nav3_B.dv[nav3_B.r2] = nav3_DW.all_waypoints[(10 * nav3_B.r2 +
            nav3_B.r1) - 1];
        }

        nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
          nav3_B.dv);
        nav3_DW.goalHeading[0] = nav3_DW.currentGoalHeading;
        nav3_DW.goalHeading[1] = 0.0;
        nav3_DW.goalHeading[2] = 0.0;
      } else {
        nav3_B.linearVel = 0.0;
        nav3_B.angularVel = 0.0;
      }
      break;
    }

    if (guard2) {
      if (nav3_DW.count + 1.0 < nav3_DW.last) {
        nav3_DW.count++;
        guard1 = true;
      }
    }

    if (guard1) {
      nav3_DW.is_REVERSE = nav3_IN_RotateToWaypoint;
      if (nav3_DW.REVERSE_STATE > 2147483646) {
        nav3_B.currentState = MAX_int32_T;
      } else {
        nav3_B.currentState = nav3_DW.REVERSE_STATE + 1;
      }

      nav3_B.linearVel = 0.0;
      nav3_B.r1 = static_cast<int32_T>(nav3_DW.count + 1.0);
      nav3_B.i = static_cast<int32_T>(nav3_DW.count);
      nav3_DW.waypointLinePID[0] = nav3_DW.all_waypoints[nav3_B.i - 1];
      nav3_DW.waypointLinePID[1] = nav3_DW.all_waypoints[nav3_B.r1 - 1];
      nav3_B.dv[0] = nav3_DW.all_waypoints[nav3_B.r1 - 1];
      nav3_DW.waypointLinePID[2] = nav3_DW.all_waypoints[nav3_B.i + 9];
      nav3_DW.waypointLinePID[3] = nav3_DW.all_waypoints[nav3_B.r1 + 9];
      nav3_B.dv[1] = nav3_DW.all_waypoints[nav3_B.r1 + 9];
      nav3_DW.currentGoalHeading = nav3_CalculateHeading(nav3_B.Switch,
        nav3_B.dv);
      nav3_DW.goalHeading[0] = nav3_DW.currentGoalHeading;
      nav3_DW.goalHeading[1] = 0.0;
      nav3_DW.goalHeading[2] = 0.0;
    }
  }

  /* End of Chart: '<S2>/State_Machine' */

  /* Outputs for Enabled SubSystem: '<S1>/Real_Actuation' incorporates:
   *  EnablePort: '<S8>/Enable'
   */
  /* MATLABSystem: '<S81>/Get Parameter5' */
  if (nav3_B.value_m > 0) {
    /* Product: '<S11>/Product' incorporates:
     *  Gain: '<S11>/Gain'
     *  MATLABSystem: '<S79>/Get Parameter1'
     */
    nav3_B.Product *= nav3_P.Gain_Gain * nav3_B.angularVel;

    /* Product: '<S11>/Divide2' incorporates:
     *  MATLABSystem: '<S79>/Get Parameter2'
     */
    nav3_B.K12 = nav3_B.linearVel / nav3_B.value;

    /* BusAssignment: '<S10>/Bus Assignment' incorporates:
     *  Constant: '<S12>/Constant'
     *  MATLABSystem: '<S79>/Get Parameter2'
     *  Product: '<S11>/Divide'
     *  Product: '<S11>/Divide1'
     *  Sum: '<S11>/Add'
     *  Sum: '<S11>/Add1'
     */
    nav3_B.BusAssignment_l = nav3_P.Constant_Value_n;
    nav3_B.BusAssignment_l.Angular.X = 1.0 / nav3_B.value * (nav3_B.K12 -
      nav3_B.Product);
    nav3_B.BusAssignment_l.Angular.Y = (nav3_B.K12 + nav3_B.Product) /
      nav3_B.value;

    /* Outputs for Atomic SubSystem: '<S10>/Publish' */
    /* MATLABSystem: '<S13>/SinkBlock' */
    Pub_nav3_442.publish(&nav3_B.BusAssignment_l);

    /* End of Outputs for SubSystem: '<S10>/Publish' */
  }

  /* End of Outputs for SubSystem: '<S1>/Real_Actuation' */

  /* Outputs for Enabled SubSystem: '<S1>/Simulation_Actuation' incorporates:
   *  EnablePort: '<S9>/Enable'
   */
  /* Logic: '<S1>/NOT' incorporates:
   *  MATLABSystem: '<S81>/Get Parameter5'
   */
  if (nav3_B.value_m == 0) {
    /* BusAssignment: '<S9>/Bus Assignment' incorporates:
     *  Constant: '<S14>/Constant'
     */
    nav3_B.BusAssignment_l = nav3_P.Constant_Value_o;
    nav3_B.BusAssignment_l.Linear.X = nav3_B.linearVel;
    nav3_B.BusAssignment_l.Angular.Z = nav3_B.angularVel;

    /* Outputs for Atomic SubSystem: '<S9>/Publish' */
    /* MATLABSystem: '<S15>/SinkBlock' */
    Pub_nav3_422.publish(&nav3_B.BusAssignment_l);

    /* End of Outputs for SubSystem: '<S9>/Publish' */
  }

  /* End of Logic: '<S1>/NOT' */
  /* End of Outputs for SubSystem: '<S1>/Simulation_Actuation' */

  /* Outputs for Atomic SubSystem: '<S85>/Live_Subscriber' */
  /* MATLABSystem: '<S93>/SourceBlock' incorporates:
   *  Inport: '<S96>/In1'
   */
  nav3_B.b_varargout_1 = Sub_nav3_187.getLatestMessage(&nav3_B.b_varargout_2_c);

  /* Outputs for Enabled SubSystem: '<S93>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S96>/Enable'
   */
  if (nav3_B.b_varargout_1) {
    nav3_B.In1_a = nav3_B.b_varargout_2_c;
  }

  /* End of MATLABSystem: '<S93>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S93>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S85>/Live_Subscriber' */

  /* MATLAB Function: '<S92>/Quaternion_2_Euler' */
  nav3_Quaternion_2_Euler(nav3_B.In1_a.Orientation.X, nav3_B.In1_a.Orientation.Y,
    nav3_B.In1_a.Orientation.Z, nav3_B.In1_a.Orientation.W,
    &nav3_B.sf_Quaternion_2_Euler);

  /* Outputs for Atomic SubSystem: '<S84>/Live_Subscriber1' */
  /* MATLABSystem: '<S89>/SourceBlock' */
  nav3_B.b_varargout_1 = Sub_nav3_166.getLatestMessage(&nav3_B.b_varargout_2_f);

  /* Outputs for Enabled SubSystem: '<S89>/Enabled Subsystem' */
  nav3_EnabledSubsystem(nav3_B.b_varargout_1, &nav3_B.b_varargout_2_f,
                        &nav3_B.EnabledSubsystem_d);

  /* End of Outputs for SubSystem: '<S89>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S84>/Live_Subscriber1' */

  /* BusAssignment: '<S4>/Bus Assignment' incorporates:
   *  Constant: '<S75>/Constant'
   *  DataStoreRead: '<S4>/goalHeading'
   *  DataTypeConversion: '<S4>/Data Type Conversion'
   */
  nav3_B.BusAssignment = nav3_P.Constant_Value;
  nav3_B.BusAssignment.Data[0] = nav3_B.currentState;
  nav3_B.BusAssignment.Data[1] = nav3_B.currentWaypoint[0];
  nav3_B.BusAssignment.Data[2] = nav3_B.currentWaypoint[1];
  nav3_B.BusAssignment.Data[3] = nav3_B.linearVel;
  nav3_B.BusAssignment.Data[4] = nav3_B.angularVel;
  nav3_B.BusAssignment.Data[5] = nav3_DW.goalHeading[0];
  nav3_B.BusAssignment.Data[8] = nav3_B.Switch[0];
  nav3_B.BusAssignment.Data[6] = nav3_DW.goalHeading[1];
  nav3_B.BusAssignment.Data[9] = nav3_B.Switch[1];
  nav3_B.BusAssignment.Data[7] = nav3_DW.goalHeading[2];
  nav3_B.BusAssignment.Data[10] = nav3_B.Switch[2];
  nav3_B.BusAssignment.Data_SL_Info.CurrentLength = nav3_B.ProbeWidth;

  /* Outputs for Atomic SubSystem: '<S4>/Publish' */
  /* MATLABSystem: '<S76>/SinkBlock' */
  Pub_nav3_516.publish(&nav3_B.BusAssignment);

  /* End of Outputs for SubSystem: '<S4>/Publish' */

  /* BusAssignment: '<S106>/Bus Assignment' incorporates:
   *  Constant: '<S106>/Constant'
   */
  nav3_B.BusAssignment_n.Data = nav3_P.Constant_Value_oa;

  /* Outputs for Atomic SubSystem: '<S106>/Publish' */
  /* MATLABSystem: '<S108>/SinkBlock' */
  Pub_nav3_494.publish(&nav3_B.BusAssignment_n);

  /* End of Outputs for SubSystem: '<S106>/Publish' */

  /* Outputs for Atomic SubSystem: '<S106>/Subscribe' */
  /* MATLABSystem: '<S110>/SourceBlock' incorporates:
   *  Inport: '<S111>/In1'
   */
  nav3_B.b_varargout_1 = Sub_nav3_499.getLatestMessage(&nav3_B.BusAssignment_n);

  /* Outputs for Enabled SubSystem: '<S110>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S111>/Enable'
   */
  if (nav3_B.b_varargout_1) {
    nav3_B.In1_d = nav3_B.BusAssignment_n;
  }

  /* End of MATLABSystem: '<S110>/SourceBlock' */
  /* End of Outputs for SubSystem: '<S110>/Enabled Subsystem' */
  /* End of Outputs for SubSystem: '<S106>/Subscribe' */

  /* Outputs for Enabled SubSystem: '<S106>/RESET_IMU' incorporates:
   *  EnablePort: '<S109>/Enable'
   */
  if (nav3_B.In1_d.Data > 0) {
    /* MATLABSystem: '<S109>/Set Parameter' incorporates:
     *  Constant: '<S109>/Constant'
     *  Sum: '<S109>/Sum'
     */
    ParamSet_nav3_475.set_parameter(nav3_P.Constant_Value_c -
      nav3_B.sf_Quaternion_2_Euler.theta);
  }

  /* End of Outputs for SubSystem: '<S106>/RESET_IMU' */

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.2, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  nav3_M->Timing.clockTick0++;
  if (!nav3_M->Timing.clockTick0) {
    nav3_M->Timing.clockTickH0++;
  }
}

/* Model initialize function */
void nav3_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* block I/O */
  (void) memset((static_cast<void *>(&nav3_B)), 0,
                sizeof(B_nav3_T));

  /* states (dwork) */
  (void) memset(static_cast<void *>(&nav3_DW), 0,
                sizeof(DW_nav3_T));

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

    static const char_T tmp_h[21] = { '/', 's', 't', 'a', 't', 'e', '/', 'i',
      'n', 'i', 't', 'i', 'a', 'l', 'i', 's', 'a', 't', 'i', 'o', 'n' };

    static const char_T tmp_i[13] = { '/', 's', 't', 'a', 't', 'e', '/', 'm',
      'a', 'n', 'u', 'a', 'l' };

    static const char_T tmp_j[14] = { '/', 's', 't', 'a', 't', 'e', '/', 'r',
      'e', 'v', 'e', 'r', 's', 'e' };

    static const char_T tmp_k[14] = { '/', 's', 't', 'a', 't', 'e', '/', 'w',
      'a', 'i', 't', 'i', 'n', 'g' };

    static const char_T tmp_l[18] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'r',
      'o', 'b', 'o', 't', 'L', 'e', 'n', 'g', 't', 'h' };

    static const char_T tmp_m[16] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'w',
      'h', 'e', 'e', 'l', 'B', 'a', 's', 'e' };

    static const char_T tmp_n[18] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'w',
      'h', 'e', 'e', 'l', 'R', 'a', 'd', 'i', 'u', 's' };

    static const char_T tmp_o[19] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'L',
      'i', 'd', 'a', 'r', '2', 'R', 'o', 'b', 'o', 't', 'Y' };

    static const char_T tmp_p[19] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'L',
      'i', 'd', 'a', 'r', '2', 'R', 'o', 'b', 'o', 't', 'X' };

    static const char_T tmp_q[23] = { '/', 'r', 'o', 'b', 'o', 't', '/', 'L',
      'i', 'd', 'a', 'r', '2', 'R', 'o', 'b', 'o', 't', 'T', 'h', 'e', 't', 'a'
    };

    static const char_T tmp_r[12] = { '/', 's', 'y', 's', 't', 'e', 'm', '/',
      'l', 'i', 'v', 'e' };

    static const char_T tmp_s[34] = { '/', 's', 'l', 'a', 'm', 'w', 'a', 'r',
      'e', '_', 'r', 'o', 's', '_', 's', 'd', 'k', '_', 's', 'e', 'r', 'v', 'e',
      'r', '_', 'n', 'o', 'd', 'e', '/', 'o', 'd', 'o', 'm' };

    static const char_T tmp_t[10] = { '/', 'a', 'm', 'c', 'l', '_', 'p', 'o',
      's', 'e' };

    static const char_T tmp_u[19] = { '/', 'a', 'm', 'r', '/', 's', 't', 'a',
      't', 'u', 's', '/', 'v', 'o', 'l', 't', 'a', 'g', 'e' };

    static const char_T tmp_v[14] = { '/', 'z', 'l', 'a', 'c', '8', '0', '1',
      '5', 'd', '/', 'r', 'p', 'm' };

    static const char_T tmp_w[12] = { '/', 's', 'i', 'm', '_', 'c', 'm', 'd',
      '_', 'v', 'e', 'l' };

    static const char_T tmp_x[24] = { '/', 't', 'i', 'n', 'k', 'e', 'r', 'f',
      'o', 'r', 'g', 'e', '_', 'i', 'm', 'u', '_', 'r', 'o', 's', '/', 'i', 'm',
      'u' };

    static const char_T tmp_y[19] = { '/', 'a', 'm', 'r', '/', 's', 't', 'a',
      't', 'u', 's', '/', 'c', 'u', 'r', 'r', 'e', 'n', 't' };

    static const char_T tmp_z[8] = { '/', 'a', 'm', 'r', '/', 'l', 'o', 'g' };

    static const char_T tmp_10[20] = { '/', 'a', 'm', 'r', '/', 's', 't', 'a',
      't', 'u', 's', '/', 'r', 'e', 's', 'e', 't', 'i', 'm', 'u' };

    /* Start for MATLABSystem: '<S77>/Get Parameter1' */
    nav3_DW.obj_gl.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_f = true;
    nav3_DW.obj_gl.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      nav3_B.cv9[i] = tmp_4[i];
    }

    nav3_B.cv9[18] = '\x00';
    ParamGet_nav3_242.initialize(nav3_B.cv9);
    ParamGet_nav3_242.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_242.set_initial_value(0.0);
    nav3_DW.obj_gl.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S77>/Get Parameter1' */

    /* Start for MATLABSystem: '<S77>/Get Parameter2' */
    nav3_DW.obj_j.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_bh = true;
    nav3_DW.obj_j.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_5[i];
    }

    tmp[10] = '\x00';
    ParamGet_nav3_274.initialize(tmp);
    ParamGet_nav3_274.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_274.set_initial_value(0.0);
    nav3_DW.obj_j.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S77>/Get Parameter2' */

    /* Start for MATLABSystem: '<S77>/Get Parameter3' */
    nav3_DW.obj_a.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_c5 = true;
    nav3_DW.obj_a.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_6[i];
    }

    tmp[10] = '\x00';
    ParamGet_nav3_275.initialize(tmp);
    ParamGet_nav3_275.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_275.set_initial_value(0.0);
    nav3_DW.obj_a.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S77>/Get Parameter3' */

    /* Start for MATLABSystem: '<S77>/Get Parameter4' */
    nav3_DW.obj_b.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_bhy = true;
    nav3_DW.obj_b.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_7[i];
    }

    tmp[10] = '\x00';
    ParamGet_nav3_276.initialize(tmp);
    ParamGet_nav3_276.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_276.set_initial_value(0.0);
    nav3_DW.obj_b.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S77>/Get Parameter4' */

    /* Start for MATLABSystem: '<S78>/Get Parameter6' */
    nav3_DW.obj_ek.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_et = true;
    nav3_DW.obj_ek.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      nav3_B.cv2[i] = tmp_8[i];
    }

    nav3_B.cv2[29] = '\x00';
    ParamGet_nav3_541.initialize(nav3_B.cv2);
    ParamGet_nav3_541.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_541.set_initial_value(0.25);
    nav3_DW.obj_ek.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S78>/Get Parameter6' */

    /* Start for MATLABSystem: '<S78>/Get Parameter5' */
    nav3_DW.obj_lb.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_a = true;
    nav3_DW.obj_lb.isInitialized = 1;
    for (i = 0; i < 33; i++) {
      nav3_B.cv1[i] = tmp_9[i];
    }

    nav3_B.cv1[33] = '\x00';
    ParamGet_nav3_368.initialize(nav3_B.cv1);
    ParamGet_nav3_368.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_368.set_initial_value(0.0);
    nav3_DW.obj_lb.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S78>/Get Parameter5' */

    /* Start for MATLABSystem: '<S78>/Get Parameter2' */
    nav3_DW.obj_kg.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_gi = true;
    nav3_DW.obj_kg.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      nav3_B.cv4[i] = tmp_a[i];
    }

    nav3_B.cv4[24] = '\x00';
    ParamGet_nav3_326.initialize(nav3_B.cv4);
    ParamGet_nav3_326.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_326.set_initial_value(0.0);
    nav3_DW.obj_kg.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S78>/Get Parameter2' */

    /* Start for MATLABSystem: '<S78>/Get Parameter3' */
    nav3_DW.obj_ef.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_mn = true;
    nav3_DW.obj_ef.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      nav3_B.cv4[i] = tmp_b[i];
    }

    nav3_B.cv4[24] = '\x00';
    ParamGet_nav3_327.initialize(nav3_B.cv4);
    ParamGet_nav3_327.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_327.set_initial_value(0.0);
    nav3_DW.obj_ef.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S78>/Get Parameter3' */

    /* Start for MATLABSystem: '<S78>/Get Parameter1' */
    nav3_DW.obj_g.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_ow = true;
    nav3_DW.obj_g.isInitialized = 1;
    for (i = 0; i < 28; i++) {
      nav3_B.cv3[i] = tmp_c[i];
    }

    nav3_B.cv3[28] = '\x00';
    ParamGet_nav3_325.initialize(nav3_B.cv3);
    ParamGet_nav3_325.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_325.set_initial_value(0.0);
    nav3_DW.obj_g.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S78>/Get Parameter1' */

    /* Start for MATLABSystem: '<S78>/Get Parameter4' */
    nav3_DW.obj_dr.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_oi = true;
    nav3_DW.obj_dr.isInitialized = 1;
    for (i = 0; i < 34; i++) {
      nav3_B.cv[i] = tmp_d[i];
    }

    nav3_B.cv[34] = '\x00';
    ParamGet_nav3_365.initialize(nav3_B.cv);
    ParamGet_nav3_365.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_365.set_initial_value(0.0);
    nav3_DW.obj_dr.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S78>/Get Parameter4' */

    /* Start for MATLABSystem: '<S78>/Get Parameter' */
    nav3_DW.obj_p.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_lk = true;
    nav3_DW.obj_p.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      nav3_B.cv2[i] = tmp_e[i];
    }

    nav3_B.cv2[29] = '\x00';
    ParamGet_nav3_237.initialize(nav3_B.cv2);
    ParamGet_nav3_237.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_237.set_initial_value(0.0);
    nav3_DW.obj_p.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S78>/Get Parameter' */

    /* Start for MATLABSystem: '<S80>/Get Parameter4' */
    nav3_DW.obj_n.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_gn = true;
    nav3_DW.obj_n.isInitialized = 1;
    for (i = 0; i < 17; i++) {
      nav3_B.cv10[i] = tmp_f[i];
    }

    nav3_B.cv10[17] = '\x00';
    ParamGet_nav3_295.initialize(nav3_B.cv10);
    ParamGet_nav3_295.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_295.set_initial_value(0);
    nav3_DW.obj_n.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S80>/Get Parameter4' */

    /* Start for MATLABSystem: '<S80>/Get Parameter3' */
    nav3_DW.obj_l.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_ou = true;
    nav3_DW.obj_l.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      nav3_B.cv11[i] = tmp_g[i];
    }

    nav3_B.cv11[16] = '\x00';
    ParamGet_nav3_294.initialize(nav3_B.cv11);
    ParamGet_nav3_294.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_294.set_initial_value(0);
    nav3_DW.obj_l.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S80>/Get Parameter3' */

    /* Start for MATLABSystem: '<S80>/Get Parameter2' */
    nav3_DW.obj_e.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_c = true;
    nav3_DW.obj_e.isInitialized = 1;
    for (i = 0; i < 21; i++) {
      nav3_B.cv6[i] = tmp_h[i];
    }

    nav3_B.cv6[21] = '\x00';
    ParamGet_nav3_293.initialize(nav3_B.cv6);
    ParamGet_nav3_293.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_293.set_initial_value(0);
    nav3_DW.obj_e.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S80>/Get Parameter2' */

    /* Start for MATLABSystem: '<S80>/Get Parameter1' */
    nav3_DW.obj_f.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_e = true;
    nav3_DW.obj_f.isInitialized = 1;
    for (i = 0; i < 13; i++) {
      tmp_0[i] = tmp_i[i];
    }

    tmp_0[13] = '\x00';
    ParamGet_nav3_292.initialize(tmp_0);
    ParamGet_nav3_292.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_292.set_initial_value(0);
    nav3_DW.obj_f.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S80>/Get Parameter1' */

    /* Start for MATLABSystem: '<S80>/Get Parameter5' */
    nav3_DW.obj_e0.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_p = true;
    nav3_DW.obj_e0.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      nav3_B.cv12[i] = tmp_j[i];
    }

    nav3_B.cv12[14] = '\x00';
    ParamGet_nav3_303.initialize(nav3_B.cv12);
    ParamGet_nav3_303.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_303.set_initial_value(0);
    nav3_DW.obj_e0.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S80>/Get Parameter5' */

    /* Start for MATLABSystem: '<S80>/Get Parameter6' */
    nav3_DW.obj_lq.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_ci = true;
    nav3_DW.obj_lq.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      nav3_B.cv12[i] = tmp_k[i];
    }

    nav3_B.cv12[14] = '\x00';
    ParamGet_nav3_306.initialize(nav3_B.cv12);
    ParamGet_nav3_306.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_306.set_initial_value(0);
    nav3_DW.obj_lq.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S80>/Get Parameter6' */

    /* Start for MATLABSystem: '<S79>/Get Parameter' */
    nav3_DW.obj_m.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_ot = true;
    nav3_DW.obj_m.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      nav3_B.cv9[i] = tmp_l[i];
    }

    nav3_B.cv9[18] = '\x00';
    ParamGet_nav3_340.initialize(nav3_B.cv9);
    ParamGet_nav3_340.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_340.set_initial_value(0.0);
    nav3_DW.obj_m.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S79>/Get Parameter' */

    /* Start for MATLABSystem: '<S79>/Get Parameter1' */
    nav3_DW.obj_h.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_l5 = true;
    nav3_DW.obj_h.isInitialized = 1;
    for (i = 0; i < 16; i++) {
      nav3_B.cv11[i] = tmp_m[i];
    }

    nav3_B.cv11[16] = '\x00';
    ParamGet_nav3_341.initialize(nav3_B.cv11);
    ParamGet_nav3_341.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_341.set_initial_value(0.0);
    nav3_DW.obj_h.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S79>/Get Parameter1' */

    /* Start for MATLABSystem: '<S79>/Get Parameter2' */
    nav3_DW.obj_my.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_i = true;
    nav3_DW.obj_my.isInitialized = 1;
    for (i = 0; i < 18; i++) {
      nav3_B.cv9[i] = tmp_n[i];
    }

    nav3_B.cv9[18] = '\x00';
    ParamGet_nav3_342.initialize(nav3_B.cv9);
    ParamGet_nav3_342.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_342.set_initial_value(0.0);
    nav3_DW.obj_my.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S79>/Get Parameter2' */

    /* Start for MATLABSystem: '<S79>/Get Parameter3' */
    nav3_DW.obj_d.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_be = true;
    nav3_DW.obj_d.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3_B.cv8[i] = tmp_o[i];
    }

    nav3_B.cv8[19] = '\x00';
    ParamGet_nav3_357.initialize(nav3_B.cv8);
    ParamGet_nav3_357.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_357.set_initial_value(0.0);
    nav3_DW.obj_d.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S79>/Get Parameter3' */

    /* Start for MATLABSystem: '<S79>/Get Parameter4' */
    nav3_DW.obj_lm.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_ba = true;
    nav3_DW.obj_lm.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3_B.cv8[i] = tmp_p[i];
    }

    nav3_B.cv8[19] = '\x00';
    ParamGet_nav3_358.initialize(nav3_B.cv8);
    ParamGet_nav3_358.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_358.set_initial_value(0.0);
    nav3_DW.obj_lm.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S79>/Get Parameter4' */

    /* Start for MATLABSystem: '<S79>/Get Parameter5' */
    nav3_DW.obj_k.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_k = true;
    nav3_DW.obj_k.isInitialized = 1;
    for (i = 0; i < 23; i++) {
      nav3_B.cv5[i] = tmp_q[i];
    }

    nav3_B.cv5[23] = '\x00';
    ParamGet_nav3_359.initialize(nav3_B.cv5);
    ParamGet_nav3_359.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_359.set_initial_value(0.0);
    nav3_DW.obj_k.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S79>/Get Parameter5' */

    /* Start for MATLABSystem: '<S81>/Get Parameter5' */
    nav3_DW.obj.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_oj = true;
    nav3_DW.obj.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_r[i];
    }

    tmp_1[12] = '\x00';
    ParamGet_nav3_381.initialize(tmp_1);
    ParamGet_nav3_381.initialize_error_codes(0, 1, 2, 3);
    ParamGet_nav3_381.set_initial_value(0);
    nav3_DW.obj.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S81>/Get Parameter5' */

    /* Start for Atomic SubSystem: '<S86>/Live_Subscriber' */
    /* Start for MATLABSystem: '<S98>/SourceBlock' */
    nav3_DW.obj_gt.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_g = true;
    nav3_DW.obj_gt.isInitialized = 1;
    for (i = 0; i < 34; i++) {
      nav3_B.cv[i] = tmp_s[i];
    }

    nav3_B.cv[34] = '\x00';
    Sub_nav3_202.createSubscriber(nav3_B.cv, 1);
    nav3_DW.obj_gt.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S98>/SourceBlock' */
    /* End of Start for SubSystem: '<S86>/Live_Subscriber' */

    /* Start for Atomic SubSystem: '<S87>/Simulation_Subscriber' */
    /* Start for MATLABSystem: '<S103>/SourceBlock' */
    nav3_DW.obj_fy.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_b = true;
    nav3_DW.obj_fy.isInitialized = 1;
    for (i = 0; i < 10; i++) {
      tmp[i] = tmp_t[i];
    }

    tmp[10] = '\x00';
    Sub_nav3_223.createSubscriber(tmp, 1);
    nav3_DW.obj_fy.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S103>/SourceBlock' */
    /* End of Start for SubSystem: '<S87>/Simulation_Subscriber' */

    /* Start for Atomic SubSystem: '<S84>/Live_Subscriber' */
    /* Start for MATLABSystem: '<S88>/SourceBlock' */
    nav3_DW.obj_nh.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_o = true;
    nav3_DW.obj_nh.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3_B.cv8[i] = tmp_u[i];
    }

    nav3_B.cv8[19] = '\x00';
    Sub_nav3_165.createSubscriber(nav3_B.cv8, 1);
    nav3_DW.obj_nh.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S88>/SourceBlock' */
    /* End of Start for SubSystem: '<S84>/Live_Subscriber' */

    /* Start for Atomic SubSystem: '<S3>/Subscribe' */
    /* Start for MATLABSystem: '<S73>/SourceBlock' */
    nav3_DW.obj_mf.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_pp = true;
    nav3_DW.obj_mf.isInitialized = 1;
    tmp_2[0] = '/';
    tmp_2[1] = 'j';
    tmp_2[2] = 'o';
    tmp_2[3] = 'y';
    tmp_2[4] = '\x00';
    Sub_nav3_555.createSubscriber(tmp_2, 1);
    nav3_DW.obj_mf.isSetupComplete = true;

    /* End of Start for SubSystem: '<S3>/Subscribe' */

    /* Start for Enabled SubSystem: '<S1>/Real_Actuation' */
    /* Start for Atomic SubSystem: '<S10>/Publish' */
    /* Start for MATLABSystem: '<S13>/SinkBlock' */
    nav3_DW.obj_ft.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_j = true;
    nav3_DW.obj_ft.isInitialized = 1;
    for (i = 0; i < 14; i++) {
      nav3_B.cv12[i] = tmp_v[i];
    }

    nav3_B.cv12[14] = '\x00';
    Pub_nav3_442.createPublisher(nav3_B.cv12, 1);
    nav3_DW.obj_ft.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S13>/SinkBlock' */
    /* End of Start for SubSystem: '<S10>/Publish' */
    /* End of Start for SubSystem: '<S1>/Real_Actuation' */

    /* Start for Enabled SubSystem: '<S1>/Simulation_Actuation' */
    /* Start for Atomic SubSystem: '<S9>/Publish' */
    /* Start for MATLABSystem: '<S15>/SinkBlock' */
    nav3_DW.obj_gv.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_gc = true;
    nav3_DW.obj_gv.isInitialized = 1;
    for (i = 0; i < 12; i++) {
      tmp_1[i] = tmp_w[i];
    }

    tmp_1[12] = '\x00';
    Pub_nav3_422.createPublisher(tmp_1, 1);
    nav3_DW.obj_gv.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S15>/SinkBlock' */
    /* End of Start for SubSystem: '<S9>/Publish' */
    /* End of Start for SubSystem: '<S1>/Simulation_Actuation' */

    /* Start for Atomic SubSystem: '<S85>/Live_Subscriber' */
    /* Start for MATLABSystem: '<S93>/SourceBlock' */
    nav3_DW.obj_dm.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_d = true;
    nav3_DW.obj_dm.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      nav3_B.cv4[i] = tmp_x[i];
    }

    nav3_B.cv4[24] = '\x00';
    Sub_nav3_187.createSubscriber(nav3_B.cv4, 1);
    nav3_DW.obj_dm.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S93>/SourceBlock' */
    /* End of Start for SubSystem: '<S85>/Live_Subscriber' */

    /* Start for Atomic SubSystem: '<S84>/Live_Subscriber1' */
    /* Start for MATLABSystem: '<S89>/SourceBlock' */
    nav3_DW.obj_n4.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_da = true;
    nav3_DW.obj_n4.isInitialized = 1;
    for (i = 0; i < 19; i++) {
      nav3_B.cv8[i] = tmp_y[i];
    }

    nav3_B.cv8[19] = '\x00';
    Sub_nav3_166.createSubscriber(nav3_B.cv8, 1);
    nav3_DW.obj_n4.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S89>/SourceBlock' */
    /* End of Start for SubSystem: '<S84>/Live_Subscriber1' */

    /* Start for Probe: '<S4>/Probe Width' */
    nav3_B.ProbeWidth = 11U;

    /* Start for Atomic SubSystem: '<S4>/Publish' */
    /* Start for MATLABSystem: '<S76>/SinkBlock' */
    nav3_DW.obj_o.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_gs = true;
    nav3_DW.obj_o.isInitialized = 1;
    for (i = 0; i < 8; i++) {
      tmp_3[i] = tmp_z[i];
    }

    tmp_3[8] = '\x00';
    Pub_nav3_516.createPublisher(tmp_3, 1);
    nav3_DW.obj_o.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S76>/SinkBlock' */
    /* End of Start for SubSystem: '<S4>/Publish' */

    /* Start for Atomic SubSystem: '<S106>/Publish' */
    /* Start for MATLABSystem: '<S108>/SinkBlock' */
    nav3_DW.obj_jx.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_m = true;
    nav3_DW.obj_jx.isInitialized = 1;
    for (i = 0; i < 20; i++) {
      nav3_B.cv7[i] = tmp_10[i];
    }

    nav3_B.cv7[20] = '\x00';
    Pub_nav3_494.createPublisher(nav3_B.cv7, 1);
    nav3_DW.obj_jx.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S108>/SinkBlock' */
    /* End of Start for SubSystem: '<S106>/Publish' */

    /* Start for Atomic SubSystem: '<S106>/Subscribe' */
    /* Start for MATLABSystem: '<S110>/SourceBlock' */
    nav3_DW.obj_jp.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty = true;
    nav3_DW.obj_jp.isInitialized = 1;
    for (i = 0; i < 20; i++) {
      nav3_B.cv7[i] = tmp_10[i];
    }

    nav3_B.cv7[20] = '\x00';
    Sub_nav3_499.createSubscriber(nav3_B.cv7, 1);
    nav3_DW.obj_jp.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S110>/SourceBlock' */
    /* End of Start for SubSystem: '<S106>/Subscribe' */

    /* Start for Enabled SubSystem: '<S106>/RESET_IMU' */
    /* Start for MATLABSystem: '<S109>/Set Parameter' */
    nav3_DW.obj_c.matlabCodegenIsDeleted = false;
    nav3_DW.objisempty_l = true;
    nav3_DW.obj_c.isInitialized = 1;
    for (i = 0; i < 29; i++) {
      nav3_B.cv2[i] = tmp_e[i];
    }

    nav3_B.cv2[29] = '\x00';
    ParamSet_nav3_475.initialize(nav3_B.cv2);
    nav3_DW.obj_c.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S109>/Set Parameter' */
    /* End of Start for SubSystem: '<S106>/RESET_IMU' */

    /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
    nav3_DW.AUTONOMOUS_CONTROL_STATE = nav3_P.DataStoreMemory_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory1' */
    nav3_DW.EMERGENCY_STATE = nav3_P.DataStoreMemory1_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory10' */
    nav3_DW.I = nav3_P.DataStoreMemory10_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory11' */
    nav3_DW.D = nav3_P.DataStoreMemory11_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory13' */
    nav3_DW.WAYPOINT_PRECISION = nav3_P.DataStoreMemory13_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory12' */
    nav3_DW.goalHeading[0] = nav3_P.DataStoreMemory12_InitialValue[0];

    /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
    nav3_DW.InitialPose[0] = nav3_P.DataStoreMemory14_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory12' */
    nav3_DW.goalHeading[1] = nav3_P.DataStoreMemory12_InitialValue[1];

    /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
    nav3_DW.InitialPose[1] = nav3_P.DataStoreMemory14_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory12' */
    nav3_DW.goalHeading[2] = nav3_P.DataStoreMemory12_InitialValue[2];

    /* Start for DataStoreMemory: '<Root>/Data Store Memory14' */
    nav3_DW.InitialPose[2] = nav3_P.DataStoreMemory14_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory2' */
    nav3_DW.INITIALISATION_STATE = nav3_P.DataStoreMemory2_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory3' */
    nav3_DW.MANUAL_CONTROL_STATE = nav3_P.DataStoreMemory3_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory4' */
    nav3_DW.REVERSE_STATE = nav3_P.DataStoreMemory4_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory5' */
    nav3_DW.WAITING_STATE = nav3_P.DataStoreMemory5_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory6' */
    nav3_DW.LOOKAHEAD_DISTANCE = nav3_P.DataStoreMemory6_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory7' */
    nav3_DW.MAX_ANGULAR_VEL = nav3_P.DataStoreMemory7_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory8' */
    nav3_DW.MAX_LINEAR_VEL = nav3_P.DataStoreMemory8_InitialValue;

    /* Start for DataStoreMemory: '<Root>/Data Store Memory9' */
    nav3_DW.P = nav3_P.DataStoreMemory9_InitialValue;
  }

  /* SystemInitialize for Atomic SubSystem: '<S86>/Live_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S98>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S101>/Out1' */
  nav3_B.In1 = nav3_P.Out1_Y0;

  /* End of SystemInitialize for SubSystem: '<S98>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S86>/Live_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S87>/Simulation_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S103>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S105>/Out1' */
  nav3_B.In1_p = nav3_P.Out1_Y0_m;

  /* End of SystemInitialize for SubSystem: '<S103>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S87>/Simulation_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S84>/Live_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S88>/Enabled Subsystem' */
  nav3_EnabledSubsystem_Init(&nav3_B.EnabledSubsystem_m,
    &nav3_P.EnabledSubsystem_m);

  /* End of SystemInitialize for SubSystem: '<S88>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S84>/Live_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S3>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S73>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S74>/Out1' */
  nav3_B.In1_i = nav3_P.Out1_Y0_ix;

  /* End of SystemInitialize for SubSystem: '<S73>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S3>/Subscribe' */
  nav3_DW.is_AUTONOMOUS_CONTROL = nav3_IN_NO_ACTIVE_CHILD;
  nav3_DW.is_REVERSE = nav3_IN_NO_ACTIVE_CHILD;
  nav3_DW.is_active_c3_nav3 = 0U;
  nav3_DW.is_c3_nav3 = nav3_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for Chart: '<S2>/State_Machine' incorporates:
   *  SubSystem: '<S16>/checkAtGoal'
   */
  nav3_checkAtGoal_Init(&nav3_B.checkAtGoal, &nav3_P.checkAtGoal);

  /* SystemInitialize for Chart: '<S2>/State_Machine' incorporates:
   *  SubSystem: '<S16>/PID'
   */
  nav3_PID_Init(&nav3_B.PID, &nav3_DW.PID, &nav3_P.PID);

  /* SystemInitialize for Atomic SubSystem: '<S85>/Live_Subscriber' */
  /* SystemInitialize for Enabled SubSystem: '<S93>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S96>/Out1' */
  nav3_B.In1_a = nav3_P.Out1_Y0_i;

  /* End of SystemInitialize for SubSystem: '<S93>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S85>/Live_Subscriber' */

  /* SystemInitialize for Atomic SubSystem: '<S84>/Live_Subscriber1' */
  /* SystemInitialize for Enabled SubSystem: '<S89>/Enabled Subsystem' */
  nav3_EnabledSubsystem_Init(&nav3_B.EnabledSubsystem_d,
    &nav3_P.EnabledSubsystem_d);

  /* End of SystemInitialize for SubSystem: '<S89>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S84>/Live_Subscriber1' */

  /* SystemInitialize for Atomic SubSystem: '<S106>/Subscribe' */
  /* SystemInitialize for Enabled SubSystem: '<S110>/Enabled Subsystem' */
  /* SystemInitialize for Outport: '<S111>/Out1' */
  nav3_B.In1_d = nav3_P.Out1_Y0_c;

  /* End of SystemInitialize for SubSystem: '<S110>/Enabled Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S106>/Subscribe' */

  /* Enable for Chart: '<S2>/State_Machine' incorporates:
   *  SubSystem: '<S16>/PID'
   */
  nav3_PID_Enable(&nav3_DW.PID);
}

/* Model terminate function */
void nav3_terminate(void)
{
  /* Terminate for MATLABSystem: '<S77>/Get Parameter1' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_gl);

  /* Terminate for MATLABSystem: '<S77>/Get Parameter2' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_j);

  /* Terminate for MATLABSystem: '<S77>/Get Parameter3' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_a);

  /* Terminate for MATLABSystem: '<S77>/Get Parameter4' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_b);

  /* Terminate for MATLABSystem: '<S78>/Get Parameter6' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_ek);

  /* Terminate for MATLABSystem: '<S78>/Get Parameter5' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_lb);

  /* Terminate for MATLABSystem: '<S78>/Get Parameter2' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_kg);

  /* Terminate for MATLABSystem: '<S78>/Get Parameter3' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_ef);

  /* Terminate for MATLABSystem: '<S78>/Get Parameter1' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_g);

  /* Terminate for MATLABSystem: '<S78>/Get Parameter4' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_dr);

  /* Terminate for MATLABSystem: '<S78>/Get Parameter' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_p);

  /* Terminate for MATLABSystem: '<S80>/Get Parameter4' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_n);

  /* Terminate for MATLABSystem: '<S80>/Get Parameter3' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_l);

  /* Terminate for MATLABSystem: '<S80>/Get Parameter2' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_e);

  /* Terminate for MATLABSystem: '<S80>/Get Parameter1' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_f);

  /* Terminate for MATLABSystem: '<S80>/Get Parameter5' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_e0);

  /* Terminate for MATLABSystem: '<S80>/Get Parameter6' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_lq);

  /* Terminate for MATLABSystem: '<S79>/Get Parameter' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_m);

  /* Terminate for MATLABSystem: '<S79>/Get Parameter1' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_h);

  /* Terminate for MATLABSystem: '<S79>/Get Parameter2' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_my);

  /* Terminate for MATLABSystem: '<S79>/Get Parameter3' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_d);

  /* Terminate for MATLABSystem: '<S79>/Get Parameter4' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_lm);

  /* Terminate for MATLABSystem: '<S79>/Get Parameter5' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj_k);

  /* Terminate for MATLABSystem: '<S81>/Get Parameter5' */
  matlabCodegenHandle_matlab_kgkc(&nav3_DW.obj);

  /* Terminate for Atomic SubSystem: '<S86>/Live_Subscriber' */
  /* Terminate for MATLABSystem: '<S98>/SourceBlock' */
  matlabCodegenHandle_matlabCo_kg(&nav3_DW.obj_gt);

  /* End of Terminate for SubSystem: '<S86>/Live_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S87>/Simulation_Subscriber' */
  /* Terminate for MATLABSystem: '<S103>/SourceBlock' */
  matlabCodegenHandle_matlabCo_kg(&nav3_DW.obj_fy);

  /* End of Terminate for SubSystem: '<S87>/Simulation_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S84>/Live_Subscriber' */
  /* Terminate for MATLABSystem: '<S88>/SourceBlock' */
  matlabCodegenHandle_matlabCo_kg(&nav3_DW.obj_nh);

  /* End of Terminate for SubSystem: '<S84>/Live_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S3>/Subscribe' */
  /* Terminate for MATLABSystem: '<S73>/SourceBlock' */
  matlabCodegenHandle_matlabCo_kg(&nav3_DW.obj_mf);

  /* End of Terminate for SubSystem: '<S3>/Subscribe' */

  /* Terminate for Enabled SubSystem: '<S1>/Real_Actuation' */
  /* Terminate for Atomic SubSystem: '<S10>/Publish' */
  /* Terminate for MATLABSystem: '<S13>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3_DW.obj_ft);

  /* End of Terminate for SubSystem: '<S10>/Publish' */
  /* End of Terminate for SubSystem: '<S1>/Real_Actuation' */

  /* Terminate for Enabled SubSystem: '<S1>/Simulation_Actuation' */
  /* Terminate for Atomic SubSystem: '<S9>/Publish' */
  /* Terminate for MATLABSystem: '<S15>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3_DW.obj_gv);

  /* End of Terminate for SubSystem: '<S9>/Publish' */
  /* End of Terminate for SubSystem: '<S1>/Simulation_Actuation' */

  /* Terminate for Atomic SubSystem: '<S85>/Live_Subscriber' */
  /* Terminate for MATLABSystem: '<S93>/SourceBlock' */
  matlabCodegenHandle_matlabCo_kg(&nav3_DW.obj_dm);

  /* End of Terminate for SubSystem: '<S85>/Live_Subscriber' */

  /* Terminate for Atomic SubSystem: '<S84>/Live_Subscriber1' */
  /* Terminate for MATLABSystem: '<S89>/SourceBlock' */
  matlabCodegenHandle_matlabCo_kg(&nav3_DW.obj_n4);

  /* End of Terminate for SubSystem: '<S84>/Live_Subscriber1' */

  /* Terminate for Atomic SubSystem: '<S4>/Publish' */
  /* Terminate for MATLABSystem: '<S76>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3_DW.obj_o);

  /* End of Terminate for SubSystem: '<S4>/Publish' */

  /* Terminate for Atomic SubSystem: '<S106>/Publish' */
  /* Terminate for MATLABSystem: '<S108>/SinkBlock' */
  matlabCodegenHandle_matlabCodeg(&nav3_DW.obj_jx);

  /* End of Terminate for SubSystem: '<S106>/Publish' */

  /* Terminate for Atomic SubSystem: '<S106>/Subscribe' */
  /* Terminate for MATLABSystem: '<S110>/SourceBlock' */
  matlabCodegenHandle_matlabCo_kg(&nav3_DW.obj_jp);

  /* End of Terminate for SubSystem: '<S106>/Subscribe' */

  /* Terminate for Enabled SubSystem: '<S106>/RESET_IMU' */
  /* Terminate for MATLABSystem: '<S109>/Set Parameter' */
  nav3_matlabCodegenHa_en(&nav3_DW.obj_c);

  /* End of Terminate for SubSystem: '<S106>/RESET_IMU' */
}
