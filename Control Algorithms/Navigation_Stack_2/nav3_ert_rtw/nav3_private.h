/*
 * nav3_private.h
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

#ifndef RTW_HEADER_nav3_private_h_
#define RTW_HEADER_nav3_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "nav3.h"

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern int32_T div_nzp_s32(int32_T numerator, int32_T denominator);
extern void nav3_checkAtGoal_Init(B_checkAtGoal_nav3_T *localB,
  P_checkAtGoal_nav3_T *localP);
extern void nav3_checkAtGoal(const real_T rtu_pose[3], const real_T
  rtu_goalWaypoint[2], real_T rtu_goalTolerance, B_checkAtGoal_nav3_T *localB);
extern void nav3_Normalise_Goal_Heading(real_T rtu_headingAngle,
  B_Normalise_Goal_Heading_nav3_T *localB);
extern void nav3_PID_Init(B_PID_nav3_T *localB, DW_PID_nav3_T *localDW,
  P_PID_nav3_T *localP);
extern void nav3_PID_Enable(DW_PID_nav3_T *localDW);
extern void nav3_PID(RT_MODEL_nav3_T * const nav3_M, const real_T rtu_pose[3],
                     real_T rtu_desiredHeading, real_T rtu_P, real_T rtu_I,
                     real_T rtu_D, B_PID_nav3_T *localB, DW_PID_nav3_T *localDW,
                     P_PID_nav3_T *localP);
extern void nav3_EnabledSubsystem_Init(B_EnabledSubsystem_nav3_T *localB,
  P_EnabledSubsystem_nav3_T *localP);
extern void nav3_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_nav3_std_msgs_Float32 *rtu_In1, B_EnabledSubsystem_nav3_T *localB);
extern void nav3_Quaternion_2_Euler(real_T rtu_x, real_T rtu_y, real_T rtu_z,
  real_T rtu_w, B_Quaternion_2_Euler_nav3_T *localB);

#endif                                 /* RTW_HEADER_nav3_private_h_ */
