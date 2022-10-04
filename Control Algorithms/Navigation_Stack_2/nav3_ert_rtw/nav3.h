/*
 * nav3.h
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

#ifndef RTW_HEADER_nav3_h_
#define RTW_HEADER_nav3_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "nav3_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block signals for system '<S16>/checkAtGoal' */
typedef struct {
  boolean_T LessThan;                  /* '<S18>/Less Than' */
} B_checkAtGoal_nav3_T;

/* Block signals for system '<S17>/Normalise_Goal_Heading' */
typedef struct {
  real_T bearingAngle;                 /* '<S17>/Normalise_Goal_Heading' */
} B_Normalise_Goal_Heading_nav3_T;

/* Block signals for system '<S16>/PID' */
typedef struct {
  real_T Saturation;                   /* '<S63>/Saturation' */
  B_Normalise_Goal_Heading_nav3_T sf_Normalise_Heading;/* '<S17>/Normalise_Heading' */
  B_Normalise_Goal_Heading_nav3_T sf_Normalise_Goal_Heading;/* '<S17>/Normalise_Goal_Heading' */
} B_PID_nav3_T;

/* Block states (default storage) for system '<S16>/PID' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S56>/Integrator' */
  real_T UD_DSTATE;                    /* '<S49>/UD' */
  real_T Integrator_PREV_U;            /* '<S56>/Integrator' */
  uint32_T PID_ELAPS_T[2];             /* '<S16>/PID' */
  uint32_T PID_PREV_T[2];              /* '<S16>/PID' */
  uint8_T Integrator_SYSTEM_ENABLE;    /* '<S56>/Integrator' */
  boolean_T PID_RESET_ELAPS_T;         /* '<S16>/PID' */
} DW_PID_nav3_T;

/* Block signals for system '<S88>/Enabled Subsystem' */
typedef struct {
  SL_Bus_nav3_std_msgs_Float32 In1;    /* '<S90>/In1' */
} B_EnabledSubsystem_nav3_T;

/* Block signals for system '<S92>/Quaternion_2_Euler' */
typedef struct {
  real_T theta;                        /* '<S92>/Quaternion_2_Euler' */
  real_T aSinInput;
  real_T c_b;
} B_Quaternion_2_Euler_nav3_T;

/* Block signals (default storage) */
typedef struct {
  SL_Bus_nav3_std_msgs_Float64MultiArray BusAssignment;/* '<S4>/Bus Assignment' */
  SL_Bus_nav3_nav_msgs_Odometry In1;   /* '<S101>/In1' */
  SL_Bus_nav3_nav_msgs_Odometry b_varargout_2;
  SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped In1_p;/* '<S105>/In1' */
  SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped b_varargout_2_m;
  SL_Bus_nav3_sensor_msgs_Imu In1_a;   /* '<S96>/In1' */
  SL_Bus_nav3_sensor_msgs_Imu b_varargout_2_c;
  SL_Bus_nav3_sensor_msgs_Joy In1_i;   /* '<S74>/In1' */
  SL_Bus_nav3_sensor_msgs_Joy b_varargout_2_k;
  creal_T eigVec[16];
  creal_T At[16];
  real_T b[18];
  real_T b_c[18];
  real_T rotm[16];
  real_T b_V[16];
  real_T b_A[16];
  real_T W2LOTransform[9];
  real_T World2RobotOdom[9];           /* '<S78>/MATLAB Function' */
  real_T Lidar2Robot[9];               /* '<S79>/MATLAB Function' */
  real_T W2RTransform_tmp[9];
  real_T W2LOTransform_b[9];
  real_T DataTypeConversion[8];
  creal_T eigVal[4];
  creal_T beta1[4];
  creal_T work1[4];
  SL_Bus_nav3_geometry_msgs_Twist BusAssignment_l;/* '<S10>/Bus Assignment' */
  real_T b_waypoints_data[6];
  char_T cv[35];
  char_T cv1[34];
  real_T varargin_1[4];
  real_T work[4];
  real_T rworka[4];
  real_T work_p[4];
  real_T b_waypoints_data_c[4];
  char_T cv2[30];
  char_T cv3[29];
  char_T cv4[25];
  char_T cv5[24];
  real_T Switch[3];                    /* '<S6>/Switch' */
  real_T tau[3];
  real_T v[3];
  real_T b_v[3];
  char_T cv6[22];
  char_T cv7[21];
  char_T cv8[20];
  char_T cv9[19];
  char_T cv10[18];
  char_T cv11[17];
  int32_T rscale[4];
  int8_T b_I[16];
  real_T currentWaypoint[2];           /* '<S2>/State_Machine' */
  real_T goalWaypoint[2];              /* '<S2>/State_Machine' */
  real_T dv[2];
  real_T dv1[2];
  real_T lookaheadStartPt[2];
  real_T lookaheadEndPt[2];
  real_T b_waypoints[2];
  real_T b_waypoints_f[2];
  real_T refPt[2];
  real_T controller_ProjectionPoint[2];
  real_T b_waypoints_g[2];
  creal_T s;
  creal_T ctemp;
  creal_T ad22;
  creal_T ascale;
  char_T cv12[15];
  real_T linearVel;                    /* '<S2>/State_Machine' */
  real_T angularVel;                   /* '<S2>/State_Machine' */
  real_T pose[3];                      /* '<S2>/State_Machine' */
  real_T goalTolerance;                /* '<S2>/State_Machine' */
  real_T pose_f[3];                    /* '<S2>/State_Machine' */
  real_T desiredHeading;               /* '<S2>/State_Machine' */
  real_T P;                            /* '<S2>/State_Machine' */
  real_T I;                            /* '<S2>/State_Machine' */
  real_T D;                            /* '<S2>/State_Machine' */
  real_T K12;
  real_T K34;
  real_T value;
  real_T value_g;
  real_T Product;                      /* '<S11>/Product' */
  real_T rtb_Lidar2Robot_tmp;
  real_T quat_idx_2;
  real_T quat_idx_3;
  real_T W2LOTransform_tmp;
  real_T W2LOTransform_tmp_m;
  real_T W2LOTransform_tmp_n;
  real_T colnorm;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T anrm;
  real_T anrmto;
  real_T mul;
  real_T d;
  real_T d1;
  real_T atmp_re;
  real_T atmp_im;
  real_T anorm;
  real_T b_atol;
  real_T absxr;
  real_T absxi;
  real_T ar;
  real_T ai;
  real_T t1_re;
  real_T t1_im;
  real_T shift_re;
  real_T shift_im;
  real_T shift_im_p;
  real_T eshift_re;
  real_T eshift_im;
  real_T scale_l;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T x;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  real_T a;
  real_T d2;
  real_T minDistance;
  real_T overshootDist;
  real_T alpha;
  real_T alpha_j;
  real_T v12;
  real_T v12_d;
  real_T scale_g;
  real_T absxk_l;
  real_T t_d;
  real_T tst;
  real_T htmp1;
  real_T htmp2;
  real_T ba;
  real_T aa;
  real_T h12;
  real_T h21s;
  real_T unusedU1;
  real_T unusedU2;
  real_T unusedU3;
  real_T p;
  real_T bcmax;
  real_T bcmis;
  real_T scale_d;
  real_T z;
  real_T tau_l;
  real_T anorm_o;
  real_T ascale_b;
  real_T temp;
  real_T acoeff;
  real_T scale_n;
  real_T dmin;
  real_T f_y;
  real_T salpha_re;
  real_T salpha_im;
  real_T work2_idx_2_im;
  real_T work2_idx_3_re;
  real_T work2_idx_3_im;
  real_T alpha1;
  real_T xnorm;
  real_T c;
  real_T alpha_b;
  real_T lookaheadEndPt_idx_1;
  real_T lookaheadEndPt_idx_0;
  real_T scale_ln;
  real_T g2_h;
  real_T f2s_b;
  real_T di_d;
  real_T x_e;
  real_T fs_re_b;
  real_T fs_im_j;
  real_T gs_re_f;
  real_T gs_im_a;
  real_T tmp;
  real_T scale_j;
  real_T sumsq;
  real_T temp1;
  real_T temp2;
  real_T cfromc;
  real_T ctoc;
  real_T cfrom1;
  real_T cto1;
  real_T mul_j;
  real_T xnorm_o;
  real_T scale_ny;
  real_T absxk_i;
  real_T t_o;
  SL_Bus_nav3_std_msgs_Int16 In1_d;    /* '<S111>/In1' */
  boolean_T b_n[6];
  int32_T currentState;                /* '<S2>/State_Machine' */
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T value_m;
  int32_T i;
  int32_T W2LOTransform_tmp_c;
  int32_T W2LOTransform_tmp_md;
  int32_T b_j;
  int32_T i_m;
  int32_T ihi;
  int32_T i_j;
  int32_T jcol;
  int32_T c_i;
  int32_T k;
  int32_T ii;
  int32_T nzcount;
  int32_T jj;
  int32_T j;
  int32_T ifirst;
  int32_T istart;
  int32_T ilast;
  int32_T ilastm1;
  int32_T iiter;
  int32_T jp1;
  int32_T jiter;
  int32_T i_h;
  int32_T ctemp_tmp;
  int32_T ctemp_tmp_tmp;
  int32_T count;
  int32_T rescaledir;
  int32_T i_c;
  int32_T waypointLine_tmp;
  int32_T trueCount;
  int32_T c_i_c;
  int32_T lookaheadIdx;
  int32_T k_p;
  int32_T i_p;
  int32_T L;
  int32_T k_a;
  int32_T m;
  int32_T nr;
  int32_T hoffset;
  int32_T j_e;
  int32_T b_j_a;
  int32_T c_j;
  int32_T ix;
  int32_T s_tmp;
  int32_T b_a;
  int32_T c_if;
  int32_T i_l;
  int32_T c_j_o;
  int32_T e_jr;
  int32_T c_x_tmp;
  int32_T c_x_tmp_tmp;
  int32_T d_re_tmp;
  int32_T work2_idx_1_re_tmp;
  int32_T d_re_tmp_tmp;
  int32_T knt;
  int32_T lastc;
  int32_T rowleft;
  int32_T iac;
  int32_T g;
  int32_T b_ia;
  int32_T jy;
  int32_T b_ix;
  int32_T lastv;
  int32_T lastc_o;
  int32_T coltop;
  int32_T ix_i;
  int32_T iac_f;
  int32_T d_i;
  int32_T b_ia_f;
  int32_T jy_g;
  int32_T trueCount_c;
  int32_T partialTrueCount;
  int32_T u0;
  int32_T u1;
  int32_T offset;
  int32_T j_o;
  int32_T j_l;
  int32_T b_m;
  int32_T i_mj;
  int32_T reAij_tmp;
  int32_T i1;
  int32_T knt_c;
  int32_T c_k;
  uint32_T ProbeWidth;                 /* '<S4>/Probe Width' */
  SL_Bus_nav3_std_msgs_Float32 b_varargout_2_f;
  int8_T c_data[3];
  int8_T c_data_p[2];
  SL_Bus_nav3_std_msgs_Int16 BusAssignment_n;/* '<S106>/Bus Assignment' */
  boolean_T b_varargout_1;
  boolean_T p_e;
  boolean_T ilascl;
  boolean_T found;
  boolean_T failed;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  boolean_T p_o;
  boolean_T b_p;
  boolean_T goto150;
  boolean_T b_h;
  boolean_T lscalea;
  boolean_T lscaleb;
  boolean_T firstNonZero;
  boolean_T notdone;
  B_Quaternion_2_Euler_nav3_T sf_Quaternion_2_Euler_g;/* '<S102>/Quaternion_2_Euler' */
  B_Quaternion_2_Euler_nav3_T sf_Quaternion_2_Euler_o;/* '<S97>/Quaternion_2_Euler' */
  B_Quaternion_2_Euler_nav3_T sf_Quaternion_2_Euler;/* '<S92>/Quaternion_2_Euler' */
  B_EnabledSubsystem_nav3_T EnabledSubsystem_d;/* '<S89>/Enabled Subsystem' */
  B_EnabledSubsystem_nav3_T EnabledSubsystem_m;/* '<S88>/Enabled Subsystem' */
  B_PID_nav3_T PID;                    /* '<S16>/PID' */
  B_checkAtGoal_nav3_T checkAtGoal;    /* '<S16>/checkAtGoal' */
} B_nav3_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  ros_slros_internal_block_GetP_T obj; /* '<S81>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_n;/* '<S80>/Get Parameter4' */
  ros_slros_internal_block_GetP_T obj_l;/* '<S80>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_e;/* '<S80>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_f;/* '<S80>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_e0;/* '<S80>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_lq;/* '<S80>/Get Parameter6' */
  ros_slros_internal_block_GetP_T obj_m;/* '<S79>/Get Parameter' */
  ros_slros_internal_block_GetP_T obj_h;/* '<S79>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_my;/* '<S79>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_d;/* '<S79>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_lm;/* '<S79>/Get Parameter4' */
  ros_slros_internal_block_GetP_T obj_k;/* '<S79>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_ek;/* '<S78>/Get Parameter6' */
  ros_slros_internal_block_GetP_T obj_lb;/* '<S78>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_kg;/* '<S78>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_ef;/* '<S78>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_g;/* '<S78>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_dr;/* '<S78>/Get Parameter4' */
  ros_slros_internal_block_GetP_T obj_p;/* '<S78>/Get Parameter' */
  ros_slros_internal_block_GetP_T obj_gl;/* '<S77>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_j;/* '<S77>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_a;/* '<S77>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_b;/* '<S77>/Get Parameter4' */
  ros_slros_internal_block_Publ_T obj_jx;/* '<S108>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_o;/* '<S76>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_gv;/* '<S15>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_ft;/* '<S13>/SinkBlock' */
  ros_slros_internal_block_Subs_T obj_jp;/* '<S110>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_fy;/* '<S103>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_gt;/* '<S98>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_dm;/* '<S93>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_n4;/* '<S89>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_nh;/* '<S88>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_mf;/* '<S73>/SourceBlock' */
  ros_slros_internal_block_SetP_T obj_c;/* '<S109>/Set Parameter' */
  real_T I;                            /* '<Root>/Data Store Memory10' */
  real_T D;                            /* '<Root>/Data Store Memory11' */
  real_T goalHeading[3];               /* '<Root>/Data Store Memory12' */
  real_T WAYPOINT_PRECISION;           /* '<Root>/Data Store Memory13' */
  real_T InitialPose[3];               /* '<Root>/Data Store Memory14' */
  real_T LOOKAHEAD_DISTANCE;           /* '<Root>/Data Store Memory6' */
  real_T MAX_ANGULAR_VEL;              /* '<Root>/Data Store Memory7' */
  real_T MAX_LINEAR_VEL;               /* '<Root>/Data Store Memory8' */
  real_T P;                            /* '<Root>/Data Store Memory9' */
  real_T last;                         /* '<S2>/State_Machine' */
  real_T finishedMission;              /* '<S2>/State_Machine' */
  real_T count;                        /* '<S2>/State_Machine' */
  real_T currentGoalHeading;           /* '<S2>/State_Machine' */
  real_T waypointLine[6];              /* '<S2>/State_Machine' */
  real_T all_waypoints[20];            /* '<S2>/State_Machine' */
  real_T finishedMissionCount;         /* '<S2>/State_Machine' */
  real_T waypointLinePID[4];           /* '<S2>/State_Machine' */
  int32_T AUTONOMOUS_CONTROL_STATE;    /* '<Root>/Data Store Memory' */
  int32_T EMERGENCY_STATE;             /* '<Root>/Data Store Memory1' */
  int32_T INITIALISATION_STATE;        /* '<Root>/Data Store Memory2' */
  int32_T MANUAL_CONTROL_STATE;        /* '<Root>/Data Store Memory3' */
  int32_T REVERSE_STATE;               /* '<Root>/Data Store Memory4' */
  int32_T WAITING_STATE;               /* '<Root>/Data Store Memory5' */
  uint8_T is_active_c3_nav3;           /* '<S2>/State_Machine' */
  uint8_T is_c3_nav3;                  /* '<S2>/State_Machine' */
  uint8_T is_REVERSE;                  /* '<S2>/State_Machine' */
  uint8_T is_AUTONOMOUS_CONTROL;       /* '<S2>/State_Machine' */
  boolean_T objisempty;                /* '<S110>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S109>/Set Parameter' */
  boolean_T objisempty_m;              /* '<S108>/SinkBlock' */
  boolean_T objisempty_b;              /* '<S103>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S98>/SourceBlock' */
  boolean_T objisempty_d;              /* '<S93>/SourceBlock' */
  boolean_T objisempty_da;             /* '<S89>/SourceBlock' */
  boolean_T objisempty_o;              /* '<S88>/SourceBlock' */
  boolean_T objisempty_oj;             /* '<S81>/Get Parameter5' */
  boolean_T objisempty_gn;             /* '<S80>/Get Parameter4' */
  boolean_T objisempty_ou;             /* '<S80>/Get Parameter3' */
  boolean_T objisempty_c;              /* '<S80>/Get Parameter2' */
  boolean_T objisempty_e;              /* '<S80>/Get Parameter1' */
  boolean_T objisempty_p;              /* '<S80>/Get Parameter5' */
  boolean_T objisempty_ci;             /* '<S80>/Get Parameter6' */
  boolean_T objisempty_ot;             /* '<S79>/Get Parameter' */
  boolean_T objisempty_l5;             /* '<S79>/Get Parameter1' */
  boolean_T objisempty_i;              /* '<S79>/Get Parameter2' */
  boolean_T objisempty_be;             /* '<S79>/Get Parameter3' */
  boolean_T objisempty_ba;             /* '<S79>/Get Parameter4' */
  boolean_T objisempty_k;              /* '<S79>/Get Parameter5' */
  boolean_T objisempty_et;             /* '<S78>/Get Parameter6' */
  boolean_T objisempty_a;              /* '<S78>/Get Parameter5' */
  boolean_T objisempty_gi;             /* '<S78>/Get Parameter2' */
  boolean_T objisempty_mn;             /* '<S78>/Get Parameter3' */
  boolean_T objisempty_ow;             /* '<S78>/Get Parameter1' */
  boolean_T objisempty_oi;             /* '<S78>/Get Parameter4' */
  boolean_T objisempty_lk;             /* '<S78>/Get Parameter' */
  boolean_T objisempty_f;              /* '<S77>/Get Parameter1' */
  boolean_T objisempty_bh;             /* '<S77>/Get Parameter2' */
  boolean_T objisempty_c5;             /* '<S77>/Get Parameter3' */
  boolean_T objisempty_bhy;            /* '<S77>/Get Parameter4' */
  boolean_T objisempty_gs;             /* '<S76>/SinkBlock' */
  boolean_T objisempty_pp;             /* '<S73>/SourceBlock' */
  boolean_T objisempty_gc;             /* '<S15>/SinkBlock' */
  boolean_T objisempty_j;              /* '<S13>/SinkBlock' */
  DW_PID_nav3_T PID;                   /* '<S16>/PID' */
} DW_nav3_T;

/* Parameters for system: '<S16>/checkAtGoal' */
struct P_checkAtGoal_nav3_T_ {
  boolean_T atGoal_Y0;                 /* Computed Parameter: atGoal_Y0
                                        * Referenced by: '<S18>/atGoal'
                                        */
};

/* Parameters for system: '<S16>/PID' */
struct P_PID_nav3_T_ {
  real_T DiscreteVaryingPID_Differentiat;
                              /* Mask Parameter: DiscreteVaryingPID_Differentiat
                               * Referenced by: '<S49>/UD'
                               */
  real_T DiscreteVaryingPID_InitialCondi;
                              /* Mask Parameter: DiscreteVaryingPID_InitialCondi
                               * Referenced by: '<S56>/Integrator'
                               */
  real_T DiscreteVaryingPID_LowerSaturat;
                              /* Mask Parameter: DiscreteVaryingPID_LowerSaturat
                               * Referenced by:
                               *   '<S47>/DeadZone'
                               *   '<S63>/Saturation'
                               */
  real_T DiscreteVaryingPID_UpperSaturat;
                              /* Mask Parameter: DiscreteVaryingPID_UpperSaturat
                               * Referenced by:
                               *   '<S47>/DeadZone'
                               *   '<S63>/Saturation'
                               */
  real_T angVel_Y0;                    /* Computed Parameter: angVel_Y0
                                        * Referenced by: '<S17>/angVel'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S47>/Constant1'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S56>/Integrator'
                                        */
  real_T Tsamp_WtEt;                   /* Computed Parameter: Tsamp_WtEt
                                        * Referenced by: '<S51>/Tsamp'
                                        */
  real_T ZeroGain_Gain;                /* Expression: 0
                                        * Referenced by: '<S47>/ZeroGain'
                                        */
};

/* Parameters for system: '<S88>/Enabled Subsystem' */
struct P_EnabledSubsystem_nav3_T_ {
  SL_Bus_nav3_std_msgs_Float32 Out1_Y0;/* Computed Parameter: Out1_Y0
                                        * Referenced by: '<S90>/Out1'
                                        */
};

/* Parameters (default storage) */
struct P_nav3_T_ {
  SL_Bus_nav3_std_msgs_Float64MultiArray Constant_Value;/* Computed Parameter: Constant_Value
                                                         * Referenced by: '<S75>/Constant'
                                                         */
  SL_Bus_nav3_nav_msgs_Odometry Out1_Y0;/* Computed Parameter: Out1_Y0
                                         * Referenced by: '<S101>/Out1'
                                         */
  SL_Bus_nav3_nav_msgs_Odometry Constant_Value_a;/* Computed Parameter: Constant_Value_a
                                                  * Referenced by: '<S98>/Constant'
                                                  */
  SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped Out1_Y0_m;/* Computed Parameter: Out1_Y0_m
                                                                 * Referenced by: '<S105>/Out1'
                                                                 */
  SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped Constant_Value_b;/* Computed Parameter: Constant_Value_b
                                                                      * Referenced by: '<S103>/Constant'
                                                                      */
  SL_Bus_nav3_sensor_msgs_Imu Out1_Y0_i;/* Computed Parameter: Out1_Y0_i
                                         * Referenced by: '<S96>/Out1'
                                         */
  SL_Bus_nav3_sensor_msgs_Imu Constant_Value_j;/* Computed Parameter: Constant_Value_j
                                                * Referenced by: '<S93>/Constant'
                                                */
  SL_Bus_nav3_sensor_msgs_Joy Out1_Y0_ix;/* Computed Parameter: Out1_Y0_ix
                                          * Referenced by: '<S74>/Out1'
                                          */
  SL_Bus_nav3_sensor_msgs_Joy Constant_Value_l;/* Computed Parameter: Constant_Value_l
                                                * Referenced by: '<S73>/Constant'
                                                */
  SL_Bus_nav3_geometry_msgs_Twist Constant_Value_n;/* Computed Parameter: Constant_Value_n
                                                    * Referenced by: '<S12>/Constant'
                                                    */
  SL_Bus_nav3_geometry_msgs_Twist Constant_Value_o;/* Computed Parameter: Constant_Value_o
                                                    * Referenced by: '<S14>/Constant'
                                                    */
  SL_Bus_nav3_std_msgs_Float32 Constant_Value_ag;/* Computed Parameter: Constant_Value_ag
                                                  * Referenced by: '<S88>/Constant'
                                                  */
  SL_Bus_nav3_std_msgs_Float32 Constant_Value_av;/* Computed Parameter: Constant_Value_av
                                                  * Referenced by: '<S89>/Constant'
                                                  */
  SL_Bus_nav3_std_msgs_Int16 Out1_Y0_c;/* Computed Parameter: Out1_Y0_c
                                        * Referenced by: '<S111>/Out1'
                                        */
  SL_Bus_nav3_std_msgs_Int16 Constant_Value_i;/* Computed Parameter: Constant_Value_i
                                               * Referenced by: '<S110>/Constant'
                                               */
  SL_Bus_nav3_std_msgs_Int16 Constant_Value_e;/* Computed Parameter: Constant_Value_e
                                               * Referenced by: '<S107>/Constant'
                                               */
  real_T Gain_Gain;                    /* Expression: pi
                                        * Referenced by: '<S11>/Gain'
                                        */
  real_T State_Machine_reversePose[3]; /* Expression: [0; 0; -pi/2]
                                        * Referenced by: '<S2>/State_Machine'
                                        */
  real_T Constant_Value_c;             /* Expression: pi/2
                                        * Referenced by: '<S109>/Constant'
                                        */
  real_T Linear_Accel_Gain;            /* Expression: 0.25
                                        * Referenced by: '<S3>/Linear_Accel'
                                        */
  real_T waypoints_Value[18];
  /* Expression: [-0.5,1; -0.5, 3; 0.5,3; 0.5,5; -0.5,5; -0.5,3; 0.5,3; 0.5,1; 0,1]
   * Referenced by: '<S2>/waypoints'
   */
  real_T DataStoreMemory10_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory10'
                                         */
  real_T DataStoreMemory11_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory11'
                                         */
  real_T DataStoreMemory12_InitialValue[3];/* Expression: [0 0 0]
                                            * Referenced by: '<Root>/Data Store Memory12'
                                            */
  real_T DataStoreMemory13_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory13'
                                         */
  real_T DataStoreMemory14_InitialValue;/* Expression: 0
                                         * Referenced by: '<Root>/Data Store Memory14'
                                         */
  real_T DataStoreMemory6_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory6'
                                        */
  real_T DataStoreMemory7_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory7'
                                        */
  real_T DataStoreMemory8_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory8'
                                        */
  real_T DataStoreMemory9_InitialValue;/* Expression: 0
                                        * Referenced by: '<Root>/Data Store Memory9'
                                        */
  int32_T Switch_Threshold;            /* Computed Parameter: Switch_Threshold
                                        * Referenced by: '<S6>/Switch'
                                        */
  int32_T DataStoreMemory_InitialValue;
                             /* Computed Parameter: DataStoreMemory_InitialValue
                              * Referenced by: '<Root>/Data Store Memory'
                              */
  int32_T DataStoreMemory1_InitialValue;
                            /* Computed Parameter: DataStoreMemory1_InitialValue
                             * Referenced by: '<Root>/Data Store Memory1'
                             */
  int32_T DataStoreMemory2_InitialValue;
                            /* Computed Parameter: DataStoreMemory2_InitialValue
                             * Referenced by: '<Root>/Data Store Memory2'
                             */
  int32_T DataStoreMemory3_InitialValue;
                            /* Computed Parameter: DataStoreMemory3_InitialValue
                             * Referenced by: '<Root>/Data Store Memory3'
                             */
  int32_T DataStoreMemory4_InitialValue;
                            /* Computed Parameter: DataStoreMemory4_InitialValue
                             * Referenced by: '<Root>/Data Store Memory4'
                             */
  int32_T DataStoreMemory5_InitialValue;
                            /* Computed Parameter: DataStoreMemory5_InitialValue
                             * Referenced by: '<Root>/Data Store Memory5'
                             */
  int16_T Constant_Value_oa;           /* Computed Parameter: Constant_Value_oa
                                        * Referenced by: '<S106>/Constant'
                                        */
  P_EnabledSubsystem_nav3_T EnabledSubsystem_d;/* '<S89>/Enabled Subsystem' */
  P_EnabledSubsystem_nav3_T EnabledSubsystem_m;/* '<S88>/Enabled Subsystem' */
  P_PID_nav3_T PID;                    /* '<S16>/PID' */
  P_checkAtGoal_nav3_T checkAtGoal;    /* '<S16>/checkAtGoal' */
};

/* Real-time Model Data Structure */
struct tag_RTM_nav3_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
  } Timing;
};

/* Block parameters (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern P_nav3_T nav3_P;

#ifdef __cplusplus

}
#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern B_nav3_T nav3_B;

#ifdef __cplusplus

}
#endif

/* Block states (default storage) */
extern DW_nav3_T nav3_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void nav3_initialize(void);
  extern void nav3_step(void);
  extern void nav3_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_nav3_T *const nav3_M;

#ifdef __cplusplus

}
#endif

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S49>/DTDup' : Unused code path elimination
 * Block '<S2>/waypoints1' : Unused code path elimination
 * Block '<Root>/CurrentPose' : Unused code path elimination
 * Block '<Root>/CurrentState' : Unused code path elimination
 * Block '<Root>/CurrentWaypoint' : Unused code path elimination
 * Block '<Root>/Input Commands' : Unused code path elimination
 * Block '<S4>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S4>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S4>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S4>/Data Type Conversion4' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'nav3'
 * '<S1>'   : 'nav3/Actuation_Subsystem'
 * '<S2>'   : 'nav3/Control_Subsystem'
 * '<S3>'   : 'nav3/Joystick_Subsystem'
 * '<S4>'   : 'nav3/Logging_Subsystem'
 * '<S5>'   : 'nav3/Parameter_Server_Subsystem'
 * '<S6>'   : 'nav3/Perception_Subsystem'
 * '<S7>'   : 'nav3/Service_Request_Subsystem'
 * '<S8>'   : 'nav3/Actuation_Subsystem/Real_Actuation'
 * '<S9>'   : 'nav3/Actuation_Subsystem/Simulation_Actuation'
 * '<S10>'  : 'nav3/Actuation_Subsystem/Real_Actuation/Actuator'
 * '<S11>'  : 'nav3/Actuation_Subsystem/Real_Actuation/Forward Kinematics'
 * '<S12>'  : 'nav3/Actuation_Subsystem/Real_Actuation/Actuator/Blank Message'
 * '<S13>'  : 'nav3/Actuation_Subsystem/Real_Actuation/Actuator/Publish'
 * '<S14>'  : 'nav3/Actuation_Subsystem/Simulation_Actuation/Blank Message'
 * '<S15>'  : 'nav3/Actuation_Subsystem/Simulation_Actuation/Publish'
 * '<S16>'  : 'nav3/Control_Subsystem/State_Machine'
 * '<S17>'  : 'nav3/Control_Subsystem/State_Machine/PID'
 * '<S18>'  : 'nav3/Control_Subsystem/State_Machine/checkAtGoal'
 * '<S19>'  : 'nav3/Control_Subsystem/State_Machine/PID/Calculate_Error'
 * '<S20>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID'
 * '<S21>'  : 'nav3/Control_Subsystem/State_Machine/PID/Normalise_Goal_Heading'
 * '<S22>'  : 'nav3/Control_Subsystem/State_Machine/PID/Normalise_Heading'
 * '<S23>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Anti-windup'
 * '<S24>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/D Gain'
 * '<S25>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter'
 * '<S26>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter ICs'
 * '<S27>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/I Gain'
 * '<S28>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain'
 * '<S29>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain Fdbk'
 * '<S30>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator'
 * '<S31>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator ICs'
 * '<S32>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Copy'
 * '<S33>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Gain'
 * '<S34>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/P Copy'
 * '<S35>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Parallel P Gain'
 * '<S36>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Reset Signal'
 * '<S37>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation'
 * '<S38>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation Fdbk'
 * '<S39>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum'
 * '<S40>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum Fdbk'
 * '<S41>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode'
 * '<S42>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode Sum'
 * '<S43>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Integral'
 * '<S44>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Ngain'
 * '<S45>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/postSat Signal'
 * '<S46>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/preSat Signal'
 * '<S47>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Anti-windup/Disc. Clamping Parallel'
 * '<S48>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/D Gain/External Parameters'
 * '<S49>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter/Differentiator'
 * '<S50>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter/Differentiator/Tsamp'
 * '<S51>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S52>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter ICs/Internal IC - Differentiator'
 * '<S53>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/I Gain/External Parameters'
 * '<S54>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain/Passthrough'
 * '<S55>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain Fdbk/Disabled'
 * '<S56>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator/Discrete'
 * '<S57>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator ICs/Internal IC'
 * '<S58>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Copy/Disabled wSignal Specification'
 * '<S59>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Gain/Passthrough'
 * '<S60>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/P Copy/Disabled'
 * '<S61>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Parallel P Gain/External Parameters'
 * '<S62>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Reset Signal/Disabled'
 * '<S63>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation/Enabled'
 * '<S64>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation Fdbk/Disabled'
 * '<S65>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum/Sum_PID'
 * '<S66>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum Fdbk/Disabled'
 * '<S67>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode/Disabled'
 * '<S68>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode Sum/Passthrough'
 * '<S69>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Integral/Passthrough'
 * '<S70>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Ngain/Passthrough'
 * '<S71>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/postSat Signal/Forward_Path'
 * '<S72>'  : 'nav3/Control_Subsystem/State_Machine/PID/Discrete Varying PID/preSat Signal/Forward_Path'
 * '<S73>'  : 'nav3/Joystick_Subsystem/Subscribe'
 * '<S74>'  : 'nav3/Joystick_Subsystem/Subscribe/Enabled Subsystem'
 * '<S75>'  : 'nav3/Logging_Subsystem/Blank Message'
 * '<S76>'  : 'nav3/Logging_Subsystem/Publish'
 * '<S77>'  : 'nav3/Parameter_Server_Subsystem/Control_Parameters'
 * '<S78>'  : 'nav3/Parameter_Server_Subsystem/Experiment_Parameters'
 * '<S79>'  : 'nav3/Parameter_Server_Subsystem/Robot_Parameters'
 * '<S80>'  : 'nav3/Parameter_Server_Subsystem/State_Parameters'
 * '<S81>'  : 'nav3/Parameter_Server_Subsystem/System_Parameters'
 * '<S82>'  : 'nav3/Parameter_Server_Subsystem/Experiment_Parameters/MATLAB Function'
 * '<S83>'  : 'nav3/Parameter_Server_Subsystem/Robot_Parameters/MATLAB Function'
 * '<S84>'  : 'nav3/Perception_Subsystem/Battery_Subsystem'
 * '<S85>'  : 'nav3/Perception_Subsystem/IMU_Subsystem'
 * '<S86>'  : 'nav3/Perception_Subsystem/SLAM_Subsystem'
 * '<S87>'  : 'nav3/Perception_Subsystem/Simulation_Subsystem'
 * '<S88>'  : 'nav3/Perception_Subsystem/Battery_Subsystem/Live_Subscriber'
 * '<S89>'  : 'nav3/Perception_Subsystem/Battery_Subsystem/Live_Subscriber1'
 * '<S90>'  : 'nav3/Perception_Subsystem/Battery_Subsystem/Live_Subscriber/Enabled Subsystem'
 * '<S91>'  : 'nav3/Perception_Subsystem/Battery_Subsystem/Live_Subscriber1/Enabled Subsystem'
 * '<S92>'  : 'nav3/Perception_Subsystem/IMU_Subsystem/Live_Pose_Parser'
 * '<S93>'  : 'nav3/Perception_Subsystem/IMU_Subsystem/Live_Subscriber'
 * '<S94>'  : 'nav3/Perception_Subsystem/IMU_Subsystem/Live_Pose_Parser/Quaternion_2_Euler'
 * '<S95>'  : 'nav3/Perception_Subsystem/IMU_Subsystem/Live_Pose_Parser/TrueNorthToIMU1'
 * '<S96>'  : 'nav3/Perception_Subsystem/IMU_Subsystem/Live_Subscriber/Enabled Subsystem'
 * '<S97>'  : 'nav3/Perception_Subsystem/SLAM_Subsystem/Live_Pose_Parser'
 * '<S98>'  : 'nav3/Perception_Subsystem/SLAM_Subsystem/Live_Subscriber'
 * '<S99>'  : 'nav3/Perception_Subsystem/SLAM_Subsystem/Live_Pose_Parser/Quaternion_2_Euler'
 * '<S100>' : 'nav3/Perception_Subsystem/SLAM_Subsystem/Live_Pose_Parser/World to Robot Transform'
 * '<S101>' : 'nav3/Perception_Subsystem/SLAM_Subsystem/Live_Subscriber/Enabled Subsystem'
 * '<S102>' : 'nav3/Perception_Subsystem/Simulation_Subsystem/Sim_Pose_Parser'
 * '<S103>' : 'nav3/Perception_Subsystem/Simulation_Subsystem/Simulation_Subscriber'
 * '<S104>' : 'nav3/Perception_Subsystem/Simulation_Subsystem/Sim_Pose_Parser/Quaternion_2_Euler'
 * '<S105>' : 'nav3/Perception_Subsystem/Simulation_Subsystem/Simulation_Subscriber/Enabled Subsystem'
 * '<S106>' : 'nav3/Service_Request_Subsystem/RESET_IMU'
 * '<S107>' : 'nav3/Service_Request_Subsystem/RESET_IMU/Blank Message'
 * '<S108>' : 'nav3/Service_Request_Subsystem/RESET_IMU/Publish'
 * '<S109>' : 'nav3/Service_Request_Subsystem/RESET_IMU/RESET_IMU'
 * '<S110>' : 'nav3/Service_Request_Subsystem/RESET_IMU/Subscribe'
 * '<S111>' : 'nav3/Service_Request_Subsystem/RESET_IMU/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_nav3_h_ */
