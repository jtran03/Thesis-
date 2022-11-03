/*
 * nav3v2.h
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

#ifndef RTW_HEADER_nav3v2_h_
#define RTW_HEADER_nav3v2_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros_initialize.h"
#include "nav3v2_types.h"

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

/* Block signals for system '<S19>/checkAtGoal' */
typedef struct {
  boolean_T LessThan;                  /* '<S21>/Less Than' */
} B_checkAtGoal_nav3v2_T;

/* Block signals for system '<S20>/Normalise_Goal_Heading' */
typedef struct {
  real_T bearingAngle;                 /* '<S20>/Normalise_Goal_Heading' */
} B_Normalise_Goal_Heading_nav3_T;

/* Block signals for system '<S19>/PID' */
typedef struct {
  real_T Saturation;                   /* '<S66>/Saturation' */
  B_Normalise_Goal_Heading_nav3_T sf_Normalise_Heading;/* '<S20>/Normalise_Heading' */
  B_Normalise_Goal_Heading_nav3_T sf_Normalise_Goal_Heading;/* '<S20>/Normalise_Goal_Heading' */
} B_PID_nav3v2_T;

/* Block states (default storage) for system '<S19>/PID' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S59>/Integrator' */
  real_T UD_DSTATE;                    /* '<S52>/UD' */
  real_T Integrator_PREV_U;            /* '<S59>/Integrator' */
  uint32_T PID_ELAPS_T[2];             /* '<S19>/PID' */
  uint32_T PID_PREV_T[2];              /* '<S19>/PID' */
  uint8_T Integrator_SYSTEM_ENABLE;    /* '<S59>/Integrator' */
  boolean_T PID_RESET_ELAPS_T;         /* '<S19>/PID' */
} DW_PID_nav3v2_T;

/* Block signals for system '<S96>/Enabled Subsystem' */
typedef struct {
  SL_Bus_nav3v2_std_msgs_Float32 In1;  /* '<S98>/In1' */
} B_EnabledSubsystem_nav3v2_T;

/* Block signals for system '<S100>/Quaternion_2_Euler' */
typedef struct {
  real_T theta;                        /* '<S100>/Quaternion_2_Euler' */
  real_T aSinInput;
  real_T c_b;
} B_Quaternion_2_Euler_nav3v2_T;

/* Block signals (default storage) */
typedef struct {
  SL_Bus_nav3v2_std_msgs_Float64MultiArray BusAssignment;/* '<S4>/Bus Assignment' */
  SL_Bus_nav3v2_nav_msgs_Odometry In1; /* '<S111>/In1' */
  SL_Bus_nav3v2_nav_msgs_Odometry b_varargout_2;
  SL_Bus_nav3v2_geometry_msgs_PoseWithCovarianceStamped In1_p;/* '<S115>/In1' */
  SL_Bus_nav3v2_geometry_msgs_PoseWithCovarianceStamped b_varargout_2_m;
  SL_Bus_nav3v2_sensor_msgs_Imu In1_a; /* '<S104>/In1' */
  SL_Bus_nav3v2_sensor_msgs_Imu b_varargout_2_c;
  SL_Bus_nav3v2_sensor_msgs_Joy In1_i; /* '<S77>/In1' */
  SL_Bus_nav3v2_sensor_msgs_Joy b_varargout_2_k;
  creal_T eigVec[16];
  creal_T At[16];
  controllerPurePursuit_nav3v2_T controller;
  real_T rotm[16];
  real_T b_V[16];
  real_T b_A[16];
  real_T W2LOTransform[9];
  real_T World2RobotOdom[9];           /* '<S85>/MATLAB Function' */
  real_T Lidar2Robot[9];               /* '<S86>/MATLAB Function' */
  real_T W2RTransform_tmp[9];
  real_T W2LOTransform_c[9];
  real_T DataTypeConversion[8];
  creal_T eigVal[4];
  creal_T beta1[4];
  creal_T work1[4];
  SL_Bus_nav3v2_geometry_msgs_Twist In1_b;/* '<S106>/In1' */
  SL_Bus_nav3v2_geometry_msgs_Twist BusAssignment_l;/* '<S13>/Bus Assignment' */
  real_T waypoints_data[6];
  char_T cv[35];
  char_T cv1[34];
  real_T varargin_1[4];
  real_T work[4];
  real_T rworka[4];
  real_T work_b[4];
  char_T cv2[30];
  char_T cv3[29];
  char_T cv4[26];
  char_T cv5[25];
  char_T cv6[24];
  real_T Switch[3];                    /* '<S6>/Switch' */
  real_T tau[3];
  real_T v[3];
  real_T b_v[3];
  char_T cv7[22];
  char_T cv8[21];
  char_T cv9[20];
  char_T cv10[19];
  char_T cv11[18];
  char_T cv12[17];
  int32_T rscale[4];
  int8_T b_I[16];
  char_T cv13[16];
  real_T currentWaypoint[2];           /* '<S2>/State_Machine' */
  real_T dv[2];
  real_T dv1[2];
  real_T varargout_3[2];
  real_T lookaheadStartPt[2];
  real_T obj[2];
  real_T waypoints[2];
  real_T refPt[2];
  creal_T s;
  creal_T ctemp;
  creal_T ad22;
  creal_T ascale;
  char_T cv14[15];
  real_T linearVel;                    /* '<S2>/State_Machine' */
  real_T angularVel;                   /* '<S2>/State_Machine' */
  real_T pose[3];                      /* '<S2>/State_Machine' */
  real_T goalWaypoint[2];              /* '<S2>/State_Machine' */
  real_T goalTolerance;                /* '<S2>/State_Machine' */
  real_T pose_j[3];                    /* '<S2>/State_Machine' */
  real_T desiredHeading;               /* '<S2>/State_Machine' */
  real_T P;                            /* '<S2>/State_Machine' */
  real_T I;                            /* '<S2>/State_Machine' */
  real_T D;                            /* '<S2>/State_Machine' */
  real_T K12;
  real_T K34;
  real_T value;
  real_T value_p;
  real_T Product;                      /* '<S14>/Product' */
  real_T rtb_Lidar2Robot_tmp;
  real_T quat_idx_2;
  real_T quat_idx_3;
  real_T W2LOTransform_tmp;
  real_T W2LOTransform_tmp_c;
  real_T W2LOTransform_tmp_f;
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
  real_T shift_im_g;
  real_T eshift_re;
  real_T eshift_im;
  real_T scale_g;
  real_T g2;
  real_T f2s;
  real_T di;
  real_T x;
  real_T fs_re;
  real_T fs_im;
  real_T gs_re;
  real_T gs_im;
  real_T a;
  real_T varargout_2;
  real_T varargout_1;
  real_T minDistance;
  real_T dist;
  real_T b_dist;
  real_T overshootDist;
  real_T lookaheadIdx;
  real_T alpha;
  real_T v12;
  real_T v12_m;
  real_T scale_n;
  real_T absxk_p;
  real_T t_l;
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
  real_T scale_j;
  real_T z;
  real_T tau_d;
  real_T anorm_g;
  real_T ascale_l;
  real_T temp;
  real_T acoeff;
  real_T scale_d;
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
  real_T scale_dy;
  real_T g2_l;
  real_T f2s_o;
  real_T di_b;
  real_T x_n;
  real_T fs_re_b;
  real_T fs_im_l;
  real_T gs_re_h;
  real_T gs_im_b;
  real_T scale_da;
  real_T sumsq;
  real_T temp1;
  real_T temp2;
  real_T cfromc;
  real_T ctoc;
  real_T cfrom1;
  real_T cto1;
  real_T mul_e;
  real_T xnorm_b;
  real_T scale_jz;
  real_T absxk_f;
  real_T t_a;
  SL_Bus_nav3v2_std_msgs_Int16 In1_d;  /* '<S121>/In1' */
  boolean_T b[6];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T value_j;
  int32_T i;
  int32_T W2LOTransform_tmp_j;
  int32_T W2LOTransform_tmp_o;
  int32_T b_j;
  int32_T i_n;
  int32_T ihi;
  int32_T i_i;
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
  int32_T i_o;
  int32_T ctemp_tmp;
  int32_T ctemp_tmp_tmp;
  int32_T count;
  int32_T rescaledir;
  int32_T i_nv;
  int32_T i1;
  int32_T i2;
  int32_T trueCount;
  int32_T partialTrueCount;
  int32_T d_i;
  int32_T c_i_tmp;
  int32_T c_i_tmp_m;
  int32_T k_c;
  int32_T i_m;
  int32_T L;
  int32_T k_m;
  int32_T m;
  int32_T nr;
  int32_T hoffset;
  int32_T j_j;
  int32_T b_j_h;
  int32_T c_j;
  int32_T ix;
  int32_T s_tmp;
  int32_T b_c;
  int32_T c_c;
  int32_T i_p;
  int32_T c_j_p;
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
  int32_T lastc_a;
  int32_T coltop;
  int32_T ix_e;
  int32_T iac_a;
  int32_T d_a;
  int32_T b_ia_i;
  int32_T jy_l;
  int32_T j_o;
  int32_T b_o;
  int32_T i_ip;
  int32_T reAij_tmp;
  int32_T i3;
  int32_T u0;
  int32_T u1;
  int32_T knt_f;
  int32_T c_k;
  uint32_T ProbeWidth;                 /* '<S4>/Probe Width' */
  int32_T currentState;                /* '<S2>/State_Machine' */
  int32_T ledState;                    /* '<S2>/State_Machine' */
  SL_Bus_nav3v2_std_msgs_Float32 b_varargout_2_i;
  SL_Bus_nav3v2_std_msgs_Float32 BusAssignment1;/* '<S4>/Bus Assignment1' */
  SL_Bus_nav3v2_std_msgs_Int32 BusAssignment2_k;/* '<S4>/Bus Assignment2' */
  SL_Bus_nav3v2_std_msgs_Int32 BusAssignment2;/* '<S8>/Bus Assignment2' */
  int8_T c_data[3];
  SL_Bus_nav3v2_std_msgs_Int16 BusAssignment_n;/* '<S116>/Bus Assignment' */
  boolean_T b_varargout_1;
  boolean_T p_f;
  boolean_T ilascl;
  boolean_T found;
  boolean_T failed;
  boolean_T goto60;
  boolean_T goto70;
  boolean_T goto90;
  boolean_T searchFlag;
  boolean_T p_g;
  boolean_T b_p;
  boolean_T goto150;
  boolean_T b_co;
  boolean_T lscalea;
  boolean_T lscaleb;
  boolean_T firstNonZero;
  boolean_T notdone;
  B_Quaternion_2_Euler_nav3v2_T sf_Quaternion_2_Euler_g;/* '<S112>/Quaternion_2_Euler' */
  B_Quaternion_2_Euler_nav3v2_T sf_Quaternion_2_Euler_o;/* '<S107>/Quaternion_2_Euler' */
  B_Quaternion_2_Euler_nav3v2_T sf_Quaternion_2_Euler;/* '<S100>/Quaternion_2_Euler' */
  B_EnabledSubsystem_nav3v2_T EnabledSubsystem_d;/* '<S97>/Enabled Subsystem' */
  B_EnabledSubsystem_nav3v2_T EnabledSubsystem_m;/* '<S96>/Enabled Subsystem' */
  B_PID_nav3v2_T PID;                  /* '<S19>/PID' */
  B_checkAtGoal_nav3v2_T checkAtGoal;  /* '<S19>/checkAtGoal' */
} B_nav3v2_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  ros_slros_internal_block_GetP_T obj; /* '<S88>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_n;/* '<S87>/Get Parameter4' */
  ros_slros_internal_block_GetP_T obj_l;/* '<S87>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_l3;/* '<S87>/Get Parameter11' */
  ros_slros_internal_block_GetP_T obj_ls;/* '<S87>/Get Parameter12' */
  ros_slros_internal_block_GetP_T obj_k;/* '<S87>/Get Parameter13' */
  ros_slros_internal_block_GetP_T obj_e;/* '<S87>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_f;/* '<S87>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_e0;/* '<S87>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_lq;/* '<S87>/Get Parameter6' */
  ros_slros_internal_block_GetP_T obj_na;/* '<S87>/Get Parameter10' */
  ros_slros_internal_block_GetP_T obj_i;/* '<S87>/Get Parameter9' */
  ros_slros_internal_block_GetP_T obj_i3;/* '<S87>/Get Parameter8' */
  ros_slros_internal_block_GetP_T obj_m;/* '<S87>/Get Parameter7' */
  ros_slros_internal_block_GetP_T obj_mt;/* '<S86>/Get Parameter' */
  ros_slros_internal_block_GetP_T obj_h;/* '<S86>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_my;/* '<S86>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_d;/* '<S86>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_lm;/* '<S86>/Get Parameter4' */
  ros_slros_internal_block_GetP_T obj_kb;/* '<S86>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_ek;/* '<S85>/Get Parameter6' */
  ros_slros_internal_block_GetP_T obj_lb;/* '<S85>/Get Parameter5' */
  ros_slros_internal_block_GetP_T obj_kg;/* '<S85>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_ef;/* '<S85>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_g;/* '<S85>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_dr;/* '<S85>/Get Parameter4' */
  ros_slros_internal_block_GetP_T obj_p;/* '<S85>/Get Parameter' */
  ros_slros_internal_block_GetP_T obj_gl;/* '<S84>/Get Parameter1' */
  ros_slros_internal_block_GetP_T obj_j;/* '<S84>/Get Parameter2' */
  ros_slros_internal_block_GetP_T obj_a;/* '<S84>/Get Parameter3' */
  ros_slros_internal_block_GetP_T obj_b;/* '<S84>/Get Parameter4' */
  ros_slros_internal_block_Publ_T obj_jx;/* '<S118>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_c;/* '<S83>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_jr;/* '<S82>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_o;/* '<S81>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_gv;/* '<S18>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_ft;/* '<S16>/SinkBlock' */
  ros_slros_internal_block_Publ_T obj_nw;/* '<S12>/SinkBlock' */
  ros_slros_internal_block_Subs_T obj_jp;/* '<S120>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_fy;/* '<S113>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_gt;/* '<S108>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_do;/* '<S105>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_dm;/* '<S101>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_n4;/* '<S97>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_nh;/* '<S96>/SourceBlock' */
  ros_slros_internal_block_Subs_T obj_mf;/* '<S76>/SourceBlock' */
  ros_slros_internal_block_SetP_T obj_cw;/* '<S119>/Set Parameter' */
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
  real_T all_waypoints[10];            /* '<S2>/State_Machine' */
  real_T all_waypoints_config[5];      /* '<S2>/State_Machine' */
  int32_T AUTONOMOUS_CONTROL_STATE;    /* '<Root>/Data Store Memory' */
  int32_T EMERGENCY_STATE;             /* '<Root>/Data Store Memory1' */
  int32_T LED_WAYPOINT_REACHED_STATE;  /* '<Root>/Data Store Memory15' */
  int32_T LED_MISSION_PAUSED_STATE;    /* '<Root>/Data Store Memory16' */
  int32_T LED_STARTING_STATE;          /* '<Root>/Data Store Memory17' */
  int32_T LED_MISSION_COMPLETE_STATE;  /* '<Root>/Data Store Memory18' */
  int32_T LED_ERROR_STATE;             /* '<Root>/Data Store Memory19' */
  int32_T INITIALISATION_STATE;        /* '<Root>/Data Store Memory2' */
  int32_T LED_EMERGENCY_STOP_STATE;    /* '<Root>/Data Store Memory20' */
  int32_T LED_BATTERY_LOW_STATE;       /* '<Root>/Data Store Memory21' */
  int32_T MANUAL_CONTROL_STATE;        /* '<Root>/Data Store Memory3' */
  int32_T REVERSE_STATE;               /* '<Root>/Data Store Memory4' */
  int32_T WAITING_STATE;               /* '<Root>/Data Store Memory5' */
  uint8_T is_active_c3_nav3v2;         /* '<S2>/State_Machine' */
  uint8_T is_c3_nav3v2;                /* '<S2>/State_Machine' */
  uint8_T is_REVERSE;                  /* '<S2>/State_Machine' */
  uint8_T is_AUTONOMOUS_CONTROL;       /* '<S2>/State_Machine' */
  boolean_T objisempty;                /* '<S120>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S119>/Set Parameter' */
  boolean_T objisempty_m;              /* '<S118>/SinkBlock' */
  boolean_T objisempty_b;              /* '<S113>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S108>/SourceBlock' */
  boolean_T objisempty_ls;             /* '<S105>/SourceBlock' */
  boolean_T objisempty_d;              /* '<S101>/SourceBlock' */
  boolean_T objisempty_da;             /* '<S97>/SourceBlock' */
  boolean_T objisempty_o;              /* '<S96>/SourceBlock' */
  boolean_T objisempty_oj;             /* '<S88>/Get Parameter5' */
  boolean_T objisempty_gn;             /* '<S87>/Get Parameter4' */
  boolean_T objisempty_ou;             /* '<S87>/Get Parameter3' */
  boolean_T objisempty_h;              /* '<S87>/Get Parameter11' */
  boolean_T objisempty_mi;             /* '<S87>/Get Parameter12' */
  boolean_T objisempty_b1;             /* '<S87>/Get Parameter13' */
  boolean_T objisempty_c;              /* '<S87>/Get Parameter2' */
  boolean_T objisempty_e;              /* '<S87>/Get Parameter1' */
  boolean_T objisempty_p;              /* '<S87>/Get Parameter5' */
  boolean_T objisempty_ci;             /* '<S87>/Get Parameter6' */
  boolean_T objisempty_dh;             /* '<S87>/Get Parameter10' */
  boolean_T objisempty_f;              /* '<S87>/Get Parameter9' */
  boolean_T objisempty_k;              /* '<S87>/Get Parameter8' */
  boolean_T objisempty_a;              /* '<S87>/Get Parameter7' */
  boolean_T objisempty_ot;             /* '<S86>/Get Parameter' */
  boolean_T objisempty_l5;             /* '<S86>/Get Parameter1' */
  boolean_T objisempty_i;              /* '<S86>/Get Parameter2' */
  boolean_T objisempty_be;             /* '<S86>/Get Parameter3' */
  boolean_T objisempty_ba;             /* '<S86>/Get Parameter4' */
  boolean_T objisempty_k5;             /* '<S86>/Get Parameter5' */
  boolean_T objisempty_et;             /* '<S85>/Get Parameter6' */
  boolean_T objisempty_an;             /* '<S85>/Get Parameter5' */
  boolean_T objisempty_gi;             /* '<S85>/Get Parameter2' */
  boolean_T objisempty_mn;             /* '<S85>/Get Parameter3' */
  boolean_T objisempty_ow;             /* '<S85>/Get Parameter1' */
  boolean_T objisempty_oi;             /* '<S85>/Get Parameter4' */
  boolean_T objisempty_lk;             /* '<S85>/Get Parameter' */
  boolean_T objisempty_fg;             /* '<S84>/Get Parameter1' */
  boolean_T objisempty_bh;             /* '<S84>/Get Parameter2' */
  boolean_T objisempty_c5;             /* '<S84>/Get Parameter3' */
  boolean_T objisempty_bhy;            /* '<S84>/Get Parameter4' */
  boolean_T objisempty_mo;             /* '<S83>/SinkBlock' */
  boolean_T objisempty_b2;             /* '<S82>/SinkBlock' */
  boolean_T objisempty_gs;             /* '<S81>/SinkBlock' */
  boolean_T objisempty_pp;             /* '<S76>/SourceBlock' */
  boolean_T objisempty_gc;             /* '<S18>/SinkBlock' */
  boolean_T objisempty_j;              /* '<S16>/SinkBlock' */
  boolean_T objisempty_pw;             /* '<S12>/SinkBlock' */
  DW_PID_nav3v2_T PID;                 /* '<S19>/PID' */
} DW_nav3v2_T;

/* Parameters for system: '<S19>/checkAtGoal' */
struct P_checkAtGoal_nav3v2_T_ {
  boolean_T atGoal_Y0;                 /* Computed Parameter: atGoal_Y0
                                        * Referenced by: '<S21>/atGoal'
                                        */
};

/* Parameters for system: '<S19>/PID' */
struct P_PID_nav3v2_T_ {
  real_T DiscreteVaryingPID_Differentiat;
                              /* Mask Parameter: DiscreteVaryingPID_Differentiat
                               * Referenced by: '<S52>/UD'
                               */
  real_T DiscreteVaryingPID_InitialCondi;
                              /* Mask Parameter: DiscreteVaryingPID_InitialCondi
                               * Referenced by: '<S59>/Integrator'
                               */
  real_T DiscreteVaryingPID_LowerSaturat;
                              /* Mask Parameter: DiscreteVaryingPID_LowerSaturat
                               * Referenced by:
                               *   '<S50>/DeadZone'
                               *   '<S66>/Saturation'
                               */
  real_T DiscreteVaryingPID_UpperSaturat;
                              /* Mask Parameter: DiscreteVaryingPID_UpperSaturat
                               * Referenced by:
                               *   '<S50>/DeadZone'
                               *   '<S66>/Saturation'
                               */
  real_T angVel_Y0;                    /* Computed Parameter: angVel_Y0
                                        * Referenced by: '<S20>/angVel'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S50>/Constant1'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S59>/Integrator'
                                        */
  real_T Tsamp_WtEt;                   /* Computed Parameter: Tsamp_WtEt
                                        * Referenced by: '<S54>/Tsamp'
                                        */
  real_T ZeroGain_Gain;                /* Expression: 0
                                        * Referenced by: '<S50>/ZeroGain'
                                        */
};

/* Parameters for system: '<S96>/Enabled Subsystem' */
struct P_EnabledSubsystem_nav3v2_T_ {
  SL_Bus_nav3v2_std_msgs_Float32 Out1_Y0;/* Computed Parameter: Out1_Y0
                                          * Referenced by: '<S98>/Out1'
                                          */
};

/* Parameters (default storage) */
struct P_nav3v2_T_ {
  SL_Bus_nav3v2_std_msgs_Float64MultiArray Constant_Value;/* Computed Parameter: Constant_Value
                                                           * Referenced by: '<S78>/Constant'
                                                           */
  SL_Bus_nav3v2_nav_msgs_Odometry Out1_Y0;/* Computed Parameter: Out1_Y0
                                           * Referenced by: '<S111>/Out1'
                                           */
  SL_Bus_nav3v2_nav_msgs_Odometry Constant_Value_a;/* Computed Parameter: Constant_Value_a
                                                    * Referenced by: '<S108>/Constant'
                                                    */
  SL_Bus_nav3v2_geometry_msgs_PoseWithCovarianceStamped Out1_Y0_m;/* Computed Parameter: Out1_Y0_m
                                                                   * Referenced by: '<S115>/Out1'
                                                                   */
  SL_Bus_nav3v2_geometry_msgs_PoseWithCovarianceStamped Constant_Value_b;/* Computed Parameter: Constant_Value_b
                                                                      * Referenced by: '<S113>/Constant'
                                                                      */
  SL_Bus_nav3v2_sensor_msgs_Imu Out1_Y0_i;/* Computed Parameter: Out1_Y0_i
                                           * Referenced by: '<S104>/Out1'
                                           */
  SL_Bus_nav3v2_sensor_msgs_Imu Constant_Value_j;/* Computed Parameter: Constant_Value_j
                                                  * Referenced by: '<S101>/Constant'
                                                  */
  SL_Bus_nav3v2_sensor_msgs_Joy Out1_Y0_ix;/* Computed Parameter: Out1_Y0_ix
                                            * Referenced by: '<S77>/Out1'
                                            */
  SL_Bus_nav3v2_sensor_msgs_Joy Constant_Value_l;/* Computed Parameter: Constant_Value_l
                                                  * Referenced by: '<S76>/Constant'
                                                  */
  SL_Bus_nav3v2_geometry_msgs_Twist Constant_Value_n;/* Computed Parameter: Constant_Value_n
                                                      * Referenced by: '<S15>/Constant'
                                                      */
  SL_Bus_nav3v2_geometry_msgs_Twist Constant_Value_o;/* Computed Parameter: Constant_Value_o
                                                      * Referenced by: '<S17>/Constant'
                                                      */
  SL_Bus_nav3v2_geometry_msgs_Twist Out1_Y0_o;/* Computed Parameter: Out1_Y0_o
                                               * Referenced by: '<S106>/Out1'
                                               */
  SL_Bus_nav3v2_geometry_msgs_Twist Constant_Value_jg;/* Computed Parameter: Constant_Value_jg
                                                       * Referenced by: '<S105>/Constant'
                                                       */
  SL_Bus_nav3v2_std_msgs_Float32 Constant_Value_ag;/* Computed Parameter: Constant_Value_ag
                                                    * Referenced by: '<S96>/Constant'
                                                    */
  SL_Bus_nav3v2_std_msgs_Float32 Constant_Value_av;/* Computed Parameter: Constant_Value_av
                                                    * Referenced by: '<S97>/Constant'
                                                    */
  SL_Bus_nav3v2_std_msgs_Float32 Constant_Value_f;/* Computed Parameter: Constant_Value_f
                                                   * Referenced by: '<S79>/Constant'
                                                   */
  SL_Bus_nav3v2_std_msgs_Int16 Out1_Y0_c;/* Computed Parameter: Out1_Y0_c
                                          * Referenced by: '<S121>/Out1'
                                          */
  SL_Bus_nav3v2_std_msgs_Int16 Constant_Value_i;/* Computed Parameter: Constant_Value_i
                                                 * Referenced by: '<S120>/Constant'
                                                 */
  SL_Bus_nav3v2_std_msgs_Int16 Constant_Value_e;/* Computed Parameter: Constant_Value_e
                                                 * Referenced by: '<S117>/Constant'
                                                 */
  SL_Bus_nav3v2_std_msgs_Int32 Constant_Value_fq;/* Computed Parameter: Constant_Value_fq
                                                  * Referenced by: '<S11>/Constant'
                                                  */
  SL_Bus_nav3v2_std_msgs_Int32 Constant_Value_g;/* Computed Parameter: Constant_Value_g
                                                 * Referenced by: '<S80>/Constant'
                                                 */
  real_T Gain_Gain;                    /* Expression: pi
                                        * Referenced by: '<S14>/Gain'
                                        */
  real_T State_Machine_reversePose[3]; /* Expression: [0; 0; -pi/2]
                                        * Referenced by: '<S2>/State_Machine'
                                        */
  real_T Constant_Value_c;             /* Expression: pi/2
                                        * Referenced by: '<S119>/Constant'
                                        */
  real_T Linear_Accel_Gain;            /* Expression: 0.25
                                        * Referenced by: '<S3>/Linear_Accel'
                                        */
  real_T waypoints_Value[12];         /* Expression: [3,7,2; 8,7,2; 8,2,2;3,2,1]
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
  int32_T DataStoreMemory15_InitialValue;
                           /* Computed Parameter: DataStoreMemory15_InitialValue
                            * Referenced by: '<Root>/Data Store Memory15'
                            */
  int32_T DataStoreMemory16_InitialValue;
                           /* Computed Parameter: DataStoreMemory16_InitialValue
                            * Referenced by: '<Root>/Data Store Memory16'
                            */
  int32_T DataStoreMemory17_InitialValue;
                           /* Computed Parameter: DataStoreMemory17_InitialValue
                            * Referenced by: '<Root>/Data Store Memory17'
                            */
  int32_T DataStoreMemory18_InitialValue;
                           /* Computed Parameter: DataStoreMemory18_InitialValue
                            * Referenced by: '<Root>/Data Store Memory18'
                            */
  int32_T DataStoreMemory19_InitialValue;
                           /* Computed Parameter: DataStoreMemory19_InitialValue
                            * Referenced by: '<Root>/Data Store Memory19'
                            */
  int32_T DataStoreMemory2_InitialValue;
                            /* Computed Parameter: DataStoreMemory2_InitialValue
                             * Referenced by: '<Root>/Data Store Memory2'
                             */
  int32_T DataStoreMemory20_InitialValue;
                           /* Computed Parameter: DataStoreMemory20_InitialValue
                            * Referenced by: '<Root>/Data Store Memory20'
                            */
  int32_T DataStoreMemory21_InitialValue;
                           /* Computed Parameter: DataStoreMemory21_InitialValue
                            * Referenced by: '<Root>/Data Store Memory21'
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
                                        * Referenced by: '<S116>/Constant'
                                        */
  P_EnabledSubsystem_nav3v2_T EnabledSubsystem_d;/* '<S97>/Enabled Subsystem' */
  P_EnabledSubsystem_nav3v2_T EnabledSubsystem_m;/* '<S96>/Enabled Subsystem' */
  P_PID_nav3v2_T PID;                  /* '<S19>/PID' */
  P_checkAtGoal_nav3v2_T checkAtGoal;  /* '<S19>/checkAtGoal' */
};

/* Real-time Model Data Structure */
struct tag_RTM_nav3v2_T {
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

  extern P_nav3v2_T nav3v2_P;

#ifdef __cplusplus

}
#endif

/* Block signals (default storage) */
#ifdef __cplusplus

extern "C" {

#endif

  extern B_nav3v2_T nav3v2_B;

#ifdef __cplusplus

}
#endif

/* Block states (default storage) */
extern DW_nav3v2_T nav3v2_DW;

#ifdef __cplusplus

extern "C" {

#endif

  /* Model entry point functions */
  extern void nav3v2_initialize(void);
  extern void nav3v2_step(void);
  extern void nav3v2_terminate(void);

#ifdef __cplusplus

}
#endif

/* Real-time Model object */
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_nav3v2_T *const nav3v2_M;

#ifdef __cplusplus

}
#endif

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Current Pose' : Unused code path elimination
 * Block '<S2>/Current State' : Unused code path elimination
 * Block '<S2>/Current Waypoint' : Unused code path elimination
 * Block '<S2>/Input Commands' : Unused code path elimination
 * Block '<S52>/DTDup' : Unused code path elimination
 * Block '<S2>/waypoints1' : Unused code path elimination
 * Block '<S2>/waypoints2' : Unused code path elimination
 * Block '<S8>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S4>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S4>/Data Type Conversion1' : Eliminate redundant data type conversion
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
 * '<Root>' : 'nav3v2'
 * '<S1>'   : 'nav3v2/Actuation_Subsystem'
 * '<S2>'   : 'nav3v2/Control_Subsystem'
 * '<S3>'   : 'nav3v2/Joystick_Subsystem'
 * '<S4>'   : 'nav3v2/Logging_Subsystem'
 * '<S5>'   : 'nav3v2/Parameter_Server_Subsystem'
 * '<S6>'   : 'nav3v2/Perception_Subsystem'
 * '<S7>'   : 'nav3v2/Service_Request_Subsystem'
 * '<S8>'   : 'nav3v2/Actuation_Subsystem/LED_Actuation'
 * '<S9>'   : 'nav3v2/Actuation_Subsystem/Real_Actuation'
 * '<S10>'  : 'nav3v2/Actuation_Subsystem/Simulation_Actuation'
 * '<S11>'  : 'nav3v2/Actuation_Subsystem/LED_Actuation/Blank Message2'
 * '<S12>'  : 'nav3v2/Actuation_Subsystem/LED_Actuation/Publish2'
 * '<S13>'  : 'nav3v2/Actuation_Subsystem/Real_Actuation/Actuator'
 * '<S14>'  : 'nav3v2/Actuation_Subsystem/Real_Actuation/Forward Kinematics'
 * '<S15>'  : 'nav3v2/Actuation_Subsystem/Real_Actuation/Actuator/Blank Message'
 * '<S16>'  : 'nav3v2/Actuation_Subsystem/Real_Actuation/Actuator/Publish'
 * '<S17>'  : 'nav3v2/Actuation_Subsystem/Simulation_Actuation/Blank Message'
 * '<S18>'  : 'nav3v2/Actuation_Subsystem/Simulation_Actuation/Publish'
 * '<S19>'  : 'nav3v2/Control_Subsystem/State_Machine'
 * '<S20>'  : 'nav3v2/Control_Subsystem/State_Machine/PID'
 * '<S21>'  : 'nav3v2/Control_Subsystem/State_Machine/checkAtGoal'
 * '<S22>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Calculate_Error'
 * '<S23>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID'
 * '<S24>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Normalise_Goal_Heading'
 * '<S25>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Normalise_Heading'
 * '<S26>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Anti-windup'
 * '<S27>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/D Gain'
 * '<S28>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter'
 * '<S29>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter ICs'
 * '<S30>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/I Gain'
 * '<S31>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain'
 * '<S32>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain Fdbk'
 * '<S33>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator'
 * '<S34>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator ICs'
 * '<S35>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Copy'
 * '<S36>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Gain'
 * '<S37>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/P Copy'
 * '<S38>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Parallel P Gain'
 * '<S39>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Reset Signal'
 * '<S40>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation'
 * '<S41>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation Fdbk'
 * '<S42>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum'
 * '<S43>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum Fdbk'
 * '<S44>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode'
 * '<S45>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode Sum'
 * '<S46>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Integral'
 * '<S47>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Ngain'
 * '<S48>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/postSat Signal'
 * '<S49>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/preSat Signal'
 * '<S50>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Anti-windup/Disc. Clamping Parallel'
 * '<S51>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/D Gain/External Parameters'
 * '<S52>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter/Differentiator'
 * '<S53>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter/Differentiator/Tsamp'
 * '<S54>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S55>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Filter ICs/Internal IC - Differentiator'
 * '<S56>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/I Gain/External Parameters'
 * '<S57>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain/Passthrough'
 * '<S58>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Ideal P Gain Fdbk/Disabled'
 * '<S59>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator/Discrete'
 * '<S60>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Integrator ICs/Internal IC'
 * '<S61>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Copy/Disabled wSignal Specification'
 * '<S62>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/N Gain/Passthrough'
 * '<S63>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/P Copy/Disabled'
 * '<S64>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Parallel P Gain/External Parameters'
 * '<S65>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Reset Signal/Disabled'
 * '<S66>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation/Enabled'
 * '<S67>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Saturation Fdbk/Disabled'
 * '<S68>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum/Sum_PID'
 * '<S69>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Sum Fdbk/Disabled'
 * '<S70>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode/Disabled'
 * '<S71>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tracking Mode Sum/Passthrough'
 * '<S72>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Integral/Passthrough'
 * '<S73>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/Tsamp - Ngain/Passthrough'
 * '<S74>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/postSat Signal/Forward_Path'
 * '<S75>'  : 'nav3v2/Control_Subsystem/State_Machine/PID/Discrete Varying PID/preSat Signal/Forward_Path'
 * '<S76>'  : 'nav3v2/Joystick_Subsystem/Subscribe'
 * '<S77>'  : 'nav3v2/Joystick_Subsystem/Subscribe/Enabled Subsystem'
 * '<S78>'  : 'nav3v2/Logging_Subsystem/Blank Message'
 * '<S79>'  : 'nav3v2/Logging_Subsystem/Blank Message1'
 * '<S80>'  : 'nav3v2/Logging_Subsystem/Blank Message2'
 * '<S81>'  : 'nav3v2/Logging_Subsystem/Publish'
 * '<S82>'  : 'nav3v2/Logging_Subsystem/Publish1'
 * '<S83>'  : 'nav3v2/Logging_Subsystem/Publish2'
 * '<S84>'  : 'nav3v2/Parameter_Server_Subsystem/Control_Parameters'
 * '<S85>'  : 'nav3v2/Parameter_Server_Subsystem/Experiment_Parameters'
 * '<S86>'  : 'nav3v2/Parameter_Server_Subsystem/Robot_Parameters'
 * '<S87>'  : 'nav3v2/Parameter_Server_Subsystem/State_Parameters'
 * '<S88>'  : 'nav3v2/Parameter_Server_Subsystem/System_Parameters'
 * '<S89>'  : 'nav3v2/Parameter_Server_Subsystem/Experiment_Parameters/MATLAB Function'
 * '<S90>'  : 'nav3v2/Parameter_Server_Subsystem/Robot_Parameters/MATLAB Function'
 * '<S91>'  : 'nav3v2/Perception_Subsystem/Battery_Subsystem'
 * '<S92>'  : 'nav3v2/Perception_Subsystem/IMU_Subsystem'
 * '<S93>'  : 'nav3v2/Perception_Subsystem/Move_Base_Subsystem'
 * '<S94>'  : 'nav3v2/Perception_Subsystem/SLAM_Subsystem'
 * '<S95>'  : 'nav3v2/Perception_Subsystem/Simulation_Subsystem'
 * '<S96>'  : 'nav3v2/Perception_Subsystem/Battery_Subsystem/Live_Subscriber'
 * '<S97>'  : 'nav3v2/Perception_Subsystem/Battery_Subsystem/Live_Subscriber1'
 * '<S98>'  : 'nav3v2/Perception_Subsystem/Battery_Subsystem/Live_Subscriber/Enabled Subsystem'
 * '<S99>'  : 'nav3v2/Perception_Subsystem/Battery_Subsystem/Live_Subscriber1/Enabled Subsystem'
 * '<S100>' : 'nav3v2/Perception_Subsystem/IMU_Subsystem/Live_Pose_Parser'
 * '<S101>' : 'nav3v2/Perception_Subsystem/IMU_Subsystem/Live_Subscriber'
 * '<S102>' : 'nav3v2/Perception_Subsystem/IMU_Subsystem/Live_Pose_Parser/Quaternion_2_Euler'
 * '<S103>' : 'nav3v2/Perception_Subsystem/IMU_Subsystem/Live_Pose_Parser/TrueNorthToIMU1'
 * '<S104>' : 'nav3v2/Perception_Subsystem/IMU_Subsystem/Live_Subscriber/Enabled Subsystem'
 * '<S105>' : 'nav3v2/Perception_Subsystem/Move_Base_Subsystem/Live_Subscriber'
 * '<S106>' : 'nav3v2/Perception_Subsystem/Move_Base_Subsystem/Live_Subscriber/Enabled Subsystem'
 * '<S107>' : 'nav3v2/Perception_Subsystem/SLAM_Subsystem/Live_Pose_Parser'
 * '<S108>' : 'nav3v2/Perception_Subsystem/SLAM_Subsystem/Live_Subscriber'
 * '<S109>' : 'nav3v2/Perception_Subsystem/SLAM_Subsystem/Live_Pose_Parser/Quaternion_2_Euler'
 * '<S110>' : 'nav3v2/Perception_Subsystem/SLAM_Subsystem/Live_Pose_Parser/World to Robot Transform'
 * '<S111>' : 'nav3v2/Perception_Subsystem/SLAM_Subsystem/Live_Subscriber/Enabled Subsystem'
 * '<S112>' : 'nav3v2/Perception_Subsystem/Simulation_Subsystem/Sim_Pose_Parser'
 * '<S113>' : 'nav3v2/Perception_Subsystem/Simulation_Subsystem/Simulation_Subscriber'
 * '<S114>' : 'nav3v2/Perception_Subsystem/Simulation_Subsystem/Sim_Pose_Parser/Quaternion_2_Euler'
 * '<S115>' : 'nav3v2/Perception_Subsystem/Simulation_Subsystem/Simulation_Subscriber/Enabled Subsystem'
 * '<S116>' : 'nav3v2/Service_Request_Subsystem/RESET_IMU'
 * '<S117>' : 'nav3v2/Service_Request_Subsystem/RESET_IMU/Blank Message'
 * '<S118>' : 'nav3v2/Service_Request_Subsystem/RESET_IMU/Publish'
 * '<S119>' : 'nav3v2/Service_Request_Subsystem/RESET_IMU/RESET_IMU'
 * '<S120>' : 'nav3v2/Service_Request_Subsystem/RESET_IMU/Subscribe'
 * '<S121>' : 'nav3v2/Service_Request_Subsystem/RESET_IMU/Subscribe/Enabled Subsystem'
 */
#endif                                 /* RTW_HEADER_nav3v2_h_ */
