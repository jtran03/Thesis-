/*
 * nav3_types.h
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

#ifndef RTW_HEADER_nav3_types_h_
#define RTW_HEADER_nav3_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_ros_time_Time_

typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_nav3_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Header_

typedef struct {
  uint32_T Seq;
  uint8_T FrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;
  SL_Bus_nav3_ros_time_Time Stamp;
} SL_Bus_nav3_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_sensor_msgs_Joy_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_sensor_msgs_Joy_

typedef struct {
  real32_T Axes[8];
  SL_Bus_ROSVariableLengthArrayInfo Axes_SL_Info;
  int32_T Buttons[13];
  SL_Bus_ROSVariableLengthArrayInfo Buttons_SL_Info;
  SL_Bus_nav3_std_msgs_Header Header;
} SL_Bus_nav3_sensor_msgs_Joy;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_MultiArrayDimension_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_MultiArrayDimension_

typedef struct {
  uint8_T Label[128];
  SL_Bus_ROSVariableLengthArrayInfo Label_SL_Info;
  uint32_T Size;
  uint32_T Stride;
} SL_Bus_nav3_std_msgs_MultiArrayDimension;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_MultiArrayLayout_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_MultiArrayLayout_

typedef struct {
  uint32_T DataOffset;
  SL_Bus_nav3_std_msgs_MultiArrayDimension Dim[16];
  SL_Bus_ROSVariableLengthArrayInfo Dim_SL_Info;
} SL_Bus_nav3_std_msgs_MultiArrayLayout;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Float64MultiArray_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Float64MultiArray_

typedef struct {
  real_T Data[11];
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;
  SL_Bus_nav3_std_msgs_MultiArrayLayout Layout;
} SL_Bus_nav3_std_msgs_Float64MultiArray;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Int16_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Int16_

typedef struct {
  int16_T Data;
} SL_Bus_nav3_std_msgs_Int16;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Vector3_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_nav3_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Twist_

typedef struct {
  SL_Bus_nav3_geometry_msgs_Vector3 Linear;
  SL_Bus_nav3_geometry_msgs_Vector3 Angular;
} SL_Bus_nav3_geometry_msgs_Twist;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Float32_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_std_msgs_Float32_

typedef struct {
  real32_T Data;
} SL_Bus_nav3_std_msgs_Float32;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Quaternion_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_nav3_geometry_msgs_Quaternion;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_sensor_msgs_Imu_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_sensor_msgs_Imu_

typedef struct {
  real_T OrientationCovariance[9];
  real_T AngularVelocityCovariance[9];
  real_T LinearAccelerationCovariance[9];
  SL_Bus_nav3_std_msgs_Header Header;
  SL_Bus_nav3_geometry_msgs_Quaternion Orientation;
  SL_Bus_nav3_geometry_msgs_Vector3 AngularVelocity;
  SL_Bus_nav3_geometry_msgs_Vector3 LinearAcceleration;
} SL_Bus_nav3_sensor_msgs_Imu;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Point_

typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_nav3_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_Pose_

typedef struct {
  SL_Bus_nav3_geometry_msgs_Point Position;
  SL_Bus_nav3_geometry_msgs_Quaternion Orientation;
} SL_Bus_nav3_geometry_msgs_Pose;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_PoseWithCovariance_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_PoseWithCovariance_

typedef struct {
  real_T Covariance[36];
  SL_Bus_nav3_geometry_msgs_Pose Pose;
} SL_Bus_nav3_geometry_msgs_PoseWithCovariance;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_TwistWithCovariance_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_TwistWithCovariance_

typedef struct {
  real_T Covariance[36];
  SL_Bus_nav3_geometry_msgs_Twist Twist;
} SL_Bus_nav3_geometry_msgs_TwistWithCovariance;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_nav_msgs_Odometry_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_nav_msgs_Odometry_

typedef struct {
  uint8_T ChildFrameId[128];
  SL_Bus_ROSVariableLengthArrayInfo ChildFrameId_SL_Info;
  SL_Bus_nav3_std_msgs_Header Header;
  SL_Bus_nav3_geometry_msgs_PoseWithCovariance Pose;
  SL_Bus_nav3_geometry_msgs_TwistWithCovariance Twist;
} SL_Bus_nav3_nav_msgs_Odometry;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped_
#define DEFINED_TYPEDEF_FOR_SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped_

typedef struct {
  SL_Bus_nav3_std_msgs_Header Header;
  SL_Bus_nav3_geometry_msgs_PoseWithCovariance Pose;
} SL_Bus_nav3_geometry_msgs_PoseWithCovarianceStamped;

#endif

#ifndef struct_tag_rkSooZHJZnr3Dpfu1LNqfH
#define struct_tag_rkSooZHJZnr3Dpfu1LNqfH

struct tag_rkSooZHJZnr3Dpfu1LNqfH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /*struct_tag_rkSooZHJZnr3Dpfu1LNqfH*/

#ifndef typedef_ros_slros_internal_block_Publ_T
#define typedef_ros_slros_internal_block_Publ_T

typedef struct tag_rkSooZHJZnr3Dpfu1LNqfH ros_slros_internal_block_Publ_T;

#endif                               /*typedef_ros_slros_internal_block_Publ_T*/

#ifndef struct_tag_9SewJ4y3IXNs5GrZti8qkG
#define struct_tag_9SewJ4y3IXNs5GrZti8qkG

struct tag_9SewJ4y3IXNs5GrZti8qkG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /*struct_tag_9SewJ4y3IXNs5GrZti8qkG*/

#ifndef typedef_ros_slros_internal_block_Subs_T
#define typedef_ros_slros_internal_block_Subs_T

typedef struct tag_9SewJ4y3IXNs5GrZti8qkG ros_slros_internal_block_Subs_T;

#endif                               /*typedef_ros_slros_internal_block_Subs_T*/

#ifndef struct_tag_KSdGoEc2IyOHz4CLi4rcCD
#define struct_tag_KSdGoEc2IyOHz4CLi4rcCD

struct tag_KSdGoEc2IyOHz4CLi4rcCD
{
  int32_T __dummy;
};

#endif                                 /*struct_tag_KSdGoEc2IyOHz4CLi4rcCD*/

#ifndef typedef_e_robotics_slcore_internal_bl_T
#define typedef_e_robotics_slcore_internal_bl_T

typedef struct tag_KSdGoEc2IyOHz4CLi4rcCD e_robotics_slcore_internal_bl_T;

#endif                               /*typedef_e_robotics_slcore_internal_bl_T*/

#ifndef struct_tag_PzhaB0v2Sx4ikuHWZx5WUB
#define struct_tag_PzhaB0v2Sx4ikuHWZx5WUB

struct tag_PzhaB0v2Sx4ikuHWZx5WUB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  e_robotics_slcore_internal_bl_T SampleTimeHandler;
};

#endif                                 /*struct_tag_PzhaB0v2Sx4ikuHWZx5WUB*/

#ifndef typedef_ros_slros_internal_block_GetP_T
#define typedef_ros_slros_internal_block_GetP_T

typedef struct tag_PzhaB0v2Sx4ikuHWZx5WUB ros_slros_internal_block_GetP_T;

#endif                               /*typedef_ros_slros_internal_block_GetP_T*/

#ifndef struct_tag_rVaL2jCthDg1Nc4cghuMrG
#define struct_tag_rVaL2jCthDg1Nc4cghuMrG

struct tag_rVaL2jCthDg1Nc4cghuMrG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                 /*struct_tag_rVaL2jCthDg1Nc4cghuMrG*/

#ifndef typedef_ros_slros_internal_block_SetP_T
#define typedef_ros_slros_internal_block_SetP_T

typedef struct tag_rVaL2jCthDg1Nc4cghuMrG ros_slros_internal_block_SetP_T;

#endif                               /*typedef_ros_slros_internal_block_SetP_T*/

/* Parameters for system: '<S16>/checkAtGoal' */
typedef struct P_checkAtGoal_nav3_T_ P_checkAtGoal_nav3_T;

/* Parameters for system: '<S16>/PID' */
typedef struct P_PID_nav3_T_ P_PID_nav3_T;

/* Parameters for system: '<S88>/Enabled Subsystem' */
typedef struct P_EnabledSubsystem_nav3_T_ P_EnabledSubsystem_nav3_T;

/* Parameters (default storage) */
typedef struct P_nav3_T_ P_nav3_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_nav3_T RT_MODEL_nav3_T;

#endif                                 /* RTW_HEADER_nav3_types_h_ */
