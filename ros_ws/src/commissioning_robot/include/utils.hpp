#pragma once

#include <commissioning_robot/ControlState.h>
#include <commissioning_robot/FeedbackState.h>
#include <commissioning_robot/MechanismReport.h>
#include <commissioning_robot/PhaseReport.h>
#include <commissioning_robot/encoder_portConfig.h>
#include <commissioning_robot/motor_portConfig.h>
#include <commissioning_robot/robot_paramConfig.h>
#include <commissioning_robot/switch_portConfig.h>
#include <control_msgs/JointControllerState.h>
#include <ds4_driver/Feedback.h>
#include <ds4_driver/Status.h>
#include <dynamic_reconfigure/server.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <jsk_rviz_plugins/OverlayMenu.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <pid_controller.hpp>
#include <position_motordrive.hpp>
#include <string>
#include <velocity_motordrive.hpp>

#define dtr(degree) ((degree)*M_PI / 180.0)
#define rtd(rad) ((rad)*180.0 / M_PI)
#define IS_INSIDE(x, y, z) ((abs(x - y) < z) ? (true) : (false))

#define NODE_FREQ (100)

#define NUM_MOTOR (16)
#define NUM_ENCODER (20)
#define NUM_SWITCH (20)

#define NUM_DRIVEUNIT (4)
#define NUM_ARM_AXIS (2)
#define NUM_MAIN_LIFT_AXIS (1)
#define NUM_CABLE_MOTOR (2)

#define NUM_TOTAL_LIFT (NUM_DRIVEUNIT + NUM_ARM_AXIS + NUM_MAIN_LIFT_AXIS)

#define QUEUE_SIZE_PUBLISHER (3)
#define QUEUE_SIZE_SUBSCRIBER (3)

#define ROBOT_WORLD_AXIS_MOVING_SPEED_MPS (2.0)  // 2.0 m/s
#define MAX_PWM_STMHAL (30000)

#define FOLLOW_PWM_ARM_X (10000)
#define FOLLOW_PWM_ARM_Y (20000)

#define ORIGIN_PWM_ARM_X (FOLLOW_PWM_ARM_X / 2)
#define ORIGIN_PWM_ARM_Y (FOLLOW_PWM_ARM_Y / 2)

#define MAX_PWM_MAIN_LIFT (15000)
#define MIN_PWM_MAIN_LIFT (1300)

#define MAX_PWM_DRIVE_LIFT
#define MAX_PWM_DRIvE_WHEEL

#define NUM_POSITION_ARM_X
#define NUM_POSITION_ARM_Y

#define REFERENCE_MAIN_LIFT (0.25)  // max 0.62 centor 0.32 honnbann 0.555
#define ACCEL_PERIOD_MAIN_LIFT (4.0)

#define REFERENCE_ARM_X (3800)
#define ERROR_ALLOWANCE_ARM_X (34)
#define ERROR_ALLOWANCE_ARM_Y (25)
#define ERROR_ALLOWANCE_MAIN_LIFT (120)
#define ERROR_ALLOWANCE_DRIVE_LIFT
#define FAST_DISABLE_ALLOWANCE_ARM_X (ERROR_ALLOWANCE_ARM_X * 8)
#define FAST_DISABLE_ALLOWANCE_ARM_Y (ERROR_ALLOWANCE_ARM_Y * 8)

#define ENCODER_INDEX_MAIN_LIFT (14)
#define SWITCH_INDEX_MAIN_LIFT (14)

#define ENCODER_INDEX_ARM_X (17)
#define SWITCH_INDEX_ARM_X (18)

#define ENCODER_INDEX_ARM_Y (16)
#define SWITCH_INDEX_ARM_Y (19)

#define ENCODER_INDEX_CABLE_ARM_MOTOR (13)
#define SWITCH_INDEX_CABLE_ARM_MOTOR (11)
#define ERROR_ALLOWANCE_CABLE_ARM_MOTOR (15)
#define REFERENCE_CABLE_ARM_MOTOR (800)  // ref900 max1000 bane
#define FOLLOW_PWM_CABLE_ARM_MOTOR (16000)
#define ORIGIN_PWM_CABLE_ARM_MOTOR (FOLLOW_PWM_CABLE_ARM_MOTOR / 1.5)

#define ENCODER_INDEX_CABLE_BELT_MOTOR (12)
#define SWITCH_INDEX_CABLE_BELT_MOTOR (9)
#define ERROR_ALLOWANCE_CABLE_BELT_MOTOR (22)  // 270:1 ->22
#define REFERENCE_CABLE_FIRST_BELT_MOTOR (70)
#define REFERENCE_CABLE_STEP_BELT_MOTOR (290)
#define FOLLOW_PWM_CABLE_BELT_MOTOR (11000)  // 270:1 -> 11000
#define ORIGIN_PWM_CABLE_BELT_MOTOR (10000)  // 270:1 -> 10000
// first 70 - 405 - 667 - x - 1233
// 1233 - 70 = 1163 (each 290)

#define CAN_DATA_LENGTH (8)

enum MAIN_AXIS {
    axis_MAIN,
};

enum ARM_AXIS {
    axis_X,
    axis_Y,
};

enum CABLE_MOTOR {
    ARM_MOTOR,
    BELT_MOTOR,
};

void tf_quat_to_rpy(double &roll, double &pitch, double &yaw, tf::Quaternion quat) {
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // rpy are Pass by Reference
}
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat) {
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // rpy are Pass by Reference
}

template <class T, class U, class V>
T constrain(T base, U mimimum_outer, V maximum_outer) {
    T ans;
    if (base > maximum_outer)
        ans = maximum_outer;
    else if (base < mimimum_outer)
        ans = mimimum_outer;
    else
        ans = base;
    return ans;
}

template <class T, class U, class V>
T constrain_with_minimum_output(T base, U mimimum_outer, V mimimum_inner, V maximum_inner, U maximum_outer) {
    T ans;
    if (base < 0) {
        if (base < mimimum_outer)
            ans = mimimum_outer;
        else if (mimimum_inner < base)
            ans = mimimum_inner;
        else
            ans = base;
    } else {
        if (maximum_outer < base)
            ans = maximum_outer;
        else if (base < maximum_inner)
            ans = maximum_inner;
        else
            ans = base;
    }
    return ans;
}

enum CONTROL_MODE {
    STOP,
    FOLLOW,
    FAST_FOLLOW,
    MANUAL,
    ORIGIN,
    NUM_CONTROL_MODE
};

enum DS4_BUTTON_LIST {
    ARROW_RIGHT,
    ARROW_UP,
    ARROW_LEFT,
    ARROW_DOWN,
    CIRCLE,
    TRIANGLE,
    SQUARE,
    CROSS,
    L1,
    R1,
    L2,
    R2,
    L3,
    R3,
    OPTIONS,
    SHARE,
    PS,
    TRACKPAD,
    NUM_BUTTON_LIST,
};