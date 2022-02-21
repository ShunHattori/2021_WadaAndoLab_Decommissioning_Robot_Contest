#include <utils.hpp>

std_msgs::Int32MultiArray RHWS_encoder_pulses;
std_msgs::UInt16MultiArray RHWS_switch_states;
void callback_RHWS_encoder_pulses(const std_msgs::Int32MultiArray &msg) {
    for (int i = 0; i < NUM_ENCODER; i++) {
        RHWS_encoder_pulses.data[i] = msg.data[i];
    }
}
void callback_RHWS_switch_states(const std_msgs::UInt16MultiArray &msg) {
    for (int i = 0; i < NUM_SWITCH; i++) {
        RHWS_switch_states.data[i] = msg.data[i];
    }
}

commissioning_robot::MechanismReport MechanismReport;

std_msgs::Int16MultiArray arm_motor_pwms;
commissioning_robot::FeedbackState FeedbackState_arm_controller;
static int bias_pulse[2] = {0, 0};

void callback_arm_controller(const commissioning_robot::ControlState &msg) {
    static int ENCODER_INDEX[NUM_ARM_AXIS] = {ENCODER_INDEX_ARM_X, ENCODER_INDEX_ARM_Y};
    static int SWITCH_INDEX[NUM_ARM_AXIS] = {SWITCH_INDEX_ARM_X, SWITCH_INDEX_ARM_Y};
    static int ERROR_ALLOWANCE[NUM_ARM_AXIS] = {ERROR_ALLOWANCE_ARM_X, ERROR_ALLOWANCE_ARM_Y};
    static int FAST_DISABLE_ALLOWANCE[NUM_ARM_AXIS] = {FAST_DISABLE_ALLOWANCE_ARM_X, FAST_DISABLE_ALLOWANCE_ARM_Y};
    static int FOLLOW_PWM[NUM_ARM_AXIS] = {FOLLOW_PWM_ARM_X, FOLLOW_PWM_ARM_Y};
    static int ORIGIN_PWM[NUM_ARM_AXIS] = {ORIGIN_PWM_ARM_X, ORIGIN_PWM_ARM_Y};
    for (uint8_t index = 0; index < NUM_ARM_AXIS; index++) {
        if (msg.mode[index] == CONTROL_MODE::FOLLOW || msg.mode[index] == CONTROL_MODE::FAST_FOLLOW) {
            if (abs(msg.reference[index] - (RHWS_encoder_pulses.data[ENCODER_INDEX[index]] - bias_pulse[index])) < ERROR_ALLOWANCE[index]) {
                arm_motor_pwms.data[index] = 0;
            } else {
                if (msg.mode[index] == CONTROL_MODE::FOLLOW) {
                    if (msg.reference[index] > (RHWS_encoder_pulses.data[ENCODER_INDEX[index]] - bias_pulse[index])) {
                        arm_motor_pwms.data[index] = FOLLOW_PWM[index];
                    } else {
                        arm_motor_pwms.data[index] = -FOLLOW_PWM[index];
                    }
                } else if (msg.mode[index] == CONTROL_MODE::FAST_FOLLOW) {
                    if (abs(msg.reference[index] - (RHWS_encoder_pulses.data[ENCODER_INDEX[index]] - bias_pulse[index])) > FAST_DISABLE_ALLOWANCE[index]) {
                        int64_t output = (double(msg.reference[index] - (RHWS_encoder_pulses.data[ENCODER_INDEX[index]] - bias_pulse[index])) * FOLLOW_PWM[index] / FAST_DISABLE_ALLOWANCE[index]) * 0.5;
                        output = constrain_with_minimum_output(output, -MAX_PWM_STMHAL, -ORIGIN_PWM[index], ORIGIN_PWM[index], MAX_PWM_STMHAL);
                        arm_motor_pwms.data[index] = output;
                    } else {
                        if (msg.reference[index] > (RHWS_encoder_pulses.data[ENCODER_INDEX[index]] - bias_pulse[index])) {
                            arm_motor_pwms.data[index] = ORIGIN_PWM[index];
                        } else {
                            arm_motor_pwms.data[index] = -ORIGIN_PWM[index];
                        }
                    }
                }
            }
            if (arm_motor_pwms.data[index] < 0 && (msg.mode[index] == CONTROL_MODE::FOLLOW || msg.mode[index] == CONTROL_MODE::FAST_FOLLOW)) {
                if (RHWS_switch_states.data[SWITCH_INDEX[index]]) {
                    arm_motor_pwms.data[index] = 0;
                }
            }
        } else if (msg.mode[index] == CONTROL_MODE::ORIGIN) {
            arm_motor_pwms.data[index] = -ORIGIN_PWM[index];
            if (RHWS_switch_states.data[SWITCH_INDEX[index]]) {
                bias_pulse[index] = RHWS_encoder_pulses.data[ENCODER_INDEX[index]];
                arm_motor_pwms.data[index] = 0;
            }
        } else {
            arm_motor_pwms.data[index] = 0;
        }

        if (msg.mode[index] == CONTROL_MODE::ORIGIN) {
            if (RHWS_switch_states.data[SWITCH_INDEX[index]]) {
                FeedbackState_arm_controller.is_ended[index] = true;
            } else {
                FeedbackState_arm_controller.is_ended[index] = false;
            }
        } else {
            if (IS_INSIDE(msg.reference[index], (RHWS_encoder_pulses.data[ENCODER_INDEX[index]] - bias_pulse[index]), ERROR_ALLOWANCE[index])) {
                FeedbackState_arm_controller.is_ended[index] = true;
            } else {
                FeedbackState_arm_controller.is_ended[index] = false;
            }
        }

        FeedbackState_arm_controller.reference_feedbackside[index] = msg.reference[index];
        FeedbackState_arm_controller.mode_feedbackside[index] = msg.mode[index];
        MechanismReport.running_mode[index] = msg.mode[index];
        MechanismReport.state_limit[index] = RHWS_switch_states.data[SWITCH_INDEX[index]];
        MechanismReport.state_pulse[index] = RHWS_encoder_pulses.data[ENCODER_INDEX[index]] - bias_pulse[index];
        MechanismReport.reference[index] = msg.reference[index];
    }

    // if (msg.mode[axis_X] == CONTROL_MODE::FOLLOW || msg.mode[axis_X] == CONTROL_MODE::FAST_FOLLOW) {
    //     if (abs(msg.reference[axis_X] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_X] - bias_X_pulse)) < ERROR_ALLOWANCE_ARM_X) {
    //         arm_motor_pwms.data[axis_X] = 0;
    //     } else {
    //         if (msg.mode[axis_X] == CONTROL_MODE::FOLLOW) {
    //             if (msg.reference[axis_X] > (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_X] - bias_X_pulse)) {
    //                 arm_motor_pwms.data[axis_X] = FOLLOW_PWM_ARM_X;
    //             } else {
    //                 arm_motor_pwms.data[axis_X] = -FOLLOW_PWM_ARM_X;
    //             }
    //         } else if (msg.mode[axis_X] == CONTROL_MODE::FAST_FOLLOW) {
    //             if (abs(msg.reference[axis_X] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_X] - bias_X_pulse)) > FAST_DISABLE_ALLOWANCE_ARM_X) {
    //                 int64_t output = (double(msg.reference[axis_X] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_X] - bias_X_pulse)) * FOLLOW_PWM_ARM_X / FAST_DISABLE_ALLOWANCE_ARM_X) * 0.5;
    //                 output = constrain_with_minimum_output(output, -MAX_PWM_STMHAL, -ORIGIN_PWM_ARM_X, ORIGIN_PWM_ARM_X, MAX_PWM_STMHAL);
    //                 arm_motor_pwms.data[axis_X] = output;
    //             } else {
    //                 if (msg.reference[axis_X] > (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_X] - bias_X_pulse)) {
    //                     arm_motor_pwms.data[axis_X] = ORIGIN_PWM_ARM_X;
    //                 } else {
    //                     arm_motor_pwms.data[axis_X] = -ORIGIN_PWM_ARM_X;
    //                 }
    //             }
    //         }
    //     }
    // } else if (msg.mode[axis_X] == CONTROL_MODE::ORIGIN) {
    //     arm_motor_pwms.data[axis_X] = -ORIGIN_PWM_ARM_X;
    //     if (RHWS_switch_states.data[SWITCH_INDEX_ARM_X]) {
    //         bias_X_pulse = RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_X];
    //         arm_motor_pwms.data[axis_X] = 0;
    //     }
    // } else {
    //     arm_motor_pwms.data[axis_X] = 0;
    // }

    // if (msg.mode[axis_Y] == CONTROL_MODE::FOLLOW || msg.mode[axis_Y] == CONTROL_MODE::FAST_FOLLOW) {
    //     if (abs(msg.reference[axis_Y] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] - bias_Y_pulse)) < ERROR_ALLOWANCE_ARM_Y) {
    //         arm_motor_pwms.data[axis_Y] = 0;
    //     } else {
    //         if (msg.mode[axis_Y] == CONTROL_MODE::FOLLOW) {
    //             if (msg.reference[axis_Y] > (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] - bias_Y_pulse)) {
    //                 arm_motor_pwms.data[axis_Y] = FOLLOW_PWM_ARM_Y;
    //             } else {
    //                 arm_motor_pwms.data[axis_Y] = -FOLLOW_PWM_ARM_Y;
    //             }
    //         } else if (msg.mode[axis_Y] == CONTROL_MODE::FAST_FOLLOW) {
    //             ROS_INFO("FAST MODE");
    //             if (abs(msg.reference[axis_Y] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] - bias_Y_pulse)) > FAST_DISABLE_ALLOWANCE_ARM_Y) {
    //                 ROS_INFO("FAST MODE RANGE");
    //                 int64_t output = (double(msg.reference[axis_Y] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] - bias_Y_pulse)) * FOLLOW_PWM_ARM_Y / FAST_DISABLE_ALLOWANCE_ARM_Y) * 0.5;
    //                 ROS_INFO("output:%ld", output);
    //                 output = constrain_with_minimum_output(output, -MAX_PWM_STMHAL, -FOLLOW_PWM_ARM_Y, FOLLOW_PWM_ARM_Y, MAX_PWM_STMHAL);
    //                 ROS_INFO("output:%ld", output);
    //                 ROS_INFO("msg.reference[axis_Y]:%lf", msg.reference[axis_Y]);
    //                 ROS_INFO("RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y]:%d", RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y]);
    //                 ROS_INFO("bias_Y_pulse:%d", bias_Y_pulse);
    //                 ROS_INFO("double(msg.reference[axis_Y] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] - bias_Y_pulse)):%lf", double(msg.reference[axis_Y] - (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] - bias_Y_pulse)));
    //                 arm_motor_pwms.data[axis_Y] = output;
    //             } else {
    //                 if (msg.reference[axis_Y] > (RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] - bias_Y_pulse)) {
    //                     arm_motor_pwms.data[axis_Y] = FOLLOW_PWM_ARM_Y;
    //                 } else {
    //                     arm_motor_pwms.data[axis_Y] = -FOLLOW_PWM_ARM_Y;
    //                 }
    //             }
    //         }
    //     }
    // } else if (msg.mode[axis_Y] == CONTROL_MODE::ORIGIN) {
    //     arm_motor_pwms.data[axis_Y] = -ORIGIN_PWM_ARM_Y;
    //     if (RHWS_switch_states.data[SWITCH_INDEX_ARM_Y]) {
    //         bias_Y_pulse = RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y];
    //         arm_motor_pwms.data[axis_Y] = 0;
    //     }
    // } else {
    //     arm_motor_pwms.data[axis_Y] = 0;
    // }
}

int main(int argv, char **argc) {
    ros::init(argv, argc, "ARM CONTROLLER");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Subscriber sub_ControlState_arm_controller = nh.subscribe("ControlState/arm_controller", QUEUE_SIZE_SUBSCRIBER, callback_arm_controller);
    ros::Publisher pub_FeedbackState_arm_controller = nh.advertise<commissioning_robot::FeedbackState>("FeedbackState/arm_controller", QUEUE_SIZE_PUBLISHER);
    FeedbackState_arm_controller.is_ended.resize(NUM_ARM_AXIS);
    FeedbackState_arm_controller.current.resize(NUM_ARM_AXIS);
    FeedbackState_arm_controller.reference_feedbackside.resize(NUM_ARM_AXIS);
    FeedbackState_arm_controller.mode_feedbackside.resize(NUM_ARM_AXIS);

    ros::Publisher pub_arm_motor_pwms = nh.advertise<std_msgs::Int16MultiArray>("arm_motor_pwms", QUEUE_SIZE_PUBLISHER);
    arm_motor_pwms.data.resize(NUM_ARM_AXIS);

    ros::Publisher pub_MechanismReport = nh.advertise<commissioning_robot::MechanismReport>("MechanismReport/arm_controller", QUEUE_SIZE_PUBLISHER);
    MechanismReport.running_mode.resize(NUM_ARM_AXIS);
    MechanismReport.state_limit.resize(NUM_ARM_AXIS);
    MechanismReport.state_pulse.resize(NUM_ARM_AXIS);
    MechanismReport.reference.resize(NUM_ARM_AXIS);

    ros::Subscriber sub_RHWS_encoder_pulses = nh.subscribe("RHWS/encoder_pulses", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_encoder_pulses);
    ros::Subscriber sub_RHWS_switch_states = nh.subscribe("RHWS/switch_states", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_switch_states);
    RHWS_encoder_pulses.data.resize(NUM_ENCODER);
    RHWS_switch_states.data.resize(NUM_SWITCH);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        pub_arm_motor_pwms.publish(arm_motor_pwms);
        pub_FeedbackState_arm_controller.publish(FeedbackState_arm_controller);
        pub_MechanismReport.publish(MechanismReport);
    }
}