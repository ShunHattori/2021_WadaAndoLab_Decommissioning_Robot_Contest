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

std_msgs::Int16MultiArray cable_motor_pwms;
commissioning_robot::FeedbackState FeedbackState_cable_manager_controller;
static int bias_cable_arm_pulse = 0, bias_cable_push_pulse = 0;

void callback_cable_manager_controller(const commissioning_robot::ControlState &msg) {
    ROS_INFO("##### callback_cable_manager_controller #####");
    ROS_INFO("RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR]:%d", RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR]);
    ROS_INFO("RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR]:%d", RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR]);
    ROS_INFO("RHWS_switch_states.data[SWITCH_INDEX_CABLE_ARM_MOTOR]:%d", RHWS_switch_states.data[SWITCH_INDEX_CABLE_ARM_MOTOR]);
    ROS_INFO("RHWS_switch_states.data[SWITCH_INDEX_CABLE_BELT_MOTOR]:%d", RHWS_switch_states.data[SWITCH_INDEX_CABLE_BELT_MOTOR]);
    ROS_INFO("bias_cable_arm_pulse:%d", bias_cable_arm_pulse);
    ROS_INFO("bias_cable_push_pulse:%d", bias_cable_push_pulse);
    ROS_INFO("#############################################");

    // 追従動作モードのとき
    if (msg.mode[ARM_MOTOR] == CONTROL_MODE::FOLLOW) {
        // 目標値と現在値の誤差が許容値以内のとき、出力をゼロに設定
        if (abs(msg.reference[ARM_MOTOR] - (RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR] - bias_cable_arm_pulse)) < ERROR_ALLOWANCE_CABLE_ARM_MOTOR) {
            cable_motor_pwms.data[ARM_MOTOR] = 0;
        }
        // 目標値と現在値の誤差が許容範囲外のとき
        else {
            // 目標値が現在値よりも大きいとき
            if (msg.reference[ARM_MOTOR] > (RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR] - bias_cable_arm_pulse)) {
                cable_motor_pwms.data[ARM_MOTOR] = FOLLOW_PWM_CABLE_ARM_MOTOR;
            } else {
                cable_motor_pwms.data[ARM_MOTOR] = -FOLLOW_PWM_CABLE_ARM_MOTOR;
            }
        }
        // モータ出力が逆回転かつ、制御モードが追従動作のとき、リミットスイッチが押されていれば出力をゼロに設定する（安全高）
        // ただし、本当にリミットスイッチの行き過ぎではない可能性があるので、パルスバイアスは更新しない（手で押した場合を想定）
        if (cable_motor_pwms.data[ARM_MOTOR] < 0 && msg.mode[ARM_MOTOR] == CONTROL_MODE::FOLLOW) {
            if (RHWS_switch_states.data[SWITCH_INDEX_CABLE_ARM_MOTOR]) {
                cable_motor_pwms.data[ARM_MOTOR] = 0;
            }
        }
    }
    // 原点回帰モードのとき
    else if (msg.mode[ARM_MOTOR] == CONTROL_MODE::ORIGIN) {
        cable_motor_pwms.data[ARM_MOTOR] = -ORIGIN_PWM_CABLE_ARM_MOTOR;
        // スイッチが押されているとき、パルスバイアスを更新して、出力をゼロに設定する
        if (RHWS_switch_states.data[SWITCH_INDEX_CABLE_ARM_MOTOR]) {
            bias_cable_arm_pulse = RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR];
            cable_motor_pwms.data[ARM_MOTOR] = 0;
        }
    } else if (msg.mode[ARM_MOTOR] == CONTROL_MODE::MANUAL) {
        cable_motor_pwms.data[ARM_MOTOR] = msg.manual[ARM_MOTOR];
    }
    // その他制御状態（想定はSTOP）のとき
    else {
        // 出力を無条件にゼロに設定
        cable_motor_pwms.data[ARM_MOTOR] = 0;
    }

    if (msg.mode[BELT_MOTOR] == CONTROL_MODE::FOLLOW) {
        if (abs(msg.reference[BELT_MOTOR] - (RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR] - bias_cable_push_pulse)) < ERROR_ALLOWANCE_CABLE_BELT_MOTOR) {
            cable_motor_pwms.data[BELT_MOTOR] = 0;
        } else {
            if (msg.mode[BELT_MOTOR] == CONTROL_MODE::FOLLOW) {
                if (msg.reference[BELT_MOTOR] > (RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR] - bias_cable_push_pulse)) {
                    cable_motor_pwms.data[BELT_MOTOR] = FOLLOW_PWM_CABLE_BELT_MOTOR;
                } else {
                    cable_motor_pwms.data[BELT_MOTOR] = -FOLLOW_PWM_CABLE_BELT_MOTOR;
                }
            }
        }
        if (cable_motor_pwms.data[BELT_MOTOR] < 0 && msg.mode[BELT_MOTOR] == CONTROL_MODE::FOLLOW) {
            if (RHWS_switch_states.data[SWITCH_INDEX_CABLE_BELT_MOTOR]) {
                cable_motor_pwms.data[BELT_MOTOR] = 0;
            }
        }
    } else if (msg.mode[BELT_MOTOR] == CONTROL_MODE::ORIGIN) {
        cable_motor_pwms.data[BELT_MOTOR] = -ORIGIN_PWM_CABLE_BELT_MOTOR;
        if (RHWS_switch_states.data[SWITCH_INDEX_CABLE_BELT_MOTOR]) {
            bias_cable_push_pulse = RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR];
            cable_motor_pwms.data[BELT_MOTOR] = 0;
        }
    } else {
        cable_motor_pwms.data[BELT_MOTOR] = 0;
    }

    if (msg.mode[ARM_MOTOR] == CONTROL_MODE::ORIGIN) {
        if (RHWS_switch_states.data[SWITCH_INDEX_CABLE_ARM_MOTOR]) {
            FeedbackState_cable_manager_controller.is_ended[ARM_MOTOR] = true;
        } else {
            FeedbackState_cable_manager_controller.is_ended[ARM_MOTOR] = false;
        }
    } else {
        if (IS_INSIDE(msg.reference[ARM_MOTOR], (RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR] - bias_cable_arm_pulse), ERROR_ALLOWANCE_CABLE_ARM_MOTOR)) {
            FeedbackState_cable_manager_controller.is_ended[ARM_MOTOR] = true;
        } else {
            FeedbackState_cable_manager_controller.is_ended[ARM_MOTOR] = false;
        }
    }

    if (msg.mode[BELT_MOTOR] == CONTROL_MODE::ORIGIN) {
        if (RHWS_switch_states.data[SWITCH_INDEX_CABLE_BELT_MOTOR]) {
            FeedbackState_cable_manager_controller.is_ended[BELT_MOTOR] = true;
        } else {
            FeedbackState_cable_manager_controller.is_ended[BELT_MOTOR] = false;
        }
    } else {
        if (IS_INSIDE(msg.reference[BELT_MOTOR], (RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR] - bias_cable_push_pulse), ERROR_ALLOWANCE_CABLE_BELT_MOTOR)) {
            FeedbackState_cable_manager_controller.is_ended[BELT_MOTOR] = true;
        } else {
            FeedbackState_cable_manager_controller.is_ended[BELT_MOTOR] = false;
        }
    }

    FeedbackState_cable_manager_controller.reference_feedbackside[ARM_MOTOR] = msg.reference[ARM_MOTOR];
    FeedbackState_cable_manager_controller.reference_feedbackside[BELT_MOTOR] = msg.reference[BELT_MOTOR];
    FeedbackState_cable_manager_controller.mode_feedbackside[ARM_MOTOR] = msg.mode[ARM_MOTOR];
    FeedbackState_cable_manager_controller.mode_feedbackside[BELT_MOTOR] = msg.mode[BELT_MOTOR];
    MechanismReport.running_mode[ARM_MOTOR] = msg.mode[ARM_MOTOR];
    MechanismReport.state_limit[ARM_MOTOR] = RHWS_switch_states.data[SWITCH_INDEX_CABLE_ARM_MOTOR];
    MechanismReport.state_pulse[ARM_MOTOR] = RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR] - bias_cable_arm_pulse;
    MechanismReport.reference[ARM_MOTOR] = msg.reference[ARM_MOTOR];
    MechanismReport.running_mode[BELT_MOTOR] = msg.mode[BELT_MOTOR];
    MechanismReport.state_limit[BELT_MOTOR] = RHWS_switch_states.data[SWITCH_INDEX_CABLE_BELT_MOTOR];
    MechanismReport.state_pulse[BELT_MOTOR] = RHWS_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR] - bias_cable_push_pulse;
    MechanismReport.reference[BELT_MOTOR] = msg.reference[BELT_MOTOR];
}

int main(int argv, char **argc) {
    ros::init(argv, argc, "CABLE MANAGER CONTROLLER");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Subscriber sub_ControlState_cable_manager_controller = nh.subscribe("ControlState/cable_manager_controller", QUEUE_SIZE_SUBSCRIBER, callback_cable_manager_controller);
    ros::Publisher pub_FeedbackState_cable_manager_controller = nh.advertise<commissioning_robot::FeedbackState>("FeedbackState/cable_manager_controller", QUEUE_SIZE_PUBLISHER);
    FeedbackState_cable_manager_controller.is_ended.resize(NUM_CABLE_MOTOR);
    FeedbackState_cable_manager_controller.current.resize(NUM_CABLE_MOTOR);
    FeedbackState_cable_manager_controller.reference_feedbackside.resize(NUM_CABLE_MOTOR);
    FeedbackState_cable_manager_controller.mode_feedbackside.resize(NUM_CABLE_MOTOR);

    ros::Publisher pub_MechanismReport = nh.advertise<commissioning_robot::MechanismReport>("MechanismReport/cable_manager_controller", QUEUE_SIZE_PUBLISHER);
    MechanismReport.running_mode.resize(NUM_CABLE_MOTOR);
    MechanismReport.state_limit.resize(NUM_CABLE_MOTOR);
    MechanismReport.state_pulse.resize(NUM_CABLE_MOTOR);
    MechanismReport.reference.resize(NUM_CABLE_MOTOR);

    ros::Subscriber sub_RHWS_encoder_pulses = nh.subscribe("RHWS/encoder_pulses", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_encoder_pulses);
    ros::Subscriber sub_RHWS_switch_states = nh.subscribe("RHWS/switch_states", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_switch_states);
    RHWS_encoder_pulses.data.resize(NUM_ENCODER);
    RHWS_switch_states.data.resize(NUM_SWITCH);

    ros::Publisher pub_cable_motor_pwms = nh.advertise<std_msgs::Int16MultiArray>("cable_motor_pwms", QUEUE_SIZE_SUBSCRIBER);
    cable_motor_pwms.data.resize(NUM_CABLE_MOTOR);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        pub_cable_motor_pwms.publish(cable_motor_pwms);
        pub_FeedbackState_cable_manager_controller.publish(FeedbackState_cable_manager_controller);
        pub_MechanismReport.publish(MechanismReport);
    }
}