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

std_msgs::Int16MultiArray wheel_motor_pwms;
constexpr double MD_p_gain = 2800,
                 MD_i_gain = 350, MD_d_gain = 1000;
velocity_motordrive motor_controllers[NUM_DRIVEUNIT] = {
    velocity_motordrive(MD_p_gain, MD_i_gain, MD_d_gain),
    velocity_motordrive(MD_p_gain, MD_i_gain, MD_d_gain),
    velocity_motordrive(MD_p_gain, MD_i_gain, MD_d_gain),
    velocity_motordrive(MD_p_gain, MD_i_gain, MD_d_gain),
};

void callback_drive_wheel_controller(const commissioning_robot::ControlState &msg) {
    ROS_INFO("callback_drive_wheel_controller called");
    if (msg.reference.size() != NUM_DRIVEUNIT || msg.mode.size() != NUM_DRIVEUNIT)
        return;

    // 注意として、基本的にコントローラ系では正負の変換は禁止
    // 入力側（SIMブリッジ、CANONブリッジ側、ROBOTコア）で正負の調整を行うこと！
    // そうしないと符号の収拾がつかんくなって頭おかしなる

    motor_controllers[0].set_current_pulse(RHWS_encoder_pulses.data[0]);
    motor_controllers[1].set_current_pulse(RHWS_encoder_pulses.data[2]);
    motor_controllers[2].set_current_pulse(RHWS_encoder_pulses.data[5]);
    motor_controllers[3].set_current_pulse(RHWS_encoder_pulses.data[7]);

    for (int i = 0; i < NUM_DRIVEUNIT; i++) {
        if (msg.mode[i] == CONTROL_MODE::STOP) {
            motor_controllers[i].reset_control_variable();
        }
        motor_controllers[i].set_reference_velocity(msg.reference[i]);
        motor_controllers[i].update();

        static constexpr int16_t MAX_MOTOR_PWM = MAX_PWM_STMHAL;
        double pwm = motor_controllers[i].get_pwm();
        wheel_motor_pwms.data[i] = constrain(pwm, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
        wheel_motor_pwms.data[i] = abs(wheel_motor_pwms.data[i]) < 1500 ? 0 : wheel_motor_pwms.data[i];
    }
    MechanismReport.state_pulse[0] = RHWS_encoder_pulses.data[0];
    MechanismReport.state_pulse[1] = RHWS_encoder_pulses.data[2];
    MechanismReport.state_pulse[2] = RHWS_encoder_pulses.data[5];
    MechanismReport.state_pulse[3] = RHWS_encoder_pulses.data[7];
}

int main(int argv, char **argc) {
    ros::init(argv, argc, "DRIVE WHEEL CONTROLLER");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Publisher pub_FeedbackState_drive_wheel_controller = nh.advertise<commissioning_robot::FeedbackState>("FeedbackState/drive_wheel_controller", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_wheel_motor_pwms = nh.advertise<std_msgs::Int16MultiArray>("wheel_motor_pwms", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_MechanismReport = nh.advertise<commissioning_robot::MechanismReport>("MechanismReport/drive_wheel_controller", QUEUE_SIZE_PUBLISHER);
    MechanismReport.state_pulse.resize(NUM_DRIVEUNIT);

    ros::Subscriber sub_ControlState_drive_wheel_controller = nh.subscribe("ControlState/drive_wheel_controller", QUEUE_SIZE_SUBSCRIBER, callback_drive_wheel_controller);
    ros::Subscriber sub_RHWS_encoder_pulses = nh.subscribe("RHWS/encoder_pulses", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_encoder_pulses);
    ros::Subscriber sub_RHWS_switch_states = nh.subscribe("RHWS/switch_states", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_switch_states);
    wheel_motor_pwms.data.resize(NUM_DRIVEUNIT);
    RHWS_encoder_pulses.data.resize(NUM_ENCODER);
    RHWS_switch_states.data.resize(NUM_SWITCH);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        pub_wheel_motor_pwms.publish(wheel_motor_pwms);
        pub_MechanismReport.publish(MechanismReport);
    }
}