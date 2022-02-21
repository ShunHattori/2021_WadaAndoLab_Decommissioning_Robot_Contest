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

std_msgs::Float32 RHWS_IMU_pitch_biased;
void callback_RHWS_IMU_pitch_biased(const std_msgs::Float32 &msg) {
    RHWS_IMU_pitch_biased.data = msg.data;
}

std_msgs::Int16MultiArray drive_lift_motor_pwms;
// constexpr double DL_p_gain = 20.00, DL_i_gain = 0, DL_d_gain = 0;
constexpr double DL_p_gain = 80.00, DL_i_gain = 0, DL_d_gain = 0;
pid_controller drive_lift[NUM_DRIVEUNIT] = {
    pid_controller(DL_p_gain, DL_i_gain, DL_d_gain),
    pid_controller(DL_p_gain, DL_i_gain, DL_d_gain),
    pid_controller(DL_p_gain, DL_i_gain, DL_d_gain),
    pid_controller(DL_p_gain, DL_i_gain, DL_d_gain),
};

// constexpr double pitch_p_gain = 5.00, pitch_i_gain = 3.0, pitch_d_gain = 0;
constexpr double pitch_p_gain = 130, pitch_i_gain = 2.2, pitch_d_gain = 0;  // p=130 i=2.2
pid_controller pid_pitch(pitch_p_gain, pitch_i_gain, pitch_d_gain);

void callback_drive_lift_controller(const commissioning_robot::ControlState &msg) {
    if (msg.mode.size() != NUM_DRIVEUNIT || msg.reference.size() != NUM_DRIVEUNIT)
        return;

    // max 0.334m and 2800pulse
    int8_t enc_sign[NUM_DRIVEUNIT] = {1, -1, 1, -1};
    int8_t lift_sign[NUM_DRIVEUNIT] = {-1, -1, 1, 1};  // 3 ok 2 ok 1 ok 4 ok
    int8_t encoder_index[NUM_DRIVEUNIT] = {1, 3, 6, 8};
    int8_t switch_index[NUM_DRIVEUNIT] = {0, 2, 5, 7};
    double drive_lift_reference_pulse[NUM_DRIVEUNIT], mutiplier_pos_to_pulse = -8383;
    static long bottom_pulse[4] = {0, 0, 0, 0};

    pid_pitch.set_reference(0.0);
    pid_pitch.set_current(RHWS_IMU_pitch_biased.data);
    pid_pitch.set_i_clamp(-40, 40);
    pid_pitch.update();
    for (int i = 0; i < NUM_DRIVEUNIT; i++) {
        double output = 0;
        drive_lift_reference_pulse[i] = lift_sign[i] * pid_pitch.get_control_variable();
        for (int k = 0; k < NUM_DRIVEUNIT; k++) {
            if (drive_lift_reference_pulse[k] > 0) {
                for (int m = 0; m < NUM_DRIVEUNIT; m++) {
                    if (m != k)
                        drive_lift_reference_pulse[m] += -drive_lift_reference_pulse[k];
                }
                drive_lift_reference_pulse[k] = 0;
            }
        }
        drive_lift[i].set_i_clamp(-5000, 5000);
        drive_lift[i].set_current(RHWS_encoder_pulses.data[encoder_index[i]] - bottom_pulse[i]);
        drive_lift[i].set_reference(drive_lift_reference_pulse[i] - msg.reference[i]);
        drive_lift[i].update();

        if (msg.mode[i] == CONTROL_MODE::FOLLOW) {
            output = constrain(drive_lift[i].get_control_variable(), -7500, 7500);
            if (RHWS_switch_states.data[switch_index[i]] == 1) {
                if (output > 0) {
                    output = 0;  // FOLLOWモードの時に下向きの出力がある時にリミットスイッチに触れたら出力をゼロに設定。
                }
            }
        } else if (msg.mode[i] == CONTROL_MODE::ORIGIN) {
            output = 5000;
            drive_lift[i].reset_control_variable();
            if (RHWS_switch_states.data[switch_index[i]] == 1) {
                output = 0;
            }
        }

        if (RHWS_switch_states.data[switch_index[i]] == 1) {
            bottom_pulse[i] = RHWS_encoder_pulses.data[encoder_index[i]];
        }

        drive_lift_motor_pwms.data[i] = output;
        if (msg.mode[i] == CONTROL_MODE::STOP) {
            drive_lift_motor_pwms.data[i] = 0;
        }

        MechanismReport.running_mode[i] = msg.mode[i];
        MechanismReport.state_limit[i] = RHWS_switch_states.data[switch_index[i]];
        MechanismReport.state_pulse[i] = RHWS_encoder_pulses.data[encoder_index[i]] - bottom_pulse[i];
        MechanismReport.reference[i] = msg.reference[i];
    }
}

int main(int argv, char **argc) {
    ros::init(argv, argc, "DRIVE LIFT CONTROLLER");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Subscriber sub_ControlState_drive_lift_controller = nh.subscribe("ControlState/drive_lift_controller", QUEUE_SIZE_SUBSCRIBER, callback_drive_lift_controller);
    ros::Publisher pub_FeedbackState_drive_lift_controller = nh.advertise<commissioning_robot::FeedbackState>("FeedbackState/drive_lift_controller", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_drive_lift_motor_pwms = nh.advertise<std_msgs::Int16MultiArray>("drive_lift_motor_pwms", QUEUE_SIZE_PUBLISHER);
    drive_lift_motor_pwms.data.resize(NUM_DRIVEUNIT);

    ros::Publisher pub_MechanismReport = nh.advertise<commissioning_robot::MechanismReport>("MechanismReport/drive_lift_controller", QUEUE_SIZE_PUBLISHER);
    MechanismReport.running_mode.resize(NUM_DRIVEUNIT);
    MechanismReport.state_limit.resize(NUM_DRIVEUNIT);
    MechanismReport.state_pulse.resize(NUM_DRIVEUNIT);
    MechanismReport.reference.resize(NUM_DRIVEUNIT);

    ros::Subscriber sub_RHWS_encoder_pulses = nh.subscribe("RHWS/encoder_pulses", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_encoder_pulses);
    ros::Subscriber sub_RHWS_switch_states = nh.subscribe("RHWS/switch_states", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_switch_states);
    ros::Subscriber sub_RHWS_IMU_pitch_biased = nh.subscribe("RHWS/IMU_pitch_biased", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_IMU_pitch_biased);
    RHWS_encoder_pulses.data.resize(NUM_ENCODER);
    RHWS_switch_states.data.resize(NUM_SWITCH);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        pub_drive_lift_motor_pwms.publish(drive_lift_motor_pwms);
        pub_MechanismReport.publish(MechanismReport);
    }
}