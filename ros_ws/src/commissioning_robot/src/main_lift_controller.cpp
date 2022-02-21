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

std_msgs::Int16MultiArray main_lift_motor_pwms;
constexpr double ML_p_gain = 30.00, ML_i_gain = 0.15, ML_d_gain = 0;
position_motordrive main_lift(ML_p_gain, ML_i_gain, ML_d_gain);
commissioning_robot::FeedbackState FeedbackState_main_lift_controller;

void callback_main_lift_controller(const commissioning_robot::ControlState &msg) {
    if (msg.mode.size() != NUM_MAIN_LIFT_AXIS || msg.reference.size() != NUM_MAIN_LIFT_AXIS)
        return;

    int8_t enc_sign = 1;
    double robot_core_reference_pulse, mutiplier_pos_to_pulse = 5384;                              //最大パルス数要計測  -> 3500 pulse が最大値  (0.65 * x = 3500) -> x = 5384
    main_lift.set_current_pulse(enc_sign * RHWS_encoder_pulses.data.at(ENCODER_INDEX_MAIN_LIFT));  // OK
    robot_core_reference_pulse = (msg.reference[axis_MAIN] + msg.manual[axis_MAIN]) * mutiplier_pos_to_pulse;

    static bool prev_CONTROL_MODE, is_accel_period = false;
    static const double ref_pulse_detect_change_threshold = 0.1 * mutiplier_pos_to_pulse;  // 0.1m -> convert to pulse
    static double prev_ref_pulse, curr_ref_pulse;
    curr_ref_pulse = robot_core_reference_pulse;

    // 目標の変化を検出。down up 変数に上下どちら向きに目標値が変化したのかフラグを格納
    // 後のS加減速計算機の計算式分岐に使用する。
    static bool is_ref_changed = false, ref_changed_to_up = false, ref_changed_to_down = false;
    if (abs(prev_ref_pulse - curr_ref_pulse) > ref_pulse_detect_change_threshold) {
        is_ref_changed = true;
        if (curr_ref_pulse > prev_ref_pulse) {
            ref_changed_to_up = true;
            ref_changed_to_down = false;
        }
        if (prev_ref_pulse > curr_ref_pulse) {
            ref_changed_to_up = false;
            ref_changed_to_down = true;
        }
    }

    double now = ros::Time::now().toSec();
    static double flag_changed_time = 0, pulse_s_curve_top_reference = 0;
    if (is_ref_changed) {
        flag_changed_time = now;
        pulse_s_curve_top_reference = main_lift.get_reference_pulse();  //下降時に最後に設定した最大値が必要だからPCクラスから取得している
        is_accel_period = true;
        is_ref_changed = false;
    }

    double period_from_flag_changed = now - flag_changed_time;
    static double result_ref_pulse;
    if (is_accel_period) {
        if (ref_changed_to_up) {
            result_ref_pulse = curr_ref_pulse * (0.5 - 0.5 * cos(period_from_flag_changed / ACCEL_PERIOD_MAIN_LIFT * M_PI));
        }
        // else if (ref_changed_to_down) {
        //     result_ref_pulse = pulse_s_curve_top_reference * (0.55 + 0.45 * cos(period_from_flag_changed / ACCEL_PERIOD_MAIN_LIFT * M_PI));
        // }
        if (period_from_flag_changed > ACCEL_PERIOD_MAIN_LIFT) {
            is_accel_period = false;
            result_ref_pulse = curr_ref_pulse;  // MANUAL動作させたときにROBOTCOREが持っている目標値と、MLCが持っているpulse_s_curve_top_referenceに相違があって急激にPIDが効く（MANUALはそんなに動かさないで・・・）
        }
    } else {
        result_ref_pulse = curr_ref_pulse;
    }

    ROS_INFO("  ");
    ROS_INFO("result_ref_pulse:%lf", result_ref_pulse);
    ROS_INFO("is_ref_changed:%d", is_ref_changed);
    ROS_INFO("ref_changed_to_up:%d", ref_changed_to_up);
    ROS_INFO("ref_changed_to_down:%d", ref_changed_to_down);
    ROS_INFO("is_accel_period:%d", is_accel_period);
    ROS_INFO("period_from_flag_changed:%lf", period_from_flag_changed);
    ROS_INFO("pulse_s_curve_top_reference:%lf", pulse_s_curve_top_reference);
    ROS_INFO("curr_ref_pulse:%lf", curr_ref_pulse);
    ROS_INFO("flag_changed_time:%lf", flag_changed_time);

    if (msg.mode[axis_MAIN] == CONTROL_MODE::FOLLOW) {
        main_lift.set_reference_pulse_from_origin(result_ref_pulse);
        main_lift.set_limit_reference_pulse(-100000, 100000);
        main_lift.set_i_clamp(-5000, 5000);
        main_lift.set_minimum_pwm(-MIN_PWM_MAIN_LIFT);
        main_lift.set_mode(position_motordrive::FOLLOW_REFERENCE);
    } else if (msg.mode[axis_MAIN] == CONTROL_MODE::ORIGIN) {
        main_lift.set_limit_state(position_motordrive::BOTTOM, RHWS_switch_states.data[SWITCH_INDEX_MAIN_LIFT]);
        main_lift.set_mode(position_motordrive::GET_ORIGIN);
        main_lift.reset_pid_cv();
    }

    prev_CONTROL_MODE = msg.mode[axis_MAIN];
    prev_ref_pulse = curr_ref_pulse;

    main_lift.update();
    // ROS_INFO("main_lift[i].get_pwm():%lf", main_lift[i].get_pwm());
    static const int8_t pwm_sign = 1;
    double pwm = pwm_sign * main_lift.get_pwm();
    main_lift_motor_pwms.data[axis_MAIN] = constrain(pwm, -MAX_PWM_MAIN_LIFT, MAX_PWM_MAIN_LIFT);  //出力最大値をここで設定
    if (msg.mode[axis_MAIN] == CONTROL_MODE::STOP) {
        main_lift.reset_pid_cv();
        main_lift_motor_pwms.data[axis_MAIN] = 0;
    }

    // ROS_INFO("abs((msg.reference[axis_MAIN] * mutiplier_pos_to_pulse) - (RHWS_encoder_pulses.data.at(ENCODER_INDEX_MAIN_LIFT) - main_lift.get_origin_pulse())):%lf", abs((msg.reference[axis_MAIN] * mutiplier_pos_to_pulse) - (RHWS_encoder_pulses.data.at(ENCODER_INDEX_MAIN_LIFT) - main_lift.get_origin_pulse())));
    if (abs((msg.reference[axis_MAIN] * mutiplier_pos_to_pulse) - (RHWS_encoder_pulses.data.at(ENCODER_INDEX_MAIN_LIFT) - main_lift.get_origin_pulse())) < ERROR_ALLOWANCE_MAIN_LIFT) {  // get_ref_pulseはS加減速のせいで急に変わらないからREF比較の時にTRUEが入ってしまう。そこでMSGのREFとENCからORIGINを引いたパルスを比較する
        FeedbackState_main_lift_controller.is_ended[axis_MAIN] = true;
    } else {
        FeedbackState_main_lift_controller.is_ended[axis_MAIN] = false;
    }
    FeedbackState_main_lift_controller.reference_feedbackside[axis_MAIN] = msg.reference[axis_MAIN];
    FeedbackState_main_lift_controller.mode_feedbackside[axis_MAIN] = msg.mode[axis_MAIN];
    MechanismReport.running_mode[axis_MAIN] = msg.mode[axis_MAIN];
    MechanismReport.state_limit[axis_MAIN] = RHWS_switch_states.data[SWITCH_INDEX_MAIN_LIFT];
    MechanismReport.state_pulse[axis_MAIN] = RHWS_encoder_pulses.data.at(ENCODER_INDEX_MAIN_LIFT) - main_lift.get_origin_pulse();
    MechanismReport.reference[axis_MAIN] = robot_core_reference_pulse;
}

int main(int argv, char **argc) {
    ros::init(argv, argc, "MAIN LIFT CONTROLLER");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Subscriber sub_ControlState_main_lift_controller = nh.subscribe("ControlState/main_lift_controller", QUEUE_SIZE_SUBSCRIBER, callback_main_lift_controller);
    ros::Publisher pub_FeedbackState_main_lift_controller = nh.advertise<commissioning_robot::FeedbackState>("FeedbackState/main_lift_controller", QUEUE_SIZE_PUBLISHER);
    FeedbackState_main_lift_controller.is_ended.resize(NUM_MAIN_LIFT_AXIS);
    FeedbackState_main_lift_controller.current.resize(NUM_MAIN_LIFT_AXIS);
    FeedbackState_main_lift_controller.reference_feedbackside.resize(NUM_MAIN_LIFT_AXIS);
    FeedbackState_main_lift_controller.mode_feedbackside.resize(NUM_MAIN_LIFT_AXIS);

    ros::Publisher pub_main_lift_motor_pwms = nh.advertise<std_msgs::Int16MultiArray>("main_lift_motor_pwms", QUEUE_SIZE_PUBLISHER);
    main_lift_motor_pwms.data.resize(NUM_MAIN_LIFT_AXIS);

    ros::Publisher pub_MechanismReport = nh.advertise<commissioning_robot::MechanismReport>("MechanismReport/main_lift_controller", QUEUE_SIZE_PUBLISHER);
    MechanismReport.reference.resize(NUM_MAIN_LIFT_AXIS);
    MechanismReport.running_mode.resize(NUM_MAIN_LIFT_AXIS);
    MechanismReport.state_limit.resize(NUM_MAIN_LIFT_AXIS);
    MechanismReport.state_pulse.resize(NUM_MAIN_LIFT_AXIS);

    ros::Subscriber sub_RHWS_encoder_pulses = nh.subscribe("RHWS/encoder_pulses", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_encoder_pulses);
    ros::Subscriber sub_RHWS_switch_states = nh.subscribe("RHWS/switch_states", QUEUE_SIZE_SUBSCRIBER, callback_RHWS_switch_states);
    RHWS_encoder_pulses.data.resize(NUM_ENCODER);
    RHWS_switch_states.data.resize(NUM_SWITCH);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        pub_main_lift_motor_pwms.publish(main_lift_motor_pwms);
        pub_FeedbackState_main_lift_controller.publish(FeedbackState_main_lift_controller);
        pub_MechanismReport.publish(MechanismReport);
    }
}