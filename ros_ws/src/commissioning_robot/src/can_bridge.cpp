#include <utils.hpp>

std_msgs::UInt8MultiArray can_buffer_ID_6, can_buffer_ID_7, can_buffer_ID_8, can_buffer_ID_9;
std_msgs::UInt8MultiArray can_buffer_ID_10, can_buffer_ID_11, can_buffer_ID_12, can_buffer_ID_13, can_buffer_ID_33;
void callback_CAN_ID_6(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_6.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_6.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_7(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_7.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_7.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_8(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_8.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_8.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_9(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_9.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_9.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_10(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_10.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_10.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_11(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_11.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_11.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_12(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_12.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_12.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_13(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_13.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_13.data[i] = msg.data[i];
    }
}
void callback_CAN_ID_33(const std_msgs::UInt8MultiArray &msg) {
    static bool init = true;
    if (init) {
        can_buffer_ID_33.data.resize(8);
        init = false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        can_buffer_ID_33.data[i] = msg.data[i];
    }
}

std_msgs::Float32 IMU_pitch, IMU_roll;
void IMU_compose() {
    if (can_buffer_ID_33.data.size() != 8)
        return;
    union IMU_data {
        float data;
        uint8_t data_byte[4];
    } pitch, roll;
    for (uint8_t i = 0; i < 4; i++) {
        roll.data_byte[i] = can_buffer_ID_33.data[i];
        pitch.data_byte[i] = can_buffer_ID_33.data[i + 4];
        // pitch.data_byte[i] |= (can_buffer_ID_33.data[i] << ((3 - i) * 8));
        // roll.data_byte[i] |= (can_buffer_ID_33.data[i + 4] << ((3 - i) * 8));
    }
    static bool init_pitch_flag = true;
    static double pitch_offset = 0;
    if (init_pitch_flag == true && pitch.data != 0) {
        init_pitch_flag = false;
        pitch_offset = pitch.data;
    }
    static constexpr int8_t IMU_sign[2] = {1, 1};
    IMU_pitch.data = pitch.data * IMU_sign[0] - pitch_offset;
    IMU_roll.data = roll.data * IMU_sign[1];
}

// RHWS -> robot hardware sensor
std_msgs::Int16MultiArray RHWS_raw_encoder_pulses;
std_msgs::Int32MultiArray RHWS_encoded_encoder_pulses;
void RHWS_encoder_pulses_compose() {  // 9-21 comfirm
    static bool init = true;
    if (init) {
        RHWS_raw_encoder_pulses.data.resize(NUM_ENCODER);
        RHWS_encoded_encoder_pulses.data.resize(NUM_ENCODER);
        init = false;
    }
    if (can_buffer_ID_6.data.size() != 8 || can_buffer_ID_7.data.size() != 8 || can_buffer_ID_8.data.size() != 8 || can_buffer_ID_9.data.size() != 8)
        return;
    if (can_buffer_ID_10.data.size() != 8 || can_buffer_ID_11.data.size() != 8 || can_buffer_ID_12.data.size() != 8 || can_buffer_ID_13.data.size() != 8)
        return;
    RHWS_raw_encoder_pulses.data[0] = int16_t(int16_t(can_buffer_ID_6.data[0] << 8) | can_buffer_ID_6.data[1]);
    RHWS_raw_encoder_pulses.data[1] = int16_t(int16_t(can_buffer_ID_6.data[2] << 8) | can_buffer_ID_6.data[3]);
    RHWS_raw_encoder_pulses.data[2] = int16_t(int16_t(can_buffer_ID_6.data[4] << 8) | can_buffer_ID_6.data[5]);
    RHWS_raw_encoder_pulses.data[3] = int16_t(int16_t(can_buffer_ID_6.data[6] << 8) | can_buffer_ID_6.data[7]);
    RHWS_raw_encoder_pulses.data[4] = int16_t(int16_t(can_buffer_ID_7.data[0] << 8) | can_buffer_ID_7.data[1]);
    RHWS_raw_encoder_pulses.data[5] = int16_t(int16_t(can_buffer_ID_8.data[0] << 8) | can_buffer_ID_8.data[1]);
    RHWS_raw_encoder_pulses.data[6] = int16_t(int16_t(can_buffer_ID_8.data[2] << 8) | can_buffer_ID_8.data[3]);
    RHWS_raw_encoder_pulses.data[7] = int16_t(int16_t(can_buffer_ID_8.data[4] << 8) | can_buffer_ID_8.data[5]);
    RHWS_raw_encoder_pulses.data[8] = int16_t(int16_t(can_buffer_ID_8.data[6] << 8) | can_buffer_ID_8.data[7]);
    RHWS_raw_encoder_pulses.data[9] = int16_t(int16_t(can_buffer_ID_9.data[0] << 8) | can_buffer_ID_9.data[1]);
    RHWS_raw_encoder_pulses.data[10] = int16_t(int16_t(can_buffer_ID_10.data[0] << 8) | can_buffer_ID_10.data[1]);
    RHWS_raw_encoder_pulses.data[11] = int16_t(int16_t(can_buffer_ID_10.data[2] << 8) | can_buffer_ID_10.data[3]);
    RHWS_raw_encoder_pulses.data[12] = int16_t(int16_t(can_buffer_ID_10.data[4] << 8) | can_buffer_ID_10.data[5]);
    RHWS_raw_encoder_pulses.data[13] = int16_t(int16_t(can_buffer_ID_10.data[6] << 8) | can_buffer_ID_10.data[7]);
    RHWS_raw_encoder_pulses.data[14] = int16_t(int16_t(can_buffer_ID_11.data[0] << 8) | can_buffer_ID_11.data[1]);
    RHWS_raw_encoder_pulses.data[15] = int16_t(int16_t(can_buffer_ID_12.data[0] << 8) | can_buffer_ID_12.data[1]);
    RHWS_raw_encoder_pulses.data[16] = int16_t(int16_t(can_buffer_ID_12.data[2] << 8) | can_buffer_ID_12.data[3]);
    RHWS_raw_encoder_pulses.data[17] = int16_t(int16_t(can_buffer_ID_12.data[4] << 8) | can_buffer_ID_12.data[5]);
    RHWS_raw_encoder_pulses.data[18] = int16_t(int16_t(can_buffer_ID_12.data[6] << 8) | can_buffer_ID_12.data[7]);
    RHWS_raw_encoder_pulses.data[19] = int16_t(int16_t(can_buffer_ID_13.data[0] << 8) | can_buffer_ID_13.data[1]);

    static constexpr int8_t sign[NUM_ENCODER] = {-1, 1, 1, 1, 1, 1, 1, -1, 1, 1};
    RHWS_raw_encoder_pulses.data[0] *= sign[0];
    RHWS_raw_encoder_pulses.data[2] *= sign[2];
    RHWS_raw_encoder_pulses.data[5] *= sign[5];
    RHWS_raw_encoder_pulses.data[7] *= sign[7];

    RHWS_raw_encoder_pulses.data[3] *= (-1);
    RHWS_raw_encoder_pulses.data[8] *= (-1);

    RHWS_raw_encoder_pulses.data[17] *= (-1);

    RHWS_raw_encoder_pulses.data[ENCODER_INDEX_CABLE_ARM_MOTOR] *= (1);
    RHWS_raw_encoder_pulses.data[ENCODER_INDEX_CABLE_BELT_MOTOR] *= (-1);
}

// works very fine!
void RHWS_encoder_count_up_anti_overflow() {
    static int16_t rolledupCount[NUM_ENCODER];
    static int32_t prev_raw_encoder_pulses[NUM_ENCODER];
    static int8_t sign_previous[NUM_ENCODER];  // 1 = positive, 0 = negative
    int8_t sign_current[NUM_ENCODER];          // 1 = positive, 0 = negative

    for (int i = 0; i < NUM_ENCODER; i++) {
        if (RHWS_raw_encoder_pulses.data.at(i) > 0)
            sign_current[i] = 1;
        else
            sign_current[i] = 0;

        if (sign_current[i] != sign_previous[i]) {
            if (prev_raw_encoder_pulses[i] > MAX_PWM_STMHAL)
                rolledupCount[i]++;
            else if (prev_raw_encoder_pulses[i] < -MAX_PWM_STMHAL)
                rolledupCount[i]--;
        }
        RHWS_encoded_encoder_pulses.data.at(i) = (256 * 256) * rolledupCount[i] + RHWS_raw_encoder_pulses.data.at(i);
        prev_raw_encoder_pulses[i] = RHWS_raw_encoder_pulses.data.at(i);
        sign_previous[i] = sign_current[i];
    }
}

std_msgs::UInt16MultiArray RHWS_switch_states;
void RHWS_switch_states_compose() {
    static bool init = true;
    if (init) {
        RHWS_switch_states.data.resize(NUM_SWITCH);
        init = false;
    }
    if (can_buffer_ID_7.data.size() != 8 || can_buffer_ID_9.data.size() != 8)
        return;
    if (can_buffer_ID_11.data.size() != 8 || can_buffer_ID_13.data.size() != 8)
        return;

    RHWS_switch_states.data[0] = can_buffer_ID_7.data[2];
    RHWS_switch_states.data[1] = can_buffer_ID_7.data[3];
    RHWS_switch_states.data[2] = can_buffer_ID_7.data[4];
    RHWS_switch_states.data[3] = can_buffer_ID_7.data[5];
    RHWS_switch_states.data[4] = can_buffer_ID_7.data[6];
    RHWS_switch_states.data[5] = can_buffer_ID_9.data[2];
    RHWS_switch_states.data[6] = can_buffer_ID_9.data[3];
    RHWS_switch_states.data[7] = can_buffer_ID_9.data[4];
    RHWS_switch_states.data[8] = can_buffer_ID_9.data[5];
    RHWS_switch_states.data[9] = can_buffer_ID_9.data[6];
    RHWS_switch_states.data[10] = can_buffer_ID_11.data[2];
    RHWS_switch_states.data[11] = can_buffer_ID_11.data[3];
    RHWS_switch_states.data[12] = can_buffer_ID_11.data[4];
    RHWS_switch_states.data[13] = can_buffer_ID_11.data[5];
    RHWS_switch_states.data[14] = can_buffer_ID_11.data[6];
    RHWS_switch_states.data[15] = can_buffer_ID_13.data[2];
    RHWS_switch_states.data[16] = can_buffer_ID_13.data[3];
    RHWS_switch_states.data[17] = can_buffer_ID_13.data[4];
    RHWS_switch_states.data[18] = can_buffer_ID_13.data[5];
    RHWS_switch_states.data[19] = can_buffer_ID_13.data[6];
}

std_msgs::Int16MultiArray wheel_motor_pwms;
void callback_wheel_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    static bool init = true;
    if (init) {
        wheel_motor_pwms.data.resize(NUM_DRIVEUNIT);
        init = false;
    }
    if (msg.data.size() != NUM_DRIVEUNIT)
        return;
    for (int i = 0; i < NUM_DRIVEUNIT; i++) {
        wheel_motor_pwms.data[i] = msg.data[i];
    }
}

std_msgs::Int16MultiArray drive_lift_motor_pwms;
void callback_drive_lift_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    static bool init = true;
    if (init) {
        drive_lift_motor_pwms.data.resize(NUM_DRIVEUNIT);
        init = false;
    }
    if (msg.data.size() != NUM_DRIVEUNIT)
        return;
    for (int i = 0; i < NUM_DRIVEUNIT; i++) {
        drive_lift_motor_pwms.data[i] = msg.data[i];
    }
}

std_msgs::Int16MultiArray main_lift_motor_pwms;
void callback_main_lift_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    static bool init = true;
    if (init) {
        main_lift_motor_pwms.data.resize(NUM_MAIN_LIFT_AXIS);
        init = false;
    }
    if (msg.data.size() != NUM_MAIN_LIFT_AXIS)
        return;
    for (int i = 0; i < NUM_MAIN_LIFT_AXIS; i++) {
        main_lift_motor_pwms.data[i] = msg.data[i];
    }
}

std_msgs::Int16MultiArray arm_motor_pwms;
void callback_arm_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    static bool init = true;
    if (init) {
        arm_motor_pwms.data.resize(NUM_ARM_AXIS);
        init = false;
    }
    if (msg.data.size() != NUM_ARM_AXIS)
        return;
    for (int i = 0; i < NUM_ARM_AXIS; i++) {
        arm_motor_pwms.data[i] = msg.data[i];
    }
}

std_msgs::Int16MultiArray cable_motor_pwms;
void callback_cable_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    static bool init = true;
    if (init) {
        cable_motor_pwms.data.resize(NUM_CABLE_MOTOR);
        init = false;
    }
    if (msg.data.size() != NUM_CABLE_MOTOR)
        return;
    for (int i = 0; i < NUM_CABLE_MOTOR; i++) {
        cable_motor_pwms.data[i] = msg.data[i];
    }
}

int16_t power_led_breathing() {
    int16_t led_strength = 0;
    static const double ACCEL_PERIOD = 1, STABLE_PERIOD = 0;
    static const uint16_t OFFSET_PWM = 0;
    static const int16_t MAX_PWM = int16_t(double(MAX_PWM_STMHAL / 16) * 12.0) - OFFSET_PWM;
    static bool is_accel_period = true, is_stable_period = true;

    double now = ros::Time::now().toSec();
    static double accel_started_period = ros::Time::now().toSec();
    static double stable_started_period = ros::Time::now().toSec();
    static double deaccel_started_period = ros::Time::now().toSec();

    if (is_accel_period) {
        double diff_time = now - accel_started_period;
        led_strength = sin(diff_time / ACCEL_PERIOD * M_PI_2) * MAX_PWM + OFFSET_PWM;
        if (diff_time > ACCEL_PERIOD) {
            is_accel_period = false;
            is_stable_period = true;
            stable_started_period = ros::Time::now().toSec();
        }
    } else if (is_stable_period) {
        double diff_time = now - stable_started_period;
        led_strength = MAX_PWM;
        if (diff_time > STABLE_PERIOD) {
            is_accel_period = false;
            is_stable_period = false;
            deaccel_started_period = ros::Time::now().toSec();
        }
    } else {
        double diff_time = now - deaccel_started_period;
        led_strength = cos(diff_time / ACCEL_PERIOD * M_PI_2) * MAX_PWM + OFFSET_PWM;
        if (diff_time > ACCEL_PERIOD) {
            is_accel_period = true;
            is_stable_period = false;
            accel_started_period = ros::Time::now().toSec();
        }
    }

    if (led_strength < 0) {
        led_strength = 0;
        // ROS_FATAL("the value led_strength is negative! check calclations!");
    }
    return led_strength;
}

int16_t power_led_flashing() {
    int16_t led_strength = 0;
    static const double STABLE_PERIOD = 0.5;
    static const int16_t MAX_PWM = int16_t(double(MAX_PWM_STMHAL / 16.8) * 12.0);
    static bool is_led_on = true;

    double now = ros::Time::now().toSec();
    static double previous_toggled_period = ros::Time::now().toSec();

    if (is_led_on) {
        double diff_time = now - previous_toggled_period;
        led_strength = MAX_PWM;
        if (diff_time > STABLE_PERIOD) {
            is_led_on = false;
            previous_toggled_period = ros::Time::now().toSec();
        }
    } else {
        double diff_time = now - previous_toggled_period;
        led_strength = 0;
        if (diff_time > STABLE_PERIOD) {
            is_led_on = true;
            previous_toggled_period = ros::Time::now().toSec();
        }
    }

    if (led_strength < 0) {
        led_strength = 0;
    }
    return led_strength;
}

int16_t power_led_ON() {
    static const int16_t MAX_PWM = int16_t(double(MAX_PWM_STMHAL / 16.8) * 12.0);
    return MAX_PWM;
}

std_msgs::Int16MultiArray RHWA_motor_pwm;
void RHWA_motor_pwm_compose() {
    static bool init = true;
    if (init) {
        RHWA_motor_pwm.data.resize(NUM_MOTOR);
        wheel_motor_pwms.data.resize(NUM_DRIVEUNIT);
        drive_lift_motor_pwms.data.resize(NUM_DRIVEUNIT);
        main_lift_motor_pwms.data.resize(NUM_MAIN_LIFT_AXIS);
        arm_motor_pwms.data.resize(NUM_ARM_AXIS);
        cable_motor_pwms.data.resize(NUM_CABLE_MOTOR);
        init = false;
    }

    // 注意として、基本的にコントローラ系では正負の変換は禁止
    // 入力側（SIMブリッジ、CANONブリッジ側、ROBOTコア）で正負の調整を行うこと！
    // そうしないと符号の収拾がつかんくなって頭おかしなる
    static const int8_t pwm_sign_drive_wheel[NUM_DRIVEUNIT] = {-1, 1, 1, -1};
    static const int8_t pwm_sign_drive_lift[NUM_DRIVEUNIT] = {-1, 1, -1, 1};

    RHWA_motor_pwm.data[4] = pwm_sign_drive_wheel[0] * wheel_motor_pwms.data[0];
    RHWA_motor_pwm.data[5] = pwm_sign_drive_wheel[1] * wheel_motor_pwms.data[1];
    RHWA_motor_pwm.data[6] = pwm_sign_drive_wheel[2] * wheel_motor_pwms.data[2];
    RHWA_motor_pwm.data[7] = pwm_sign_drive_wheel[3] * wheel_motor_pwms.data[3];
    // RHWA_motor_pwm.data[0] = arm_motor_pwms.data[0];
    // RHWA_motor_pwm.data[1] = arm_motor_pwms.data[1];
    // RHWA_motor_pwm.data[2] = main_lift_motor_pwms.data[0];

    // honnbann drive lift setting
    //  RHWA_motor_pwm.data[4] = pwm_sign_drive_lift[0] * drive_lift_motor_pwms.data[0];
    //  RHWA_motor_pwm.data[5] = pwm_sign_drive_lift[1] * drive_lift_motor_pwms.data[1];
    //  RHWA_motor_pwm.data[6] = pwm_sign_drive_lift[2] * drive_lift_motor_pwms.data[2];
    //  RHWA_motor_pwm.data[7] = pwm_sign_drive_lift[3] * drive_lift_motor_pwms.data[3];

    RHWA_motor_pwm.data[0] = pwm_sign_drive_lift[0] * drive_lift_motor_pwms.data[0];
    RHWA_motor_pwm.data[1] = pwm_sign_drive_lift[1] * drive_lift_motor_pwms.data[1];
    RHWA_motor_pwm.data[2] = pwm_sign_drive_lift[2] * drive_lift_motor_pwms.data[2];
    RHWA_motor_pwm.data[3] = pwm_sign_drive_lift[3] * drive_lift_motor_pwms.data[3];

    static const int8_t pwm_sign_main_lift = 1;
    RHWA_motor_pwm.data[8] = pwm_sign_main_lift * main_lift_motor_pwms.data[0];

    static const int8_t pwm_sign_arm_x = 1, pwm_sign_arm_y = -1;
    RHWA_motor_pwm.data[9] = pwm_sign_arm_x * arm_motor_pwms.data[0];
    RHWA_motor_pwm.data[10] = pwm_sign_arm_y * arm_motor_pwms.data[1];

    static const int8_t pwm_sign_cable_arm = -1, pwm_sign_cable_push = 1;
    RHWA_motor_pwm.data[12] = pwm_sign_cable_arm * cable_motor_pwms.data[0];
    RHWA_motor_pwm.data[13] = pwm_sign_cable_push * cable_motor_pwms.data[1];

    static uint16_t led_PWM = 0;
    static const double lpf_gain = 0.1;
    static uint16_t previous_led_PWM = 0;
    if (abs(RHWA_motor_pwm.data[8]) > 0) {
        RHWA_motor_pwm.data[15] = power_led_flashing();
    } else {
        if (abs(RHWA_motor_pwm.data[4]) > 1000 || abs(RHWA_motor_pwm.data[5]) > 1000 || abs(RHWA_motor_pwm.data[6]) > 1000 || abs(RHWA_motor_pwm.data[7]) > 1000) {
            led_PWM = power_led_ON();
        } else {
            led_PWM = power_led_breathing();
        }
        RHWA_motor_pwm.data[15] = led_PWM * lpf_gain + previous_led_PWM * (1 - lpf_gain);
        previous_led_PWM = RHWA_motor_pwm.data[15];
    }
}

std_msgs::UInt8MultiArray CAN_ID_1_data, CAN_ID_2_data, CAN_ID_3_data, CAN_ID_4_data;
void CAN_ID_1_data_compose() {
    static bool init = true;
    if (init) {
        CAN_ID_1_data.data.resize(8);
        init = false;
    }
    CAN_ID_1_data.data[0] = (RHWA_motor_pwm.data[0] & 0xff);
    CAN_ID_1_data.data[1] = ((RHWA_motor_pwm.data[0] >> 8) & 0xff);
    CAN_ID_1_data.data[2] = (RHWA_motor_pwm.data[1] & 0xff);
    CAN_ID_1_data.data[3] = ((RHWA_motor_pwm.data[1] >> 8) & 0xff);
    CAN_ID_1_data.data[4] = (RHWA_motor_pwm.data[2] & 0xff);
    CAN_ID_1_data.data[5] = ((RHWA_motor_pwm.data[2] >> 8) & 0xff);
    CAN_ID_1_data.data[6] = (RHWA_motor_pwm.data[3] & 0xff);
    CAN_ID_1_data.data[7] = ((RHWA_motor_pwm.data[3] >> 8) & 0xff);
}
void CAN_ID_2_data_compose() {
    static bool init = true;
    if (init) {
        CAN_ID_2_data.data.resize(8);
        init = false;
    }
    CAN_ID_2_data.data[0] = (RHWA_motor_pwm.data[4] & 0xff);
    CAN_ID_2_data.data[1] = ((RHWA_motor_pwm.data[4] >> 8) & 0xff);
    CAN_ID_2_data.data[2] = (RHWA_motor_pwm.data[5] & 0xff);
    CAN_ID_2_data.data[3] = ((RHWA_motor_pwm.data[5] >> 8) & 0xff);
    CAN_ID_2_data.data[4] = (RHWA_motor_pwm.data[6] & 0xff);
    CAN_ID_2_data.data[5] = ((RHWA_motor_pwm.data[6] >> 8) & 0xff);
    CAN_ID_2_data.data[6] = (RHWA_motor_pwm.data[7] & 0xff);
    CAN_ID_2_data.data[7] = ((RHWA_motor_pwm.data[7] >> 8) & 0xff);
}
void CAN_ID_3_data_compose() {
    static bool init = true;
    if (init) {
        CAN_ID_3_data.data.resize(8);
        init = false;
    }
    CAN_ID_3_data.data[0] = (RHWA_motor_pwm.data[8] & 0xff);
    CAN_ID_3_data.data[1] = ((RHWA_motor_pwm.data[8] >> 8) & 0xff);
    CAN_ID_3_data.data[2] = (RHWA_motor_pwm.data[9] & 0xff);
    CAN_ID_3_data.data[3] = ((RHWA_motor_pwm.data[9] >> 8) & 0xff);
    CAN_ID_3_data.data[4] = (RHWA_motor_pwm.data[10] & 0xff);
    CAN_ID_3_data.data[5] = ((RHWA_motor_pwm.data[10] >> 8) & 0xff);
    CAN_ID_3_data.data[6] = (RHWA_motor_pwm.data[11] & 0xff);
    CAN_ID_3_data.data[7] = ((RHWA_motor_pwm.data[11] >> 8) & 0xff);
}
void CAN_ID_4_data_compose() {
    static bool init = true;
    if (init) {
        CAN_ID_4_data.data.resize(8);
        init = false;
    }
    CAN_ID_4_data.data[0] = (RHWA_motor_pwm.data[12] & 0xff);
    CAN_ID_4_data.data[1] = ((RHWA_motor_pwm.data[12] >> 8) & 0xff);
    CAN_ID_4_data.data[2] = (RHWA_motor_pwm.data[13] & 0xff);
    CAN_ID_4_data.data[3] = ((RHWA_motor_pwm.data[13] >> 8) & 0xff);
    CAN_ID_4_data.data[4] = (RHWA_motor_pwm.data[14] & 0xff);
    CAN_ID_4_data.data[5] = ((RHWA_motor_pwm.data[14] >> 8) & 0xff);
    CAN_ID_4_data.data[6] = (RHWA_motor_pwm.data[15] & 0xff);
    CAN_ID_4_data.data[7] = ((RHWA_motor_pwm.data[15] >> 8) & 0xff);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_bridge");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Publisher pub_RHWS_encoder_pulses = nh.advertise<std_msgs::Int32MultiArray>("RHWS/encoder_pulses", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_RHWS_switch_states = nh.advertise<std_msgs::UInt16MultiArray>("RHWS/switch_states", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_RHWS_IMU_pitch = nh.advertise<std_msgs::Float32>("RHWS/IMU_pitch", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_RHWS_IMU_roll = nh.advertise<std_msgs::Float32>("RHWS/IMU_roll", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_CAN_ID_1 = nh.advertise<std_msgs::UInt8MultiArray>("canout1", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_CAN_ID_2 = nh.advertise<std_msgs::UInt8MultiArray>("canout2", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_CAN_ID_3 = nh.advertise<std_msgs::UInt8MultiArray>("canout3", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_CAN_ID_4 = nh.advertise<std_msgs::UInt8MultiArray>("canout4", QUEUE_SIZE_PUBLISHER);

    ros::Subscriber sub_wheel_motor_pwms = nh.subscribe("wheel_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_wheel_motor_pwms);
    ros::Subscriber sub_drive_lift_motor_pwms = nh.subscribe("drive_lift_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_drive_lift_motor_pwms);
    ros::Subscriber sub_main_lift_motor_pwms = nh.subscribe("main_lift_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_main_lift_motor_pwms);
    ros::Subscriber sub_arm_motor_pwms = nh.subscribe("arm_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_arm_motor_pwms);
    ros::Subscriber sub_cable_motor_pwms = nh.subscribe("cable_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_cable_motor_pwms);

    ros::Subscriber sub_CAN_ID_6 = nh.subscribe("canin6", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_6);
    ros::Subscriber sub_CAN_ID_7 = nh.subscribe("canin7", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_7);
    ros::Subscriber sub_CAN_ID_8 = nh.subscribe("canin8", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_8);
    ros::Subscriber sub_CAN_ID_9 = nh.subscribe("canin9", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_9);
    ros::Subscriber sub_CAN_ID_10 = nh.subscribe("canin10", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_10);
    ros::Subscriber sub_CAN_ID_11 = nh.subscribe("canin11", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_11);
    ros::Subscriber sub_CAN_ID_12 = nh.subscribe("canin12", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_12);
    ros::Subscriber sub_CAN_ID_13 = nh.subscribe("canin13", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_13);
    ros::Subscriber sub_CAN_ID_33 = nh.subscribe("canin33", QUEUE_SIZE_SUBSCRIBER, callback_CAN_ID_33);

    while (ros::ok()) {
        RHWS_encoder_pulses_compose();
        RHWS_encoder_count_up_anti_overflow();
        RHWS_switch_states_compose();
        IMU_compose();

        RHWA_motor_pwm_compose();
        CAN_ID_1_data_compose();
        CAN_ID_2_data_compose();
        CAN_ID_3_data_compose();
        CAN_ID_4_data_compose();

        pub_CAN_ID_1.publish(CAN_ID_1_data);
        pub_CAN_ID_2.publish(CAN_ID_2_data);
        pub_CAN_ID_3.publish(CAN_ID_3_data);
        pub_CAN_ID_4.publish(CAN_ID_4_data);

        pub_RHWS_encoder_pulses.publish(RHWS_encoded_encoder_pulses);
        pub_RHWS_switch_states.publish(RHWS_switch_states);

        pub_RHWS_IMU_pitch.publish(IMU_pitch);
        pub_RHWS_IMU_roll.publish(IMU_roll);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}