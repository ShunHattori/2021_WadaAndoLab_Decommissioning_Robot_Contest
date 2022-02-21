#pragma once
#include <utils.hpp>

// 何がしたい：上流から流れてくる位置指令に対してPWM値を算出したい
// 入力：PIDパラメーター・入力（目標）パルス・現在のエンコーダパルス・上下リミット接点情報
// 出力：PWM値

class position_motordrive {
   public:
    position_motordrive(double, double, double);
    ~position_motordrive();

    enum limit_type {
        BOTTOM,
        TOP,
        TYPE_SIZE,
    };
    enum operation_mode {
        GET_ORIGIN,
        FOLLOW_REFERENCE,
        MODE_SIZE,
    };

    void initialize();
    void update();
    void set_pid_gain(double, double, double);
    void set_mode(operation_mode);
    void set_current_pulse(int64_t);
    void set_reference_pulse(int64_t);
    void set_reference_pulse_from_origin(int64_t);
    void set_limit_state(limit_type, bool);
    void set_limit_reference_pulse(int64_t, int64_t);
    void set_minimum_pwm(int16_t);
    void set_i_clamp(double, double);
    void reset_pid_cv();
    double get_pwm();
    double get_limit_state(limit_type);
    double get_reference_pulse();
    double get_origin_pulse();

   private:
    pid_controller *pwm_CT;

    bool limit_state[TYPE_SIZE];
    double pwm, minimum_pwm;
    uint8_t current_mode;
    int64_t pulse_current, pulse_reference, pulse_origin;
    int64_t limit_reference_pulse[TYPE_SIZE];
};

position_motordrive::position_motordrive(double p, double i, double d) {
    this->pwm_CT = new pid_controller(p, i, d);
    this->pwm_CT->set_i_clamp(-30000, 30000);
    pwm = 0;
    current_mode = 0;
    pulse_current = 0;
    pulse_reference = 0;
}

position_motordrive::~position_motordrive() {
}

void position_motordrive::initialize() {
}

void position_motordrive::update() {
    pwm_CT->set_current(pulse_current);
    if (current_mode == GET_ORIGIN) {
        pwm_CT->reset_control_variable();
        if (limit_state[BOTTOM]) {
            pulse_origin = pulse_current;
            pwm = 0;
        } else {
            pwm = minimum_pwm;
        }
    } else if (current_mode == FOLLOW_REFERENCE) {
        pwm_CT->set_reference(pulse_reference);
        pwm_CT->update();
        pwm = pwm_CT->get_control_variable();
    }

    // ROS_INFO("pwm_CT->get_process_variable(pid_controller::P):%lf", pwm_CT->get_process_variable(pid_controller::P));
    // ROS_INFO("pwm_CT->get_process_variable(pid_controller::I):%lf", pwm_CT->get_process_variable(pid_controller::I));
    // ROS_INFO("pwm_CT->get_process_variable(pid_controller::D):%lf", pwm_CT->get_process_variable(pid_controller::D));
}

void position_motordrive::set_pid_gain(double p, double i, double d) {
    this->pwm_CT->set_gain(pid_controller::P, p);
    this->pwm_CT->set_gain(pid_controller::P, i);
    this->pwm_CT->set_gain(pid_controller::P, d);
}

void position_motordrive::set_mode(operation_mode mode) {
    this->current_mode = mode;
}
void position_motordrive::set_current_pulse(int64_t pulse) {
    this->pulse_current = pulse;
}
void position_motordrive::set_reference_pulse(int64_t pulse) {
    if (pulse > this->limit_reference_pulse[TOP]) {
        ROS_ERROR("limit_reference_pulse[TOP] range error");
        return;
    }
    if (pulse < this->limit_reference_pulse[BOTTOM]) {
        ROS_ERROR("limit_reference_pulse[BOTTOM] range error");
        return;
    }
    this->pulse_reference = pulse;
}

void position_motordrive::set_reference_pulse_from_origin(int64_t pulse) {
    this->set_reference_pulse(this->pulse_origin + pulse);
}

void position_motordrive::set_limit_state(limit_type type, bool state) {
    this->limit_state[type] = state;
}

void position_motordrive::set_limit_reference_pulse(int64_t pulse_min, int64_t pulse_max) {
    this->limit_reference_pulse[BOTTOM] = pulse_min;
    this->limit_reference_pulse[TOP] = pulse_max;
}

void position_motordrive::set_minimum_pwm(int16_t pwm) {
    this->minimum_pwm = pwm;
}
void position_motordrive::set_i_clamp(double clamp_low, double clamp_high) {
    this->pwm_CT->set_i_clamp(clamp_low, clamp_high);
}

void position_motordrive::reset_pid_cv() {
    this->pwm_CT->reset_control_variable();
}

double position_motordrive::get_pwm() {
    return this->pwm;
}

double position_motordrive::get_limit_state(limit_type type) {
    return this->limit_state[type];
}

double position_motordrive::get_reference_pulse() {
    return this->pulse_reference;
}

double position_motordrive::get_origin_pulse() {
    return this->pulse_origin;
}