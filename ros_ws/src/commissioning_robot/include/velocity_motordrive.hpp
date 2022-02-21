#pragma once
#include <utils.hpp>

// 何がしたい：上位から流れてくるモーターの速度指令に対してPWM値を算出したい
// 入力：PIDパラメーター・入力（目標）速度・現在のエンコーダパルス
// 出力：PWM値

class velocity_motordrive {
   private:
    pid_controller *pwm_CT;
    double pwm;
    double pps;
    double rps, rps_lpf;
    std_msgs::Int64MultiArray pulse_history;
    std_msgs::Float64MultiArray period_history;
    ros::Time current_time;

    int64_t sum_prev_pulse;
    double prev_period;

   public:
    velocity_motordrive(double p, double i, double d);
    ~velocity_motordrive();

    void initialize();
    void update();
    void set_pid_gain(double, double, double);
    void set_reference_velocity(double);
    void set_current_pulse(int64_t);
    void reset_control_variable();

    double get_pwm();
    double get_rps() {
        return rps_lpf;
    }
};

velocity_motordrive::velocity_motordrive(double p, double i, double d) {
    this->pwm_CT = new pid_controller(p, i, d);
    this->pwm_CT->set_i_clamp(-10000, 10000);
    this->pwm = 0;
    this->pps = 0;
    this->rps = 0;
}

velocity_motordrive::~velocity_motordrive() {
}

void velocity_motordrive::set_pid_gain(double p, double i, double d) {
    this->pwm_CT->set_gain(pid_controller::P, p);
    this->pwm_CT->set_gain(pid_controller::P, i);
    this->pwm_CT->set_gain(pid_controller::P, d);
}

double velocity_motordrive::get_pwm() {
    return this->pwm_CT->get_control_variable();
}

void velocity_motordrive::set_reference_velocity(double reference) {
    ROS_INFO("reference:%lf", reference);
    this->pwm_CT->set_reference(reference);
}

void velocity_motordrive::set_current_pulse(int64_t current) {
    // //we need to calc pulse/s for pid current (ref value has unit of velocity)
    double now = ros::Time::now().toSec();
    pulse_history.data.push_back(current);
    period_history.data.push_back(now);

    // //0.08より前のデータを全消去
    for (int i = 0; i < period_history.data.size(); i++) {
        if ((now - period_history.data.at(i)) > 0.08) {
            pulse_history.data.erase(pulse_history.data.begin());
            period_history.data.erase(period_history.data.begin());
        } else {
            break;
        }
    }

    //領域チェック
    if (pulse_history.data.size() < 1)
        return;

    //パルス総和計算→pps 差分のみを足す
    int64_t sum_curr_pulse = 0;
    for (int i = 0; i < pulse_history.data.size() - 1; i++) {
        sum_curr_pulse += pulse_history.data.at(i + 1) - pulse_history.data.at(i);
    }

    double duration = now - period_history.data.at(0);

    // nanの処理
    // https://www.jpcert.or.jp/sc-rules/c-flp04-c.html
    if (isnan(duration))
        return;
    if (isnan(sum_curr_pulse))
        return;
    if (duration == 0)
        return;

    pps = sum_curr_pulse / duration;
    sum_prev_pulse = sum_curr_pulse;

    //エンコーダの分解能・ていばいからrpsを計算
    const double res = 100 * 2;
    rps = pps / res;
    constexpr double lpf_gain = 0.1;
    rps_lpf = rps_lpf * (1 - lpf_gain) + rps * lpf_gain;

    ROS_INFO("duration:%lf", duration);
    ROS_INFO("pulse_history.data.size():%ld", pulse_history.data.size());
    ROS_INFO("sum_curr_pulse:%ld", sum_curr_pulse);
    ROS_INFO("pps:%lf", pps);
    ROS_INFO("rps:%lf", rps);
    ROS_INFO("rps_lpf:%lf", rps_lpf);

    if (isnan(rps))
        return;

    // this->pwm_CT->set_current(rps);
    this->pwm_CT->set_current(rps_lpf);
}

void velocity_motordrive::reset_control_variable() {
    this->pwm_CT->reset_control_variable();
}

void velocity_motordrive::update() {
    this->pwm_CT->update();
}