#pragma once
#include <utils.hpp>

// #define DEBUG_pid_controller

class pid_controller {
   private:
    double reference_variable, current_variable, control_variable, error, previous_error;
    struct term_data {
        double process_variable;
        double gain;
        double clamp_low, clamp_high;
    } data[3];

   public:
    pid_controller(double, double, double);
    ~pid_controller() {}

    enum term_type {
        P,
        I,
        D,
        terms_num
    };

    void update();
    void set_gain(term_type, double);
    void set_reference(double);
    void set_current(double);
    void set_i_clamp(double, double);
    void reset_control_variable();

    double get_process_variable(term_type);
    double get_control_variable();
};

pid_controller::pid_controller(double p_gain, double i_gain, double d_gain) {
    data[P].gain = p_gain;
    data[I].gain = i_gain;
    data[D].gain = d_gain;
    for (int i = 0; i < terms_num; i++) {
        data[i].process_variable = 0;
        data[i].clamp_low = -(256 * 256.0 / 2.0);
        data[i].clamp_high = (256 * 256.0 / 2.0);
    }
    reference_variable = 0;
    current_variable = 0;
    control_variable = 0;
    error = 0;
    previous_error = 0;
}

void pid_controller::update() {
    error = reference_variable - current_variable;
    data[P].process_variable = error * data[P].gain;
    data[I].process_variable += error * data[I].gain;
    data[D].process_variable = (error - previous_error) * data[D].gain;

    if (data[I].process_variable < data[I].clamp_low) {
        data[I].process_variable = data[I].clamp_low;
    }
    if (data[I].process_variable > data[I].clamp_high) {
        data[I].process_variable = data[I].clamp_high;
    }

    // ROS_INFO("error:%lf", error);
    // ROS_INFO("data[P].process_variable:%lf", data[P].process_variable);
    // ROS_INFO("data[I].process_variable:%lf", data[I].process_variable);
    // ROS_INFO("data[D].process_variable:%lf", data[D].process_variable);

    control_variable = data[P].process_variable + data[I].process_variable + data[D].process_variable;
    previous_error = error;
}

void pid_controller::set_gain(term_type term, double gain) {
    data[term].gain = gain;
#ifdef DEBUG_pid_controller
    ROS_INFO("[pid_controller] gain updated: type->%d, gain->%.5lf", term, gain);
#endif
}

void pid_controller::set_reference(double reference) {
    this->reference_variable = reference;
}

void pid_controller::set_current(double current) {
    this->current_variable = current;
}

void pid_controller::set_i_clamp(double clamp_low, double clamp_high) {
    data[I].clamp_low = clamp_low;
    data[I].clamp_high = clamp_high;
#ifdef DEBUG_pid_controller
    ROS_INFO("[pid_controller] clamp updated: low->%.5lf, high->%.5lf", data[I].clamp_low, data[I].clamp_high);
#endif
}

void pid_controller::reset_control_variable() {
    data[P].process_variable = 0;
    data[I].process_variable = 0;
    data[D].process_variable = 0;
    reference_variable = 0;
    current_variable = 0;
    control_variable = 0;
}

double pid_controller::get_process_variable(term_type term) {
    switch (term) {
        case P:
            return data[P].process_variable;
        case I:
            return data[I].process_variable;
        case D:
            return data[D].process_variable;
    }
    return 0;
}

double pid_controller::get_control_variable() {
    // ROS_INFO("control_variable:%lf", control_variable);
    return control_variable;
}