#include <utils.hpp>

double constexpr target_roll = 0, roll_p_gain = 0.002, roll_i_gain = 0.000001,
                 pitch_p_gain = 0.002, pitch_i_gain = 0.00001,
                 i_clamp = 0.0002;
pid_controller orientation_roll_controller(roll_p_gain, roll_i_gain, 0);
pid_controller orientation_pitch_controller(pitch_p_gain, pitch_i_gain, 0);

geometry_msgs::Twist robot_velocity;
geometry_msgs::Pose2D robot_position;
sensor_msgs::Joy state_joy;
std_msgs::Float64 robot_drive_lift_position[4];
std_msgs::Float64 robot_main_lift_position[4];
std_msgs::Float64 drive_lift_manual_control_value;
std_msgs::Float64 arm_lift_position;
std_msgs::Float64 wheel_vel[4];
std_msgs::Float64MultiArray wheels_vel;
std_msgs::Float64MultiArray lifts_pos;
std_msgs::Int8MultiArray lifts_mode;
double roll, pitch, yaw;
bool slope_mode = 0, main_lift_rise_flag = 0;
int8_t arm_lift_flag = 0;  // 0 = get origin, 1 = ref bottom, 2 = ref top
double roll_i = 0;
double pitch_i = 0;

//足回り上下機構用変数
bool drive_lift_bottom_flag = 0;
double drive_lift_height = 0;

void callback_wheel_vel_tf_publish(std_msgs::Float32MultiArray wheels_vel) {
    if (wheels_vel.data.empty())
        return;

    wheel_vel[0].data = wheels_vel.data.at(0);
    wheel_vel[1].data = wheels_vel.data.at(1);
    wheel_vel[2].data = wheels_vel.data.at(2);
    wheel_vel[3].data = wheels_vel.data.at(3);
}

// COREに移植予定　MODE=MANUALとしてMANUALに↓の値を渡す
void robot_drive_lift_compute() {
    static constexpr double lift_max = 0;
    static constexpr double lift_min = -0.334;

    //全リフト一番下に設定
    if (drive_lift_bottom_flag) {
        for (int i = 0; i < 4; i++) {
            lifts_mode.data.at(i) = 1;  //bottom lock state
            robot_drive_lift_position[i].data = 0;
        }
    } else {
        robot_drive_lift_position[0].data += -robot_velocity.linear.z;
        robot_drive_lift_position[1].data += -robot_velocity.linear.z;
        robot_drive_lift_position[2].data += -robot_velocity.linear.z;
        robot_drive_lift_position[3].data += -robot_velocity.linear.z;
        robot_drive_lift_position[0].data += -drive_lift_manual_control_value.data;
        robot_drive_lift_position[1].data += -drive_lift_manual_control_value.data;
        robot_drive_lift_position[2].data += +drive_lift_manual_control_value.data;
        robot_drive_lift_position[3].data += +drive_lift_manual_control_value.data;

        //可動域（下限）を超えた場合、他のリフトに移動量を加算してそのリフトの制御量をゼロにする。
        for (int i = 0; i < 4; i++) {
            if (robot_drive_lift_position[i].data > 0) {
                for (int j = 0; j < 4; j++) {
                    if (j != i)
                        robot_drive_lift_position[j].data += -robot_drive_lift_position[i].data;
                }
                robot_drive_lift_position[i].data = 0;
            }
        }
        for (int i = 0; i < 4; i++) {
            lifts_mode.data.at(i) = 0;  //free lift state
        }
    }

    for (int i = 0; i < 4; i++) {
        robot_drive_lift_position[i].data = constrain(robot_drive_lift_position[i].data, lift_min, lift_max);
    }
    ROS_INFO("drive_lift:%lf, %lf, %lf, %lf", robot_drive_lift_position[0].data, robot_drive_lift_position[1].data, robot_drive_lift_position[2].data, robot_drive_lift_position[3].data);
}

// POSE CTのFOLLOWに移植予定　MODEだけよみとりかなー
void robot_drive_lift_horizontal_compute() {
    orientation_roll_controller.set_reference(0);
    orientation_pitch_controller.set_reference(0);
    orientation_roll_controller.set_current(roll);
    orientation_pitch_controller.set_current(pitch);

    orientation_roll_controller.update();
    orientation_pitch_controller.update();

    double roll_control_term = orientation_roll_controller.get_control_variable();
    double pitch_control_term = orientation_pitch_controller.get_control_variable();

    robot_drive_lift_position[0].data -= (roll_control_term);
    robot_drive_lift_position[1].data -= (roll_control_term);
    robot_drive_lift_position[2].data += (roll_control_term);
    robot_drive_lift_position[3].data += (roll_control_term);

    robot_drive_lift_position[0].data += (pitch_control_term);
    robot_drive_lift_position[1].data -= (pitch_control_term);
    robot_drive_lift_position[2].data -= (pitch_control_term);
    robot_drive_lift_position[3].data += (pitch_control_term);

    //可動域（下限）を超えた場合、他のリフトに移動量を加算してそのリフトの制御量をゼロにする。
    for (int i = 0; i < 4; i++) {
        if (robot_drive_lift_position[i].data > 0) {
            for (int j = 0; j < 4; j++) {
                if (j != i)
                    robot_drive_lift_position[j].data += -robot_drive_lift_position[i].data;
            }
            robot_drive_lift_position[i].data = 0;
        }
    }

    ROS_INFO("lift1:%.5lf,lift2:%.5lf,lift3:%.5lf,lift4:%.5lf\r\n", robot_drive_lift_position[0].data, robot_drive_lift_position[1].data, robot_drive_lift_position[2].data, robot_drive_lift_position[3].data);

    static constexpr double lift_max = 0;
    static constexpr double lift_min = -0.334;
    for (int i = 0; i < 4; i++) {
        robot_drive_lift_position[i].data = constrain(robot_drive_lift_position[i].data, lift_min, lift_max);
        // ROS_INFO("%lf", robot_drive_lift_position[i].data);
    }
    ROS_INFO("lift1:%.5lf,lift2:%.5lf,lift3:%.5lf,lift4:%.5lf\r\n", robot_drive_lift_position[0].data, robot_drive_lift_position[1].data, robot_drive_lift_position[2].data, robot_drive_lift_position[3].data);
}

void robot_main_lift_compute() {
    static constexpr double lift_max = 0.680;
    static constexpr double lift_min = 0;
    if (main_lift_rise_flag) {
        robot_main_lift_position[1].data += 0.0005;
        robot_main_lift_position[2].data += 0.0005;
        robot_main_lift_position[3].data += 0.0005;
    } else {
        robot_main_lift_position[1].data += -0.0005;
        robot_main_lift_position[2].data += -0.0005;
        robot_main_lift_position[3].data += -0.0005;
    }

    for (int i = 0; i < 3; i++) {
        robot_main_lift_position[i].data = constrain(robot_main_lift_position[i].data, lift_min, lift_max);
    }
}

void arm_lift_compute() {
    //リファレンス位置はS字制御で生成＋PWM計算はPID制御で追従
    // top -> 4000, max -> 8200, 一回転 -> 1000p
    static int8_t prev_arm_lift_flag = 0;
    static bool is_accel_period = false;
    bool is_flag_changed = arm_lift_flag != prev_arm_lift_flag ? true : false;
    static double flag_changed_time = 0, pulse_s_curve_top_reference = 0;
    constexpr double ACCEL_PERIOD = 3.0, PULSE_REFERENCE = 4000;
    double now = ros::Time::now().toSec();
    double period_from_flag_changed = now - flag_changed_time;
    if (arm_lift_flag == 0) {
        arm_lift_position.data = -1;
    } else {
        if (is_flag_changed) {
            flag_changed_time = ros::Time::now().toSec();
            pulse_s_curve_top_reference = arm_lift_position.data;
            is_accel_period = true;
        } else {
            if (is_accel_period) {
                if (arm_lift_flag == 2) {  //pressed up
                    arm_lift_position.data = PULSE_REFERENCE * (0.5 - 0.5 * cos(period_from_flag_changed / ACCEL_PERIOD * M_PI));
                } else {  //pressed down
                    arm_lift_position.data = pulse_s_curve_top_reference * (0.55 + 0.45 * cos(period_from_flag_changed / ACCEL_PERIOD * M_PI));
                }
                if (period_from_flag_changed > ACCEL_PERIOD) {
                    is_accel_period = false;
                    arm_lift_position.data = arm_lift_flag == 2 ? PULSE_REFERENCE : pulse_s_curve_top_reference * 0.1;  //top-> pulse_ref, bm-> 0
                }
            }
        }
    }

    prev_arm_lift_flag = arm_lift_flag;
}

void callback_joy_parse(const sensor_msgs::Joy &joy_msg) {
    //calc robot vel
    robot_velocity.linear.x = -joy_msg.axes[0] * 2.0;  //max 2.0 m/s
    robot_velocity.linear.y = joy_msg.axes[1] * 2.0;
    double left_rot_center_fixed = (-joy_msg.axes[2] / 2.0) + 1.0;
    double right_rot_center_fixed = (-joy_msg.axes[5] / 2.0) + 1.0;
    robot_velocity.angular.z = (right_rot_center_fixed - left_rot_center_fixed);
    if (joy_msg.axes[2] == 0 || joy_msg.axes[5] == 0)
        robot_velocity.angular.z = 0;  //初回、片方だけの旋回ボタンを押したときに+1.0が影響してバグるのを防ぐ

    //vel for lift
    robot_velocity.linear.z = joy_msg.axes[4] * 0.0001;

    static double prev_arrow_value = 0;
    double arrow_value = joy_msg.axes[7];
    if (arrow_value != prev_arrow_value) {
        if (arrow_value != 0) {
            if (arrow_value > 0) {
                arm_lift_flag++;
                if (arm_lift_flag > 2)
                    arm_lift_flag = 2;
            } else {
                arm_lift_flag--;
                if (arm_lift_flag < 0)
                    arm_lift_flag = 0;
            }
        }
    }
    prev_arrow_value = arrow_value;
    ROS_WARN("arm_lift_flag:%d", arm_lift_flag);
    // ROS_WARN("arrow_value:%lf", arrow_value);

    //button for slope_mode
    static bool circle_prev;
    bool circle_curr = joy_msg.buttons[1];
    if (circle_curr != circle_prev && circle_curr == 1)
        slope_mode = !slope_mode;
    circle_prev = circle_curr;

    //button for drive lift bottom
    static bool triangle_prev;
    bool triangle_curr = joy_msg.buttons[2];
    if (triangle_curr != triangle_prev && triangle_curr == 1)
        drive_lift_bottom_flag = !drive_lift_bottom_flag;
    triangle_prev = triangle_curr;

    // button for drive lift control
    static bool L1_prev, R1_prev;
    bool L1_curr = joy_msg.buttons[4];
    bool R1_curr = joy_msg.buttons[5];
    if (L1_curr != L1_prev && L1_curr == 1) {
        drive_lift_manual_control_value.data = -0.0001;
    } else if (R1_curr != R1_prev && R1_curr == 1) {
        drive_lift_manual_control_value.data = 0.0001;
    } else {
        drive_lift_manual_control_value.data = 0;
    }
    L1_prev = L1_curr;
    R1_prev = R1_curr;

    //button for main_lift_rise_flag
    static bool cross_prev;
    bool cross_curr = joy_msg.buttons[0];
    if (cross_curr != cross_prev && cross_curr == 1)
        main_lift_rise_flag = !main_lift_rise_flag;
    cross_prev = cross_curr;
}

void callback_robot_pose(const geometry_msgs::Pose &robot_pose) {
    double x = robot_pose.position.x;
    double y = robot_pose.position.y;
    double z = robot_pose.position.z;

    tf::Quaternion quat;
    quat.setW(robot_pose.orientation.w);
    quat.setX(robot_pose.orientation.x);
    quat.setY(robot_pose.orientation.y);
    quat.setZ(robot_pose.orientation.z);
    tf_quat_to_rpy(roll, pitch, yaw, quat);

    // ROS_INFO("[pose] x:%.3lf, y:%.3lf, z:%.3lf", x, y, z);
    // ROS_INFO("[ori] x:%.1lf, y:%.1lf, z:%.1lf\r\n", rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw));
}

void compress_lifts_pos() {
    static bool init = 1;
    if (init) {
        lifts_pos.data.resize(6);
        init = 0;
    }
    for (size_t i = 0; i < 4; i++) {
        lifts_pos.data.at(i) = robot_drive_lift_position[i].data;
    }
    lifts_pos.data.at(4) = 0;
    lifts_pos.data.at(5) = arm_lift_position.data;
}

void compress_wheel_vel() {
    static bool init = 1;
    if (init) {
        wheels_vel.data.resize(4);
        init = 0;
    }
    for (size_t i = 0; i < 4; i++) {
        wheels_vel.data.at(i) = wheel_vel[i].data;
    }
}

void callback(commissioning_robot::robot_paramConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request:[RP]%.3lf,[RI]%.3lf,[RD]%.3lf,[PP]%.3lf,[PI]%.3lf,[PD]%.3lf",
             config.roll_p, config.roll_i, config.roll_d, config.pitch_p, config.pitch_i, config.pitch_d);
    orientation_roll_controller.set_gain(pid_controller::P, config.roll_p);
    orientation_roll_controller.set_gain(pid_controller::I, config.roll_i);
    orientation_roll_controller.set_gain(pid_controller::D, config.roll_d);
    orientation_pitch_controller.set_gain(pid_controller::P, config.pitch_p);
    orientation_pitch_controller.set_gain(pid_controller::I, config.pitch_i);
    orientation_pitch_controller.set_gain(pid_controller::D, config.pitch_d);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_core");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Publisher pub_inv_kin = nh.advertise<geometry_msgs::Twist>("robot_global_vel", 1);
    ros::Publisher pub_wheels_vel = nh.advertise<std_msgs::Float64MultiArray>("wheels_vel", 1);
    ros::Publisher pub_lifts_mode = nh.advertise<std_msgs::Int8MultiArray>("lifts_mode", 1);
    ros::Publisher pub_lifts_pos = nh.advertise<std_msgs::Float64MultiArray>("lifts_pos", 1);

    ros::Subscriber sub_joy = nh.subscribe("joy", 10, callback_joy_parse);
    ros::Subscriber sub_robot_pose = nh.subscribe("robot_pose", 10, callback_robot_pose);
    ros::Subscriber sub_wheel_vel = nh.subscribe("inv_kin_wheels_vel", 10, callback_wheel_vel_tf_publish);

    dynamic_reconfigure::Server<commissioning_robot::robot_paramConfig> server;
    dynamic_reconfigure::Server<commissioning_robot::robot_paramConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    robot_drive_lift_position[0].data = 0;
    robot_drive_lift_position[1].data = 0;
    robot_drive_lift_position[2].data = 0;
    robot_drive_lift_position[3].data = 0;
    robot_main_lift_position[0].data = 0;
    robot_main_lift_position[1].data = 0;
    robot_main_lift_position[2].data = 0;
    robot_main_lift_position[3].data = 0;
    lifts_mode.data.resize(6);

    while (ros::ok()) {
        // 足回りの上下機構を制御する
        // slope_modeのとき、車体姿勢が常に水平になるように自動制御
        // ではないとき、コントローラによる手動姿勢制御
        if (slope_mode) {
            // robot_drive_lift_horizontal_compute();
        } else {
            roll_i = 0;
            pitch_i = 0;
            robot_drive_lift_compute();
        }

        // robot_main_lift_compute();
        // arm_lift_compute();

        // publishing robot global velocity to IK for getting velocity of each wheel
        pub_inv_kin.publish(robot_velocity);

        // publishing wheels vel to gazebo
        compress_wheel_vel();
        compress_lifts_pos();

        pub_wheels_vel.publish(wheels_vel);
        pub_lifts_pos.publish(lifts_pos);
        pub_lifts_mode.publish(lifts_mode);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
