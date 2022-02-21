#include <utils.hpp>
/*
・移動
        ・横　axis_left_x
        ・縦　axis_left_y
        ・旋回　axis_r2 & axis_l2
・足回り上下
        ・IMUで水平維持　toggle button_options
        ・手動操作　(pitch only) press button_triangle & button_cross
        ・原点固定　toggle button_options
・メイン展開上下
        ・展開有効化　toggle button_share
        ・微調整用入力　press button_l1 & button_r1
        ・原点固定　toggle button_share
・アーム部上下
        ・展開パターン保存　button_dpad_up & button_dpad_down
        ・微調整用入力　axis_right_y
        ・原点固定　button_square
・展開全原点
        ・全リセット　button_ps
*/

std_msgs::Float64 target_lift_vel[NUM_TOTAL_LIFT], target_wheel_vel[NUM_DRIVEUNIT], arm_vel[2];
geometry_msgs::Pose robot_pose;

std_msgs::Int32MultiArray RHWS_encoder_pulses;
std_msgs::UInt16MultiArray RHWS_switch_states;

void callback_DynCfgPrm_motor(commissioning_robot::motor_portConfig &config, uint32_t level) {
}

void setup_DynCfgPrm_motor() {
    dynamic_reconfigure::Server<commissioning_robot::motor_portConfig> server;
    dynamic_reconfigure::Server<commissioning_robot::motor_portConfig>::CallbackType f;

    f = boost::bind(&callback_DynCfgPrm_motor, _1, _2);
    server.setCallback(f);
}

void callback_DynCfgPrm_encoder(commissioning_robot::encoder_portConfig &config, uint32_t level) {
}

void setup_DynCfgPrm_encoder() {
    dynamic_reconfigure::Server<commissioning_robot::encoder_portConfig> server;
    dynamic_reconfigure::Server<commissioning_robot::encoder_portConfig>::CallbackType f;

    f = boost::bind(&callback_DynCfgPrm_encoder, _1, _2);
    server.setCallback(f);
}

void callback_DynCfgPrm_switch(commissioning_robot::switch_portConfig &config, uint32_t level) {
}

void setup_DynCfgPrm_switch() {
    dynamic_reconfigure::Server<commissioning_robot::switch_portConfig> server;
    dynamic_reconfigure::Server<commissioning_robot::switch_portConfig>::CallbackType f;

    f = boost::bind(&callback_DynCfgPrm_switch, _1, _2);
    server.setCallback(f);
}

void callback_wheel_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    if (msg.data.size() != NUM_DRIVEUNIT)
        return;
    // 注意として、基本的にコントローラ系では正負の変換は禁止
    // 入力側（SIMブリッジ、CANONブリッジ側、ROBOTコア）で正負の調整を行うこと！
    // そうしないと符号の収拾がつかんくなって頭おかしなる
    int8_t enc_sign[NUM_DRIVEUNIT] = {1, 1, 1, 1};
    int8_t pwm_sign[NUM_DRIVEUNIT] = {-1, -1, -1, -1};
    static double encoder_pulses_float_support[NUM_DRIVEUNIT];
    for (size_t i = 0; i < NUM_DRIVEUNIT; i++) {
        // 32000pwm -> 3.0m/s
        target_wheel_vel[i].data = pwm_sign[i] * msg.data.at(i) / 1500.0;
        // enc 100ppr vel 1m/s sample=2
        //  r=0.04029　制御時間必要やからめんどくさいし、上の/500で曖昧やからここ頑張っても意味ない
        //けどrqt見ながらそれなりにまともな値になってるはず
        encoder_pulses_float_support[i] += enc_sign[i] * msg.data.at(i) / 2000.0;  //適当
    }
    RHWS_encoder_pulses.data[0] = encoder_pulses_float_support[0];
    RHWS_encoder_pulses.data[2] = encoder_pulses_float_support[1];
    RHWS_encoder_pulses.data[5] = encoder_pulses_float_support[2];
    RHWS_encoder_pulses.data[7] = encoder_pulses_float_support[3];
}

void callback_drive_lift_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    if (msg.data.size() != NUM_DRIVEUNIT)
        return;
    int8_t pwm_sign[NUM_DRIVEUNIT] = {1, 1, 1, 1};
    for (size_t i = 0; i < NUM_DRIVEUNIT; i++) {
        target_lift_vel[i].data = pwm_sign[i] * msg.data.at(i) / 100000.0;
    }
}

void callback_main_lift_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    if (msg.data.size() != 1)
        return;
    int8_t pwm_sign[4] = {1, 1, 1, 1};
    for (size_t i = 0; i < 4; i++) {
        target_lift_vel[i + 4].data = pwm_sign[i] * msg.data.at(0) / 35000.0;
        // target_lift_pos[i + 4].data = 1; test
        // ROS_WARN("%d", msg.data.at(0));
    }
}

void callback_arm_motor_pwms(const std_msgs::Int16MultiArray &msg) {
    if (msg.data.size() != 2)
        return;
    int8_t pwm_sign[2] = {1, 1};
    arm_vel[0].data = pwm_sign[0] * msg.data.at(0) / 150000.0;
    arm_vel[1].data = pwm_sign[1] * msg.data.at(1) / 320000.0;
}

double roll, pitch, yaw;
std_msgs::Float32 pitch_float32;
geometry_msgs::Quaternion pose;
void callback_model_states(const gazebo_msgs::ModelStates &model_states) {
    if (model_states.pose.size() <= 2)
        return;

    robot_pose.position.x = model_states.pose.at(2).position.x;
    robot_pose.position.y = model_states.pose.at(2).position.y;
    robot_pose.position.z = model_states.pose.at(2).position.z;

    robot_pose.orientation.w = model_states.pose.at(2).orientation.w;
    robot_pose.orientation.x = model_states.pose.at(2).orientation.x;
    robot_pose.orientation.y = model_states.pose.at(2).orientation.y;
    robot_pose.orientation.z = model_states.pose.at(2).orientation.z;

    pose.x = robot_pose.orientation.x;
    pose.y = robot_pose.orientation.y;
    pose.z = robot_pose.orientation.z;
    pose.w = robot_pose.orientation.w;
    geometry_quat_to_rpy(roll, pitch, yaw, pose);
    pitch_float32.data = roll / M_PI * 180.0;  //がぜぼで確認したらROLLが想定してるPITCHになってたから修正
}

nav_msgs::Odometry p3d_drive_wheel1_pose, p3d_drive_wheel2_pose, p3d_drive_wheel3_pose, p3d_drive_wheel4_pose;
nav_msgs::Odometry p3d_drive_lift1_pose, p3d_drive_lift2_pose, p3d_drive_lift3_pose, p3d_drive_lift4_pose;
nav_msgs::Odometry p3d_main_lift_pose;
nav_msgs::Odometry p3d_unit_y_pose, p3d_unit_x_pose;
void callback_p3d_drive_wheel1_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_wheel1_pose = msg;
}
void callback_p3d_drive_wheel2_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_wheel2_pose = msg;
}
void callback_p3d_drive_wheel3_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_wheel3_pose = msg;
}
void callback_p3d_drive_wheel4_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_wheel4_pose = msg;
}
void callback_p3d_drive_lift1_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_lift1_pose = msg;
}
void callback_p3d_drive_lift2_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_lift2_pose = msg;
}
void callback_p3d_drive_lift3_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_lift3_pose = msg;
}
void callback_p3d_drive_lift4_pose(const nav_msgs::Odometry &msg) {
    p3d_drive_lift4_pose = msg;
}
void callback_p3d_main_lift_pose(const nav_msgs::Odometry &msg) {
    p3d_main_lift_pose = msg;
}
void callback_p3d_unit_y_pose(const nav_msgs::Odometry &msg) {
    p3d_unit_y_pose = msg;
}
void callback_p3d_unit_x_pose(const nav_msgs::Odometry &msg) {
    p3d_unit_x_pose = msg;
}

void drive_lift_fake_sensor_compose() {
    static constexpr double DW_SCAFAC = 1, DL_SCAFAC = -8383, ML_SCAFAC = 5384;
    static constexpr int8_t enc_sign_drive_wheel[NUM_DRIVEUNIT] = {-1, -1, +1, +1};
    static constexpr int8_t enc_sign_drive_lift[NUM_DRIVEUNIT] = {-1, -1, -1, -1};
    static constexpr int8_t enc_sign_main_lift[1] = {+1};

    // RHWS_encoder_pulses.data[0] += enc_sign_drive_wheel[0] * (p3d_drive_wheel1_pose.twist.twist.angular.x) * DW_SCAFAC;
    // RHWS_encoder_pulses.data[2] += enc_sign_drive_wheel[1] * (p3d_drive_wheel1_pose.twist.twist.angular.x) * DW_SCAFAC;
    // RHWS_encoder_pulses.data[5] += enc_sign_drive_wheel[2] * (p3d_drive_wheel1_pose.twist.twist.angular.x) * DW_SCAFAC;
    // RHWS_encoder_pulses.data[7] += enc_sign_drive_wheel[3] * (p3d_drive_wheel1_pose.twist.twist.angular.x) * DW_SCAFAC;

    RHWS_encoder_pulses.data[1] = enc_sign_drive_lift[0] * (p3d_drive_lift1_pose.pose.pose.position.z - 0.1) * DL_SCAFAC;
    RHWS_encoder_pulses.data[3] = enc_sign_drive_lift[1] * (p3d_drive_lift2_pose.pose.pose.position.z - 0.1) * DL_SCAFAC;
    RHWS_encoder_pulses.data[6] = enc_sign_drive_lift[2] * (p3d_drive_lift3_pose.pose.pose.position.z - 0.1) * DL_SCAFAC;
    RHWS_encoder_pulses.data[8] = enc_sign_drive_lift[3] * (p3d_drive_lift4_pose.pose.pose.position.z - 0.1) * DL_SCAFAC;
    RHWS_switch_states.data[0] = (p3d_drive_lift1_pose.pose.pose.position.z - 0.1) > -0.01 ? 1 : 0;
    RHWS_switch_states.data[2] = (p3d_drive_lift2_pose.pose.pose.position.z - 0.1) > -0.01 ? 1 : 0;
    RHWS_switch_states.data[5] = (p3d_drive_lift3_pose.pose.pose.position.z - 0.1) > -0.01 ? 1 : 0;
    RHWS_switch_states.data[7] = (p3d_drive_lift4_pose.pose.pose.position.z - 0.1) > -0.01 ? 1 : 0;

    RHWS_encoder_pulses.data[ENCODER_INDEX_MAIN_LIFT] = enc_sign_main_lift[0] * (p3d_main_lift_pose.pose.pose.position.z - 0.2022) * ML_SCAFAC;
    RHWS_switch_states.data[SWITCH_INDEX_MAIN_LIFT] = (abs((p3d_main_lift_pose.pose.pose.position.z - 0.2022)) < 0.01) ? 1 : 0;

    RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_X] = (1) * (p3d_unit_x_pose.pose.pose.position.x + 0.25) * 7200;
    RHWS_switch_states.data[SWITCH_INDEX_ARM_X] = (abs((p3d_unit_x_pose.pose.pose.position.x + 0.25)) < ERROR_ALLOWANCE_ARM_X / 7200.0) ? 1 : 0;
    RHWS_encoder_pulses.data[ENCODER_INDEX_ARM_Y] = (1) * (p3d_unit_y_pose.pose.pose.position.z + 0.36) * 10735;
    RHWS_switch_states.data[SWITCH_INDEX_ARM_Y] = (abs((p3d_unit_y_pose.pose.pose.position.z + 0.36)) < ERROR_ALLOWANCE_ARM_Y / 10735.0) ? 1 : 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sim_bridge");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Subscriber sub_wheel_motor_pwms = nh.subscribe("wheel_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_wheel_motor_pwms);
    ros::Subscriber sub_drive_lift_motor_pwms = nh.subscribe("drive_lift_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_drive_lift_motor_pwms);
    ros::Subscriber sub_main_lift_motor_pwms = nh.subscribe("main_lift_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_main_lift_motor_pwms);
    ros::Subscriber sub_arm_motor_pwms = nh.subscribe("arm_motor_pwms", QUEUE_SIZE_SUBSCRIBER, callback_arm_motor_pwms);
    ros::Subscriber sub_model_state = nh.subscribe("/gazebo/model_states", QUEUE_SIZE_SUBSCRIBER, callback_model_states);

    ros::Publisher pub_RHWS_IMU_pitch = nh.advertise<std_msgs::Float32>("RHWS/IMU_pitch", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_robot_state = nh.advertise<geometry_msgs::Pose>("robot_pose", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_drive_lift1_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/lift1_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_drive_lift2_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/lift2_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_drive_lift3_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/lift3_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_drive_lift4_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/lift4_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_wheel1_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/wheel1_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_wheel2_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/wheel2_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_wheel3_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/wheel3_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_wheel4_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/wheel4_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_bottom_lift_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/bottom_unit_link_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_second_lift_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/second_unit_link_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_third_lift_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/third_unit_link_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_top_lift_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/top_unit_link_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_arm_lift_x_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/arm_unit_x_link_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_arm_lift_y_vel = nh.advertise<std_msgs::Float64>("/commissioning_robot/arm_unit_y_link_velocity_controller/command", QUEUE_SIZE_PUBLISHER);
    ros::Subscriber sub_p3d_drive_wheel1_pose = nh.subscribe("/p3d_drive_wheel1_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_wheel1_pose);
    ros::Subscriber sub_p3d_drive_wheel2_pose = nh.subscribe("/p3d_drive_wheel2_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_wheel2_pose);
    ros::Subscriber sub_p3d_drive_wheel3_pose = nh.subscribe("/p3d_drive_wheel3_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_wheel3_pose);
    ros::Subscriber sub_p3d_drive_wheel4_pose = nh.subscribe("/p3d_drive_wheel4_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_wheel4_pose);
    ros::Subscriber sub_p3d_drive_lift1_pose = nh.subscribe("/p3d_drive_lift1_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_lift1_pose);
    ros::Subscriber sub_p3d_drive_lift2_pose = nh.subscribe("/p3d_drive_lift2_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_lift2_pose);
    ros::Subscriber sub_p3d_drive_lift3_pose = nh.subscribe("/p3d_drive_lift3_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_lift3_pose);
    ros::Subscriber sub_p3d_drive_lift4_pose = nh.subscribe("/p3d_drive_lift4_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_drive_lift4_pose);
    ros::Subscriber sub_p3d_main_lift_pose = nh.subscribe("/p3d_main_lift_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_main_lift_pose);
    ros::Subscriber sub_p3d_unit_y_pose = nh.subscribe("/p3d_unit_y_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_unit_y_pose);
    ros::Subscriber sub_p3d_unit_x_pose = nh.subscribe("/p3d_unit_x_pose", QUEUE_SIZE_SUBSCRIBER, callback_p3d_unit_x_pose);

    ros::Publisher pub_RHWS_encoder_pulses = nh.advertise<std_msgs::Int32MultiArray>("RHWS/encoder_pulses", QUEUE_SIZE_PUBLISHER);
    ros::Publisher pub_RHWS_switch_states = nh.advertise<std_msgs::UInt16MultiArray>("RHWS/switch_states", QUEUE_SIZE_PUBLISHER);
    RHWS_encoder_pulses.data.resize(NUM_ENCODER);
    RHWS_switch_states.data.resize(NUM_SWITCH);

    dynamic_reconfigure::Server<commissioning_robot::motor_portConfig> server1;
    dynamic_reconfigure::Server<commissioning_robot::motor_portConfig>::CallbackType f1;
    f1 = boost::bind(&callback_DynCfgPrm_motor, _1, _2);
    server1.setCallback(f1);
    dynamic_reconfigure::Server<commissioning_robot::encoder_portConfig> server2;
    dynamic_reconfigure::Server<commissioning_robot::encoder_portConfig>::CallbackType f2;
    f2 = boost::bind(&callback_DynCfgPrm_encoder, _1, _2);
    server2.setCallback(f2);
    dynamic_reconfigure::Server<commissioning_robot::switch_portConfig> server3;
    dynamic_reconfigure::Server<commissioning_robot::switch_portConfig>::CallbackType f3;
    f3 = boost::bind(&callback_DynCfgPrm_switch, _1, _2);
    server3.setCallback(f3);

    while (ros::ok()) {
        drive_lift_fake_sensor_compose();

        pub_robot_state.publish(robot_pose);
        pub_RHWS_IMU_pitch.publish(pitch_float32);

        pub_drive_lift1_vel.publish(target_lift_vel[0]);
        pub_drive_lift2_vel.publish(target_lift_vel[1]);
        pub_drive_lift3_vel.publish(target_lift_vel[2]);
        pub_drive_lift4_vel.publish(target_lift_vel[3]);
        pub_bottom_lift_vel.publish(target_lift_vel[4]);
        pub_second_lift_vel.publish(target_lift_vel[5]);
        pub_third_lift_vel.publish(target_lift_vel[6]);
        pub_top_lift_vel.publish(target_lift_vel[7]);

        pub_wheel1_vel.publish(target_wheel_vel[0]);
        pub_wheel2_vel.publish(target_wheel_vel[1]);
        pub_wheel3_vel.publish(target_wheel_vel[2]);
        pub_wheel4_vel.publish(target_wheel_vel[3]);

        pub_arm_lift_x_vel.publish(arm_vel[0]);
        pub_arm_lift_y_vel.publish(arm_vel[1]);

        pub_RHWS_encoder_pulses.publish(RHWS_encoder_pulses);
        pub_RHWS_switch_states.publish(RHWS_switch_states);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
