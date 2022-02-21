#include <utils.hpp>

double constexpr target_roll = 0, roll_p_gain = 0.002, roll_i_gain = 0.000001,
                 pitch_p_gain = 0.002, pitch_i_gain = 0.00001,
                 i_clamp = 0.0002;
pid_controller orientation_roll_controller(roll_p_gain, roll_i_gain, 0);
pid_controller orientation_pitch_controller(pitch_p_gain, pitch_i_gain, 0);
std_msgs::Float64 lift_position[NUM_DRIVEUNIT];

std_msgs::Float64MultiArray drive_lift_position;

void callback_pose_controller(const commissioning_robot::ControlState &msg) {
    if (msg.mode.size() != NUM_DRIVEUNIT || msg.manual.size() != NUM_DRIVEUNIT)
        return;

    if (msg.mode[0] == CONTROL_MODE::FOLLOW) {
        double roll = 0, pitch = 0;  // taking data from sensor feedback topic "TODO"
        orientation_roll_controller.set_reference(0);
        orientation_pitch_controller.set_reference(0);
        orientation_roll_controller.set_current(roll);
        orientation_pitch_controller.set_current(pitch);

        orientation_roll_controller.update();
        orientation_pitch_controller.update();

        double roll_control_term = orientation_roll_controller.get_control_variable();
        double pitch_control_term = orientation_pitch_controller.get_control_variable();

        lift_position[0].data -= (roll_control_term);
        lift_position[1].data -= (roll_control_term);
        lift_position[2].data += (roll_control_term);
        lift_position[3].data += (roll_control_term);

        lift_position[0].data += (pitch_control_term);
        lift_position[1].data -= (pitch_control_term);
        lift_position[2].data -= (pitch_control_term);
        lift_position[3].data += (pitch_control_term);

        //可動域（下限）を超えた場合、他のリフトに移動量を加算してそのリフトの制御量をゼロにする。
        for (int i = 0; i < NUM_DRIVEUNIT; i++) {
            if (lift_position[i].data > 0) {
                for (int j = 0; j < NUM_DRIVEUNIT; j++) {
                    if (j != i)
                        lift_position[j].data += -lift_position[i].data;
                }
                lift_position[i].data = 0;
            }
        }

        ROS_INFO("lift1:%.5lf,lift2:%.5lf,lift3:%.5lf,lift4:%.5lf\r\n", lift_position[0].data, lift_position[1].data, lift_position[2].data, lift_position[3].data);

        static constexpr double lift_max = 0;
        static constexpr double lift_min = -0.334;
        for (int i = 0; i < NUM_DRIVEUNIT; i++) {
            lift_position[i].data = constrain(lift_position[i].data, lift_min, lift_max);
        }
        ROS_INFO("lift1:%.5lf,lift2:%.5lf,lift3:%.5lf,lift4:%.5lf\r\n", lift_position[0].data, lift_position[1].data, lift_position[2].data, lift_position[3].data);

    } else if (msg.mode[0] == CONTROL_MODE::MANUAL) {
        for (int i = 0; i < NUM_DRIVEUNIT; i++) {
            drive_lift_position.data[i] = msg.manual[i];
        }
    } else if (msg.mode[0] == CONTROL_MODE::STOP) {
        for (int i = 0; i < NUM_DRIVEUNIT; i++) {
            drive_lift_position.data[i] = 0;
        }
    }
}

int main(int argv, char **argc) {
    ros::init(argv, argc, "POSE CONTROLLER");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Subscriber sub_ControlState_pose_controller = nh.subscribe("ControlState/pose_controller", QUEUE_SIZE_SUBSCRIBER, callback_pose_controller);
    ros::Publisher pub_FeedbackState_pose_controller = nh.advertise<commissioning_robot::FeedbackState>("FeedbackState/pose_controller", QUEUE_SIZE_PUBLISHER);

    ros::Publisher pub_drive_lift_position = nh.advertise<std_msgs::Float64MultiArray>("drive_lift_position", QUEUE_SIZE_PUBLISHER);
    drive_lift_position.data.resize(NUM_DRIVEUNIT);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        pub_drive_lift_position.publish(drive_lift_position);
    }
}