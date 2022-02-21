#include <utils.hpp>

std_msgs::Float32MultiArray wheel_velocity;

void callback_drive_inverse_kinematics(const geometry_msgs::Twist &msg) {
    double x = msg.linear.x;
    double y = msg.linear.y;
    double theta = msg.angular.z;
    static double center_from_wheel = 1;
    static double wheel_angle = dtr(45);
    wheel_velocity.data.resize(NUM_DRIVEUNIT);
    wheel_velocity.data.at(0) = -sin(wheel_angle) * x + cos(wheel_angle) * y - center_from_wheel * theta;
    wheel_velocity.data.at(1) = +sin(wheel_angle) * x + cos(wheel_angle) * y + center_from_wheel * theta;
    wheel_velocity.data.at(2) = -sin(wheel_angle) * x + cos(wheel_angle) * y + center_from_wheel * theta;
    wheel_velocity.data.at(3) = +sin(wheel_angle) * x + cos(wheel_angle) * y - center_from_wheel * theta;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_inverse_kinematics");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);

    ros::Publisher pub_wheel_velocity = nh.advertise<std_msgs::Float32MultiArray>("inverse_kinematics_drive_wheels_velocity", QUEUE_SIZE_PUBLISHER);
    ros::Subscriber sub_drive_inverse_kinematics = nh.subscribe("drive_inverse_kinematics_Twist", QUEUE_SIZE_SUBSCRIBER, callback_drive_inverse_kinematics);

    while (ros::ok()) {
        pub_wheel_velocity.publish(wheel_velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
