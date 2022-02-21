#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <string>

geometry_msgs::Twist robot_velocity;
geometry_msgs::Pose2D robot_position;
sensor_msgs::Joy state_joy;
sensor_msgs::JointState js;
std_msgs::Float32MultiArray robot_lift_position;

void callback_wheel_vel_tf_publish(std_msgs::Float32MultiArray wheels_vel) {
    if (wheels_vel.data.empty())
        return;

    static tf::TransformBroadcaster br;
    static std_msgs::Float32MultiArray wheels_angle;
    tf::Transform transform_w1, transform_w2, transform_w3, transform_w4;
    tf::Quaternion quaternion_w1, quaternion_w2, quaternion_w3, quaternion_w4;
    wheels_angle.data.resize(4);
    wheels_angle.data.at(0) += wheels_vel.data.at(0);
    wheels_angle.data.at(1) += wheels_vel.data.at(1);
    wheels_angle.data.at(2) += wheels_vel.data.at(2);
    wheels_angle.data.at(3) += wheels_vel.data.at(3);
    quaternion_w1.setRPY(wheels_angle.data.at(0), 0, 0);
    quaternion_w2.setRPY(wheels_angle.data.at(1), 0, 0);
    quaternion_w3.setRPY(wheels_angle.data.at(2), 0, 0);
    quaternion_w4.setRPY(wheels_angle.data.at(3), 0, 0);

    static constexpr double wheel_joint_x = 0.0495;
    static constexpr double wheel_joint_y = 0.0500;
    static constexpr double wheel_joint_z = 0.0720;

    transform_w1.setOrigin(tf::Vector3(+wheel_joint_x, +wheel_joint_y, -wheel_joint_z));
    transform_w2.setOrigin(tf::Vector3(-wheel_joint_x, +wheel_joint_y, -wheel_joint_z));
    transform_w3.setOrigin(tf::Vector3(-wheel_joint_x, -wheel_joint_y, -wheel_joint_z));
    transform_w4.setOrigin(tf::Vector3(+wheel_joint_x, -wheel_joint_y, -wheel_joint_z));
    transform_w1.setRotation(quaternion_w1);
    transform_w2.setRotation(quaternion_w2);
    transform_w3.setRotation(quaternion_w3);
    transform_w4.setRotation(quaternion_w4);
    br.sendTransform(tf::StampedTransform(transform_w1, ros::Time::now(), "lift1_link", "wheel1_link"));
    br.sendTransform(tf::StampedTransform(transform_w2, ros::Time::now(), "lift2_link", "wheel2_link"));
    br.sendTransform(tf::StampedTransform(transform_w3, ros::Time::now(), "lift3_link", "wheel3_link"));
    br.sendTransform(tf::StampedTransform(transform_w4, ros::Time::now(), "lift4_link", "wheel4_link"));
}

void robot_lift_publush() {
    // if (lift_position.data.empty())
    // {
    //     ROS_INFO("empty");
    //     return;
    // }

    static tf::TransformBroadcaster br;
    tf::Transform transform_l1, transform_l2, transform_l3, transform_l4;
    tf::Quaternion quaternion_l1, quaternion_l2, quaternion_l3, quaternion_l4;
    robot_lift_position.data.resize(4);
    robot_lift_position.data.at(0) += robot_velocity.linear.z;
    robot_lift_position.data.at(1) += robot_velocity.linear.z;
    robot_lift_position.data.at(2) += robot_velocity.linear.z;
    robot_lift_position.data.at(3) += robot_velocity.linear.z;
    static constexpr double lift_max = 0;
    static constexpr double lift_min = -0.334;
    for (auto &&i : robot_lift_position.data) {
        if (i > lift_max)
            i = lift_max;
        if (i < lift_min)
            i = lift_min;
        ROS_INFO("%lf", i);
    }

    static constexpr double lift_joint_x = 0.18;
    static constexpr double lift_joint_y = 0.2065;

    quaternion_l1.setRPY(0, 0, 0);
    quaternion_l2.setRPY(0, 0, 0);
    quaternion_l3.setRPY(0, 0, 0);
    quaternion_l4.setRPY(0, 0, 0);
    transform_l1.setOrigin(tf::Vector3(+lift_joint_x, +lift_joint_y, robot_lift_position.data.at(0)));
    transform_l2.setOrigin(tf::Vector3(-lift_joint_x, +lift_joint_y, robot_lift_position.data.at(1)));
    transform_l3.setOrigin(tf::Vector3(-lift_joint_x, -lift_joint_y, robot_lift_position.data.at(2)));
    transform_l4.setOrigin(tf::Vector3(+lift_joint_x, -lift_joint_y, robot_lift_position.data.at(3)));
    // ROS_INFO("x:%.3lf,y:%.3lf,z:%.3lf", robot_lift_position.data.at(0), robot_lift_position.data.at(1), robot_lift_position.data.at(3));
    transform_l1.setRotation(quaternion_l1);
    transform_l2.setRotation(quaternion_l2);
    transform_l3.setRotation(quaternion_l3);
    transform_l4.setRotation(quaternion_l4);
    br.sendTransform(tf::StampedTransform(transform_l1, ros::Time::now(), "base_frame_link", "lift1_link"));
    br.sendTransform(tf::StampedTransform(transform_l2, ros::Time::now(), "base_frame_link", "lift2_link"));
    br.sendTransform(tf::StampedTransform(transform_l3, ros::Time::now(), "base_frame_link", "lift3_link"));
    br.sendTransform(tf::StampedTransform(transform_l4, ros::Time::now(), "base_frame_link", "lift4_link"));
}

void callback_joy_parse(const sensor_msgs::Joy &joy_msg) {
    robot_velocity.linear.x = -joy_msg.axes[0] * 0.15;
    robot_velocity.linear.y = joy_msg.axes[1] * 0.15;
    double left_rot_center_fixed = (-joy_msg.axes[2] / 2.0) + 1.0;
    double right_rot_center_fixed = (-joy_msg.axes[5] / 2.0) + 1.0;
    robot_velocity.angular.z = (right_rot_center_fixed - left_rot_center_fixed) * 0.2;

    robot_velocity.linear.z = joy_msg.axes[4] * 0.005;
}

void robot_pose_publish(geometry_msgs::Pose2D pose) {
    robot_position.x += (robot_velocity.linear.x * cos(robot_position.theta) - robot_velocity.linear.y * sin(robot_position.theta)) / 50.0;
    robot_position.y += (robot_velocity.linear.x * sin(robot_position.theta) + robot_velocity.linear.y * cos(robot_position.theta)) / 50.0;
    robot_position.theta -= robot_velocity.angular.z / 25.0;
    // ROS_INFO("x:%.3lf,y:%.3lf,z:%.3lf", robot_position.x, robot_position.y, robot_position.theta);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.x, pose.y, 0.1));
    tf::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_frame_link"));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(144);
    ros::Publisher pub_joint = nh.advertise<sensor_msgs::JointState>("joint_states", 2);
    ros::Publisher pub_inv_kin = nh.advertise<geometry_msgs::Twist>("robot_global_vel", 2);
    ros::Subscriber sub_wheel_vel = nh.subscribe("inv_kin_wheels_vel", 10, callback_wheel_vel_tf_publish);
    ros::Subscriber sub_joy = nh.subscribe("joy", 10, callback_joy_parse);

    float step = 1;
    robot_position.x = 0;
    robot_position.y = 0;
    robot_position.theta = 0;
    robot_pose_publish(robot_position);

    js.header.stamp = ros::Time::now();
    js.name.resize(4);
    js.position.resize(4);
    js.velocity.resize(4);
    js.effort.resize(4);
    js.name[0] = "wheel1_joint";
    js.name[1] = "wheel2_joint";
    js.name[2] = "wheel3_joint";
    js.name[3] = "wheel4_joint";
    js.position[0] = 0;
    js.position[1] = 0;
    js.position[2] = 0;
    js.position[3] = 0;
    pub_joint.publish(js);

    js.header.stamp = ros::Time::now();
    js.name[0] = "lift1_joint";
    js.name[1] = "lift2_joint";
    js.name[2] = "lift3_joint";
    js.name[3] = "lift4_joint";
    js.position[0] = 0;
    js.position[1] = 0;
    js.position[2] = 0;
    js.position[3] = 0;
    pub_joint.publish(js);
    ros::spinOnce();

    while (ros::ok()) {
        pub_inv_kin.publish(robot_velocity);
        robot_pose_publish(robot_position);
        robot_lift_publush();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
