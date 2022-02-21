#include <utils.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "RHWS_joint_state_publisher");
    ros::NodeHandle nh;
    ros::Rate loop_rate(NODE_FREQ);
    ros::Publisher pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", QUEUE_SIZE_PUBLISHER);

    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();
    // js.name.push_back("lift1_link_joint");
    // js.name.push_back("lift2_link_joint");
    // js.name.push_back("lift3_link_joint");
    // js.name.push_back("lift4_link_joint");
    // js.name.push_back("mechanum1_link_joint");
    // js.name.push_back("mechanum2_link_joint");
    // js.name.push_back("mechanum3_link_joint");
    // js.name.push_back("mechanum4_link_joint");
    // js.name.push_back("bottom_unit_link_joint");
    // js.name.push_back("second_unit_link_joint");
    // js.name.push_back("third_unit_link_joint");
    // js.name.push_back("top_unit_link_joint");
    // js.name.push_back("arm_unit_y_link_joint");
    // js.name.push_back("arm_unit_x_base_link_joint");
    // js.name.push_back("arm_unit_x_link_joint");
    // js.name.push_back("mechanum1_link_barrel1_joint");
    // js.name.push_back("mechanum1_link_barrel2_joint");
    // js.name.push_back("mechanum1_link_barrel3_joint");
    // js.name.push_back("mechanum1_link_barrel4_joint");
    // js.name.push_back("mechanum1_link_barrel5_joint");
    // js.name.push_back("mechanum1_link_barrel6_joint");
    // js.name.push_back("mechanum1_link_barrel7_joint");
    // js.name.push_back("mechanum1_link_barrel8_joint");
    // js.name.push_back("mechanum1_link_barrel9_joint");
    // js.name.push_back("mechanum2_link_barrel1_joint");
    // js.name.push_back("mechanum2_link_barrel2_joint");
    // js.name.push_back("mechanum2_link_barrel3_joint");
    // js.name.push_back("mechanum2_link_barrel4_joint");
    // js.name.push_back("mechanum2_link_barrel5_joint");
    // js.name.push_back("mechanum2_link_barrel6_joint");
    // js.name.push_back("mechanum2_link_barrel7_joint");
    // js.name.push_back("mechanum2_link_barrel8_joint");
    // js.name.push_back("mechanum2_link_barrel9_joint");
    // js.name.push_back("mechanum3_link_barrel1_joint");
    // js.name.push_back("mechanum3_link_barrel2_joint");
    // js.name.push_back("mechanum3_link_barrel3_joint");
    // js.name.push_back("mechanum3_link_barrel4_joint");
    // js.name.push_back("mechanum3_link_barrel5_joint");
    // js.name.push_back("mechanum3_link_barrel6_joint");
    // js.name.push_back("mechanum3_link_barrel7_joint");
    // js.name.push_back("mechanum3_link_barrel8_joint");
    // js.name.push_back("mechanum3_link_barrel9_joint");
    // js.name.push_back("mechanum4_link_barrel1_joint");
    // js.name.push_back("mechanum4_link_barrel2_joint");
    // js.name.push_back("mechanum4_link_barrel3_joint");
    // js.name.push_back("mechanum4_link_barrel4_joint");
    // js.name.push_back("mechanum4_link_barrel5_joint");
    // js.name.push_back("mechanum4_link_barrel6_joint");
    // js.name.push_back("mechanum4_link_barrel7_joint");
    // js.name.push_back("mechanum4_link_barrel8_joint");
    // js.name.push_back("mechanum4_link_barrel9_joint");

    js.position.resize(js.name.size());

    for (int i = 0; i < js.name.size(); i++) {
        js.position.at(i) = 0;
    }

    while (ros::ok()) {
        js.header.stamp = ros::Time::now();
        pub_joint_states.publish(js);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
