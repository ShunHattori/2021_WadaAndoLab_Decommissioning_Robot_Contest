#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray


def talker():
    pub = rospy.Publisher("/canout1", UInt8MultiArray, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(60)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        fake_can_data_list = list()
        fake_can_data_list.append(0)
        fake_can_data_list.append(128)
        fake_can_data_list.append(0)
        fake_can_data_list.append(128)
        fake_can_data_list.append(0)
        fake_can_data_list.append(128)
        fake_can_data_list.append(0)
        fake_can_data_list.append(128)
        fake_can_data = UInt8MultiArray(data=fake_can_data_list)
        pub.publish(fake_can_data)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
