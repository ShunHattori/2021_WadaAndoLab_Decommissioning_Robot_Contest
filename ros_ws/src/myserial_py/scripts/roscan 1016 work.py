#!/usr/bin/env python
# license removed for brevity
from os import wait
from struct import calcsize
import rospy
import serial
from rospy import topics
from rospy.topics import Subscriber
from std_msgs.msg import UInt8MultiArray
import signal
import sys
import time


can_all_id_list = list()
can_canin_publisher_list = list()

port_name = "/dev/ttyACM0"
port_speed = 516000
NODE_FREQ = 1500

# convert bytes data to int format e.g. port.read() method returned data
def conv_int(bytes_formated_data):
    return int.from_bytes(bytes_formated_data, "big")


def CAN_ROS_work(port):
    if port.inWaiting() > 11:
        # for i in range(11):
        #     rospy.loginfo(port.read())
        # return
        # rospy.loginfo("filled up FIFO")
        if conv_int(port.read()) != 0xFF:
            return
        # rospy.loginfo("first")
        if conv_int(port.read()) != 0xFE:
            return
        # rospy.loginfo("second")
        id = conv_int(port.read())
        # if id != 6:
        #     return

        read_data = list()
        for i in range(8):
            read_data.append(conv_int(port.read()))
        frame_str = f"ID:{id}, {read_data[0]} {read_data[1]} {read_data[2]} {read_data[3]} {read_data[4]} {read_data[5]} {read_data[6]} {read_data[7]}"
        port.flushInput()  # 超重要！

        # rospy.loginfo(frame_str)
        # rospy.loginfo(String(len(read_data)))

        # new id registration
        new_can_id_flag = 1
        for i in range(len(can_all_id_list)):
            if can_all_id_list[i] == id:
                new_can_id_flag = 0
        if new_can_id_flag == 1:
            can_all_id_list.append(id)
            can_canin_publisher_list.append(
                rospy.Publisher("/canin" + str(id), UInt8MultiArray, queue_size=5)
            )

        publisher_index = can_all_id_list.index(id)
        topic_data = UInt8MultiArray(data=read_data)
        can_canin_publisher_list[publisher_index].publish(topic_data)

        return

        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)


ros_all_topic_list = list()
ros_canout_topic_list = list()


class ArraySubscriber:
    def __init__(self, _port, _topic_name):
        self.port = serial.Serial()
        self.port = _port
        self.topic_name = _topic_name
        self.datas = list(range(8))
        self.calc_checksum = 0
        prefix_topic = "/" + "canout"
        self.topic_id = int(self.topic_name[len(prefix_topic) :])  # canoutよりあとすべてを取得
        self.subscriber = rospy.Subscriber(
            self.topic_name, UInt8MultiArray, self.callback, queue_size=5
        )

    def callback(self, _data):
        # print(type(_data))
        self.datas[0] = _data.data[0]
        self.datas[1] = _data.data[1]
        self.datas[2] = _data.data[2]
        self.datas[3] = _data.data[3]
        self.datas[4] = _data.data[4]
        self.datas[5] = _data.data[5]
        self.datas[6] = _data.data[6]
        self.datas[7] = _data.data[7]
        data_sum = (
            self.datas[0]
            + self.datas[1]
            + self.datas[2]
            + self.datas[3]
            + self.datas[4]
            + self.datas[5]
            + self.datas[6]
            + self.datas[7]
        )
        self.calc_checksum = 255 - ((self.topic_id + data_sum) % 256)
        # rospy.loginfo("")
        # rospy.loginfo(0xFF)
        # rospy.loginfo(0xFE)
        # rospy.loginfo(self.topic_id)
        # rospy.loginfo(data.data[0])
        # rospy.loginfo(data.data[1])
        # rospy.loginfo(data.data[2])
        # rospy.loginfo(data.data[3])
        # rospy.loginfo(data.data[4])
        # rospy.loginfo(data.data[5])
        # rospy.loginfo(data.data[6])
        # rospy.loginfo(data.data[7])
        # rospy.loginfo(calc_checksum)
        return

    def send(self):
        print("sending data")
        self.port.flushOutput()
        self.port.write((0xFF).to_bytes(1, "big"))
        self.port.write((0xFE).to_bytes(1, "big"))
        self.port.write(self.topic_id.to_bytes(1, "big"))
        self.port.write(self.datas[0].to_bytes(1, "big"))
        self.port.write(self.datas[1].to_bytes(1, "big"))
        self.port.write(self.datas[2].to_bytes(1, "big"))
        self.port.write(self.datas[3].to_bytes(1, "big"))
        self.port.write(self.datas[4].to_bytes(1, "big"))
        self.port.write(self.datas[5].to_bytes(1, "big"))
        self.port.write(self.datas[6].to_bytes(1, "big"))
        self.port.write(self.datas[7].to_bytes(1, "big"))
        self.port.write(self.calc_checksum.to_bytes(1, "big"))


prev_time = 0
Sub_list = list()
Sub_index = 0


def ROS_CAN_work(port):
    ros_all_topic_list = rospy.get_published_topics()
    for i in range(len(ros_all_topic_list)):
        topic_name = ros_all_topic_list[i][0]
        if topic_name.find("/canout") != -1:  # (見つからなければ-1) LISTは[トピック名,メッセージ型]の形式
            new_topic_name_flag = 1
            for k in range(len(ros_canout_topic_list)):
                if ros_canout_topic_list[k].find(topic_name) >= 0:  # TOPIC NAMEが見つかったとき
                    new_topic_name_flag = 0  # すでに登録されていたらフラグ消去
            if new_topic_name_flag == 1:
                ros_canout_topic_list.append(topic_name)  # /canoutから始まるトピック名を保存する
                Sub_list.append(ArraySubscriber(port, topic_name))

    # print(ros_all_topic_list)
    now = rospy.Time.now().to_nsec()
    global prev_time
    global Sub_index

    if now - prev_time > 5 * 1000 * 1000 and len(ros_canout_topic_list) > 0:
        Sub_list[Sub_index].send()
        Sub_index += 1
        prev_time = rospy.Time.now().to_nsec()
        if Sub_index >= len(ros_canout_topic_list):
            Sub_index = 0

    # print(ros_canout_topic_list)
    return


class SignalHandler(object):
    def __init__(self, controller):
        self.controller = controller

    def __call__(self, signum, frame):
        rospy.loginfo("Shutting down...")
        self.controller.exit()
        sys.exit(0)


def work():
    rospy.init_node("roscan_serial")
    rate = rospy.Rate(NODE_FREQ)  # 2000Hz
    port = serial.Serial(port_name, port_speed, timeout=2)  # 516kHz
    time.sleep(2)  # wait for arudino boot
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    while not rospy.is_shutdown():
        try:
            if port.isOpen():
                # start CAN -> ROS processing
                CAN_ROS_work(port)
                # end of CAN -> ROS process

                # start ROS -> CAN processing
                ROS_CAN_work(port)
                # end of ROS -> CAN process
            rate.sleep()
        except (serial.SerialException, OSError) as e:
            print("catched IO error. check USB connection:" + str(e))
            print(type(e))
            try:
                port.close()
            except:
                print("cant close the port")
                pass

            while True:
                time.sleep(0.1)
                try:
                    print("1")
                    if port == None:
                        print("port is None")
                        port.open()

                    if not port == None:
                        print("port is not None")
                        if port.isOpen():
                            port.close()
                        port.open()

                    if port.isOpen():
                        print("port is open")
                        try:
                            port.flushInput()
                            port.flushOutput()
                        except:
                            print(
                                "Catched exception when execute port.flushInput or port.flushOutput"
                            )
                        break

                except (serial.SerialException, OSError) as e:
                    print("reopen process failed.")
                    print(e)


class ROSCANBridge:
    def __init__(self):
        self.baudrate = 516000
        self.port = "dev/ttyUSB0"
        pass

    def initializePort(self):
        pass

    def retryOpeningPort(self):
        pass

    def sendDataToCAN(self):
        pass

    def work(self):
        pass


if __name__ == "__main__":
    try:
        work()
    except rospy.ROSInterruptException:
        rospy.loginfo("catched ROSInterruptException")
        pass
