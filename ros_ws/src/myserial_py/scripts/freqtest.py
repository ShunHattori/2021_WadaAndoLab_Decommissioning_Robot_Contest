#!/usr/bin/env python
# license removed for brevity
from os import read, wait
from struct import calcsize

from serial.serialutil import SerialBase
import rospy
import serial
from rospy import topics
from rospy.core import deprecated
from rospy.topics import Subscriber
from std_msgs.msg import UInt8MultiArray
import signal
import sys
import time
from collections import deque


class canoutTopicSubscriber:
    def __init__(self, _port, _topicName):
        self.port = serial.Serial()
        self.port = _port
        self.topicName = _topicName
        self.topicData = list(range(8))
        self.calc_checksum = 0
        prefix_topic = "/canout"
        self.topic_id = int(self.topicName[len(prefix_topic) :])  # canoutよりあとすべてを取得
        self.subscriber = rospy.Subscriber(self.topicName, UInt8MultiArray, self.callbackStoreData, queue_size=5)

    def callbackStoreData(self, _data):
        data_sum = 0
        for i in range(8):
            self.topicData[i] = _data.data[i]
            data_sum += self.topicData[i]
        self.calc_checksum = 255 - ((self.topic_id + data_sum) % 256)

    def send(self):
        try:
            self.port.flushOutput()
            self.port.write((0xFF).to_bytes(1, "big"))
            self.port.write((0xFE).to_bytes(1, "big"))
            self.port.write(self.topic_id.to_bytes(1, "big"))
            self.port.write(self.topicData[0].to_bytes(1, "big"))
            self.port.write(self.topicData[1].to_bytes(1, "big"))
            self.port.write(self.topicData[2].to_bytes(1, "big"))
            self.port.write(self.topicData[3].to_bytes(1, "big"))
            self.port.write(self.topicData[4].to_bytes(1, "big"))
            self.port.write(self.topicData[5].to_bytes(1, "big"))
            self.port.write(self.topicData[6].to_bytes(1, "big"))
            self.port.write(self.topicData[7].to_bytes(1, "big"))
            self.port.write(self.calc_checksum.to_bytes(1, "big"))
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
        except:
            rospy.logfatal("error has occured when sending can data via seiral protocol")


class ROSCANBridge:
    def __init__(self):
        self.portname = PORT_NAME
        self.baudrate = PORT_SPEED
        self.port = serial.Serial(self.portname, self.baudrate, timeout=2)
        time.sleep(1)  # wait for microcontroller boot
        if not self.port == None:
            rospy.loginfo("ROSCANBridge initialize successful.")
        else:
            rospy.logfatal("ROSCANBridge initialize failed. ")

        self.timePrevious = 0
        self.canoutSubIndex = 0
        self.canoutSubscriberList = []
        self.caninPublisherList = []
        self.canoutTopicList = []
        self.readDataCAN = []
        self.CANAllIDList = []

    def retryOpeningPort(self):
        while True:
            try:
                rospy.logfatal_throttle(0.5, "attempting port close")
                self.port.close()
            except:
                rospy.logfatal_throttle(0.5, "port close attempt failed.")
            try:
                self.port.open()
                self.port.flushInput()
                self.port.flushOutput()
                rospy.loginfo("port reopening success")
                time.sleep(1)
                break
            except:
                rospy.logwarn_throttle(0.5, "self.port.open() failed and catched except")

    # convert bytes data to int format e.g. port.read() method returned data
    def conv_int(self, bytes_formed_data):
        return int.from_bytes(bytes_formed_data, "big")

    def sendDataToROS(self):
        pass

    def searchCANMessage(self):
        if self.port.inWaiting() > 11:
            # for i in range(11):
            #     rospy.loginfo(self.port.read())
            # return
            # rospy.loginfo("filled up FIFO")
            if self.conv_int(self.port.read()) != 0xFF:
                return
            # rospy.loginfo("first")
            if self.conv_int(self.port.read()) != 0xFE:
                return
            # rospy.loginfo("second")
            canid = self.conv_int(self.port.read())
            # if canid != 6:
            #     return

            readData = list()
            for i in range(8):
                readData.append(self.conv_int(self.port.read()))
            frame_str = f"ID:{canid}, {readData[0]} {readData[1]} {readData[2]} {readData[3]} {readData[4]} {readData[5]} {readData[6]} {readData[7]}"
            self.readDataCAN = readData
            self.port.flushInput()  # 超重要！

            rospy.loginfo(frame_str)
            # rospy.loginfo(String(len(read_data)))

            # new canid registration
            newCANIDFlag = True
            for i in range(len(self.CANAllIDList)):
                if self.CANAllIDList[i] == canid:
                    newCANIDFlag = False
            if newCANIDFlag == True:
                self.CANAllIDList.append(canid)
                self.caninPublisherList.append(rospy.Publisher("/canin" + str(canid), UInt8MultiArray, queue_size=5))

            publisherIndex = self.CANAllIDList.index(canid)
            topicData = UInt8MultiArray(data=self.readDataCAN)
            self.caninPublisherList[publisherIndex].publish(topicData)

    def sendDataToCAN(self):
        timeNow = rospy.Time.now().to_nsec()

        if (timeNow - self.timePrevious) < (5 * 1000 * 1000):
            return  # return ealry when interval comes within 5ms
        if len(self.canoutTopicList) <= 0:
            return  # return ealry when no canout topic available

        self.canoutSubscriberList[self.canoutSubIndex].send()
        self.canoutSubIndex += 1
        if self.canoutSubIndex >= len(self.canoutTopicList):
            self.canoutSubIndex = 0
        self.timePrevious = rospy.Time.now().to_nsec()

    def searchROSTopic(self):
        allAvailableROSTopic = rospy.get_published_topics()

        for i in range(len(allAvailableROSTopic)):
            topicName = allAvailableROSTopic[i][0]
            if topicName.find("/canout") == -1:  # (見つからなければ-1) LISTは[トピック名,メッセージ型]の形式
                continue

            newTopicNameFlag = True
            for k in range(len(self.canoutTopicList)):
                if self.canoutTopicList[k].find(topicName) >= 0:  # TOPIC NAMEが見つかったとき
                    newTopicNameFlag = False  # すでに登録されているのでフラグ消去

            if newTopicNameFlag == True:
                self.canoutTopicList.append(topicName)  # /canoutから始まるトピック名を保存する
                self.canoutSubscriberList.append(canoutTopicSubscriber(self.port, topicName))

    def work(self):
        try:
            if self.port.isOpen():
                # start CAN -> ROS processing
                self.searchCANMessage()
                # end of CAN -> ROS process

                # start ROS -> CAN processing
                self.searchROSTopic()
                self.sendDataToCAN()
                # end of ROS -> CAN process

        except (serial.SerialException, OSError) as e:
            rospy.logwarn("catched exception. " + str(e))
            self.retryOpeningPort()


def work():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.init_node("ROSCANBridge_node")
    rate = rospy.Rate(NODE_FREQ)

    instance = ROSCANBridge()

    while not rospy.is_shutdown():
        instance.work()
        rate.sleep()


PORT_NAME = "/dev/ttyACM2"
PORT_SPEED = 516000
NODE_FREQ = 100000

if __name__ == "__main__":
    try:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        rospy.init_node("ROSCANBridge_node")
        rate = rospy.Rate(NODE_FREQ)
        pub_datafreq = rospy.Publisher("datafreq", UInt8MultiArray, queue_size=5)
        port = serial.Serial(PORT_NAME, PORT_SPEED, timeout=2)
        # changing input buffer is NOT supported (only windows)
        # https://stackoverflow.com/questions/12302155/how-to-expand-input-buffer-size-of-pyserial
        # https://forums.raspberrypi.com/viewtopic.php?t=222088
        count = 0
        buffering = deque(range(100))
        tx = list(range(12))
        while True:
            # count += 1
            # if count > 100:
            #     port.write((55).to_bytes(1, "big"))
            #     rospy.loginfo(port.out_waiting)
            #     rospy.loginfo(port.in_waiting)
            #     port.flushInput()
            #     count = 0
            # tx[0] = int.from_bytes(port.read(), "big")
            # topicData = UInt8MultiArray(data=tx)
            # pub_datafreq.publish(topicData)
            # rate.sleep()
            for i in range(port.in_waiting):
                buffering.append(int.from_bytes(port.read(), "big"))
            count += 1
            if count > 100:
                rospy.loginfo(f"port.in_waiting:{port.in_waiting}")
                rospy.loginfo(f"len(buffering):{len(buffering)}")
                count = 0

            if len(buffering) > 11:
                tx[0] = buffering.popleft()
                if tx[0] != 0xFF:
                    continue
                for i in range(11):
                    tx[i + 1] = buffering.popleft()
                    topicData = UInt8MultiArray(data=tx)
                    pub_datafreq.publish(topicData)

            # rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("catched ROSInterruptException")
