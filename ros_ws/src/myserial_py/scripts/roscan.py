#!/usr/bin/env python
# license removed for brevity

import signal
import time
from xmlrpc.client import ProtocolError
import rospy
import serial
from collections import deque
from std_msgs.msg import UInt8MultiArray

PORT_NAME = "/dev/ttyUSB0"
PORT_SPEED = 516000
NODE_FREQ = 1000
ENABLE_DEBUG = False


def debugLog(printMode, interval, msg):
    if ENABLE_DEBUG == False:
        return
    if printMode == "log":
        rospy.loginfo_throttle(interval, msg)
        return
    if printMode == "warn":
        rospy.logwarn_throttle(interval, msg)
        return
    if printMode == "error":
        rospy.logerr_throttle(interval, msg)
        return
    if printMode == "fetal":
        rospy.logdebug_throttle(interval, msg)
        return


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

        self.timePrevious_CANsend = 0
        self.timePrevious_ROSTopicSearch = 0
        self.canoutSubIndex = 0
        self.canoutSubscriberList = []
        self.caninPublisherList = []
        self.canoutTopicList = []
        self.readDataCAN = []
        self.CANAllIDList = []
        self.CANMessageBuffer = deque()
        self.loopcount = 0
        self.timePrevious_loopcount = 0

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
                time.sleep(2)
                break
            except:
                rospy.logwarn_throttle(0.5, "self.port.open() failed and catched except")
            time.sleep(0.5)

    # convert bytes data to int format e.g. port.read() method returned data
    def conv_int(self, bytes_formed_data):
        return int.from_bytes(bytes_formed_data, "big")

    def sendDataToROS(self):
        pass

    def searchCANMessage(self):
        rospy.loginfo_throttle(1, f"self.port.in_waiting:{self.port.in_waiting}")
        rospy.loginfo_throttle(1, f"len(self.CANMessageBuffer):{len(self.CANMessageBuffer)}")
        if self.port.in_waiting > 4000:  # linux rx serial buffer is 4096 (CAN NOT BE CHANGED)
            self.port.flushInput()

        if len(self.CANMessageBuffer) > 2000:
            self.CANMessageBuffer.clear()
            rospy.logfatal(f"CANMessageBuffer size is over 2000, erased all data")

        for i in range(self.port.in_waiting):
            self.CANMessageBuffer.append(int.from_bytes(self.port.read(), "big"))

        while len(self.CANMessageBuffer) > 11:
            if self.CANMessageBuffer.popleft() != 0xFF:
                continue
            if self.CANMessageBuffer.popleft() != 0xFE:
                continue
            canid = self.CANMessageBuffer.popleft()

            readData = list()
            for i in range(8):
                readData.append(self.CANMessageBuffer.popleft())
            frame_str = f"ID:{canid}, {readData[0]} {readData[1]} {readData[2]} {readData[3]} {readData[4]} {readData[5]} {readData[6]} {readData[7]}"
            self.readDataCAN = readData
            # self.port.flushInput()  # 超重要！

            # rospy.loginfo(frame_str)
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

        if (timeNow - self.timePrevious_CANsend) < (5 * 1000 * 1000):
            return  # return ealry when interval comes within 5ms
        if len(self.canoutTopicList) <= 0:
            return  # return ealry when no canout topic available

        self.canoutSubscriberList[self.canoutSubIndex].send()
        self.canoutSubIndex += 1
        if self.canoutSubIndex >= len(self.canoutTopicList):
            self.canoutSubIndex = 0
        self.timePrevious_CANsend = rospy.Time.now().to_nsec()

    def searchROSTopic(self, search_interval):
        timeNow = rospy.Time.now().to_sec()
        if (timeNow - self.timePrevious_ROSTopicSearch) < (search_interval):
            return  # return ealry when interval comes within 5ms

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

        self.timePrevious_ROSTopicSearch = rospy.Time.now().to_sec()

    def work(self):
        try:
            if self.port.isOpen():
                self.loopcount += 1
                if rospy.Time.now().secs - self.timePrevious_loopcount > 1:
                    rospy.logwarn(f"self.loopcount:{self.loopcount}")
                    self.loopcount = 0
                    self.timePrevious_loopcount = rospy.Time.now().secs

                # start CAN -> ROS processing
                self.searchCANMessage()
                # end of CAN -> ROS process

                # start ROS -> CAN processing
                self.searchROSTopic(2)  # interval with arg Ms
                self.sendDataToCAN()
                # end of ROS -> CAN process

        except (serial.SerialException, OSError) as e:
            rospy.logwarn("catched exception. " + str(e))
            self.retryOpeningPort()


def work():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.init_node("ROSCANBridge_node")
    rate = rospy.Rate(NODE_FREQ)

    global PORT_NAME
    PORT_NAME = rospy.get_param("~port_name")
    instance = ROSCANBridge()

    while not rospy.is_shutdown():
        instance.work()
        rate.sleep()


if __name__ == "__main__":
    try:
        work()
    except rospy.ROSInterruptException:
        rospy.logfatal_once("catched ROSInterruptException")
