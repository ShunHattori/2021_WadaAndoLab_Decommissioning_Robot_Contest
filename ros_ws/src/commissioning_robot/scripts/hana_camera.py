#!/usr/bin/env python3

from logging import Formatter, debug
from typing import DefaultDict
import rospy
import signal
from collections import deque
from rospy.core import rospydebug, rospyinfo
from sensor_msgs.msg import Image
from socket import *
import rosgraph
import rostopic
import numpy as np
import cv2
import time
import math
import sys

HOST = ""
PORT = 7777
ADDRESS = "192.168.11.6"
udp = socket(AF_INET, SOCK_DGRAM)
# udp.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

NODE_FREQ = 20
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


class Camera_streaming:
    def __init__(self) -> None:
        self.inputImage = []
        self.exportImage = []
        # self.subscribe_image_back = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_image_back, queue_size=5)
        self.subscribe_image_back = rospy.Subscriber("/image_back", Image, self.callback_image_back, queue_size=5)
        pass

    def callback_image_back(self, _data: Image) -> None:
        rospy.loginfo("callback called")
        if len(_data.data) != _data.height * _data.width * 3:
            rospy.logfatal(len(_data.data))
            rospy.logfatal(_data.height)
            rospy.logfatal(_data.width)
            return

        inputImage = np.zeros(_data.height * _data.width * 3, np.uint8)
        inputImage = _data.data
        rospy.loginfo(f"type(inputImage):{type(inputImage)}")
        rospy.loginfo(f"len(inputImage):{len(inputImage)}")
        image_ndarray = np.frombuffer(inputImage, dtype=np.uint8)
        rospy.loginfo(f"type(image_ndarray):{type(image_ndarray)}")
        rospy.loginfo(f"image_ndarray.size:{image_ndarray.size}")
        self.exportImage = image_ndarray.reshape(_data.height, _data.width, 3)

        if len(self.exportImage) == 0:
            pass
        jpg_str = cv2.imencode(".jpeg", np.float32(self.exportImage), [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        i = jpg_str[1]
        aa = udp.sendto(i.tobytes(), (ADDRESS, PORT))
        rospy.loginfo(f"socket_send:{aa}")
        pass

    def socket_send(self) -> None:

        pass

    def work(self) -> None:
        self.socket_send()
        pass


def work():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.init_node("Camera_streaming_node")
    rate = rospy.Rate(NODE_FREQ)

    # global PORT_NAME
    # PORT_NAME = rospy.get_param("~port_name")
    instance = Camera_streaming()

    while not rospy.is_shutdown():
        instance.work()
        rate.sleep()


if __name__ == "__main__":
    try:
        work()
    except rospy.ROSInterruptException:
        rospy.logfatal_once("catched ROSInterruptException")
