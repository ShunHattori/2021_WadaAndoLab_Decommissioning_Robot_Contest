#!/usr/bin/env python3

from logging import Formatter, debug
from typing import DefaultDict
import rospy
import signal
import time
import math
from collections import deque
from rospy.core import rospydebug
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from commissioning_robot.msg import ControlState
from commissioning_robot.msg import FeedbackState
from commissioning_robot.msg import MechanismReport
from commissioning_robot.msg import PhaseReport

import rosgraph
import rostopic


try:
    from jsk_rviz_plugins.msg import *
except:
    import roslib

    roslib.load_manifest("jsk_rviz_plugins")
    from jsk_rviz_plugins.msg import *

NODE_FREQ = 100
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


class RVIZ_JSK:
    def __init__(self) -> None:
        self.robot_vel = Float32()
        self.robot_pose = Float32()
        self.main_height = Float32()
        self.overlayTextOverview = OverlayText()
        self.overlayTextMotorOutputs = OverlayText()
        self.overlayTextLimitSwitch = OverlayText()
        self.overlayTextEncoder = OverlayText()
        self.encoderData = Int32MultiArray()
        self.switchData = UInt16MultiArray()
        self.robotOutline = PolygonStamped()

        self.robot_vel.data = 0
        self.robot_pose.data = 0
        self.main_height.data = 0

        self.IMUPitchBiased = 0
        self.IMURoll = 0
        self.cantopicHz = 0
        self.prevCantopicHz = 0

        self.PhaseArm = 0
        self.PhaseCable = 0

        self.ModeDriveLift = ["", "", "", ""]
        self.ModeMainLift = ""
        self.ModeArm = ["", ""]
        self.ModeCable = ["", ""]

        self.ReferenceDriveLift = [0, 0, 0, 0]
        self.ReferenceMainLift = 0
        self.ReferenceArm = [0, 0]
        self.ReferenceCable = [0, 0]

        self.PWMDriveWheel = [0, 0, 0, 0]
        self.PWMDriveLift = [0, 0, 0, 0]
        self.PWMMainLift = 0
        self.PWMArm = [0, 0]
        self.PWMCable = [0, 0]

        self.EncoderDriveWheel = [0, 0, 0, 0]
        self.EncoderDriveLift = [0, 0, 0, 0]
        self.EncoderMainLift = 0
        self.EncoderArm = [0, 0]
        self.EncoderCable = [0, 0]

        self.SwitchDriveLift = ["□", "□", "□", "□"]
        self.SwitchMainLift = "□"
        self.SwitchArm = ["□", "□"]
        self.SwitchCable = ["□", "□"]
        self.switchFrontRight = "□"
        self.switchFrontLeft = "□"

        self.MacroArmPhase = 0
        self.MacroArmDescrption = ""
        self.MacroCablePhase = 0
        self.MacroCableDescrption = ""

        self.robotOutline.header.seq = 1
        self.robotOutline.header.frame_id = "base_link"
        self.robotOutline.header.stamp = rospy.Time.now()
        robot_x = 785.0 / 2 / 1000
        robot_y = 1050.0 / 2 / 1000
        self.robotOutline.polygon.points = [Point32(x=-robot_x, y=-robot_y, z=0), Point32(x=-robot_x, y=robot_y, z=0), Point32(x=robot_x, y=robot_y, z=0), Point32(x=robot_x, y=-robot_y, z=0)]
        self.publisherRobotOutline = rospy.Publisher("/jsk_p2d_robot_outline", PolygonStamped, queue_size=5)

        self.subscriberRobotTwist = rospy.Subscriber("/drive_inverse_kinematics_Twist", Twist, self.callbackRobotTwist, queue_size=5)
        self.subscriberEncoderPulses = rospy.Subscriber("/RHWS/encoder_pulses", Int32MultiArray, self.callbackEncoderPulses, queue_size=5)
        self.subscriberSwitchStates = rospy.Subscriber("/RHWS/switch_states", UInt16MultiArray, self.callbackSwitchStates, queue_size=5)
        self.subscriberIMUPitchBiased = rospy.Subscriber("/RHWS/IMU_pitch_biased", Float32, self.callbackIMUPitchBiased, queue_size=5)
        self.subscriberIMURoll = rospy.Subscriber("/RHWS/IMU_roll", Float32, self.callbackIMURoll, queue_size=5)

        self.subscriberReportDriveWheelController = rospy.Subscriber("/MechanismReport/drive_wheel_controller", MechanismReport, self.callbackReportDriveWheelController, queue_size=5)
        self.subscriberReportDriveLiftController = rospy.Subscriber("/MechanismReport/drive_lift_controller", MechanismReport, self.callbackReportDriveLiftController, queue_size=5)
        self.subscriberReportMainLiftController = rospy.Subscriber("/MechanismReport/main_lift_controller", MechanismReport, self.callbackReportMainLiftController, queue_size=5)
        self.subscriberReportArmController = rospy.Subscriber("/MechanismReport/arm_controller", MechanismReport, self.callbackReportArmController, queue_size=5)
        self.subscriberReportCableController = rospy.Subscriber("/MechanismReport/cable_manager_controller", MechanismReport, self.callbackReportCableController, queue_size=5)

        self.subscriberPhaseReportArmController = rospy.Subscriber("/PhaseReport/arm_controller", PhaseReport, self.callbackPhaseReportArmController, queue_size=5)
        self.subscriberPhaseReportCableController = rospy.Subscriber("/PhaseReport/cable_manager_controller", PhaseReport, self.callbackPhaseReportCableController, queue_size=5)

        self.subscriber_wheel_motor_pwms = rospy.Subscriber("/wheel_motor_pwms", Int16MultiArray, self.callback_wheel_motor_pwms, queue_size=5)
        self.subscriber_main_lift_motor_pwms = rospy.Subscriber("/main_lift_motor_pwms", Int16MultiArray, self.callback_main_lift_motor_pwms, queue_size=5)
        self.subscriber_drive_lift_motor_pwms = rospy.Subscriber("/drive_lift_motor_pwms", Int16MultiArray, self.callback_drive_lift_motor_pwms, queue_size=5)
        self.subscriber_arm_motor_pwms = rospy.Subscriber("/arm_motor_pwms", Int16MultiArray, self.callback_arm_motor_pwms, queue_size=5)
        self.subscriber_cable_motor_pwms = rospy.Subscriber("/cable_motor_pwms", Int16MultiArray, self.callback_cable_motor_pwms, queue_size=5)

        self.publisherRobotVel = rospy.Publisher("/jsk_p2d_robot_vel", Float32, queue_size=5)
        self.publisherRobotPose = rospy.Publisher("/jsk_p2d_robot_pose", Float32, queue_size=5)
        self.publisherMainHeight = rospy.Publisher("/jsk_p2d_main_height", Float32, queue_size=5)
        self.publisherOverlayTextOverview = rospy.Publisher("/jsk_Overlay_text_Overview", OverlayText, queue_size=5)
        self.publisherOverlayTextMotorOutputs = rospy.Publisher("/jsk_Overlay_text_MotorOutputs", OverlayText, queue_size=5)
        self.publisherOverlayTextEncoder = rospy.Publisher("/jsk_Overlay_text_Encoder", OverlayText, queue_size=5)
        self.publisherOverlayTextLimitSwitch = rospy.Publisher("/jsk_Overlay_text_LimitSwitch", OverlayText, queue_size=5)

        self.rostopicHzMonitor = rostopic.ROSTopicHz(-1)
        self.rostopicHzSubscriber = rospy.Subscriber("/canin33", rospy.AnyMsg, self.rostopicHzMonitor.callback_hz)  # IMU message

        pass

    def callbackReportDriveWheelController(self, _data: MechanismReport) -> None:
        for i in range(4):
            self.EncoderDriveWheel[i] = _data.state_pulse[i]
        pass

    def callbackReportDriveLiftController(self, _data: MechanismReport) -> None:
        for i in range(4):
            if _data.running_mode[i] == 1:
                self.ModeDriveLift[i] = "FOLLOW"
            elif _data.running_mode[i] == 4:
                self.ModeDriveLift[i] = "ORIGIN"
            else:
                self.ModeDriveLift[i] = "STOP"
            self.ReferenceDriveLift[i] = _data.reference[i]
            self.EncoderDriveLift[i] = _data.state_pulse[i]
            self.SwitchDriveLift[i] = "■" if _data.state_limit[i] else "□"
        pass

    def callbackReportMainLiftController(self, _data: MechanismReport) -> None:
        if _data.running_mode[0] == 1:
            self.ModeMainLift = "FOLLOW"
        elif _data.running_mode[0] == 4:
            self.ModeMainLift = "ORIGIN"
        else:
            self.ModeMainLift = "STOP"
        self.ReferenceMainLift = _data.reference[0]
        self.EncoderMainLift = _data.state_pulse[0]
        self.SwitchMainLift = "■" if _data.state_limit[0] else "□"
        pass

    def callbackReportArmController(self, _data: MechanismReport) -> None:
        for i in range(2):
            if _data.running_mode[i] == 1:
                self.ModeArm[i] = "FOLLOW"
            elif _data.running_mode[i] == 2:
                self.ModeArm[i] = "FAST FOLLOW"
            elif _data.running_mode[i] == 4:
                self.ModeArm[i] = "ORIGIN"
            else:
                self.ModeArm[i] = "STOP"
            self.ReferenceArm[i] = _data.reference[i]
            self.EncoderArm[i] = _data.state_pulse[i]
            self.SwitchArm[i] = "■" if _data.state_limit[i] else "□"
        pass

    def callbackReportCableController(self, _data: MechanismReport) -> None:
        for i in range(2):
            if _data.running_mode[i] == 1:
                self.ModeCable[i] = "FOLLOW"
            elif _data.running_mode[i] == 2:
                self.ModeArm[i] = "FAST FOLLOW"
            elif _data.running_mode[i] == 4:
                self.ModeCable[i] = "ORIGIN"
            else:
                self.ModeCable[i] = "STOP"
            self.ReferenceCable[i] = _data.reference[i]
            self.EncoderCable[i] = _data.state_pulse[i]
            self.SwitchCable[i] = "■" if _data.state_limit[i] else "□"
        pass

    def callbackPhaseReportArmController(self, _data: PhaseReport) -> None:
        self.MacroArmPhase = _data.phase
        self.MacroArmDescrption = _data.description
        pass

    def callbackPhaseReportCableController(self, _data: PhaseReport) -> None:
        self.MacroCablePhase = _data.phase
        self.MacroCableDescrption = _data.description
        pass

    def callback_wheel_motor_pwms(self, _data: Int16MultiArray) -> None:
        for i in range(4):
            self.PWMDriveWheel[i] = _data.data[i]
        pass

    def callback_main_lift_motor_pwms(self, _data: Int16MultiArray) -> None:
        self.PWMMainLift = _data.data[0]
        pass

    def callback_drive_lift_motor_pwms(self, _data: Int16MultiArray) -> None:
        for i in range(4):
            self.PWMDriveLift[i] = _data.data[i]
        pass

    def callback_arm_motor_pwms(self, _data: Int16MultiArray) -> None:
        for i in range(2):
            self.PWMArm[i] = _data.data[i]
        pass

    def callback_cable_motor_pwms(self, _data: Int16MultiArray) -> None:
        for i in range(2):
            self.PWMCable[i] = _data.data[i]
        pass

    def callbackRobotTwist(self, _data: Twist) -> None:
        self.robot_vel.data = math.sqrt((_data.linear.x * _data.linear.x) + (_data.linear.y * _data.linear.y))
        pass

    def callbackEncoderPulses(self, _data: Int32MultiArray) -> None:
        self.encoderData = _data
        pass

    def callbackSwitchStates(self, _data: UInt16MultiArray) -> None:
        self.switchFrontRight = "■" if _data.data[13] else "□"
        self.switchFrontLeft = "■" if _data.data[12] else "□"
        # self.SwitchDriveLift[0] = "■" if _data.data[0] else "□"
        # self.SwitchDriveLift[1] = "■" if _data.data[2] else "□"
        # self.SwitchDriveLift[2] = "■" if _data.data[5] else "□"
        # self.SwitchDriveLift[3] = "■" if _data.data[7] else "□"
        # self.SwitchMainLift = "■" if _data.data[14] else "□"
        # self.SwitchArm[0] = "■" if _data.data[18] else "□"
        # self.SwitchArm[1] = "■" if _data.data[19] else "□"
        # self.SwitchCable[0] = "■" if _data.data[11] else "□"
        # self.SwitchCable[1] = "■" if _data.data[9] else "□"
        pass

    def callbackIMUPitchBiased(self, _data: Float32) -> None:
        self.IMUPitchBiased = _data.data
        pass

    def callbackIMURoll(self, _data: Float32) -> None:
        self.IMURoll = _data.data
        pass

    def getCantopicHz(self) -> float:
        cantopicHzTuple = self.rostopicHzMonitor.get_hz()
        self.cantopicHz = cantopicHzTuple[0] if cantopicHzTuple != None else self.prevCantopicHz
        self.prevCantopicHz = self.cantopicHz if self.cantopicHz != 0 else self.prevCantopicHz
        return self.cantopicHz
        pass

    # https://note.nkmk.me/python-f-strings/
    def createOverlayTextOverview(self) -> None:
        self.overlayTextOverview.action = OverlayText.ADD
        self.overlayTextOverview.font = "ubuntu"
        self.overlayTextOverview.text = f"""Commissioning Robot Wada Ando Lab
        DriveLift ___: {self.ModeDriveLift[0]}
        _____________: [{int(self.ReferenceDriveLift[0])}, {int(self.ReferenceDriveLift[1])}, {int(self.ReferenceDriveLift[2])}, {int(self.ReferenceDriveLift[3])}]
        _____________: ({self.EncoderDriveLift[0]}, {self.EncoderDriveLift[1]}, {self.EncoderDriveLift[2]}, {self.EncoderDriveLift[3]})
        MainLift ____: {self.ModeMainLift} [{int(self.ReferenceMainLift)} -> ({self.EncoderMainLift})]
        Arm X _______: {self.ModeArm[0]} [{int(self.ReferenceArm[0])} -> ({self.EncoderArm[0]})]
        Arm Y _______: {self.ModeArm[1]} [{int(self.ReferenceArm[1])} -> ({self.EncoderArm[1]})]
        CableLift ___: {self.ModeCable[0]} [{int(self.ReferenceCable[0])} -> ({self.EncoderCable[0]})]
        CableBelt ___: {self.ModeCable[1]} [{int(self.ReferenceCable[1])} -> ({self.EncoderCable[1]})]
        CAN Rate ____: {self.getCantopicHz():.1f}
        Macro Arm ___: [{self.MacroArmPhase} : {self.MacroArmDescrption}] 
        Macro Cable _: [{self.MacroCablePhase} : {self.MacroCableDescrption}] 
        """
        pass

    def createOverlayTextMotorOutputs(self) -> None:
        self.overlayTextMotorOutputs.action = OverlayText.ADD
        self.overlayTextMotorOutputs.font = "ubuntu"
        self.overlayTextMotorOutputs.text = f"""MotorDriver Outputs
        DriveWheel : {self.PWMDriveWheel[0]}, {self.PWMDriveWheel[1]}, {self.PWMDriveWheel[2]}, {self.PWMDriveWheel[3]}
        DriveLift _: {self.PWMDriveLift[0]}, {self.PWMDriveLift[1]}, {self.PWMDriveLift[2]}, {self.PWMDriveLift[3]}
        MainLift __: {self.PWMMainLift}
        Arm XY ____: {self.PWMArm[0]}, {self.PWMArm[1]}
        Cable _____: {self.PWMCable[0]}, {self.PWMCable[1]}
        """
        pass

    def createOverlayTextEncoder(self) -> None:
        self.overlayTextEncoder.action = OverlayText.ADD
        self.overlayTextEncoder.font = "ubuntu"
        self.overlayTextEncoder.text = f"""Encoder Pulses
        DriveWheel : {self.EncoderDriveWheel[0]}, {self.EncoderDriveWheel[1]}, {self.EncoderDriveWheel[2]}, {self.EncoderDriveWheel[3]}
        DriveLift _: {self.EncoderDriveLift[0]}, {self.EncoderDriveLift[1]}, {self.EncoderDriveLift[2]}, {self.EncoderDriveLift[3]}
        MainLift __: {self.EncoderMainLift}
        Arm XY ____: {self.EncoderArm[0]}, {self.EncoderArm[1]}
        Cable _____: {self.EncoderCable[0]}, {self.EncoderCable[1]}
        """
        pass

    def createOverlayTextLimitSwitch(self) -> None:
        self.overlayTextLimitSwitch.action = OverlayText.ADD
        self.overlayTextLimitSwitch.font = "ubuntu"
        self.overlayTextLimitSwitch.text = f"""LimitSwtich States
        FrontBar ____: [{self.switchFrontLeft}]  -  [{self.switchFrontRight}]
        DriveLift ___: [{self.SwitchDriveLift[0]}] - [{self.SwitchDriveLift[1]}] - [{self.SwitchDriveLift[2]}] - [{self.SwitchDriveLift[3]}]
        MainLift ____: [{self.SwitchMainLift}]
        Arm XY ______: [{self.SwitchArm[0]}] - [{self.SwitchArm[1]}]
        Cable _______: [{self.SwitchCable[0]}] - [{self.SwitchCable[1]}]
        """
        pass

    def work(self) -> None:
        self.createOverlayTextOverview()
        self.createOverlayTextMotorOutputs()
        self.createOverlayTextEncoder()
        self.createOverlayTextLimitSwitch()
        self.publishTopic()
        pass

    def publishTopic(self) -> None:
        self.publisherRobotVel.publish(self.robot_vel)
        self.publisherRobotPose.publish(self.robot_pose)
        if len(self.encoderData.data) != 0:
            self.main_height = self.encoderData.data[14] / 5384.0 * 4
        self.publisherMainHeight.publish(self.main_height)
        self.publisherOverlayTextOverview.publish(self.overlayTextOverview)
        self.publisherOverlayTextMotorOutputs.publish(self.overlayTextMotorOutputs)
        self.publisherOverlayTextEncoder.publish(self.overlayTextEncoder)
        self.publisherOverlayTextLimitSwitch.publish(self.overlayTextLimitSwitch)
        self.publisherRobotOutline.publish(self.robotOutline)
        pass


def work():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    rospy.init_node("rviz_jsk")
    rate = rospy.Rate(NODE_FREQ)

    # global PORT_NAME
    # PORT_NAME = rospy.get_param("~port_name")
    instance = RVIZ_JSK()

    while not rospy.is_shutdown():
        instance.work()
        rate.sleep()


if __name__ == "__main__":
    try:
        work()
    except rospy.ROSInterruptException:
        rospy.logfatal_once("catched ROSInterruptException")
