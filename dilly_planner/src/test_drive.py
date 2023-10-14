#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Bool, String, Int64MultiArray, Float64MultiArray, Int64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd
from morai_msgs.srv import MoraiEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import pathReader, findLocalPath, purePursuit
from std_msgs.msg import Int64

import tf
import time
from math import *

warnings.simplefilter(action='ignore', category=FutureWarning)

class PurePursuit():
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        self.ctrl_cmd_msg = SkidSteer6wUGVCtrlCmd()
        self.ctrl_cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=1)

        rate = rospy.Rate(15) 
                                           
        while not rospy.is_shutdown():

            ############################ 일반 주행 조향값 및 속도 설정 ##################################

            # int32 cmd_type
            # Type 1 : 전/후/좌/우 입력
            # Type 2 : 6개의 Wheel RPM 직접 입력
            # Type 3 : Target 선속도(m/s) / 각속도(rad/s) 입력
            
            # [Type 1]
            # bool Forward_input
            # bool Backward_input
            # bool Left_Turn_input
            # bool Right_Turn_input

            # [Type 2]
            # float32 left_front_wheel_rpm
            # float32 left_middle_wheel_rpm
            # float32 left_rear_wheel_rpm
            # float32 right_front_wheel_rpm
            # float32 right_middle_wheel_rpm
            # float32 right_rear_wheel_rpm

            # [Type 3]
            # float32 Target_linear_velocity
            # float32 Target_angular_velocity
            
            # self.ctrl_cmd_msg.cmd_type = 1
            # self.ctrl_cmd_msg.Forward_input = True
            # self.ctrl_cmd_msg.Backward_input = False
            # self.ctrl_cmd_msg.Left_Turn_input = False
            # self.ctrl_cmd_msg.Right_Turn_input = False

            # self.ctrl_cmd_msg.cmd_type = 2
            # self.ctrl_cmd_msg.left_front_wheel_rpm = 60.0
            # self.ctrl_cmd_msg.left_middle_wheel_rpm = 60.0
            # self.ctrl_cmd_msg.left_rear_wheel_rpm = 60.0
            # self.ctrl_cmd_msg.right_front_wheel_rpm = 30.0
            # self.ctrl_cmd_msg.right_middle_wheel_rpm = 0.0
            # self.ctrl_cmd_msg.right_rear_wheel_rpm = 0.0

            
        
   
            ########################################################################################################################################################

            self.ctrl_cmd_msg.cmd_type = 3
            
            self.back()
            
            rate.sleep()
            ########################################################################################################################################################

    def back(self) :
        print("후진 중")
        for _ in range(150000):
            self.ctrl_cmd_msg.Target_linear_velocity = -2.0
            self.ctrl_cmd_msg.Target_angular_velocity = 0.0 
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
        print("후진 끝")   
            
        print("도는 중")
        for _ in range(300000):
            self.ctrl_cmd_msg.Target_linear_velocity = 0.0
            self.ctrl_cmd_msg.Target_angular_velocity = 2.0
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
        print('돌기 끝')
        
if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass