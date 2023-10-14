#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os,cv2
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Bool, String, Int64MultiArray, Float64MultiArray, Int64
from sensor_msgs.msg import Imu, CompressedImage, Image
from geometry_msgs.msg import Point, PoseStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import SkidSteer6wUGVCtrlCmd, GPSMessage, DillyCmd, DillyCmdResponse, WoowaDillyStatus
from morai_msgs.srv import WoowaDillyEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import pathReader,findLocalPath, purePursuit, latticePlanner, rotLidar2Gps
from std_msgs.msg import Int64
from cv_bridge import CvBridge

import tf
import time
from math import *

warnings.simplefilter(action='ignore', category=FutureWarning)

ONE = 1
TWO = 2
THREE = 3
FOUR = 4
FIVE = 5

INDOOR = 'INDOOR'
OUTDOOR = 'OUTDOOR'

INDOOR_PORTAL = [3.009380386036355, -42.06917754141614]
INDOOR_INIT = [8.2829999923706055,-38.854999542236328]
INDOOR_YAW = -87.954

OUTDOOR_PORTAL = [429.97326664760476, -139.99766722600907]
OUTDOOR_INIT = [434.99951176560717, -140.03]
OUTDOOR_YAW = 91.15208480877877

PICKUP = True
DROPOFF = False

DEFAULT_LFD = 3


# pedestrain List
pedestrainList = [[0,0,0],
                  [393.93301391601563,-79.504997253417969,0.2720000147819519], 
                  [263.42001342773438,-79.168998718261719,0.3449999988079071],
                  [229.86000061035156,-129.15899658203125,0.20000000298023224],
                  [334.93099975585938,-117.61399841308594,0.20000000298023224],
                  [372.14498901367188,-116.39600372314453,0.20000000298023224]]
# Pick up place List
pickupList = [[0,0,0],
              [-42.400001525878906,-135.03999328613281,0.0],
              [51.549999237060547,-41.340000152587891,0.0],
              [65.260002136230469,44.090000152587891,0.0],
              [-77.69000244140625,7.690000057220459,0.0],
              [30.715000152587891,-41.173000335693359,0.0]]



class dilly_status:
    def __init__(self):
        self.position = Vector3()
        self.heading = 0.0
        self.velocity = Vector3()


class DillyPlanner():
    def __init__(self):
        rospy.init_node('dilly_planner', anonymous=True)
        
        # Lattice Planner Parameters
        lattice_path_length = 5
        lattice_current_lane = 2

        # Read path
        self.path_reader = pathReader('path_maker') ## 경로 파일의 위치
        self.path_name = "indoor_portal_5"
        # self.goal = int(self.path_name[-1])

        # Publisher
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        self.ctrl_cmd_pub = rospy.Publisher('/6wheel_skid_ctrl_cmd', SkidSteer6wUGVCtrlCmd, queue_size=1)
        self.object_converted_pub = rospy.Publisher('/object_converted', MarkerArray, queue_size=1)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        ########################  lattice  ########################
        for i in range(1,lattice_path_length+1):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################
        

        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/imu", Imu, self.imuCB) ## Vehicle Status Subscriber
        rospy.Subscriber("/marker", MarkerArray, self.ObjectCB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.cameraCB)
        rospy.Subscriber("/WoowaDillyStatus", WoowaDillyStatus, self.dillyStatusCB)

        # def
        self.is_status = False
        self.is_gps = False
        self.is_imu = False
        self.is_pickup = [False, False, False, False, False, False]
        self.is_dropoff = [False, False, False, False, False, False]

        # Class
        self.status_msg = dilly_status()
        self.pure_pursuit = purePursuit() ## purePursuit import

        self.global_path = self.path_reader.read_txt(self.path_name + '.txt')

        self.steering_angle_to_servo_offset = 0.0 ## servo moter offset
        self.target_x = 0.0
        self.target_y = 0.0

        # Msg
        self.object_markerArray = MarkerArray()
        self.ctrl_cmd_msg = SkidSteer6wUGVCtrlCmd()

        self.motor_msg = 0.0
        self.servo_msg = 0.0

        self.object_info = []

        self.current_waypoint = 1

        # Camera
        self.tf_broadcasteridge = CvBridge()
        self.image1 = None
        self.img_cnt = 0
        self.img_similarity = 0

        # GPS
        self.location = None
        self.change_location = False
        self.vehicle_status = [0, 0]
        self.previous_gps = [0, 0]
        self.gps_cnt = 0
        self.gps_similarity = 0

        # Service
        rospy.wait_for_service('/WoowaDillyEventCmd')
        self.dilly_client = rospy.ServiceProxy('/WoowaDillyEventCmd', WoowaDillyEventCmdSrv)
        self.dilly_request = DillyCmd()

        # Delivery Status
        self.deliveryItem = []
        self.indoor_success_cnt = 0
        self.outdoor_success_cnt = 0
        self.indoor_priority = [5,4,1,2,3,0]
        self.outdoor_priority = [5,4,1,2,3,0]
        self.indoor_goal = self.indoor_priority[self.indoor_success_cnt]
        self.outdoor_goal = self.outdoor_priority[self.outdoor_success_cnt]

        self.starting_point = None
        self.end_point = None
        self.turn_flag = 1
        self.euler_data = [0,0,0,0]

        # tf
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # time var
        rate = rospy.Rate(15) 
                  
        while not rospy.is_shutdown():
            
            
            if self.is_status == True and self.is_imu == True and self.is_gps == True:
                # count
                self.img_cnt += 1
                self.gps_cnt += 1

                
                self.getDillyStatus() 

                if self.status_msg.position.x < 150:
                    if self.location == OUTDOOR :
                        self.change_location = True
                    else :
                        self.change_location = False
                    self.location = INDOOR
                elif self.status_msg.position.x >= 150:
                    if self.location == INDOOR :
                        self.change_location = True
                    else :
                        self.change_location = False
                    self.location = OUTDOOR


                self.ctrl_cmd_msg.cmd_type = 3
                self.motor_msg = 2.0

                if self.img_similarity >= 0.98 and INDOOR:
                    
                    self.emergencyEscape()
                    
                    self.img_similarity = 0
                    continue
                
                elif self.gps_similarity < 0.2 and OUTDOOR:
                    self.emergencyEscape()
                    
                    self.gps_similarity = 999
                    continue
                
                
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                local_path, self.current_waypoint = findLocalPath(self.global_path, self.status_msg)
                
                # print("CURRENT: ", self.current_waypoint)

                self.vehicle_status=[self.status_msg.position.x, self.status_msg.position.y, (self.status_msg.heading)/180*pi, self.motor_msg]
                # print(self.vehicle_status[0], self.vehicle_status[1], (self.status_msg.heading))
                
                self.global_valid_object = rotLidar2Gps(self.object_info, self.status_msg.position.x, self.status_msg.position.y, self.vehicle_status, self.location)
                # self.object_converted_visualize(self.global_valid_object)

                self.starting_point = self.path_name.split('_')[1]
                self.end_point = self.path_name.split('_')[2]
                

               

                ################################################  lattice  ################################################
                lattice_path, selected_lane, avoid_lane = latticePlanner(local_path, self.global_valid_object, self.vehicle_status, lattice_current_lane)
                
                ################ lattice 무시 ################
                if self.end_point == 'portal': 
                    selected_lane = 2
                    
                    if ((-124.61 < self.vehicle_status[0] <-1.96 and -16.04 < self.vehicle_status[1] < 36.38) or  # 4번
                        (-22.44 < self.vehicle_status[0] < -1.9 and -113.79 < self.vehicle_status[1] < -72.58)) : # 1번 
                        selected_lane = avoid_lane
                        
                    
                if self.getDistance(self.vehicle_status, pedestrainList[self.outdoor_goal]) <= 3.0 :
                    selected_lane = 2
                    self.motor_msg = 1.0
                    
              
                if ((391.82 < self.vehicle_status[0] < 400.26 and -117.93 < self.vehicle_status[1] < -108.3) and
                    (self.outdoor_goal == 1 and self.location == OUTDOOR)) :
                    selected_lane = 2
                    self.motor_msg = 1.0
                    
                ################ lattice 무시 ################

                lattice_current_lane = selected_lane
        
                if selected_lane != -1: 
                    local_path = lattice_path[selected_lane]  
                
             
                
                # if len(lattice_path)==lattice_path_length:                    
                #     for i in range(1,lattice_path_length+1):
                #         globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ################################################  lattice  ################################################

               
        
                ########################################### COLLISION RESPAWN ###########################################
                if self.location == INDOOR :
                    
                    # if (abs(INDOOR_YAW-self.status_msg.heading) <= 5.0 and self.getDistance(self.vehicle_status, INDOOR_INIT) <= 3.0):
                    if (self.getDistance(self.vehicle_status, INDOOR_INIT) <= 3.0):
                        if len(self.deliveryItem) == 0:
                            self.changePath('indoor_respawn_' + str(self.indoor_goal))
                        
                        if len(self.deliveryItem) == 1 : # 5 -> 4 갈 때, 1 -> portal 갈 때 충돌하면
                            if self.deliveryItem[0] == 5 :
                                self.changePath('indoor_respawn_4')
                            elif self.deliveryItem[0] <= 3 :
                                self.changePath('indoor_respawn_portal')

                        if len(self.deliveryItem) == 2 : # 4 -> portal 갈 때
                            self.changePath('indoor_respawn_portal')

                        
                if self.location == OUTDOOR :
                    
                    if (self.getDistance(self.vehicle_status, OUTDOOR_INIT) <= 2.0):
                        if len(self.deliveryItem) == 1 :
                            self.changePath('outdoor_portal_' + str(self.deliveryItem[0])) 
                        elif len(self.deliveryItem) == 0 :
                            if self.path_name != 'outdoor_respawn_portal' :
                                self.turnLeft(100000)
                                self.changePath('outdoor_respawn_portal')
                            self.changePath('outdoor_respawn_portal')
                        
                        
                ########################################### COLLISION RESPAWN ###########################################

        
            
            
                self.pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                self.pure_pursuit.getEgoStatus(self.status_msg, self.motor_msg) 

                self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(DEFAULT_LFD)

                if self.location == INDOOR:
                    self.servo_msg = self.steering / 30 # 15
                elif self.location == OUTDOOR:
                    self.servo_msg = self.steering / 30 

                # self.local_path_pub.publish(local_path) ## Local Path 출력
                # self.global_path_pub.publish(self.global_path)
                # self.object_converted_pub.publish(self.object_markerArray)

                distance = 10000
                if self.location == INDOOR :

                    if self.getDistance(self.vehicle_status, INDOOR_PORTAL) <= 5.0 :
                        self.motor_msg = 1.0
                        
                    
                    if len(self.deliveryItem) == 0 :
                        self.changePath('indoor_portal_' + str(self.indoor_goal))
                    
                    distance = self.getDistance(self.vehicle_status, pickupList[self.indoor_goal])
                    
        
                    
                    self.stopForPickUp(distance, self.indoor_goal)
                        

                elif self.location == OUTDOOR: 
                    
                    if self.getDistance(self.vehicle_status, OUTDOOR_PORTAL) <= 5.0 :
                        self.motor_msg = 1.0
                        
                    
                    distance = self.getDistance(self.vehicle_status, pedestrainList[self.outdoor_goal])
                    if distance < 5.0 :
                        self.motor_msg = 1.0
                        
                    ### 배달 우선 순위 만큼 self.goal 변경
                    if len(self.deliveryItem) > 0 and self.change_location == True:
                        self.outdoor_goal = self.outdoor_priority[self.outdoor_success_cnt]
                        self.changePath('outdoor_portal_' + str(self.outdoor_goal))

                        # for i in range(5) :
                        #     if self.outdoor_priority[i] in self.deliveryItem :
                        #         self.outdoor_goal = self.outdoor_priority[i]
                                # break
                    self.stopForDropOff(distance, self.outdoor_goal)

                    # self.stopForPickUp(distance, self.goal)
              
                print(self.path_name)
                
                
                self.publishCtrlCmd(self.motor_msg, self.servo_msg)
                rate.sleep()
            # else:
            #     print("Waiting")

###################################################################### Call Back ######################################################################

    def getDillyStatus(self): ## Vehicle Status Subscriber 
        self.status_msg.position.x = self.xy_zone[0] - 334212.29
        self.status_msg.position.y = self.xy_zone[1] - 4143082.44
        self.status_msg.heading = self.euler_data[2] * 180 / pi
        self.status_msg.velocity.x = 2.0 #self.velocity
        
        
        self.tf_broadcaster.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "base_link",
                        "map")
        
        self.is_status=True


    def gpsCB(self, data):
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)
        
        self.tf_broadcaster.sendTransform((3.681446969494573e-08, -8.688178354532283e-08, 0.8067518472671509),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "gps",
                        "base_link")

        # 유사도 검사
        if self.gps_cnt == 1:
            self.previous_gps = [self.vehicle_status[0], self.vehicle_status[1]]
        if self.gps_cnt % 150 == 0 and self.location == OUTDOOR :
            
            self.gps_similarity = sqrt((self.previous_gps[0] - self.vehicle_status[0])**2 + (self.previous_gps[1] - self.vehicle_status[1])**2)
                        
            self.gps_cnt = 0

        self.is_gps = True
        self.is_status = True
        
        
        
    def cameraCB(self, data) :
        #유사도 검사
        if self.img_cnt == 1 :
            self.image1 = self.tf_broadcasteridge.compressed_imgmsg_to_cv2(data) 
        if self.img_cnt % 150 == 0 and self.location == INDOOR :
            image2 = self.tf_broadcasteridge.compressed_imgmsg_to_cv2(data)    

            # 이미지를 그레이스케일로 변환
            image1_gray = cv2.cvtColor(self.image1, cv2.COLOR_BGR2GRAY)
            image2_gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

            # 히스토그램 계산
            hist1 = cv2.calcHist([image1_gray], [0], None, [256], [0, 256])
            hist2 = cv2.calcHist([image2_gray], [0], None, [256], [0, 256])

            # 히스토그램 비교
            self.img_similarity = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
          
            self.img_cnt = 0

        self.tf_broadcaster.sendTransform((0.3570001721382141, -2.7135014533996582e-05, 0.5981801748275757),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "Camera",
                        "base_link")
        
        
        # return similarity


    def imuCB(self, data):
        self.euler_data = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))

        self.tf_broadcaster.sendTransform((0.08018457889556885, 2.076849341392517e-05, 0.75),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "imu",
                        "base_link")

        self.is_imu = True
        
        
    def ObjectCB(self, data):
        self.object_info = []

        for marker in data.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.object_info.append([x,y])

    
    def dillyStatusCB(self, data):
        self.deliveryItem = data.deliveryItem
        
 
###################################################################### Function ######################################################################

    def publishCtrlCmd(self, motor_msg, servo_msg):
        self.ctrl_cmd_msg.Target_linear_velocity = motor_msg
        self.ctrl_cmd_msg.Target_angular_velocity = servo_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)


    def object_converted_visualize(self, object_list):
        self.object_markerArray.markers = []
        
        if len(object_list) > 0:
            for i in range(len(object_list)):
                object_marker = Marker()
                object_marker.header.frame_id = 'map'
                object_marker.type = Marker.CYLINDER
                object_marker.action = Marker.ADD
                object_marker.id = i
                object_marker.scale.x = 1.0
                object_marker.scale.y = 1.0
                object_marker.scale.z = 0.0
                object_marker.color.a = 1
                object_marker.color.r = 0
                object_marker.color.g = 0
                object_marker.color.b = 1
                object_marker.pose.orientation.w = 1.0
                object_marker.pose.position.x = object_list[i][0]
                object_marker.pose.position.y = object_list[i][1]
                object_marker.pose.position.z = 0
                
                self.object_markerArray.markers.append(object_marker)
        

    def pickUpRequest(self, isPickup, goal):
        self.dilly_request.isPickup = isPickup
        self.dilly_request.deliveryItemIndex = goal

        response = self.dilly_client(self.dilly_request)

        
        return response.response.result


    def dropOffRequest(self, isPickup, goal):
        self.dilly_request.isPickup = isPickup
        self.dilly_request.deliveryItemIndex = goal

        response = self.dilly_client(self.dilly_request)

        return response.response.result


    def stopForPickUp(self, distance, goal):
        if ((0 < distance < 1.3 and self.is_pickup[goal] == False and self.indoor_goal != 5) or
            (0 < distance < 0.7 and self.is_pickup[goal] == False and self.indoor_goal == 5)) :
            self.brake(30000)
         
            if(self.pickUpRequest(PICKUP, goal) == True):
                print("######### Delivery Pickup #########")
                self.is_pickup[goal] = True
                
                self.indoor_success_cnt += 1
                self.indoor_goal = self.indoor_priority[self.indoor_success_cnt]

                self.emergencyEscape()

                if len(self.deliveryItem) == 2 :
                    print('indoor_' + str(self.end_point) + '_portal')
                    self.changePath('indoor_' + str(self.end_point) + '_portal')
                elif len(self.deliveryItem) == 1 :
                    print('indoor_' + str(self.end_point) + '_' + str(self.indoor_goal))
                    if self.indoor_goal == 2 :
                        self.changePath('indoor_1_portal')
                    
                    elif self.indoor_goal == 3:
                        self.changePath('indoor_2_portal')
                        
                    elif self.indoor_goal == 0:
                        self.changePath('indoor_3_portal')
                    
                    else :
                        self.changePath('indoor_' + str(self.end_point) + '_' + str(self.indoor_goal))

            
    

    def stopForDropOff(self, distance, goal):
        if ((0 < distance < 1.3 and self.is_dropoff[goal] == False and self.outdoor_goal != 1) or
            (0 < distance < 1.4 and self.is_dropoff[goal] == False and self.outdoor_goal == 1)) :
            
            self.brake(30000)
    
            if(self.dropOffRequest(DROPOFF, goal) == True):
                self.is_dropoff[goal] = True
                print('######### Delivery Dropoff #########')

                self.outdoor_success_cnt += 1
                self.outdoor_goal = self.outdoor_priority[self.outdoor_success_cnt]

                rospy.sleep(1)
                
                if len(self.deliveryItem) == 0 :
                    self.changePath('outdoor_' + str(self.end_point) + '_portal')
                elif len(self.deliveryItem) > 0 :
                    self.changePath('outdoor_' + str(self.end_point) + '_' + str(self.outdoor_goal))
                    
    
    # 약 time 290000 == 180도
    
    def brake(self, time):
        for _ in range(time):
            self.ctrl_cmd_msg.Target_linear_velocity = 0
            self.ctrl_cmd_msg.Target_angular_velocity = 0
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    
                    
    def changePath(self, new_path):
        self.path_name = new_path
        self.global_path = self.path_reader.read_txt(self.path_name + '.txt')
    

    ### 잘못 실내, 실외 이동하였을 경우
    def wrongChange(self) :
        if self.location == INDOOR and self.change_location == True:
            # self.path_name = 
            self.global_path = self.path_reader.read_txt(self.path_name + '.txt')
        elif self.location == OUTDOOR and self.change_location == True :
            # self.path_name = 
            self.global_path = self.path_reader.read_txt(self.path_name + '.txt')

    
    def getDistance(self, a, b):
        return sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    
    
    def rear(self, time) :
        for _ in range(time) :
            self.ctrl_cmd_msg.Target_linear_velocity = -2.0
            self.ctrl_cmd_msg.Target_angular_velocity = 0.0 
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
        
        
    def turnLeft(self, time):
        self.brake(30000)
  
        for _ in range(time):
            self.ctrl_cmd_msg.Target_linear_velocity = 0.0
            self.ctrl_cmd_msg.Target_angular_velocity = -2.0 
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            
        self.brake(30000)

    def emergencyEscape(self) :
        self.brake(30000)
        self.rear(40000)
        self.brake(30000)
        
        self.turn_flag *= -1
        for _ in range(250000):
            self.ctrl_cmd_msg.Target_linear_velocity = 0.0
            self.ctrl_cmd_msg.Target_angular_velocity = -2.0 * self.turn_flag
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            
        self.brake(30000)



if __name__ == '__main__':
    try:
        dilly_planner= DillyPlanner()
    except rospy.ROSInterruptException:
        pass