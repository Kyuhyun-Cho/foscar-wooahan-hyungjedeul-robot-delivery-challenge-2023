#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospkg,rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
from morai_msgs.msg import CtrlCmd
from math import cos,sin,sqrt,pow,atan2,pi

INDOOR = 'INDOOR'
OUTDOOR = 'OUTDOOR'

class pathReader :
    def __init__(self,pkg_name):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name = self.file_path + "/path/" + file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()        
        out_path.header.frame_id='map'
        line = openFile.readlines()
        for i in line :
            tmp = i.split()
            read_pose = PoseStamped()
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.z = float(tmp[2])
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            out_path.poses.append(read_pose)
        
        openFile.close()
        return out_path
      

def findLocalPath(ref_path, status_msg):
    out_path=  Path()
    current_x = status_msg.position.x
    current_y = status_msg.position.y
    current_waypoint = 0
    min_dis = float('inf')

    for i in range(len(ref_path.poses)) :
        dx = current_x - ref_path.poses[i].pose.position.x
        dy = current_y - ref_path.poses[i].pose.position.y
        dis = sqrt(dx*dx + dy*dy)

        # WeBot 기준 가장 가까운 waypoint 식별하기
        if dis < min_dis :
            min_dis = dis
            current_waypoint = i
        
        interval_distance = 0
        if current_waypoint >= 20:
            interval_distance = 20
        else:
            interval_distance = current_waypoint


        first_local_waypoint = current_waypoint - interval_distance

    # print(current_waypoint, len(ref_path.poses))
    # 가장 가까운 waypoint에서 +50번째에 있는 waypoint까지를 하나의 local_path로 간주하기
    if current_waypoint + 70 > len(ref_path.poses) : # 만약 가장 가까운 waypoint에서 +50번째에 있는 waypoint를 찾고자 했으나 전체 global_path 인덱스를 초과한다면 도착점에 가까워진 것이므로 global_path의 마지막 waypoint를 local_path의 종점으로 활용
        last_local_waypoint = len(ref_path.poses)
    else :
        last_local_waypoint = current_waypoint + 70


    out_path.header.frame_id = 'map'
    for i in range(first_local_waypoint,last_local_waypoint) :
        tmp_pose = PoseStamped()
        tmp_pose.pose.position.x = ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y = ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z = ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x = 0
        tmp_pose.pose.orientation.y = 0
        tmp_pose.pose.orientation.z = 0
        tmp_pose.pose.orientation.w = 1
        out_path.poses.append(tmp_pose)
    # print ("Current Waypoint: ", current_waypoint)
    return out_path, current_waypoint 


class velocityPlanning :
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction
 
    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan = []
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num,len(global_path.poses)-point_num):
            x_list = []
            y_list = []
            for box in  range(-point_num,point_num):
                x = global_path.poses[i+box].pose.position.x
                y = global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T
            
            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan


class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_position=Point()
        self.is_look_forward_point=False
        self.lfd = 2
        # self.min_lfd = 5
        # self.max_lfd = 10
        self.vehicle_length = 0.6
        self.steering = 0
        
    def getPath(self,msg):
        self.path = msg  #nav_msgs/Path 
    

    def getEgoStatus(self, msg, current_vel):

        self.current_vel = current_vel #kph
        self.vehicle_yaw=msg.heading/180*pi   # rad
        self.current_position.x=msg.position.x
        self.current_position.y=msg.position.y
        self.current_position.z=msg.position.z
        

    def steering_angle(self, lfd):
        vehicle_position = self.current_position
        rotated_point = Point()
        self.is_look_forward_point = False

        self.lfd = 1
        if (len(self.path.poses) <= 20) :
            for i in range(len(self.path.poses)) :
                # print(i)
                path_point = self.path.poses[i].pose.position
                
                dx = path_point.x - vehicle_position.x
                dy = path_point.y - vehicle_position.y
                
                rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
                rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
                
                
                if rotated_point.x > 0 :
                    dis = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))


                    if dis >= self.lfd :
                        self.lfd = lfd
                        self.forward_point = path_point
                        self.is_look_forward_point=True
                        break
        else:
            for i in range(20,len(self.path.poses)) :
                # print(i)
                path_point = self.path.poses[i].pose.position
                
                dx = path_point.x - vehicle_position.x
                dy = path_point.y - vehicle_position.y
                
                rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
                rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
                
                
                if rotated_point.x > 0 :
                    dis = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))


                    if dis >= self.lfd :
                        self.lfd = lfd
                        self.forward_point = path_point
                        self.is_look_forward_point=True
                        break
                
        # print("LFD: ", lfd)
        theta = atan2(rotated_point.y,rotated_point.x)
        self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi

        return self.steering, path_point.x, path_point.y
    

def rotLidar2Gps(obj, global_x, global_y, vehicle_status, location) :

    if len(obj) == 0: 
        return []

    ret = []

    theta = vehicle_status[2] # vehicle heading
    
    for object in (obj) :
        # dis = (object[0]**2 + object[1]**2)**0.5
        x = object[0]
        y = object[1]

        if location == INDOOR and x > 12:
            continue

        flag = True
        if x < 0 :
            flag = False
        else :
            flag = True
        # if dis <= 10:
        new_x = x*cos(theta) - y*sin(theta) + global_x
        new_y = x*sin(theta) + y*cos(theta) + global_y
        ret.append([new_x, new_y, flag])

    return ret


########################  lattice  ########################

def latticePlanner(ref_path,global_vaild_object,vehicle_status,current_lane):
    out_path=[]
    selected_lane = -1
    avoid_lane = -1
    lattic_current_lane=current_lane
    # look_distance=int(vehicle_status[3]*3.6*0.2*2)

    # if look_distance < 1 :
    #     look_distance = 1    
    # if look_distance > 3 :
    #     look_distance = 3  
    look_distance = 1
    if len(ref_path.poses)>look_distance :
        global_ref_start_point=(ref_path.poses[0].pose.position.x,ref_path.poses[0].pose.position.y)
        global_ref_start_next_point=(ref_path.poses[1].pose.position.x,ref_path.poses[1].pose.position.y)
        global_ref_end_point=(ref_path.poses[look_distance].pose.position.x,ref_path.poses[look_distance].pose.position.y)
        
        theta=atan2(global_ref_start_next_point[1]-global_ref_start_point[1],global_ref_start_next_point[0]-global_ref_start_point[0])
        translation=[global_ref_start_point[0],global_ref_start_point[1]]

        t=np.array([[cos(theta), -sin(theta),translation[0]],[sin(theta),cos(theta),translation[1]],[0,0,1]])
        det_t=np.array([[t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])   ],[t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])   ],[0,0,1]])

        world_end_point=np.array([[global_ref_end_point[0]],[global_ref_end_point[1]],[1]])
        local_end_point=det_t.dot(world_end_point)
        world_ego_vehicle_position=np.array([[vehicle_status[0]],[vehicle_status[1]],[1]])
        local_ego_vehicle_position=det_t.dot(world_ego_vehicle_position)

        # lattice 간격
        lane_off_set=[2.0,1.0,0,-1.0,-2.0]

        local_lattice_points=[]

        ############################# lattice path 생성  #############################
        for i in range(len(lane_off_set)):
            local_lattice_points.append([local_end_point[0][0],local_end_point[1][0]+lane_off_set[i],1])
            

        
        for end_point in local_lattice_points :
            lattice_path=Path()
            lattice_path.header.frame_id='map'
            x=[]
            y=[]
            x_interval=0.5
            xs=0
            xf=end_point[0]
            ps=local_ego_vehicle_position[1][0]

            pf=end_point[1]
            x_num=xf/x_interval

            for i in range(xs,int(x_num)) : 
                x.append(i*x_interval)
            
            a=[0.0,0.0,0.0,0.0]
            a[0]=ps
            a[1]=0
            a[2]=3.0*(pf-ps)/(xf*xf)
            a[3]=-2.0*(pf-ps)/(xf*xf*xf)

            for i in x:
                result=a[3]*i*i*i+a[2]*i*i+a[1]*i+a[0]
                y.append(result)


            for i in range(0,len(y)) :
                local_result=np.array([[x[i]],[y[i]],[1]])
                global_result=t.dot(local_result)

                read_pose=PoseStamped()
                read_pose.pose.position.x=global_result[0][0]
                read_pose.pose.position.y=global_result[1][0]
                read_pose.pose.position.z=0
                read_pose.pose.orientation.x=0
                read_pose.pose.orientation.y=0
                read_pose.pose.orientation.z=0
                read_pose.pose.orientation.w=1
                lattice_path.poses.append(read_pose)

            out_path.append(lattice_path)
        ############################# lattice path 생성  #############################    

        add_point_size=int(vehicle_status[3]*4*3.6)
        if add_point_size>len(ref_path.poses)-2:
            add_point_size=len(ref_path.poses)

        elif add_point_size < 70 :
            add_point_size = 70

        # print('add point',add_point_size)
        
        
         
        for i in range(look_distance,add_point_size):
            if i+1 < len(ref_path.poses):
                tmp_theta=atan2(ref_path.poses[i+1].pose.position.y-ref_path.poses[i].pose.position.y,ref_path.poses[i+1].pose.position.x-ref_path.poses[i].pose.position.x)
                
                tmp_translation=[ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])
                tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],[tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],[0,0,1]])

                for lane_num in range(len(lane_off_set)) :
                    local_result=np.array([[0],[lane_off_set[lane_num]],[1]])
                    global_result=tmp_t.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x=global_result[0][0]
                    read_pose.pose.position.y=global_result[1][0]
                    read_pose.pose.position.z=0
                    read_pose.pose.orientation.x=0
                    read_pose.pose.orientation.y=0
                    read_pose.pose.orientation.z=0
                    read_pose.pose.orientation.w=1
                    out_path[lane_num].poses.append(read_pose)

        lane_weight=[2,1,0,1,2] #reference path 
        collision_bool=[False,False,False,False,False]

        # lattice path 내 장애물 탐색하여 가중치 조절
        if len(global_vaild_object)>0:

            for obj in global_vaild_object :
                
                if ((391.69 <= obj[0] <= 399.94 and -117.13 <= obj[1] <= -108.88) or    # 첫번째 평행봉
                    (389.77 <= obj[0] <= 401.28 and -93.30 <= obj[1] <= -88.30) or      # 두번째 평행봉
                    (435.25 <= obj[0] <= 438.76 and -132.96 <= obj[1] <= -116.77) or    # 야외 입구 나무
                    (429.43 <= obj[0] <= 432.78 and -146.81 <= obj[1] <= -118.97) or    # 야외 입구 왼쪽 벽
                    (397.13 <= obj[0] <= 398.79 and -86.41 <= obj[1] <= -84.92) or      # 두번째 평행봉 오른쪽 위 나무
                    (323.73 <= obj[0] <= 424.01 and -79.9 <= obj[1] <= -64.76) or       # 1번 뒤 숲
                    (388.98 <= obj[0] <= 390.31 and -86.32 <= obj[1] <= -84.96) or      # 두번째 평행봉 왼쪽 위 나무
                    (333.83 <= obj[0] <= 335.98 and -188.76 <= obj[1] <= -116.92) or    # 4번 수령자
                    (247.23 <= obj[0] <= 501.36 and -92.36 <= obj[1] <= -84.45)):       # 2
                    continue
                
                for path_num in range(len(out_path)) :
                    
                    for path_pos in out_path[path_num].poses : #path_pos = PoseStamped()
                        
                        dis= sqrt(pow(obj[0]-path_pos.pose.position.x,2)+pow(obj[1]-path_pos.pose.position.y,2))
                        dis_car_obj = max(sqrt((obj[0]-vehicle_status[0])**2 + (obj[1]-vehicle_status[1])**2),0.1)
                        # print(path_num, dis)

                        if dis <= 2.25:
                            collision_bool[path_num]=True
                            # lane_weight[path_num] += 100
                            if obj[2] == True :
                                lane_weight[path_num] += 2*dis**-1 * (1000/dis_car_obj)# 패스에 장애물이 가까울 경우 높은 weight, 멀 경우 낮은 weight 더하기
                            else : 
                                lane_weight[path_num] += dis**-1 * (1000/dis_car_obj)
                            # break

                    

        # else :
        #     print("No Obstacle")
        # print("LANE WEIGHTS: ", lane_weight)
        selected_lane = lane_weight.index(min(lane_weight))
        
        if sum(lane_weight) != 6:
            avoid_lane = lane_weight.index(max(lane_weight))
        else:
            avoid_lane = 2
        
        
        all_lane_collision=True

        # print(lane_weight)
    
    else :
        print("NO Reference Path")
        selected_lane=-1    

    return out_path, selected_lane, avoid_lane

########################  lattice  ########################