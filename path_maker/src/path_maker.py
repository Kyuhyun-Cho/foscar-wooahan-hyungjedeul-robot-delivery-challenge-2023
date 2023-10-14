#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage
from math import pi, sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf

class PathMaker :
    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)
    
        self.cnt = 0

        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]
        
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0

        rospy.Subscriber("/gps",GPSMessage, self.gps_callback)

        self.prev_latitude = 0
        self.prev_longitude = 0

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f = open(full_path, 'w')

        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        latitude = self.latitude
        longitude = self.longitude
        altitude = 0
        
        distance = sqrt(pow(latitude - self.prev_latitude, 2) + pow(longitude - self.prev_longitude, 2))
        # print(distance)
        mode = 0
        if distance > 0.000005:   #0.3
            data='{0}\t{1}\t{2}\t{3}\n'.format(latitude, longitude, altitude, mode)
            self.f.write(data)
            self.cnt += 1
            self.prev_latitude = latitude
            self.prev_longitude = longitude
            print(self.cnt, latitude, longitude)

    def gps_callback(self, msg): 
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass

