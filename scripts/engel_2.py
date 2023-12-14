#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys

class LazerAlgılama():
    def __init__(self):
        rospy.init_node("lazer_dugumu")
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.hiz_mesaji = Twist()
        rospy.Subscriber("scan",LaserScan,self.lazerCallback)
        rospy.spin()
        
    def lazerCallback(self,lidar_verileri):
        sol_on = list(lidar_verileri.ranges[0:15])
        sag_on = list(lidar_verileri.ranges[345:359])
        on = sol_on + sag_on
        sol = list(lidar_verileri.ranges[75:105])
        sag = list(lidar_verileri.ranges[255:285])
        arka = list(lidar_verileri.ranges[165:195])
        min_on = min(on)
        min_sol = min(sol)
        min_sag = min(sag)
        min_arka = min(arka)
        print(min_on,min_sol,min_sag,min_arka)
        if min_on < 3.0:
            self.hiz_mesaji.linear.x = 0.0
            self.pub.publish(self.hiz_mesaji)
            
        else:
            self.hiz_mesaji.linear.x = 0.25
            self.pub.publish(self.hiz_mesaji)

nesne = LazerAlgılama()

