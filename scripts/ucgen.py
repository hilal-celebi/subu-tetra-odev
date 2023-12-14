#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import math

class RobotHareketSınıfı:
    def __init__(self,hız,acisal_hız):
        self.hız = hız
        self.acisal_hız = acisal_hız
        rospy.init_node("HareketDugum")
        
        self.cmd_vel = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.hiz_mesajı = Twist()
        
    def Yayinla(self):
        self.cmd_vel.publish(self.hiz_mesajı)
    
    def Dur(self):
        self.hiz_mesajı.linear.x = 0
        self.hiz_mesajı.angular.z = 0
        self.Yayinla()
        
    def SurayaKadarGit(self,mesafe):
        rate = rospy.Rate(10)
        rate.sleep()
        baslangıc_zaman = rospy.Time.now().to_sec()
        kalan_yol = 999999
        
        while kalan_yol > 0:
            guncel_zaman = rospy.Time.now().to_sec()
            #print("GecenZaman: {}".format((guncel_zaman - baslangıc_zaman)))
            gidilen_yol = self.hız * (guncel_zaman - baslangıc_zaman)
            kalan_yol = mesafe - gidilen_yol
            print("Kalan Yol: {}".format(kalan_yol))
            self.hiz_mesajı.linear.x = self.hız
            self.Yayinla()
            rate.sleep()
        
        self.Dur()
        print("Duruldu")
        
    def SurayaKadarGitAcisal(self,mesafe):
        rate = rospy.Rate(10)
        rate.sleep()
        baslangıc_zaman =  rospy.Time.now().to_sec()
        kalan_yol = 98765
        
        while kalan_yol > 0:
            guncel_zaman = rospy.Time.now().to_sec()
            gidilen_yol = self.acisal_hız * (guncel_zaman - baslangıc_zaman)
            kalan_yol = mesafe - gidilen_yol
            print("AcisalKalan Yol: {}".format(kalan_yol))
            self.hiz_mesajı.angular.z= self.acisal_hız
            self.Yayinla()
            rate.sleep()
        self.Dur()
        print("Duruldu")
        
robot_sınıfı = RobotHareketSınıfı(0.1,math.radians(5))
robot_sınıfı.SurayaKadarGit(0.2)
robot_sınıfı.SurayaKadarGitAcisal(math.radians(120))
robot_sınıfı.SurayaKadarGit(0.2)
robot_sınıfı.SurayaKadarGitAcisal(math.radians(120))
robot_sınıfı.SurayaKadarGit(0.2)
