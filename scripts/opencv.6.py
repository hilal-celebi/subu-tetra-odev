#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist


class TurtleBotKamera():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.hiz_mesaji = Twist()
        self.nesne_bulundu = False
        self.varildi = False

        rospy.Subscriber("camera/rgb/image_raw", Image, self.kamera_callback)
        rospy.Subscriber("scan", LaserScan, self.lazerCallback)
        

        rospy.init_node("camera")

    def kamera_callback(self, mesaj):
        frame = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        kenarlar = cv2.Canny(frame, 100, 200, 5)
        cv2.imshow("Frame", frame)
        cv2.imshow("Canny Kenarlar", kenarlar)
        cv2.imshow("Robot Kamera", frame)
        cv2.waitKey(1)

        self.nesne_tani(kenarlar)

    def lazerCallback(self, lidar_verileri):
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
        print(min_on, min_sol, min_sag, min_arka)
        self.hareket_et(min_on)
            
    def nesne_tani(self, kenarlar):
        
        kenar_sayisi = np.sum(kenarlar > 0)
        
        if kenar_sayisi > 750: 
            print("Nesne Bulundu")
            self.nesne_bulundu = True
        else:
            print("Herhangi bir nesne bulunamadı!!!")
            self.nesne_bulundu = False


    def hareket_et(self, min_on):
        hareket_komutlari = Twist()
        if self.varildi:
            hareket_komutlari.linear.x = 0.0
            if min_on > 2.0:
                    hareket_komutlari.linear.x = 0.0
                    self.varildi = False
        else:
            if self.nesne_bulundu:
                if min_on < 2.0:
                    hareket_komutlari.linear.x = 0.0
                    self.varildi = True
                else:
                    hareket_komutlari.linear.x = 0.25
            else:
                hareket_komutlari.linear.x = 0.0  
                hareket_komutlari.angular.z = 0.3  
                if min_on < 2.0:
                        hareket_komutlari.linear.x = 0.0
                        self.varildi = True
                        
        self.pub.publish(hareket_komutlari)


if __name__ == "__main__":
    turtlebot_kamera = TurtleBotKamera()

    rospy.spin()