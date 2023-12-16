#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from find_object_2d.msg import ObjectsStamped

class EngeliGor():
    def __init__(self):
        rospy.Subscriber("objectsStamped", ObjectsStamped, self.nesne_tani)

    def nesne_tani(self, mesaj):
        try:
            nesne_id = mesaj.objects.data[0]
            print(nesne_id)
            if nesne_id == 1:
                print("Nesne Bulundu")
                
        except IndexError:
            print("Herhangi bir nesne bulunamadı!!!")
            hareket_komutlari = Twist()
            hareket_komutlari.angular.z = 0.3 
            turtlebot_kamera.pub.publish(hareket_komutlari)
            
            

class TurtleBotKamera():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("camera/rgb/image_raw", Image, self.kamera_callback)

    def kamera_callback(self, mesaj):
        frame = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        kenarlar = cv2.Canny(frame, 100, 200, 5)
        gri = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, maske = cv2.threshold(gri, 50, 255, cv2.THRESH_BINARY_INV)
        and_is = cv2.bitwise_and(frame, frame, mask=maske)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", maske)
        cv2.imshow("Mask Goruntu", and_is)
        cv2.imshow("Canny Kenarlar", kenarlar)
        cv2.imshow("Robot Kamera", frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("kamera_ve_nesne_tanima_dugumleri")

    turtlebot_kamera = TurtleBotKamera()
    nesne_tanima = EngeliGor()
    
    rospy.spin()

    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
       
        hareket_komutlari = Twist()
        hareket_komutlari.linear.x = 0.2 
        turtlebot_kamera.pub.publish(hareket_komutlari)

        rate.sleep()
    
    
    rospy.spin()
