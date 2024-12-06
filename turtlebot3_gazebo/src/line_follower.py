#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class YellowLineFollower:
    def __init__(self):
        rospy.init_node("yellow_line_follower")
        
        # Kamera görüntüsü için subscriber ve hareket için publisher tanımla
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        self.bridge = CvBridge()
        self.twist = Twist()
        
        # Parametreler
        self.max_linear_speed = 0.50  # Maksimum ileri hız
        self.max_angular_speed = 0.5
        self.search_start_time = None
        self.search_duration = 10  # Arama süresi 10 saniye
        self.centering_threshold = 16  # Ortalanma hassasiyeti (piksel cinsinden)

    def image_callback(self, msg):
        # Görüntüyü ROS formatından OpenCV formatına çevir
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Görüntüyü HSV formatına çevir ve sarı renk aralığını belirle
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 120, 120])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Sarı yolun merkezi noktasını bul
        h, w, d = image.shape
        search_top = 3 * h // 4
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        # Sarı piksel merkezini bul ve hata miktarını hesapla
        M = cv2.moments(mask)
        if M["m00"] > 0:
            # Sarı yol algılandıysa, sapma miktarını hesapla
            cx = int(M["m10"] / M["m00"])
            err = cx - w / 2
            
            # Sapma varsa robotun yönünü ayarla
            rospy.loginfo("Sarı çizgiye göre hizalanıyor...")
            self.twist.linear.x = self.max_linear_speed
            self.twist.angular.z = -float(err) / 200  # Yolun merkezine dön

            # Açısal hızı sınırlama
            self.twist.angular.z = max(-self.max_angular_speed, min(self.twist.angular.z, self.max_angular_speed))

            self.cmd_vel_pub.publish(self.twist)
            self.search_start_time = None  # Sarı yolu bulduğumuz için arama zamanlayıcısını sıfırla
        else:
            # Eğer sarı yol görünmüyorsa arama moduna geç
            if self.search_start_time is None:
                self.search_start_time = time.time()  # Arama süresini başlat

            # Arama moduna başla (dönme hareketi)
            if time.time() - self.search_start_time < self.search_duration:
                rospy.loginfo("Sarı yol kayboldu, arama moduna geçildi.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.2  # Yavaşça dönme hareketi
                self.cmd_vel_pub.publish(self.twist)
            else:
                # Eğer arama süresi sona erdiyse dur
                rospy.loginfo("Sarı yol bulunamadı, robot duruyor.")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self.twist)
        
        # Görüntüleri göster
        cv2.imshow("mask", mask)
        cv2.imshow("output", image)
        cv2.waitKey(3)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    follower = YellowLineFollower()
    follower.run()
