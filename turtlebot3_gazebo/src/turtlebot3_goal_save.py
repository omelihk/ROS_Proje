#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class YellowLineFollower:
    def __init__(self):
        rospy.init_node("yellow_line_follower")
        
        # Kamera görüntüsü için subscriber ve hareket için publisher tanımla
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.bridge = CvBridge()
        self.twist = Twist()
        
        # Parametreler
        self.max_linear_speed = 0.60  # Maksimum ileri hız
        self.max_angular_speed = 1.2
        self.search_start_time = None
        self.search_duration = 100  # Arama süresi 2 saniye
        self.centering_threshold = 16 # Ortalanma hassasiyeti (piksel cinsinden)
        self.line_following_enabled = False  # Çizgi takibini başlatma/değiştirme durumu
        
        self.last_log_time = time.time()  # Son kaydedilen zaman
        self.goal_file_path = "/home/melih/goals.txt"  # Konumların kaydedileceği dosya yolu
        self.current_position = None  # Robotun mevcut pozisyonu

    def odom_callback(self, msg):
        # Robotun pozisyonunu odometry mesajından al
        self.current_position = msg.pose.pose.position

    def log_position(self):
        # Robot pozisyonunu dosyaya kaydet
        if self.current_position is not None:
            with open(self.goal_file_path, "a") as file:
                file.write(f"{self.current_position.x:.2f}, {self.current_position.y:.2f}\n")
            rospy.loginfo(f"Konum kaydedildi: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}")

    def image_callback(self, msg):
        # Görüntüyü ROS formatından OpenCV formatına çevir
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Görüntüyü HSV formatına çevir ve sarı, yeşil, kırmızı renk aralıklarını belirle
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 120, 120])
        upper_yellow = np.array([35, 255, 255])
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Sarı, yeşil ve kırmızı maskeleri oluştur
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Yeşil kutu algılaması
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in green_contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # Sadece yeterince büyük yeşil alanları dikkate al
                rospy.loginfo("Yeşil kutu görüldü, çizgi takibi başlatılıyor.")
                self.line_following_enabled = True
                break

        # Kırmızı kutu algılaması
        if cv2.countNonZero(red_mask) > 0:
            rospy.loginfo("Kırmızı kutu görüldü, robot duruyor.")
            self.line_following_enabled = False
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # Çizgi takibi
        if self.line_following_enabled:
            h, w, d = image.shape
            search_top = 3 * h // 4
            search_bot = search_top + 20
            yellow_mask[0:search_top, 0:w] = 0
            yellow_mask[search_bot:h, 0:w] = 0
            
            M = cv2.moments(yellow_mask)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                err = cx - w / 2
                
                rospy.loginfo("Sarı çizgiye göre hizalanıyor...")
                self.twist.linear.x = self.max_linear_speed
                self.twist.angular.z = -float(err) / 150
                self.twist.angular.z = max(-self.max_angular_speed, min(self.twist.angular.z, self.max_angular_speed))
                
                # Hız ve dönüş oranını optimize et
                if abs(err) > self.centering_threshold:
                    self.twist.linear.x = self.max_linear_speed * 0.75  # Hızı azalt
                self.cmd_vel_pub.publish(self.twist)
                self.search_start_time = None

                # Pozisyonu düzenli olarak kaydet
                current_time = time.time()
                if current_time - self.last_log_time > 10:  # 10 saniye aralıkla
                    self.log_position()
                    self.last_log_time = current_time
            else:
                if self.search_start_time is None:
                    self.search_start_time = time.time()
                if time.time() - self.search_start_time < self.search_duration:
                    rospy.loginfo("Sarı yol kayboldu, arama moduna geçildi.")
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.4  # Dönmeye devam et
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    rospy.loginfo("Sarı yol bulunamadı, robot duruyor.")
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
        
        # Görüntüleri göster
        cv2.imshow("Camera View", image)         # Kamera görüntüsü
        cv2.imshow("Yellow Mask", yellow_mask)    # Sarı maske
        cv2.imshow("Green Mask", green_mask)      # Yeşil maske
        cv2.imshow("Red Mask", red_mask)          # Kırmızı maske
        cv2.waitKey(3)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    follower = YellowLineFollower()
    follower.run()
