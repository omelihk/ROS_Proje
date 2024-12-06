#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
import math

class YellowLineFollower:
    def __init__(self):
        rospy.init_node("yellow_line_follower")
        
        # Kamera ve lazer tarayıcı için subscriber, hareket için publisher tanımla
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        self.bridge = CvBridge()
        self.twist = Twist()
        self.obstacle_detected = False
        self.goal_sent = False
        self.line_following_enabled = False
        self.current_position = (0.0, 0.0)
        
        # Hedef noktaları .txt dosyasından oku
        self.goal_positions = self.load_goals_from_file("/home/melih/goals.txt")
        
        # Parametreler
        self.max_linear_speed = 0.60
        self.max_angular_speed = 1.2
        self.search_start_time = None
        self.search_duration = 100
        self.centering_threshold = 16

    def load_goals_from_file(self, file_path):
        """
        TXT dosyasından hedef konumlarını okur.
        Dosya formatı: her satırda `x,y` şeklinde değerler.
        """
        goals = []
        try:
            with open(file_path, "r") as file:
                for line in file:
                    try:
                        x, y = map(float, line.strip().split(","))
                        goals.append((x, y))
                    except ValueError:
                        rospy.logwarn(f"Geçersiz satır atlanıyor: {line.strip()}")
        except FileNotFoundError:
            rospy.logerr(f"Hedef dosyası bulunamadı: {file_path}")
        except Exception as e:
            rospy.logerr(f"Hedef dosyasını okurken hata: {e}")
        return goals

    def odom_callback(self, msg):
        # Robotun mevcut pozisyonunu al
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def calculate_distance(self, position1, position2):
        # İki konum arasındaki Öklid mesafesini hesapla
        return math.sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2)

    def find_nearest_goal(self):
        # Mevcut pozisyonu kullanarak en yakın hedef konumu bul
        min_distance = float("inf")
        nearest_goal = None
        for goal in self.goal_positions:
            distance = self.calculate_distance(self.current_position, goal)
            if distance < min_distance:
                min_distance = distance
                nearest_goal = goal
        return nearest_goal

    def laser_callback(self, msg):
        # Mevcut pozisyon hedeflerden birine çok yakınsa o hedefi sil
        close_threshold = 1.0  # Hedefe olan yakınlık eşiği (metre cinsinden)
        for goal in self.goal_positions[:]:  # Listeyi kopyalayarak dolaş
            distance = self.calculate_distance(self.current_position, goal)
            if distance < close_threshold:
                rospy.loginfo(f"Hedef ({goal[0]}, {goal[1]}) çok yakın, siliniyor.")
                self.goal_positions.remove(goal)

        # Lazer tarayıcı verilerini kontrol ederek engel algılamasını yap
        if min(msg.ranges[:10] + msg.ranges[-10:]) < 0.9:
            rospy.loginfo("Engel algılandı, en yakın hedefe gidiliyor.")
            self.obstacle_detected = True
            
            # Hareketi durdur
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            
            # Hedefe gitme işlemi sadece bir kez tetiklenir
            if not self.goal_sent:
                nearest_goal = self.find_nearest_goal()
                if nearest_goal:
                    self.move_to_goal(nearest_goal[0], nearest_goal[1])
                self.goal_sent = True
        else:
            self.obstacle_detected = False
            self.goal_sent = False

    def move_to_goal(self, x, y):
        # MoveBaseAction için bir client oluştur
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        # Hedef konum için bir hedef oluştur
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        # Hedef gönder
        rospy.loginfo(f"En yakın hedefe ({x}, {y}) gidiliyor...")
        client.send_goal(goal)
        client.wait_for_result()

        if client.get_state() == 3:
            rospy.loginfo("Hedefe ulaşıldı, çizgi takibine devam ediliyor.")
            self.line_following_enabled = True
        else:
            rospy.loginfo("Hedefe ulaşılamadı")

    def image_callback(self, msg):
        if self.obstacle_detected:
            return

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

        # Maskeleri oluştur
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Yeşil kutu algılaması
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in green_contours:
            area = cv2.contourArea(cnt)
            if area > 500:
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
                
                if abs(err) > self.centering_threshold:
                    self.twist.linear.x = self.max_linear_speed * 0.75
                self.cmd_vel_pub.publish(self.twist)
                self.search_start_time = None
            else:
                if self.search_start_time is None:
                    self.search_start_time = time.time()
                if time.time() - self.search_start_time < self.search_duration:
                    rospy.loginfo("Sarı yol kayboldu, arıyor...")
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.max_angular_speed
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    rospy.loginfo("Sarı yol bulunamadı, çizgi takibi durduruluyor.")
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                    self.line_following_enabled = False

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    follower = YellowLineFollower()
    follower.run()
