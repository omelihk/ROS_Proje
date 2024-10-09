#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x, y):
    # ROS node başlatılıyor
    rospy.init_node('turtlebot3_move_to_goal', anonymous=True)

    # MoveBaseAction için bir client oluşturuluyor
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Hedef konum için bir hedef oluşturuluyor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Hedefin koordinat sistemi
    goal.target_pose.header.stamp = rospy.Time.now()

    # Hedef koordinatları ayarlanıyor
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # Hedefin oryantasyonu (sadece ileri bakacak)

    # Hedef gönderiliyor
    rospy.loginfo("Yeni Koordinatlar Gonderiliyor...")
    client.send_goal(goal)

    # Hedefin tamamlanması bekleniyor
    client.wait_for_result()

    # Hedefin durumu kontrol ediliyor
    if client.get_state() != 3:
        rospy.loginfo("Hedefe Ulaşılamadı")

if __name__ == '__main__':
    try:
        move_to_goal(-1.00, 1.00)  # Belirtilen koordinatlara git
        rospy.loginfo("1. Gorev Tamamlandı!")
        rospy.loginfo("2. Goreve Gidiliyor!")
        move_to_goal(-4.00, 4.00) 
        rospy.loginfo("2. Gorev Tamamlandı!")
        rospy.loginfo("3. Goreve Gidiliyor!")
        move_to_goal(5.00, 1.00)
        rospy.loginfo("Tum Gorevler Tamamlandi!")
    except rospy.ROSInterruptException:
        pass

