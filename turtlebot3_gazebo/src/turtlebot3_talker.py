#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

# Global değişkenler
current_position = None
goal_position = None

def odometry_callback(data):
    global current_position
    current_position = data.pose  # 'pose' niteliği ile doğrudan pozisyonu alıyoruz

def goal_callback(data):
    global goal_position
    goal_position = data.pose  # Burada da doğrudan 'pose' kullanmalısınız

def calculate_distance(pos1, pos2):
    """ İki pozisyon arasındaki mesafeyi hesaplar """
    return math.sqrt((pos1.x - pos2.x) ** 2 + 
                     (pos1.y - pos2.y) ** 2)

def talker():
    global current_position, goal_position

    rospy.init_node('goal_completed_talker', anonymous=True)
    pub = rospy.Publisher('goal_completed', String, queue_size=10)

    # Odometry ve hedef konumları dinleyicileri
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, odometry_callback)  # Mesaj türü burada doğru ayarlandı
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)  # Hedef konum

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        if current_position is not None and goal_position is not None:
            # current_position ve goal_position'dan pozisyonları alıyoruz
            current_pos = current_position.pose.position  # 'pose' niteliğinden 'position' alıyoruz
            goal_pos = goal_position.position  # Hedef konumu doğrudan alıyoruz
            
            distance = calculate_distance(current_pos, goal_pos)
            rospy.loginfo(f'Mesafe: {distance:.2f} m')

            # Mesafe eşik değerini belirleyin
            threshold_distance = 0.2  # 20 cm

            if distance < threshold_distance:
                message = "Hedefe ulaşıldı!"
                rospy.loginfo(message)
                pub.publish(message)
                # Hedef ve mevcut konumu sıfırlayabiliriz
                current_position = None
                goal_position = None
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

