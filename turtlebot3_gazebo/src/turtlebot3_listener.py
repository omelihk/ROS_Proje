#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f'Mesaj alındı: {data.data}')

def listener():
    rospy.init_node('goal_completed_listener', anonymous=True)
    rospy.Subscriber('goal_completed', String, callback)

    rospy.spin()  # Dinlemeyi devam ettir

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

