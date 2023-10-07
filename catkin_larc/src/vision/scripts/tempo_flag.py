#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def true_publisher():
    pub = rospy.Publisher('flag', Bool, queue_size=10)
    rospy.init_node('true_publisher_node', anonymous=True)
    rate = rospy.Rate(10)  # Frecuencia de publicacion (10 Hz en este caso)

    while not rospy.is_shutdown():
        pub.publish(False)  # Publica constantemente 'True' en el tema 'my_true_topic'
        rate.sleep()

if __name__ == '__main__':
    try:
        true_publisher()
    except rospy.ROSInterruptException:
        pass
