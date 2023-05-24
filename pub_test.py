import rospy
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('jeongwooKIM', Float64MultiArray, queue_size=10)
    rospy.init_node('Marin', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        array_for_send = [1,2,3,4,5]        # Joint 1234, griper.
        topic_for_send = Float64MultiArray(data=array_for_send)
        rospy.loginfo(topic_for_send)
        pub.publish(topic_for_send)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass