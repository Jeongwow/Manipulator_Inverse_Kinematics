import rospy
from std_msgs.msg import Float32MultiArray

def callback(data):
    rospy.loginfo("Caller ID: %s, Data: %s", rospy.get_caller_id(), str(data.data))
    
def listener():
    rospy.init_node('Army', anonymous=True)
    rospy.Subscriber('ik_position', Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    