import rospy
from std_msgs.msg import Float64MultiArray

def callback(data):
    rospy.loginfo("Caller ID: %s, Data: %s", rospy.get_caller_id(), str(data.data))
    
def listener():
    rospy.init_node('Army', anonymous=True)
    rospy.Subscriber('jeongwooKIM', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()