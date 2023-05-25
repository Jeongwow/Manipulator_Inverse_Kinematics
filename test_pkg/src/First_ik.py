import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
# 0.240 0.000 0.030
# init H/W
# 기준 cm
L1 = 7.7
L2 = 13
L3 = 12.4
L4 = 12.6
L34 = 25

def get_now_pos():
    now_pos_x=0
    now_pos_y=0
    now_pos_z=0
    
    
    pass

def check_collision():
    """
    Joint Can Move To Goal Pos With Out Collision?
    """
    
    pass

def solve_ik(_x,_y,_z):
    """
    Input : _x,_y,_z (kinematics position with keyboard input)
    Output : theta1,2,3****
    Notice !! 
     - changed format to cm
     - return to Degree    (np.arctan2() output is radian)
    
    TODOr
    Last theta2 should change to REALtheta2
    REALtheta2 = pi/2 - theta2-beta
    
    """
    # in_theta3_arg1 = np.square(_x) + np.square(_y) + np.square(_z-L1)-np.square(L2)-np.square(L34)
    # in_theta3_arg2 = 2*L2*L34
    # th3 = np.arccos(in_theta3_arg1/in_theta3_arg2)      # this Line possible + or -
    # th1 = np.arctan2(_y,_x)       # theta, x=3,y=4 -> tan(theta)=4/3  |   theta=arctan2(4,3)
    # th2 = np.arctan2(_z-L1,np.sqrt(np.square(_x) + np.square(_y))) - np.arcsin((L34*np.sin(th3))/(np.square(_x)+np.square(_y)+np.square(_z-L1)))
    
    
    
    in_theta3_arg1 = np.square(_x) + np.square(_y) + np.square(_z-L1)-np.square(L2)-np.square(L34)
    in_theta3_arg2 = 2*L2*L34
    th3 = np.arccos(in_theta3_arg1/in_theta3_arg2)      # this Line possible + or -
    th1 = np.arctan2(_y,_x)       # theta, x=3,y=4 -> tan(theta)=4/3  |   theta=arctan2(4,3)
    th2 = np.arctan2(_z-L1,np.sqrt(np.square(_x) + np.square(_y))) - np.arctan2(L34*np.sin(-th3),L2+L34*np.cos(-th3))
    
    beta = np.arctan2(0.024,0.128)      # Little Triangle with Joint2, 3
    REALtheta2 = np.pi/2-th2-beta
    return th1, REALtheta2, th3
         
    pass

def publish_ik_pos(th1,th2,th3,th4):
    pub = rospy.Publisher('ik_position', Float32MultiArray, queue_size=10)
    rospy.init_node('Please_Solve_IK', anonymous=True)

    
    rate = rospy.Rate(1) # 2hz
    while not rospy.is_shutdown():
        array_for_send = [th1,th2,th3,th4]        # Joint 1234, griper.
        topic_for_send = Float32MultiArray(data=array_for_send)
        rospy.loginfo(topic_for_send)
        pub.publish(topic_for_send)
        rate.sleep()


def main():
    while 1:
        x,y,z = map(float,input("x,y,z 입력 (스페이스바 구분)").split())
        if x=='q':
            break
        x=x*100
        y=y*100
        z=z*100
        print(z,y,x)
        print(type(x))
        print("JUMP To Solve_ik")
        th1,th2,th3 = solve_ik(x,y,z)
        th4 = 0

        print("goal joint degree is : ")
        print("joint1 : {0}(deg)".format(th1))
        print("joint2 : {0}(deg)".format(th2))
        print("joint3 : {0}(deg)".format(th3))
        
        publish_ik_pos(th1,th2,th3,th4)
    

if __name__=='__main__':
    main()