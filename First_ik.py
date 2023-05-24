import numpy as np

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
    
    TODO
    Last theta2 should change to REALtheta2
    REALtheta2 = pi/2 - theta2-beta
    
    """
    in_theta3_arg1 = np.square(_x) + np.square(_y) + np.square(_z-L1)-np.square(L2)-np.square(L34)
    in_theta3_arg2 = 2*L2*L34
    th3 = -np.arccos(in_theta3_arg1/in_theta3_arg2)      # this Line possible + or -
    th1 = np.arctan2(_y,_x)       # theta, x=3,y=4 -> tan(theta)=4/3  |   theta=arctan2(4,3)
    th2 = np.arctan2(_z-L1,np.sqrt(np.square(_x) + np.square(_y))) - np.arctan2(L34*np.sin(th3),L2+L34*np.cos(th3))
    
    beta = np.arctan2(0.024,0.128)      # Little Triangle with Joint2, 3
    REALtheta2 = np.pi/2-th2-beta
    return th1, REALtheta2, th3
        
    pass



def main():
    x,y,z = map(float,input("x,y,z 입력 (스페이스바 구분)").split())
    x=x*100
    y=y*100
    z=z*100
    print(z,y,x)
    print(type(x))
    print("JUMP To Solve_ik")
    th1,th2,th3 = solve_ik(x,y,z)
    print("goal joint degree is : ")
    print("joint1 : {0}(deg)".format(th1))
    print("joint2 : {0}(deg)".format(th2))
    print("joint3 : {0}(deg)".format(th3))
    

if __name__=='__main__':
    main()