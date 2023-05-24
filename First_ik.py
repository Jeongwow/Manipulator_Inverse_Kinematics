import numpy as np

# init H/W
# 기준 cm
L1 = 7.7
L2 = 13
L3 = 12.4
L4 = 12.6
L34 = 25



def solve_ik(_x,_y,_z):
    """
    should return theta1,2,3****(change format to cm)
    
    TODO
    Last theta2 should change to REALtheta2
    REALtheta2 = pi/2 - theta2-beta
    
    """
    
    
        
    pass



def main():
    x,y,z = map(float,input("x,y,z 입력 (스페이스바 구분)").split())

    print(z,y,x)
    print(type(x))
    print("JUMP To Solve_ik")
    solve_ik(x,y,z)
    

if __name__=='__main__':
    main()