# coding:utf-8

import numpy as np
import math
from sympy.geometry import Point,Line,Segment
import learn_image_maker as lim

ROBOT_SPEED = 5
TIME_STEP = 0.5
ROBOT_BACK_MARKER_REWARD = 500
ROBOT_SIDE_MARKER_REWARD = 300
FIELD_MARKER_REWARD = 200
CAN_READ_DISTANCE =39

REWARD_CIRCLE1 = 50
REWARD_CIRCLE2 = 60
REWARD_CIRCLE3 = 70

def distance(x1,y1,x2,y2):

    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

def theta_calc(theta,delta):

    result = theta+delta

    if result >= 360:
        result = result-360

    return result

def theta_calc2(theta,delta):

    result = theta+delta

    if result > 4:
        result = result-4

    return result

def collision_check(rx,ry,ex,ey):

    center_xylist = np.array([[125,50],[200,125],[125,200],[50,125],[125,125]])

    #center_xylist = np.array([125,88],[162,125],[125,162],[88,125],[125,125])

    collision_flag = 0
    
    for i in range(4):
        if distance(center_xylist[i][0],center_xylist[i][1],rx,ry)<= 26:
            collision_flag = 1
            #return 1
    if distance(center_xylist[4][0],center_xylist[4][1],rx,ry) <= 38:
        collision_flag = 1
        #return 1

    #wall
    if ry<=19 or rx>=230 or ry>=230 or rx<=19:
        collision_flag = 1
        #return 1

    if distance(rx,ry,ex,ey) < 26:
        collision_flag = 1
    
    return collision_flag

#new_x,new_y,new_theta,collision_flag
def robot_move(action,rx,ry,theta,ex,ey):

    collision_flag =0

    """
    if action == 1:
        if collision_check(rx,ry):
            return rx,ry,theta,1
        else:
            rx = rx+int(ROBOT_SPEED/math.sqrt(2))
            ry = ry+int(ROBOT_SPEED/math.sqrt(2))
    elif action == 2:
        if collision_check(rx,ry):
            return rx,ry,theta,1
        rx = rx+int(ROBOT_SPEED/math.sqrt(2))
        ry = ry+int(ROBOT_SPEED/math.sqrt(2))
    """

    last_rx = rx
    last_ry = ry
    if action == 1:
        if theta == 1:
            rx = rx-int(ROBOT_SPEED/math.sqrt(2))
            ry = ry-int(ROBOT_SPEED/math.sqrt(2))
        elif theta ==2:
            rx = rx+int(ROBOT_SPEED/math.sqrt(2))
            ry = ry-int(ROBOT_SPEED/math.sqrt(2))
        elif theta == 3:
            rx = rx+int(ROBOT_SPEED/math.sqrt(2))
            ry = ry+int(ROBOT_SPEED/math.sqrt(2))
        elif theta == 4:
            rx = rx-int(ROBOT_SPEED/math.sqrt(2))
            ry = ry+int(ROBOT_SPEED/math.sqrt(2))
        if collision_check(rx,ry,ex,ey):
            rx = last_rx
            ry = last_ry
            collision_flag = 1

    elif action == 2:
            
        if theta == 1:
            rx = rx+int(ROBOT_SPEED/math.sqrt(2))
            ry = ry+int(ROBOT_SPEED/math.sqrt(2))
        elif theta ==2:
            rx = rx-int(ROBOT_SPEED/math.sqrt(2))
            ry = ry+int(ROBOT_SPEED/math.sqrt(2))
        elif theta == 3:
            rx = rx-int(ROBOT_SPEED/math.sqrt(2))
            ry = ry-int(ROBOT_SPEED/math.sqrt(2))
        elif theta == 4:
            rx = rx+int(ROBOT_SPEED/math.sqrt(2))
            ry = ry-int(ROBOT_SPEED/math.sqrt(2))
        if collision_check(rx,ry,ex,ey):
            rx = last_rx
            ry = last_ry
            collision_flag = 1
    
    elif action == 3:
        theta = theta_calc2(theta,1)

    elif action == 4:
        theta = theta_calc2(theta,2)
    elif action == 5:
        theta = theta_calc2(theta,3)

    return rx,ry,theta,collision_flag
        
def point_check(rx,ry,theta,erx,ery,e_theta,point_list,enemy_flag=1):# 敵機の点数チェックではenemy_flag = -1
    reward = 0
    rp = Point(rx,ry)

    #1-1
    l1 = Line(Point(125,36),Point(111,50))
    s = Segment(Point(125,36),Point(111,50))
    l2 = Line(Point(125,50),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) < CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 3 and point_list[0] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[0] = 1*enemy_flag
        return point_list,reward

    #1-2
    l1 = Line(Point(139,50),Point(125,64))
    s = Segment(Point(139,50),Point(125,64))
    l2 = Line(Point(200,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 1 and point_list[1] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[1] = 1*enemy_flag
        return point_list,reward


    
    #2-1
    l1 = Line(Point(200,111),Point(186,125))
    s = Segment(Point(200,111),Point(186,125))
    l2 = Line(Point(200,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 3 and point_list[2] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[2] = 1*enemy_flag
        return point_list,reward

    #2-2
    l1 = Line(Point(214,125),Point(200,139))
    s = Segment(Point(214,125),Point(200,139))
    l2 = Line(Point(200,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 1 and point_list[3] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[3] = 1*enemy_flag
        return point_list,reward
    

    #3-1
    l1 = Line(Point(125,186),Point(111,200))
    s = Segment(Point(125,186),Point(111,200))
    l2 = Line(Point(125,200),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 3 and point_list[4] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[4] = 1*enemy_flag
        return point_list,reward

    #3-2
    l1 = Line(Point(139,200),Point(125,214))
    s = Segment(Point(139,200),Point(125,214))
    l2 = Line(Point(125,200),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 1 and point_list[5] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[5] = 1*enemy_flag
        return point_list,reward



    #4-1
    l1 = Line(Point(50,111),Point(36,125))
    s = Segment(Point(50,111),Point(36,125))
    l2 = Line(Point(50,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 3 and point_list[6] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[6] = 1*enemy_flag
        return point_list,reward

    #4-2
    l1 = Line(Point(64,125),Point(50,139))
    s = Segment(Point(64,125),Point(50,139))
    l2 = Line(Point(50,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 1 and point_list[7] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[7] = 1*enemy_flag
        return point_list,reward


     #c1
    l1 = Line(Point(125,100),Point(100,125))
    s = Segment(Point(125,100),Point(100,125))
    l2 = Line(Point(125,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 3 and point_list[8] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[8] = 1*enemy_flag
        return point_list,reward

    #c2
    l1 = Line(Point(125,100),Point(150,125))
    s = Segment(Point(125,100),Point(150,125))
    l2 = Line(Point(125,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 4 and point_list[9] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[9] = 1*enemy_flag
        return point_list,reward

    #c3
    l1 = Line(Point(150,125),Point(125,150))
    s = Segment(Point(150,125),Point(125,150))
    l2 = Line(Point(125,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 1 and point_list[10] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[10] = 1*enemy_flag
        return point_list,reward

    #c4
    l1 = Line(Point(125,150),Point(100,125))
    s = Segment(Point(125,150),Point(100,125))
    l2 = Line(Point(125,125),s.midpoint)
    a,b,c = l1.coefficients

    if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 2 and point_list[11] != 1*enemy_flag:

        reward = enemy_flag*FIELD_MARKER_REWARD
        point_list[11] = 1*enemy_flag
        return point_list,reward


    if enemy_flag ==1:
        #enemy_target1
        l1 = Line(Point(erx-13,ery),Point(erx,ery-13))
        s = Segment(Point(erx,ery-13),Point(erx-13,ery))
        l2 = Line(Point(erx,ery),s.midpoint)
        a,b,c = l1.coefficients

        if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 3:

            if e_theta == 4 and point_list[15]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[15] = 1
                return point_list,reward
            elif e_theta == 3 and point_list[16]==0:
                reward = ROBOT_BACK_MARKER_REWARD
                point_list[16] = 1
                return point_list,reward
            elif e_theta == 2 and point_list[17]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[17] = 1
                return point_list,reward

                
        #enemy_target2
        l1 = Line(Point(erx,ery-13),Point(erx+13,ery))
        s = Segment(Point(erx,ery-13),Point(erx+13,ery))
        l2 = Line(Point(erx,ery),s.midpoint)
        a,b,c = l1.coefficients
        
        if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) > 0 and theta == 4:

            if e_theta == 1 and point_list[15]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[15] = 1
                return point_list,reward
            elif e_theta == 4 and point_list[16]==0:
                reward = ROBOT_BACK_MARKER_REWARD
                point_list[16] = 1
                return point_list,reward
            elif e_theta == 3 and point_list[17]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[17] = 1
                return point_list,reward


         #enemy_target3
        l1 = Line(Point(erx,ery+13),Point(erx+13,ery))
        s = Segment(Point(erx,ery+13),Point(erx+13,ery))
        l2 = Line(Point(erx,ery),s.midpoint)
        a,b,c = l1.coefficients
        
        if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 1:

            if e_theta == 2 and point_list[15]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[15] = 1
                return point_list,reward
            elif e_theta == 1 and point_list[16]==0:
                reward = ROBOT_BACK_MARKER_REWARD
                point_list[16] = 1
                return point_list,reward
            elif e_theta == 4 and point_list[17]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[17] = 1
                return point_list,reward


         #enemy_target4
        l1 = Line(Point(erx,ery+13),Point(erx-13,ery))
        s = Segment(Point(erx,ery+13),Point(erx-13,ery))
        l2 = Line(Point(erx,ery),s.midpoint)
        a,b,c = l1.coefficients
        
        if l1.distance(rp) <  CAN_READ_DISTANCE and l2.distance(rp) < 10 and (a*rx+b*ry+c) < 0 and theta == 2:

            if e_theta == 3 and point_list[15]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[15] = 1
                return point_list,reward
            elif e_theta == 2 and point_list[16]==0:
                reward = ROBOT_BACK_MARKER_REWARD
                point_list[16] = 1
                return point_list,reward
            elif e_theta == 1 and point_list[17]==0:
                reward = ROBOT_SIDE_MARKER_REWARD
                point_list[17] = 1
                return point_list,reward

        erp = Point(erx,ery)
        #robot_target1
        l1 = Line(Point(rx-13,ry),Point(rx,ry-13))
        s = Segment(Point(rx,ry-13),Point(rx-13,ry))
        l2 = Line(Point(rx,ry),s.midpoint)
        a,b,c = l1.coefficients

        if l1.distance(erp) <  CAN_READ_DISTANCE and l2.distance(erp) < 10 and (a*erx+b*ery+c) > 0 and e_theta == 3:

            if theta == 4 and point_list[12]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[12] = 1
                return point_list,reward
            elif theta == 3 and point_list[13]==0:
                reward = -ROBOT_BACK_MARKER_REWARD
                point_list[13] = 1
                return point_list,reward
            elif theta == 2 and point_list[14]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[14] = 1
                return point_list,reward

                
        #robot_target2
        l1 = Line(Point(rx,ry-13),Point(rx+13,ry))
        s = Segment(Point(rx,ry-13),Point(rx+13,ry))
        l2 = Line(Point(rx,ry),s.midpoint)
        a,b,c = l1.coefficients
        
        if l1.distance(erp) <  CAN_READ_DISTANCE and l2.distance(erp) < 10 and (a*erx+b*ery+c) > 0 and e_theta == 4:

            if theta == 1 and point_list[12]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[12] = 1
                return point_list,reward
            elif theta == 4 and point_list[13]==0:
                reward = -ROBOT_BACK_MARKER_REWARD
                point_list[13] = 1
                return point_list,reward
            elif theta == 3 and point_list[14]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[14] = 1
                return point_list,reward


         #robot_target3
        l1 = Line(Point(rx,ry+13),Point(rx+13,ry))
        s = Segment(Point(rx,ry+13),Point(rx+13,ry))
        l2 = Line(Point(rx,ry),s.midpoint)
        a,b,c = l1.coefficients
        
        if l1.distance(erp) <  CAN_READ_DISTANCE and l2.distance(erp) < 10 and (a*erx+b*ery+c) < 0 and e_theta == 1:

            if theta == 2 and point_list[12]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[12] = 1
                return point_list,reward
            elif theta == 1 and point_list[13]==0:
                reward = -ROBOT_BACK_MARKER_REWARD
                point_list[13] = 1
                return point_list,reward
            elif theta == 4 and point_list[14]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[14] = 1
                return point_list,reward


         #robot_target4
        l1 = Line(Point(rx,ry+13),Point(rx-13,ry))
        s = Segment(Point(rx,ry+13),Point(rx-13,ry))
        l2 = Line(Point(rx,ry),s.midpoint)
        a,b,c = l1.coefficients
        
        if l1.distance(erp) <  CAN_READ_DISTANCE and l2.distance(erp) < 10 and (a*erx+b*ery+c) < 0 and e_theta == 2:

            if theta == 3 and point_list[12]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[12] = 1
                return point_list,reward
            elif theta == 2 and point_list[13]==0:
                reward = -ROBOT_BACK_MARKER_REWARD
                point_list[13] = 1
                return point_list,reward
            elif theta == 1 and point_list[14]==0:
                reward = -ROBOT_SIDE_MARKER_REWARD
                point_list[14] = 1
                return point_list,reward

    return point_list,reward


def is_rewatd_circle_in(rx,ry,reward_circle_list):

    rp = Point(rx,ry)
    reward = 0

    
    #m1
    l1 = Line(Point(125,50),Point(111,50))
    l2 = Line(Point(125,50),Point(125,36))
    a1,b1,c1 = l1.coefficients
    a2,b2,c2 = l1.coefficients
    
    if a1*rx+b1*ry+c1 >0 and a2*rx+b2*ry+c2 < 0:

        if distance(rx,ry,125,50) <=  REWARD_CIRCLE1 and reward_circle_list[0][0]==0:
            reward = 10
            reward_circle_list[0][0] = 1
        elif distance(rx,ry,125,50) <=  REWARD_CIRCLE2 and reward_circle_list[0][1]==0:
            reward = 10
            reward_circle_list[0][1] = 1
        elif distance(rx,ry,125,50) <=  REWARD_CIRCLE3 and reward_circle_list[0][2]==0:
            reward =10
            reward_circle_list[0][2] = 1
            
    if a1*rx+b1*ry+c1 <0 and a2*rx+b2*ry+c2 > 0 and reward_circle_list[1][0]==0:
        if distance(rx,ry,125,50) <=  REWARD_CIRCLE1 and reward_circle_list[1][0]==0:
            reward = 10
            reward_circle_list[1][0] = 1
        elif distance(rx,ry,125,50) <=  REWARD_CIRCLE2 and reward_circle_list[1][1]==0:
            reward = 10
            reward_circle_list[1][1] = 1
        elif distance(rx,ry,125,50) <=  REWARD_CIRCLE3 and reward_circle_list[1][2]==0:
            reward =10
            reward_circle_list[1][2] = 1


    #m2
    l1 = Line(Point(200,125),Point(186,125))
    l2 = Line(Point(200,125),Point(200,111))
    a1,b1,c1 = l1.coefficients
    a2,b2,c2 = l1.coefficients
    
    if a1*rx+b1*ry+c1 >0 and a2*rx+b2*ry+c2 < 0:

        if distance(rx,ry,200,125) <=  REWARD_CIRCLE1 and reward_circle_list[2][0]==0:
            reward = 10
            reward_circle_list[2][0] = 1
        elif distance(rx,ry,200,125) <=  REWARD_CIRCLE2 and reward_circle_list[2][1]==0:
            reward = 10
            reward_circle_list[2][1] = 1
        elif distance(rx,ry,200,125) <=  REWARD_CIRCLE3 and reward_circle_list[2][2]==0:
            reward =10
            reward_circle_list[2][2] = 1
            
    if a1*rx+b1*ry+c1 <0 and a2*rx+b2*ry+c2 > 0 and reward_circle_list[3][0]==0:
        if distance(rx,ry,200,125) <=  REWARD_CIRCLE1 and reward_circle_list[3][0]==0:
            reward = 10
            reward_circle_list[3][0] = 1
        elif distance(rx,ry,200,125) <=  REWARD_CIRCLE2 and reward_circle_list[3][1]==0:
            reward = 10
            reward_circle_list[3][1] = 1
        elif distance(rx,ry,200,125) <=  REWARD_CIRCLE3 and reward_circle_list[3][2]==0:
            reward =10
            reward_circle_list[3][2] = 1

            
    #m3
    l1 = Line(Point(125,200),Point(111,200))
    l2 = Line(Point(125,200),Point(125,186))
    a1,b1,c1 = l1.coefficients
    a2,b2,c2 = l1.coefficients
    
    if a1*rx+b1*ry+c1 >0 and a2*rx+b2*ry+c2 < 0:

        if distance(rx,ry,125,200) <=  REWARD_CIRCLE1 and reward_circle_list[4][0]==0:
            reward = 10
            reward_circle_list[4][0] = 1
        elif distance(rx,ry,125,200) <=  REWARD_CIRCLE2 and reward_circle_list[4][1]==0:
            reward = 10
            reward_circle_list[4][1] = 1
        elif distance(rx,ry,125,200) <=  REWARD_CIRCLE3 and reward_circle_list[4][2]==0:
            reward =10
            reward_circle_list[4][2] = 1
            
    if a1*rx+b1*ry+c1 <0 and a2*rx+b2*ry+c2 > 0 and reward_circle_list[5][0]==0:
        if distance(rx,ry,125,200) <=  REWARD_CIRCLE1 and reward_circle_list[5][0]==0:
            reward = 10
            reward_circle_list[5][0] = 1
        elif distance(rx,ry,125,200) <=  REWARD_CIRCLE2 and reward_circle_list[5][1]==0:
            reward = 10
            reward_circle_list[5][1] = 1
        elif distance(rx,ry,125,200) <=  REWARD_CIRCLE3 and reward_circle_list[5][2]==0:
            reward =10
            reward_circle_list[5][2] = 1

                
    #m4
    l1 = Line(Point(50,125),Point(36,125))
    l2 = Line(Point(50,125),Point(50,111))
    a1,b1,c1 = l1.coefficients
    a2,b2,c2 = l1.coefficients
    
    if a1*rx+b1*ry+c1 >0 and a2*rx+b2*ry+c2 < 0:

        if distance(rx,ry,50,125) <=  REWARD_CIRCLE1 and reward_circle_list[6][0]==0:
            reward = 10
            reward_circle_list[6][0] = 1
        elif distance(rx,ry,50,125) <=  REWARD_CIRCLE2 and reward_circle_list[6][1]==0:
            reward = 10
            reward_circle_list[6][1] = 1
        elif distance(rx,ry,50,125) <=  REWARD_CIRCLE3 and reward_circle_list[6][2]==0:
            reward =10
            reward_circle_list[6][2] = 1
            
    if a1*rx+b1*ry+c1 <0 and a2*rx+b2*ry+c2 > 0 and reward_circle_list[7][0]==0:
        if distance(rx,ry,50,125) <=  REWARD_CIRCLE1 and reward_circle_list[7][0]==0:
            reward = 10
            reward_circle_list[7][0] = 1
        elif distance(rx,ry,50,125) <=  REWARD_CIRCLE2 and reward_circle_list[7][1]==0:
            reward = 10
            reward_circle_list[7][1] = 1
        elif distance(rx,ry,50,125) <=  REWARD_CIRCLE3 and reward_circle_list[7][2]==0:
            reward =10
            reward_circle_list[7][2] = 1


    return reward,reward_circle_list


            
        
def run(t,r_action,e_action,rx,ry,r_theta,ex,ey,e_theta,point_list,reward_circle_list):

    t += TIME_STEP
    
    #自機の動き
    rx,ry,r_theta,collision_flag = robot_move(r_action,rx,ry,r_theta,ex,ey)
    #敵機の動き
    ex,ey,e_theta,e_collision_flag = robot_move(e_action,ex,ey,e_theta,rx,ry)

    #自分の得点計算
    point_list,reward = point_check(rx,ry,r_theta,ex,ey,e_theta,point_list)

    #相手の得点計算
    point_list,_ = point_check(ex,ey,e_theta,rx,ry,r_theta,point_list,enemy_flag=-1)

    #試合終了判定
    #オブジェクト部分の配列を分割し，1と-1の数を数える
    marker_point_list = point_list[0:12]
    robot_point_list = point_list[12:]

    my_point = np.count_nonzero(marker_point_list==1)
    enemy_point = np.count_nonzero(marker_point_list==-1)
    #残りの配列の得点を加える
    if robot_point_list[0] == 1:
        enemy_point+=3
    if robot_point_list[1] == 1:
        enemy_point+=5
    if robot_point_list[2] == 1:
        enemy_point+=3
    if robot_point_list[3] == 1:
        my_point+=3
    if robot_point_list[4] == 1:
        my_point+=5
    if robot_point_list[5] == 1:
        my_point+=3
    #どちらかが16ならば終了
    if abs(my_point-enemy_point) >= 10 or t >= 180:
        terminal = True
    else:
        terminal = False

    if collision_flag == 1:
        reward = -1
    if reward == 0:
        reward,reward_circle_list = is_rewatd_circle_in(rx,ry,reward_circle_list)
    if len(np.where(reward_circle_list==-1)[0]):

        check_kouhox = np.where(reward_circle_list==-1)[0]
        check_kouhoy = np.where(reward_circle_list==-1)[1]

        for i in len(check_kouhox):
            if reward_circle_list[check_kouhox[i]][check_kouhoy[i]] ==1:
                reward_circle_list[check_kouhox[i]][check_kouhoy[i]] = 0

        
    return reward,terminal,point_list,t,rx,ry,r_theta,ex,ey,e_theta,point_list,reward_circle_list


if __name__ =='__main__':

    rx = 150
    ry = 150
    point_list = np.zeros(18)
    reward,terminal,point_list,t,rx,ry,r_theta,ex,ey,e_theta,point_list,reward_circle_list = run(0,0,0,rx,ry,1,0,0,1,point_list,np.zeros(1))

    print(reward)

    observation = lim.learn_image_maker(rx,ry,1,0,0,point_list[0],point_list[1],point_list[2],point_list[3],point_list[4],point_list[5],point_list[6],point_list[7],point_list[8],point_list[9],point_list[10],point_list[11],point_list[12],point_list[13],point_list[14],point_list[15],point_list[16],point_list[17])
    import cv2
    cv2.imwrite('test.png',observation)
