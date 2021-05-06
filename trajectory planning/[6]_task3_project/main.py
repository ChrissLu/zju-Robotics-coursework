from vision import Vision
from action import Action
from debug import Debugger
import time
import math
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np

from dwa import config,dwa_control,motion
from myrrt import RRT_STAR


def get(goal,trajectory,x):
    while True:
        # Do something(path planning)
        debugger.draw_circle(goal[0], goal[1])
        
        if goal==[-2400,-1500] or goal==[2400,1500]: 
            # robot parameter in final track
            action.sendCommand(vx=300,vy=0,vw=x[4])
            time.sleep(config.dt*0.5)
            config.max_speed = 300  # [m/s]
            config.max_yaw_rate = 900.0 * math.pi / 180.0  # [rad/s]
            config.max_accel = 400  # [m/ss]
            config.max_delta_yaw_rate = 900.0 * math.pi / 180.0  # [rad/ss]
            config.v_resolution = 12 # [m/s]
            config.yaw_rate_resolution = 2.4 * math.pi / 180.0  # [rad/s]
            dist_judge = 100

        else:
             # robot parameter
            config.max_speed = 500  # [m/s]
            config.max_yaw_rate = 1200.0 * math.pi / 180.0  # [rad/s]
            config.max_accel = 550  # [m/ss]
            config.max_delta_yaw_rate = 1200.0 * math.pi / 180.0  # [rad/ss]
            config.v_resolution = 13 # [m/s]
            config.yaw_rate_resolution = 3.2 * math.pi / 180.0  # [rad/s]
            dist_judge = 200

        x=np.array([vision.my_robot.x, vision.my_robot.y, vision.my_robot.orientation, x[3], x[4]])
        print(x[0],x[1],x[2],x[3],x[4])

        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal<=750 and goal!=[-2400,-1500] and goal!=[2400,1500]:
            break

        ob = config.ob
        for i in range(0,16):
            ob[i]=[vision.my_robot_yellow[i].x, vision.my_robot_yellow[i].y]
            
        u, predicted_trajectory = dwa_control(x, config, goal, ob)

        action.sendCommand(vx=u[0], vy=0, vw=u[1])
        time.sleep(config.dt)
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history
        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= dist_judge:
            print("Goal!!")
            break

    return trajectory,x

def rrt_main(goal):
    obstacle_list=list()
    for i in [0,1,2,3,5,7,9,11,13,14,15]:
        obstacle_list.append((vision.my_robot_yellow[i].x, vision.my_robot_yellow[i].y,300))
    print("******************")
    print(obstacle_list)
    print("******************")

    start = [vision.my_robot.x , vision.my_robot.y]
    #rrt = RRT_STAR(start=start, goal=goal, obstacle_list=obstacle_list,rand_area=[-4000, 4000])
    rrt = RRT_STAR(start=start, end=goal, obstacle_list=obstacle_list)

    path_x=list()
    path_y=list()
    path,nodelist = rrt.planning()
    for i in range(len(path)):
        path_x.append(path[i][0])
        path_y.append(path[i][1])

    debugger.draw_all(path_x,path_y,nodelist)
    time.sleep(1)
    return path

def onetime(j):
    if j%2==1:
        x=np.array([2400,1500,math.pi,vision.my_robot.vel_x,0.0])
        goal = [-2400, -1500]
    else:
        x=np.array([-2400,-1500,math.pi,vision.my_robot.vel_x,0.0])
        goal = [2400, 1500]
    trajectory=np.array(x)

    path=rrt_main(goal)
    
    if j!=0:
        for i in range(len(path)):
            trajectory,x = get(path[len(path)-i-1],trajectory,x)

        action.sendCommand(vx= 0, vy=0, vw=0)
        time.sleep(config.dt)
    
        tx=trajectory[:,0]
        ty=trajectory[:,1]
        print(len(trajectory))
        debuggerT.draw_alltra(tx,ty)
        time.sleep(3)
        print("Done") 


if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    debuggerT=Debugger()
    debuggerG=Debugger()

    print(__file__ + " start!!")
        # goal position [x(m), y(m)]
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    for j in range(11):
        onetime(j)