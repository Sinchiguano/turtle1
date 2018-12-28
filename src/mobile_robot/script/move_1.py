#!/usr/bin/env python
import os
import time
import sys
from turtle.turtle_class import Turtlebot1, Turtlebot2, Turtlebot3
import numpy as np
sys.path.insert(0, os.getcwd())

def main():
    while ((robot1.get_odometry() is None) or (robot2.get_odometry() is None) or (robot3.get_odometry() is None)):
        print("Waiting...")
        print(robot1.get_odometry())
        print(robot2.get_odometry())
        print(robot3.get_odometry())
    robot1.reset_odometry()
    robot2.reset_odometry()
    robot3.reset_odometry()
    time.sleep(1)
    
    l = 0.2
    kp = 0.3
    dis = [[0.5, 0.5], [1, 0], [-1, 0]]
    pos_init = np.array([robot1.get_globPos()[0:3],robot2.get_globPos()[0:3], robot3.get_globPos()[0:3]])
    odo = np.zeros((3, 3))
    st = time.time()
    fst = False
    v1=[0,0]

    while (not (robot1.is_shutting_down() or robot2.is_shutting_down() or robot3.is_shutting_down())):
        odo[0] = robot1.get_odometry()
        odo[1] = robot2.get_odometry()
        odo[2] = robot3.get_odometry()
        odo = odo + pos_init
        pos = odo[:, 0:2] + l*np.transpose([np.cos(odo[:, 2]), np.sin(odo[:, 2])])
        print(pos_init)
        input("Press Enter..")

        vel = np.zeros((3,2))
        vel[1] = kp*(pos[0]-dis[0]-pos[1]+pos[2]-dis[1]-pos[1])
        vel[2] = kp*(pos[1]-dis[2]-pos[2])

     #   if ((vel[2,0] + vel[2,1]) < 0.05 and fst == False):
     #       v1 = [0, 0.5]
     #       fst = True
     #   elif (fst == False):
     #       v1 = [0, 0]
        if(time.time()-st > 20):
            v1 = [0, 0.5]
        if (time.time() -st >22):
            v1 = [0, 0]
        vel[0] = v1

        move(robot1, vel[0], odo[0,2], l)
        move(robot2, vel[1], odo[1,2], l)
        move(robot3, vel[2], odo[2,2], l)

def move(robot, goal, ang, l):
    ang = [np.cos(ang), np.sin(ang)]
    dest = [[goal[0], goal[1]], [goal[1]/l, -goal[0]/l]]
    u = np.dot(dest, ang)
    #print(u)
    #raw_input("Press Enter..")
    robot.cmd_velocity(linear=u[0], angular=u[1])
    return



if __name__ == "__main__":
    robot1=Turtlebot1()
    robot2=Turtlebot2()
    robot3=Turtlebot3()
    main()
