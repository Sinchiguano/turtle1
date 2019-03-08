#!/usr/bin/env python
import os
import time
import sys
import math
from turtle.turtle_class import Turtlebot1, Turtlebot2, Turtlebot3
import numpy as np
sys.path.insert(0, os.getcwd())

def main():
    rdy = False
    nbot = len(robot)
    while (not rdy):
        rdy = True
        for bot in robot:
            rdy = rdy and (bot.get_odometry() is not None)
        print("Waiting...")
    for bot in robot:
        bot.reset_odometry()
    time.sleep(1)
    
    running = True;
    l = 0.2
    kp = 0.3
    N = np.asarray([[0, 0, 0],[1, 0, 1],[0, 1, 0]])
    dis = np.asarray([[0.5, 0.5], [1, 0], [-1, 0]])
    pos_init = np.zeros((nbot, 3))
    for idx, bot in enumerate(robot):
        pos_init[idx] = bot.get_globPos()
    print(pos_init)
    odo = np.zeros((nbot, 3))
    pos_glob = np.zeros((nbot, 3))
    dis_curr = np.zeros(dis.shape)
    st = time.time()
    fst = False
    v1=[0,0.3]
    run = 0

    while (running):
        for bot in robot:
            running = running and not bot.is_shutting_down()
        run += 1

        for idx, bot in enumerate(robot):
            odo[idx] = calcPos(bot.get_odometry(), pos_init[idx])
            pos_glob[idx] = bot.get_globPos()
        dis_curr = dis#rot(dis, odo[0,2])

        pos = odo[:, 0:2] + l*np.transpose([np.cos(odo[:, 2]), np.sin(odo[:, 2])])
        if(run == 1000):
            print(pos_glob)
            diff = np.zeros((3,2));
            diff[0] = pos_glob[0][0:2]-pos_glob[1][0:2] - dis_curr[0]
            diff[1] = pos_glob[2][0:2]-pos_glob[1][0:2] - dis_curr[1]
            diff[2] = pos_glob[1][0:2]-pos_glob[2][0:2] - dis_curr[2]
            print(diff)
            run=0

        vel = np.zeros((nbot,2))
        ndisp = 0
        for idx1, con in enumerate(N):
            for idx2, rel in enumerate(con):
                if(rel):
                    vel[idx1] +=rel*kp*(pos[idx2]-dis_curr[ndisp]-pos[idx1])
                    ndisp += 1

     #   if ((vel[2,0] + vel[2,1]) < 0.05 and fst == False):
     #       v1 = [0, 0.5]
     #       fst = True
     #   elif (fst == False):
     #       v1 = [0, 0]
        if (time.time() -st >10):
            v1 = [0.3, 0]
        if (time.time()-st >20):
            v1 = [0, 0]
        vel[0] = v1

        for idx, bot in enumerate(robot):
            move(bot, vel[idx], odo[idx,2], l)

def move(robot, goal, ang, l):
    ang = [np.cos(ang), np.sin(ang)]
    dest = [[goal[0], goal[1]], [goal[1]/l, -goal[0]/l]]
    u = np.dot(dest, ang)
    #print(u)
    #raw_input("Press Enter..")
    robot.cmd_velocity(linear=u[0], angular=u[1])
    return

def calcPos(odo, init):
    pos = np.zeros(3)
    pos[0:2] = init[0:2] + rot(odo[0:2], init[2])
    pos[2] = odo[2]+init[2]
    return pos

def rot(pt, ang):
    pos = np.zeros(pt.shape)
    c,s = math.cos(ang), math.sin(ang)
    if(pt.ndim == 2):
        pos[:,0] = pt[:,0]*c + pt[:,1]*-s
        pos[:,1] = pt[:,0]*s + pt[:,1]*c
    else:
        pos[0] = pt[0]*c + pt[1]*-s
        pos[1] = pt[0]*s + pt[1]*c
        
    return pos


if __name__ == "__main__":
    robot = []
    robot.append(Turtlebot1())
    robot.append(Turtlebot2())
    robot.append(Turtlebot3())
    main()
