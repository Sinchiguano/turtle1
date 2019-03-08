import os
import threading
import time
import sys
import math
import rospy
import matplotlib.pyplot as plt
from turtle.turtle_class import Turtlebot
import numpy as np
sys.path.insert(0, os.getcwd())


class TurtleFormation(object):
    def __init__(self, numAgents=None,ledr=None, handle=None, gain=None, nei=None, displ=None, way=None, navigation=None):
        #initialize ros node
        rospy.init_node('turtlebot_node')
        #initialize the Robots according to the number of desired agents
        self.robot = []
        if (numAgents is None or numAgents<=0):
            numAgents = 2
        self.nbot = numAgents
        for ele in range(numAgents):
            self.robot.append(Turtlebot(ele + 1))
            print('Creating Turtlebot No.: {}'.format(ele+1))
            
        #initialize handle length, controler gain, neighborhoug graph, displacements, navigation mode and logs
        if(handle is None or handle <= 0):
            handle = 0.2
        if(gain is None or gain <= 0):
            gain = 0.3
        if(nei is None):
            nei = [[]]
        if(displ is None):
            displ = [[]]
        if(way is None):
            way = [[0,0]]
        if(navigation is None):
            navigation = 'cam'
        if(ledr is None or ledr < 0 or ledr>=numAgents):
            ledr = 0
        self.leader = ledr
        self.l = handle
        self.kp = gain
        self.N = nei
        self.dis = displ
        self.wps = np.asarray(way)
        self.nav = navigation
        self.log_odo = []
        self.log_cam = []
        #initialize the threading variables: Thread, exit interupt, shared data mutex
        self.thread = threading.Thread(target=self.moveTh, args=())
        self.thread.daemon = True;
        self.extInt = False
        self.connLck = threading.Lock()

    def move(self): #start the formation control thread
        self.extInt = False
        print("Starting Thread")
        self.thread.start()

    def stop(self): #signal thread to stop and wait for it to terminate
        self.extInt = True
        self.thread.join()
        print("Thread exited")

    def changeConn(self, nei, displ): #change the connection setup and displacement vectors. THREADSAFE!
        with self.connLck:
            self.N = nei
            self.dis = displ
            
    def moveTh(self): #Background thread that executes the formation control
        #wait for the subscriber to receive valid data
        rdy = False
        print("Wait for subscriber connection")
        while (not rdy):
            rdy = True
            for bot in self.robot:
                rdy = rdy and (bot.get_globPos() is not None)
                rdy = rdy and (bot.get_odometry() is not None)
        print("Starting")

        #reset odometry, just in case ;)
        #also remember initial position, not used right now, but might come in handy
        pos_init = np.zeros((self.nbot, 3))
        for idx, bot in enumerate(self.robot):
            pos_init[idx] = bot.get_globPos()
            bot.reset_odometry()
        time.sleep(1)
        
        #set up needed data structures
        running = True;
        pos_glob = np.zeros((self.nbot, 3))
        dis_curr = np.zeros(self.dis.shape)
        #save starting time
        st = time.time()
        timer = time.time()
        #initialize loop couter, and current waypoint index
        run = 0
        curr_wp=0

        print(self.wps)

        while (running and not self.extInt):
            pos_cam = np.zeros((self.nbot, 3))
            pos_odo = np.zeros((self.nbot, 3))
            #check if any of the robots is shutting down
            for bot in self.robot:
                running = running and not bot.is_shutting_down()
            #increment the loop counter
            run += 1

            #the loop is running to often and flooding the network with messages This loop simply waits to ensure a lower loop frequency
            while(time.time()-timer < 0.05):
                time.sleep(0.01)
            #reset the timer
            timer = time.time()

            #get current positions both from camera system and from odometry
            for idx, bot in enumerate(self.robot):
                pos_odo[idx] = calcPos(bot.get_odometry(), pos_init[idx])
                pos_cam[idx] = bot.get_globPos()

            #select the position according to desired navigation scheme
            if (self.nav == 'odo'):
                pos_glob = pos_odo
            elif (self.nav == 'cam'):
                pos_glob = pos_cam

            #calculate handle position according to current robot position, angle and handle length l
            pos = pos_glob[:, 0:2] + self.l*np.transpose([np.cos(pos_glob[:, 2]), np.sin(pos_glob[:, 2])])

            #log data only every 20 loops, to keep memory usage from logs small
            #then reset loop counter
            if(run == 20):
                self.log_odo.append(pos_odo)
                self.log_cam.append(pos_cam)
                run=0

            #calculate control inputs according to formation control algorithm
            # this is done threadsafe, so that the connection graph and displacement vectors can be changed during execution
            vel = np.zeros((self.nbot,2))
            with self.connLck:
                dis_curr = self.dis #rot(dis, odo[0,2])
                ndisp = 0
                for idx1, con in enumerate(self.N):
                    for idx2, rel in enumerate(con):
                        if(rel):
                            vel[idx1] += rel*self.kp*(pos[idx2]-dis_curr[ndisp]-pos[idx1])
                            ndisp += 1

            #switch to next waypoint if close enough and there is more waypoints to go
            if(np.linalg.norm(pos[0]-self.wps[curr_wp]) < 0.1):
                if(self.wps.shape[0] > curr_wp+1):
                    curr_wp += 1

            #additional control input for leader robot as designed for single robot unicycle robot movement
            alpha = 0.4/(0.2 + np.linalg.norm(self.wps[curr_wp] - pos[0]))
            vel[self.leader] += alpha*(self.wps[curr_wp]-pos[0])

            #execute drive comands for all robots
            for idx, bot in enumerate(self.robot):
                self.drive(idx, vel[idx], pos_glob[idx,2])

    def drive(self, idx, goal, ang):
        #drive function: calculates for the current position, angle, and desired heading
        #the linear and angular velocity, that will make the unicycle robot go to that point
        turn = [np.cos(ang), np.sin(ang)]
        dest = [[goal[0], goal[1]], [goal[1]/self.l, -goal[0]/self.l]]
        u = np.dot(dest, turn)
        self.robot[idx].cmd_velocity(linear=u[0], angular=u[1])
        return
    
    def plot(self, tp, src):
        #plot function: line plots work genericly for any number of agents
        #trinagle plots only work as intended for 3 agent systems
        if(tp == 'lines' or tp == 'all'):
            plt.figure(1)
            plt.plot(self.wps[:,0], self.wps[:,1], 'ro', label='Waypoints')
            if(src == 'cam' or src == 'all'):
                for idx in range(self.nbot):
                    plt.plot(np.asarray(self.log_cam)[:,idx,0], np.asarray(self.log_cam)[:,idx,1], color='#ff0000', label="R"+str(idx)+" pos from cam")
            if(src == 'odo' or src == 'all'):
                for idx in range(self.nbot):
                    plt.plot(np.asarray(self.log_odo)[:,idx,0], np.asarray(self.log_odo)[:,idx,1], color='#0000ff', label="R"+str(idx)+" pos from odo")
            plt.legend()
        if(tp == 'triangle' or tp == 'all'):
            plt.figure(2)
            plt.plot(self.wps[:,0], self.wps[:,1], 'ro', label='Waypoints')
            if(src == 'cam' or src == 'all'):
                plt.plot(np.asarray(self.log_cam)[:,self.leader,0], np.asarray(self.log_cam)[:,self.leader,1], color='#000000', label='Leader pos from cam')
                for idx in range(0, len(self.log_cam), 3):
                    plt.plot(np.asarray(self.log_cam)[idx,0:2,0], np.asarray(self.log_cam)[idx,0:2,1], color='black', marker='o')
                    plt.plot(np.asarray(self.log_cam)[idx,1:3,0], np.asarray(self.log_cam)[idx,1:3,1], color='black', marker='o')
                    plt.plot(np.asarray(self.log_cam)[idx,0:3:2,0], np.asarray(self.log_cam)[idx,0:3:2,1], color='black', marker='o')
            if(src == 'odo'):
                plt.plot(np.asarray(self.log_odo)[:,0,0], np.asarray(self.log_odo)[:,0,1], color='#440000', label='R0 pos from odo')
                for idx in range(0, len(self.log_odo), 15):
                    plt.plot(np.asarray(self.log_odo)[idx,0:2,0], np.asarray(self.log_odo)[idx,0:2,1], color='black', marker='o')
                    plt.plot(np.asarray(self.log_odo)[idx,1:3,0], np.asarray(self.log_odo)[idx,1:3,1], color='black', marker='o')
                    plt.plot(np.asarray(self.log_odo)[idx,0:3:2,0], np.asarray(self.log_odo)[idx,0:3:2,1], color='black', marker='o')
            plt.legend()
            
        plt.show()


def calcPos(pos_orig, init):
    #generic helper function, that calculates the current robot position according to the robots initial position and subsequent movement.
    #useful when using odometry, which starts at 0, but robots might have non-zero initial positions
    pos = np.zeros(3)
    pos[0:2] = init[0:2] + rot(pos_orig[0:2], init[2])
    pos[2] = pos_orig[2]+init[2]
    return pos

def rot(pt, ang):
    #generic helper function, that rotates a point or a set of points for an angle
    pos = np.zeros(pt.shape)
    c,s = math.cos(ang), math.sin(ang)
    if(pt.ndim == 2):
        pos[:,0] = pt[:,0]*c + pt[:,1]*-s
        pos[:,1] = pt[:,0]*s + pt[:,1]*c
    else:
        pos[0] = pt[0]*c + pt[1]*-s
        pos[1] = pt[0]*s + pt[1]*c
        
    return pos


