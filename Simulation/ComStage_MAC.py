# -*- coding: utf-8 -*-

import math
import sys
isCupy = False
import random
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import copy
import matplotlib.pyplot as plt
from matplotlib import cbook
from matplotlib import cm
from matplotlib.colors import LightSource
import networkx as net
from Common.DrKDtree import KDtree
from Common.utils import *
import Common.settings as settings
from Simulation.ComObjectCollection import *
from Simulation.ComObject import ComObject
from Simulation.ComStage import ComStage
from Simulation.ComRobotAF_MAC import ComRobotAF_MAC


class ComStageMAC(ComStage):
    nowTime = 0  # 当前时间点记录

    def __init__(self):
        super().__init__()
        self.numRobot = 100  # 实验最大多少个机器人

        self.frameHZ = 10  # 一秒多少帧
        self.formerSlots = 20  # 一帧多少正式槽
        self.preSlots = 5  # 一帧多少预备槽

        self.rbtID_Time = []  # 这个数组里面，为每个slot定义一个数组，里面存放在该时刻发消息的机器人id
        for i in range(self.formerSlots + self.preSlots):
            self.rbtID_Time.append(set())

        '''
        SLOT-CSMA: slot base CSMA
        BASE-TDMA: base TDMA
        C-MSTDMA: cross-layer multi-slots TDMA
        C-CMSTDMA: cluster based cross-layer multi-slots TDMA
        '''
        self.macProtocol = 'base-TDMA'

        # 测试指标需要的变量
        self.acesSuccessFlag = 0  # 机器人成功接入网络，再生成新的机器人进来。
        self.accessCountHistory = []  # 储存每个机器人接入时间

        # 以下变量就是为了构建地图，对通信本身没啥用
        self.map = []
        self.leaveHistory = []
        self.linenum = 12
        self.scale = self.mEnvSize[0]

    def initMap(self):
        """Method to initialize the map of the robot's environment.
        """
        # Create an empty map as a list of lists.
        self.map = [[] for i in range(self.linenum-2)]
        
        # Iterate over the rows and columns of the map.
        for i in range(1, self.linenum - 2):
            for j in range(1, self.linenum - 2):
                # Calculate the x and y coordinates of the cell.
                x = i/self.linenum * self.scale - self.scale/2
                y = j/self.linenum * self.scale - self.scale/2
                
                # Add a dictionary representing the cell to the map.
                self.map[i-1].append({'status': 'idle', # Can be idle (空闲), waiting (等待机器人到位), busy (占用), useless (该位置无法接入).
                                    'pos': (x, y), # The position of the cell.
                                    'angle': None, # The angle at which the robot should approach the cell.
                                    'slots': set()}) # A set of indices of slots in the cell.


    def updateMap(self, robot: ComRobotAF_MAC):
        """Method to update the robot's map with the current status of a given robot.

        Args:
            robot (ComRobotAF_MAC): A robot object representing the robot whose status is being updated.
        """

        # If the robot is leaving, mark its location as idle and update the relevant information in the map.
        if robot.status == 'leaving':
            i = round((robot.mPos[0] + self.scale/2)/self.scale * self.linenum - 1)
            j = round((robot.mPos[1] + self.scale/2)/self.scale * self.linenum - 1)
            self.leaveHistory.append((i, j))
            self.map[i][j]['status'] = 'idle'
            self.map[i][j]['angle'] = robot.mDirection
            self.map[i][j]['slots'] = robot.slots
            self.leaveHistory.append((i, j))
            return
        
        # If the robot is entering, determine whether it needs to pick up from the leave history or find a new location to enter.
        if robot.status == 'entering':
            if robot.mTarget[2] == 400:
                # If there are locations in the leave history, pick the most recent one and set the robot's target accordingly.
                if len(self.leaveHistory) > 0:
                    i, j = self.leaveHistory.pop()
                    robot.mTarget = np.array([self.map[i][j]['pos'][0],
                                            self.map[i][j]['pos'][1],
                                            500], dtype=float)
                    robot.p_acsInfo[robot.mId] = {'type': 'slot-rpl', 'slots': self.map[i][j]['slots']}
                    self.map[i][j]['status'] = 'waiting'
                    return
                else:
                    # If there are no locations in the leave history, find a new location for the robot to enter.
                    for i in range(5):
                        mi = 4 - i
                        ma = 5 + i
                        for p in range(mi, ma + 1):
                            if self.map[mi][p]['status'] == 'idle':
                                self.map[mi][p]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[mi][p]['pos'][0],
                                                        self.map[mi][p]['pos'][1],
                                                        500], dtype=float)
                                return
                        for p in range(mi, ma + 1):
                            if self.map[ma][p]['status'] == 'idle':
                                self.map[ma][p]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[ma][p]['pos'][0],
                                                        self.map[ma][p]['pos'][1],
                                                        500], dtype=float)
                                return
                        for p in range(mi, ma + 1):
                            if self.map[p][mi]['status'] == 'idle':
                                self.map[p][mi]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[p][mi]['pos'][0],
                                                        self.map[p][mi]['pos'][1],
                                                        500], dtype=float)
                                return
                        for p in range(mi, ma + 1):
                            if self.map[p][ma]['status'] == 'idle':
                                self.map[p][ma]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[p][ma]['pos'][0],
                                                        self.map[p][ma]['pos'][1],
                                                        500], dtype=float)
                                return
            else:
                return


    def update(self):
        """This method updates the state of robots and the network.
        """

        # Creating chart coordinate system
        if self.mAx:  # Preparing to draw a 3D graph
            self.mAx.cla()  # Clear old plot
            self.mAx.grid(False)  # Disable grid lines
            self.mAx.set_xlim(-self.mEnvSize[0], self.mEnvSize[0])  # Set limits on x axis
            self.mAx.set_ylim(-self.mEnvSize[1], self.mEnvSize[1])  # Set limits on y axis
            self.mAx.set_zlim(-self.mEnvSize[2], self.mEnvSize[2])  # Set limits on z axis
        if self.mGraphAx:  # Preparing to draw a connected graph
            self.mGraphAx.cla()  # Clear old plot

        # Clarifying the network structure, determining the directed connected graph
        self.mRobotList.clear_edges()  # Remove all edges between nodes in the graph
        for ind, pos in enumerate(self.mRobotPosList):
            robot = list(self.mRobotList.nodes)[ind]
            robot.refresh()  # Clear robot's temporary variables
            if robot.isCommunicating:
                robot.sense()  # Get robots within communication range
        for ind, pos in enumerate(self.mRobotPosList):
            robot = list(self.mRobotList.nodes)[ind]
            if robot.isCommunicating:
                for rbt in robot.robots_inRs:
                    if robot in rbt.out_robots:
                        robot.in_robots.append(rbt)  # Get incoming neighbors
            edges_color = []
        for ind, pos in enumerate(self.mRobotPosList):
            robot = list(self.mRobotList.nodes)[ind]
            if robot.isCommunicating:
                robot.in_robots, rept_robot = self.judgeConflict(robot.in_robots)  # Judge conflicts between robots with overlapping communication range
                for rpt_rbt in rept_robot:
                    rpt_rbt.out_robots.remove(robot)
                    self.mRobotList.add_edge(rpt_rbt, robot)  # Add an edge between two given nodes
                    edges_color.append('r')
                if len(rept_robot) > 0:
                    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                    print(f'out: {[rbt.mId for rbt in rept_robot]} in: {robot.mId}')
                    print(f'id: {robot.mId} in neighbors: {[rbt.mId for rbt in robot.in_robots]}')
                    for r in rept_robot:
                        print(f'id: {r.mId} out neighbors: {[rbt.mId for rbt in r.out_robots]}')
                    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                for rbt in robot.in_robots:
                    self.mRobotList.add_edge(rbt, robot)  # Add an edge between two given nodes
                    edges_color.append('b')

        # Perform the correct communication action
        print(f'nowTime: {ComStageMAC.nowTime} with id: {self.rbtID_Time[ComStageMAC.nowTime]} can send msg.')
        for vrobot in self.mRobotList.nodes:
            for rbt in vrobot.out_robots:
                vrobot.communicateWith(rbt, ComStageMAC.nowTime)  # Transmit information to robots within communication range
            if vrobot.isCommunicating and (ComStageMAC.nowTime in vrobot.slots):
                vrobot.sendMsg.clear()  # Clear message sent
                vrobot.hasSendLeaFlag = 2  # Set the leaving flag for robots that are about to leave the network. These robots can only leave the network once this flag is set.

        # Perform robot's self-update program, merge data
        for ind, vrobot in enumerate(self.mRobotList.nodes):
            self.updateMap(vrobot)  # Update the robot's view of its environment

            vrobot.update()  # Update the robot's status

            self.mRobotPosList[ind] = vrobot.mPos  # Add new position info to the list of robot positions
            vrobot.draw(self.mAx)  # Draw the robot on the chart

        # Display graph
        if self.isPlotGraph and len(self.mRobotList.nodes) > 1:
            graph_pos = net.circular_layout(self.mRobotList)  # Position the nodes around a circle
            net.draw(self.mRobotList, graph_pos, ax=self.mGraphAx,
                    with_labels=False, node_size=30, edge_color=edges_color)  # Draw the graph

        edges_color.clear()

        if self.count == 35:  # Check if count is equal to 35
            self.acesSuccessFlag = 1

        if self.acesSuccessFlag:  # If Aces Success Flag is set
            self.calcNetworkMetrics()  # Calculate network metrics
            print('**********a new robot has generate*****************')
            range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
            range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
            x = random.uniform(range_x[0], range_x[1])
            y = random.uniform(range_y[0], range_y[1])  # Generate new robot at random location
            newrobot = ComRobotAF_MAC((x, y, 400), stage)
            newrobot.isPlotTrail = True
            newrobot.setRadius(2)
            stage.addRobot(newrobot)  # Add new robot to the stage
            self.acesSuccessFlag = 0

        ComStageMAC.nowTime += 1  # Increment nowTime
        if ComStageMAC.nowTime >= (self.formerSlots + self.preSlots):
            ComStageMAC.nowTime = 0

        if self.count % 300 == 0:  # Check if count modulo 300 is equal to 0
            print(self.accessCountHistory)  # Print access count history


    def run(self):
        """
        A brief description of what the run() method does.
        """
        super().run()

    def judgeConflict(self, in_rbt):
        """This method checks whether there is a conflict between robots trying to communicate with me.
        
        Args:
            in_rbt (list): A list of robots that are trying to communicate with me.
            
        Returns:
            tuple: A tuple containing two lists - 
                1. A list of robots that are not conflicting, and 
                2. A list of robots that are conflicting with each other.
        """
        if len(in_rbt) == 0:
            return [], []   # If there are no robots, return an empty tuple
            
        rept_rbt = []  # Create an empty list to store the conflicting robots
        in_rbt_temp = []  # Create an empty list to store the non-conflicting robots
        
        # Get the IDs of all the robots trying to communicate with me at the current time step
        in_rbt_id = [robot.mId for robot in in_rbt]
        rept_set = set(in_rbt_id) & set(self.rbtID_Time[ComStageMAC.nowTime])
        
        if len(rept_set) > 1:  
            # If there are more than 1 robots trying to communicate with me at the same time step, it's a conflict
            
            # Identify the robots that are conflicting and add them to the 'rept_rbt' list
            # Add the non-conflicting robots to the 'in_rbt_temp' list
            for rbt in in_rbt:
                if rbt.mId in rept_set:
                    rept_rbt.append(rbt)
                else:
                    in_rbt_temp.append(rbt)
                    
        else:
            # If there is only 1 robot communicating with me, it's not a conflict
            return in_rbt, []

        # Return two lists - one with the non-conflicting robots and one with the conflicting robots
        return in_rbt_temp, rept_rbt


    def calcNetworkMetrics(self):
        """This method calculates and prints network metrics for the robots in the network.
        """
        num = len(self.mRobotList)  # Get the number of robots in the list
        degrees = net.degree_histogram(self.mRobotList)  # Calculate the degree distribution of the network
        print('robots num: ', num, ' with degrees: ', degrees)  # Print the number of robots and their degrees
        
        clusters = net.average_clustering(self.mRobotList)  # Calculate the average clustering coefficient of the network
        print('robots num: ', num, ' with clusters: ', clusters)  # Print the number of robots and their average clustering coefficient
        
        shortpaths = net.average_shortest_path_length(self.mRobotList)  # Calculate the average shortest path length of the network
        print('robots num: ', num, ' with shortest path: ', shortpaths)  # Print the number of robots and their average shortest path length



if __name__ == "__main__":
    stage = ComStageMAC()
    stage.initMap()
    # 初始化4个机器人作为原始机器人
    robot0 = ComRobotAF_MAC((stage.map[4][4]['pos'][0], stage.map[4][4]['pos'][1], 500), stage)
    stage.map[4][4]['status'] = 'busy'
    robot0.setDirection(60/180*math.pi)
    robot1 = ComRobotAF_MAC((stage.map[4][5]['pos'][0], stage.map[4][5]['pos'][1], 500), stage)
    stage.map[4][5]['status'] = 'busy'
    robot1.setDirection(-40/180*math.pi)
    robot2 = ComRobotAF_MAC((stage.map[5][4]['pos'][0], stage.map[5][4]['pos'][1], 500), stage)
    stage.map[5][4]['status'] = 'busy'
    robot2.setDirection(135/180*math.pi)
    robot3 = ComRobotAF_MAC((stage.map[5][5]['pos'][0], stage.map[5][5]['pos'][1], 500), stage)
    stage.map[5][5]['status'] = 'busy'
    robot3.setDirection(-100/180*math.pi)
    robot0.slots.add(0)
    robot0.slots.add(1)
    robot1.slots.add(2)
    robot1.slots.add(3)
    robot2.slots.add(4)
    robot2.slots.add(5)
    robot3.slots.add(6)
    robot3.slots.add(7)
    robotlist = [robot0, robot1, robot2, robot3]
    for robot in robotlist:
        robot.setRadius(2)
        robot.status = 'online'
        robot.resFlag = 1
        robot.hasSlotChangedFlag = 1
        robot.p_acsInfo[robot.mId]['slots'] = copy.deepcopy(robot.slots)
        for s in robot.slots:
            stage.rbtID_Time[s].add(robot.mId)
    stage.addRobot(robot0)
    stage.addRobot(robot1)
    stage.addRobot(robot2)
    stage.addRobot(robot3)

    # for i in range(2):
    #     range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
    #     range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
    #     range_z = (-stage.mEnvSize[2], stage.mEnvSize[2])
    #     x = random.uniform(range_x[0], range_x[1])
    #     y = random.uniform(range_y[0], range_y[1])
    #     z = random.uniform(range_z[0], range_z[1])
    #     robot = ComRobotAF_MAC((x, y, 400), stage)
    #     # robot.mTarget = np.array([s5[i], s6[i], 500], dtype=float)
    #     robot.isPlotTrail = True
    #     stage.addRobot(robot)

    # robot = ComRobotAF_MAC((0, 0, 400), stage)
    # robot.setDirection(-175 / 180 * math.pi)
    # robot1 = ComRobotAF_MAC((-10, 100, 400), stage)
    # robot.setRadius(2)
    # robot1.setRadius(2)
    # stage.addRobot(robot)
    # stage.addRobot(robot1)

    stage.run()
    print(stage.accessCountHistory)