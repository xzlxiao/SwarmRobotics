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
        self.map = [[] for i in range(self.linenum-2)]
        for i in range(1, self.linenum - 2):
            for j in range(1, self.linenum - 2):
                x = i/self.linenum * self.scale - self.scale/2
                y = j/self.linenum * self.scale - self.scale/2
                self.map[i-1].append({'status': 'idle',  # idle空闲，waiting等待机器人到位，busy占用, useless该位置无法接入
                                      'pos': (x, y),
                                      'angle': None,
                                      'slots': set()})

    def updateMap(self, robot: ComRobotAF_MAC):
        if robot.status == 'leaving':
            i = round((robot.mPos[0] + self.scale/2)/self.scale * self.linenum - 1)
            j = round((robot.mPos[1] + self.scale/2)/self.scale * self.linenum - 1)
            self.leaveHistory.append((i, j))
            self.map[i][j]['status'] = 'idle'
            self.map[i][j]['angle'] = robot.mDirection
            self.map[i][j]['slots'] = robot.slots
            self.leaveHistory.append((i, j))
            return
        if robot.status == 'entering':
            if robot.mTarget[2] == 400:
                if len(self.leaveHistory) > 0:
                    i, j = self.leaveHistory.pop()
                    robot.mTarget = np.array([self.map[i][j]['pos'][0],
                                              self.map[i][j]['pos'][1],
                                              500], dtype=np.float32)
                    robot.p_acsInfo[robot.mId] = {'type': 'slot-rpl', 'slots': self.map[i][j]['slots']}
                    self.map[i][j]['status'] = 'waiting'
                    return
                else:  # 找一个位置让机器人过去，作为一个新机器人接入
                    for i in range(5):
                        mi = 4 - i
                        ma = 5 + i
                        for p in range(mi, ma + 1):
                            if self.map[mi][p]['status'] == 'idle':
                                self.map[mi][p]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[mi][p]['pos'][0],
                                                          self.map[mi][p]['pos'][1],
                                                          500], dtype=np.float32)
                                return
                        for p in range(mi, ma + 1):
                            if self.map[ma][p]['status'] == 'idle':
                                self.map[ma][p]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[ma][p]['pos'][0],
                                                          self.map[ma][p]['pos'][1],
                                                          500], dtype=np.float32)
                                return
                        for p in range(mi, ma + 1):
                            if self.map[p][mi]['status'] == 'idle':
                                self.map[p][mi]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[p][mi]['pos'][0],
                                                          self.map[p][mi]['pos'][1],
                                                          500], dtype=np.float32)
                                return
                        for p in range(mi, ma + 1):
                            if self.map[p][ma]['status'] == 'idle':
                                self.map[p][ma]['status'] = 'waiting'
                                robot.mTarget = np.array([self.map[p][ma]['pos'][0],
                                                          self.map[p][ma]['pos'][1],
                                                          500], dtype=np.float32)
                                return
            else:
                return

    def update(self):
        # 图坐标系建立
        if self.mAx:  # 画3d图的准备
            self.mAx.cla()
            self.mAx.grid(False)
            self.mAx.set_xlim(-self.mEnvSize[0], self.mEnvSize[0])
            self.mAx.set_ylim(-self.mEnvSize[1], self.mEnvSize[1])
            self.mAx.set_zlim(-self.mEnvSize[2], self.mEnvSize[2])
        if self.mGraphAx:  # 画连通图的准备
            self.mGraphAx.cla()

        # 梳理网络结构，确定有向连通图
        self.mRobotList.clear_edges()
        # kd_tree = KDtree(self.mRobotPosList)
        for ind, pos in enumerate(self.mRobotPosList):
            robot = list(self.mRobotList.nodes)[ind]
            # if robot.status != 'entering':  # 对于正在进入的机器人，等其到达指定位置上了再进行感知。
            robot.refresh()
            if robot.isCommunicating:
                # 获得通信范围内的所有机器人,获得出邻居机器人
                robot.sense()
        for ind, pos in enumerate(self.mRobotPosList):
            robot = list(self.mRobotList.nodes)[ind]
            if robot.isCommunicating:
                for rbt in robot.robots_inRs:
                    if robot in rbt.out_robots:
                        robot.in_robots.append(rbt)  # 获得入邻居机器人
        edges_color = []
        for ind, pos in enumerate(self.mRobotPosList):
            robot = list(self.mRobotList.nodes)[ind]
            if robot.isCommunicating:
                # 连通图内各个机器人通信动作冲突性判断
                robot.in_robots, rept_robot = self.judgeConflict(robot.in_robots)
                for rpt_rbt in rept_robot:
                    rpt_rbt.out_robots.remove(robot)
                    self.mRobotList.add_edge(rpt_rbt, robot)
                    edges_color.append('r')
                if len(rept_robot) > 0:
                    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                    print('out:', [rbt.mId for rbt in rept_robot], ' in:', robot.mId)
                    print('id:', robot.mId, 'in neighbors: ', [rbt.mId for rbt in robot.in_robots])
                    for r in rept_robot:
                        print('id:', r.mId, 'out neighbors: ', [rbt.mId for rbt in r.out_robots])
                    print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                for rbt in robot.in_robots:
                    self.mRobotList.add_edge(rbt, robot)
                    edges_color.append('b')

        # 执行正确通信动作
        print('nowTime: ', ComStageMAC.nowTime, ' with id:', self.rbtID_Time[ComStageMAC.nowTime], 'can send msg.')
        for vrobot in self.mRobotList.nodes:
            for rbt in vrobot.out_robots:
                vrobot.communicateWith(rbt, ComStageMAC.nowTime)
            if vrobot.isCommunicating and (ComStageMAC.nowTime in vrobot.slots):
                vrobot.sendMsg.clear()  # 发送完成后清空
                vrobot.hasSendLeaFlag = 2  # 对于要离开的机器人来说，直到这时才将离开洪泛包发出去，然后机器人update时就可以离开网络了

        # 执行机器人自身的更新程序，融合数据
        for ind, vrobot in enumerate(self.mRobotList.nodes):
            self.updateMap(vrobot)

            vrobot.update()

            # 将新的机器人位置信息填入到self.mRobotPosList中
            self.mRobotPosList[ind] = vrobot.mPos
            vrobot.draw(self.mAx)

        # 图形显示
        if self.isPlotGraph and len(self.mRobotList.nodes) > 1:
            graph_pos = net.circular_layout(self.mRobotList)  # 布置框架
            net.draw(self.mRobotList, graph_pos, ax=self.mGraphAx,
                     with_labels=False, node_size=30, edge_color=edges_color)
        edges_color.clear()

        if self.count == 35:
            self.acesSuccessFlag = 1

        if self.acesSuccessFlag:
            self.calcNetworkMetrics()
            print('**********a new robot has generate*****************')
            range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
            range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
            x = random.uniform(range_x[0], range_x[1])
            y = random.uniform(range_y[0], range_y[1])
            newrobot = ComRobotAF_MAC((x, y, 400), stage)
            newrobot.isPlotTrail = True
            newrobot.setRadius(2)
            stage.addRobot(newrobot)
            self.acesSuccessFlag = 0

        # 更新时间点
        ComStageMAC.nowTime += 1
        if ComStageMAC.nowTime >= (self.formerSlots + self.preSlots):
            ComStageMAC.nowTime = 0

        if self.count % 300 == 0:
            print(self.accessCountHistory)

    def run(self):
        super().run()

    def judgeConflict(self, in_rbt):
        if len(in_rbt) == 0:
            return [], []
        rept_rbt = []
        in_rbt_temp = []
        in_rbt_id = [robot.mId for robot in in_rbt]
        rept_set = set(in_rbt_id) & set(self.rbtID_Time[ComStageMAC.nowTime])
        if len(rept_set) > 1:  # 大于1表示有多个机器人在该时刻同时发给我
            # 记录冲突的机器人
            for rbt in in_rbt:
                if rbt.mId in rept_set:
                    rept_rbt.append(rbt)
                else:
                    in_rbt_temp.append(rbt)
        else:
            return in_rbt, []

        return in_rbt_temp, rept_rbt

    def calcNetworkMetrics(self):
        num = len(self.mRobotList)
        degrees = net.degree_histogram(self.mRobotList)  # 度分布
        print('robots num: ', num, ' with degrees: ', degrees)
        clusters = net.average_clustering(self.mRobotList)
        print('robots num: ', num, ' with clusters: ', clusters)
        shortpaths = net.average_shortest_path_length(self.mRobotList)
        print('robots num: ', num, ' with shortest path: ', shortpaths)


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
    #     # robot.mTarget = np.array([s5[i], s6[i], 500], dtype=np.float32)
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





