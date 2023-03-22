# -*- coding: utf-8 -*-

from Simulation.ComRobot import ComRobot 
from Simulation.ComObject import ComObject 
import copy
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Common import settings
from Common.DrKDtree import KDtree
import Common.settings as mySettings 
import random
from Common import utils


AF_SPEED = 1            # 每次移动距离
AF_MAXPREYNUM = 50     # 每次最大进行觅食尝试的次数
AF_POPULATIONNUM = 20   # 人工鱼数量
AF_FOODSIZE = 5         # 最大食物数量
AF_MAXITERNUM = 1000    # 最大迭代次数
AF_INTERVAL = 0.05      # 两次迭代间隔的时间
AF_SENSEDIST = 50       # 感知距离
AF_MAXCROWDED = 1 / 10  # 拥挤度因子, 分母代表人工鱼的数目
AF_GETFOODDIST = 1      # 找到食物的最小距离



class ComRobotAF(ComRobot):
    def __init__(self, pos):
        super(ComRobotAF, self).__init__(pos)
        self.mPopulation = {}       # 鱼群      感知到的族群及其坐标
        self.mFood = []             # 食物      目标
        self.mFitness = 0.0
        self.mMaxPreyNum = AF_MAXPREYNUM
        self.mMaxCrowded = AF_MAXCROWDED
        self.mFollowedAgentID = None
        self.mObjectType = "ComRobotAF"       # 用于标识当前物体类别
        self.mFoodName = "ComFish"         # 设定用作食物的目标

    def update(self):
        # if (self.pos == self.target).all():
        #     self.chooseRandotarget()
        self.sense()
        self.processInfo()
        
        if len(self.mPopulation) > 0:
            self.mMaxCrowded = 1 / len(self.mPopulation)
        else:
            self.mMaxCrowded = 0
        if self.isStopping():       # 只有当机器人处于停止状态，才重新选择目标
            # 试探聚群和跟随
            
            swarm_fitness = self.swarm()
            follow_fitness = self.follow()
            
            # 比较试探的结果
            if swarm_fitness > self.mFitness or follow_fitness > self.mFitness:
                # 选出最优的决策，执行
                if swarm_fitness > follow_fitness:
                    self.swarm()
                    # print("swarm")
                else:
                    self.follow()
                    # print("follow")
            else:  # 聚群和跟随都不合适则执行觅食
                prey_fitness = self.prey()
                # print("prey_fitness: %f"%prey_fitness)
        
        self.move()
        
        
    @staticmethod
    def randomTrue(probability=0.5):
        """
        随机真假
        :param probability:  结果为True的概率
        :return:
        """
        if np.random.rand() < probability:
            return True
        else:
            return False

    def sense(self):
        super().sense()
        # 添加更新自身fitness的算法
        # self.mFitness = self.getPosFit(self.pos)
        # self.mFood = self.mProcessedInfo['ComFish'].values()
        # self.mPopulation = self.mProcessedInfo[self.mObjectType]
    
    def prey(self):
        """
        觅食算子，若达到最大觅食尝试尝试次数，目标适应度依然小于当前适应度，则返回随机位置
        :return:
        """
        for _ in range(self.mMaxPreyNum):
            pos = self.getRandomSensePos()
            fitness = self.getPosFit(pos)
            if fitness > self.mFitness:
                # print("prey")
                self.target = pos
                return fitness
        pos = self.getRandomSensePos()
        self.target = pos
        self.mFollowedAgentID = None
        # print("random move")
        return self.getPosFit(pos)

    def swarm(self):
        """
        聚群算子，计算聚群目标，计算目标适应度
        :return: 目标适应度
        """
        # 若人工鱼数量大于0，计算中心点
        if len(self.mPopulation) > 0:
            center = np.array([0.0, 0.0, 0.0], dtype=np.float32)
            for agent_pos in self.mPopulation.values():
                center += agent_pos
            center /= len(self.mPopulation)
            if center[0] > settings.CS_ENVSIZE[0]:
                center[0] = settings.CS_ENVSIZE[0]
            if center[1] > settings.CS_ENVSIZE[1]:
                center[1] = settings.CS_ENVSIZE[1]
            if center[2] > settings.CS_ENVSIZE[2]:
                center[2] = settings.CS_ENVSIZE[2]
            if center[0] < 0:
                center[0] = 0
            if center[1] < 0:
                center[1] = 0
            if center[2] < 0:
                center[2] = 0

            # 计算该点适应度
            fitness = self.getPosFit(center)
            # print("swarm_fitness %f"%fitness)
            
            if fitness > self.mFitness and not self.isCrowded(fitness, self.mFitness, center):
                self.target = center
                return fitness
        # 若人工鱼数量等于0，或适应度无法改善，或过于拥挤

        self.mFollowedAgentID = None
        return -1

    def getAgentsInRangeOfPos(self, pos: np.ndarray, r: float):
        """
        获得点pos半径range内的Agent
        :return: [[id: pos], [id: pos], ...]
        """
        agents_pos_group = list(self.mPopulation.values())
        agents_keys_group = list(self.mPopulation.keys())
        kd_tree = KDtree(agents_pos_group)
        inds, _ = kd_tree.query_radius(pos, r)
        inds = inds[0]
        return [(agents_keys_group[ind], agents_pos_group[ind]) for ind in inds]

    def follow(self):
        """
        追尾算子，计算追尾目标，计算目标适应度
        1、找到感知范围内适应度最大的人工鱼
        2、如果适应度比自己大，且不拥挤
        3、则向该人工鱼移动一步
        :return: 目标适应度
        """
        # 找到感知范围内适应度最大的人工鱼
        agent_max_pos = None
        agent_max_id = None
        fitness_max = 0
        for agent_id, agent_pos in self.mPopulation.items():
            # fitness = self.getPosFit(fish.pos, food_sense)
            fitness = self.getPosFit(agent_pos)
            if fitness > fitness_max:
                fitness_max = fitness
                agent_max_pos = agent_pos
                agent_max_id = agent_id
        # 如果适应度比自己大
        # print("follow_fitness %f"%fitness_max)
        if fitness_max > self.mFitness and agent_max_pos is not None:
            if self.isCrowded(fitness_max, self.mFitness, agent_max_pos):  # 感知该人工鱼的拥挤度，如果拥挤度不大，则向该人工鱼移动一步
                self.target = agent_max_pos
                return fitness_max
        self.mFollowedAgentID = agent_max_id
        # 否则返回-1
        return -1

    def isCrowded(self, target_fitness, current_fitness, target_pos):
        """
        判断是否拥挤
        :param target_fitness:      目标适应度
        :param current_fitness:     当前适应度
        :param target_pos:          目标坐标
        :return:                    是否拥挤
        """
        agent_num_in_range = len(self.getAgentsInRangeOfPos(target_pos, mySettings.CS_CROWDEDRANGE))

        if agent_num_in_range == 0:
            return False
        elif (target_fitness / agent_num_in_range) < (self.mMaxCrowded * current_fitness):
            return True
        else:
            return False

    def getPosFit(self, position):
        """
        适应度计算
        :param position:
        :return: 适应度
        """
        fitness = 0.0
        
        if len(self.mFood) > 0:
            for food_pos in self.mFood:
                fitness_tmp = 1 - (np.linalg.norm(np.array(position, dtype=np.float32) - food_pos, ord=2) / self.mSenseDistance)
                # fitness_tmp = 1 / (np.linalg.norm(np.array(position, dtype=np.float32) - food_pos, ord=2) + 0.000000000000001)
                if fitness_tmp > fitness:
                    fitness = fitness_tmp
        # fitness = utils.sigmoid(fitness, 0.5, 0.5)
        return fitness

    def randomSensePosFit(self):
        """
        获得当前感知范围内随机位置的适应度
        :return: 适应度, 位置
        """
        pos = self.getRandomSensePos()
        fitness = self.getPosFit(pos)
        return fitness, pos

    def getRandomSensePos(self):
        """
        获得视野内的随机位置
        :return: 位置 np.float32
        """
        if self.pos[0] - self.mSenseDistance > -mySettings.CS_ENVSIZE[0]:
            x_min = self.pos[0] - self.mSenseDistance
        else:
            x_min = -mySettings.CS_ENVSIZE[0]
        if self.pos[0] + self.mSenseDistance < mySettings.CS_ENVSIZE[0]:
            x_max = self.pos[0] + self.mSenseDistance
        else:
            x_max = mySettings.CS_ENVSIZE[0]
        if self.pos[1] - self.mSenseDistance > -mySettings.CS_ENVSIZE[1]:
            y_min = self.pos[1] - self.mSenseDistance
        else:
            y_min = -mySettings.CS_ENVSIZE[1]
        if self.pos[1] + self.mSenseDistance < mySettings.CS_ENVSIZE[1]:
            y_max = self.pos[1] + self.mSenseDistance
        else:
            y_max = mySettings.CS_ENVSIZE[1]
        if self.pos[2] - self.mSenseDistance/2 > -mySettings.CS_ENVSIZE[2]:
            z_min = self.pos[2] - self.mSenseDistance/2
        else:
            z_min = -mySettings.CS_ENVSIZE[2]
        if self.pos[2] + self.mSenseDistance/2 < mySettings.CS_ENVSIZE[2]:
            z_max = self.pos[2] + self.mSenseDistance/2
        else:
            z_max = mySettings.CS_ENVSIZE[2]

        while True:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            z = random.uniform(z_min, z_max)
            new_pos = np.array([x, y, z], dtype=np.float32)
            angle_in_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='xy') - self.mDirection     # xy平面内的偏航角度
            if self.mRobotType == '3D':
                angle_with_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='o-xy')   # 与xy平面的夹角
            if np.linalg.norm(new_pos - self.pos) < self.mSenseDistance:
                if angle_in_xy >= -self.mSenseAngle and angle_in_xy <= self.mSenseAngle:
                    if self.mRobotType == '3D':
                        if angle_with_xy >= -self.mSenseAngle and angle_with_xy <= self.mSenseAngle:                
                            return new_pos
                    else:
                        return new_pos