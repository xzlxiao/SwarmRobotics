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
from Simulation.ComObjectCollection import *

_W = 0.9
_C1 = 0.5
_C2 = 0.7

class ComRobotPSO(ComRobot):
    def __init__(self, pos):
        super(ComRobotPSO, self).__init__(pos)
        self.mPopulation = {}       # 鸟群      感知到的族群及其坐标
        self.mPopulation_agents = []    # 鸟群  对象
        self.mFood = []             # 食物      目标
        self.mFitness = 0.0
        self.mBestFitness = -100000.0
        self.mPSO_speed = np.array([.0, .0, .0])
        self.mBestFitAgent = None 
        self._mBestPosition = np.array([.0, .0, .0])
        self.best_pos = self.pos
        self.mW = _W
        self.mC1 = _C1 
        self.mC2 = _C2
        self.mTargetLineLen = 300.0       # 目标线的长度
        
        self.mObjectType = "ComRobotPSO"       # 用于标识当前物体类别
        self.mFoodName = "ComFish"         # 设定用作食物的目标
        self.mProcessedInfo[self.mObjectType] = {}

    @property
    def best_pos(self):
        if self.mRobotType == '2D':
            return self._mBestPosition[0:2]
        elif self.mRobotType == '3D':
            return self._mBestPosition[0:3]
        else:
            return self._mBestPosition

    @best_pos.setter
    def best_pos(self, value):
        if self.mRobotType == '2D':
            self._mBestPosition[0:2] = np.array(value[0:2], dtype=np.float32)
        elif self.mRobotType == '3D':
            self._mBestPosition[0:3] = np.array(value[0:3], dtype=np.float32)
        else:
            self._mBestPosition = np.array(value, dtype=np.float32)

    def pso(self):
        if self.mFitness > self.mBestFitness:
            self.mBestFitness = self.mFitness
            self.best_pos = self.pos

        for bird in self.mPopulation_agents:
            # 查看是否需要更新全局最优
            if self.mBestFitAgent is None: 
                self.mBestFitAgent = bird
            if bird.mFitness > self.mBestFitAgent.mFitness:
                self.mBestFitAgent = bird

        # if self.isStopping():       # 只有当机器人处于停止状态，才重新选择目标
            # 更新速度
        speed = self.mPSO_speed 
        w = self.mW
        c1 = self.mC1
        c2 = self.mC2
        lBestPosition = self.best_pos
        self_position = self.pos

        best_agent_position = None
        if self.mBestFitAgent is not None:
            best_agent_position = self.mBestFitAgent.pos
        if best_agent_position is not None:
            speed = w * speed + c1 * np.random.rand() * (lBestPosition - self_position) + c2 * np.random.rand() * (best_agent_position - self_position)
            
            self.mPSO_speed = speed
        # if self.mId == 0:
        #     print('target', utils.unitVector(self.mPSO_speed)*self.mTargetLineLen)
        #     print('mPSO_speed', self.mPSO_speed)
        self.setTarget(self.mPos + self.mPSO_speed)

    def update(self):
        # if (self.pos == self.target).all():
        #     self.chooseRandotarget()
        self.sense()
        self.processInfo()
        # if self.mId == 0:
        #     best_id = -1
        #     if self.mBestFitAgent is not None:
        #         best_id = self.mBestFitAgent.mId
        #     print(self.pos, self.mFitness, self.mBestFitness, best_id)
        # # 跟新适应度
        # self.mFitness = self.getPosFit(self.mPos)
        # 查看是否需要更新经验最优
        self.pso()
        
        self.move()
        
    def isStopping(self):
        if np.linalg.norm(self.mPSO_speed) < 0.0001:
            return True 
        else: 
            return False
    # def move(self):
    #     self.mPos += self.mPSO_speed

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
        self.mFitness = self.getPosFit(self.pos)
        self.mBestFitness = self.getPosFit(self.best_pos)
        if self.getInformationState() == 'global':
            self.mFood = getPosByType(self.mFoodName)
            self.mPopulation_agents = getObjectByType(self.mObjectType)
            for agent in self.mPopulation_agents:
                self.mPopulation[agent.mId] = agent.pos
        elif self.getInformationState() == 'local':
            self.mFood = self.mProcessedInfo[self.mFoodName].values()
            self.mPopulation = self.mProcessedInfo['Pos']
        elif self.getInformationState() == 'no':
            self.mFood = self.mProcessedInfo[self.mFoodName].values()
            self.mPopulation = self.mProcessedInfo[self.mRobotType]
        else:
            raise
        # 添加更新自身fitness的算法
        # self.mFitness = self.getPosFit(self.pos)
        # self.mFood = self.mProcessedInfo['ComFish'].values()
        # self.mPopulation = self.mProcessedInfo[self.mObjectType]

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