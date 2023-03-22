
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Simulation.ComModuleBase import ComModuleBase
from Simulation.ComObjectCollection import *
import random
from Common.utils import distance

class ComModuleDWorld(ComModuleBase):
    def __init__(self, robot=None) -> None:
        super().__init__(robot=robot)
        self.query_num = 10	# 如果通信请求失败，重复新请求的次数
        self.query_posibility = 0.01	# 发起请求的概率    
        self.k = 6		# 平均链接个数
        self.links_max = 10		# 最大链接数量
        self.links_min = 2		# 最小链接数量
        self.T1 = 100.0			# 目标衰变时间
        self.d_i = 10.0		# 距离影响权重
        self.epsilon = 0.03	# 循环间隔时间
        self.alpha = 0.1	# 重连控制概率
        self.mCommunicationSigma = {}           # 通信连接稳定性
        self.mCommunicationTime = {}            # 稳定性重置时间
        self.availableRobot = None
        self.edges_value = []

    def setRobot(self, robot):
        """设置模块所属的机器人

        Args:
            robot ([type]): [description]
        """
        self.mRobot = robot

    def setAvailableComRobots(self, robots:np.ndarray):
        """设置通信范围内的机器人

        Args:
            robots ([type]): [description]
        """
        self.availableRobot = robots
    
    def update(self):
        super().update()
        new_robot_partner_id = None
        new_robot = None

        self.removeCommunicatonObjectNotInRange()

        if self.getNumOfComNeighbors() < self.k:    # 当正在通信的邻居的数量小于平均数量
            if random.random() < self.query_posibility or self.getNumOfComNeighbors() <= self.links_min:    # 如果正在通信的邻居的数量小于最小数量或者达到请求概率，则发起请求
                for i in range(self.query_num): # 多次尝试
                    if self.availableRobot is not None:
                        availableRobotList = self.availableRobot.tolist()
                        if len(availableRobotList) > 0:
                            new_robot_partner_id = random.sample(self.availableRobot.tolist(), k=1)[0]
                            if new_robot_partner_id != self.mRobot.mId and new_robot_partner_id not in self.getComNeighborsId():
                                new_robot = self.mRobot.getRobotById(new_robot_partner_id)
                                if new_robot.mNetworkModule is None:
                                    self.addCommunicatonObject(new_robot_partner_id)
                                    break
                                else:
                                    if not new_robot.mNetworkModule.reachMaxComNum():
                                        self.addCommunicatonObject(new_robot_partner_id)
                                        break
        robot_ids = self.getComNeighborsId()
        for robot_id in robot_ids:
            self.mCommunicationTime[robot_id] += self.epsilon
            t = self.mCommunicationTime[robot_id]
            decay_robot = self.mRobot.getRobotById(robot_id)

            p1_p2_dist = distance(self.mRobot.mPos,  decay_robot.mPos)
            T = self.T1 / (1 + (p1_p2_dist * self.d_i))
            self.mCommunicationSigma[robot_id] = 0.5**(t/T)
            if random.random() < self.alpha * (1 - self.mCommunicationSigma[robot_id]) * self.epsilon:
                for robot_id_iter in self.mCommunicationTime.keys():
                    self.mCommunicationTime[robot_id_iter] = 0
                for robot_id_iter in decay_robot.mNetworkModule.mCommunicationTime.keys():
                    decay_robot.mNetworkModule.mCommunicationTime[robot_id_iter] = 0
                self.removeCommunicatonObject(robot_id)

        return self.mCommunicationSigma

    def getNumOfComNeighbors(self)->int:
        """获得正在通信的邻居的数量

        Returns:
            int: 数量
        """
        return len(self.mCommunicationSigma)

    def getComNeighborsId(self):
        """获得正在通信的邻居的id

        Returns:
            dict_keys: [description]
        """
        return list(self.mCommunicationSigma.keys())

    def addCommunicatonObject(self, robot_id):
        """加入通信对象

        Args:
            robot ([type]): [description]
        """
        self.mCommunicationSigma[robot_id] = 1.0
        self.mCommunicationTime[robot_id] = 0.0
    
    def removeCommunicatonObject(self, robot_id):
        """删除通信对象

        Args:
            robot_id ([type]): [description]
        """
        self.mCommunicationSigma.pop(robot_id)
        self.mCommunicationTime.pop(robot_id)

    def removeCommunicatonObjectNotInRange(self):
        robot_ids = list(self.mCommunicationSigma.keys())
        for robot_id in robot_ids:
            if robot_id not in self.availableRobot:
                self.removeCommunicatonObject(robot_id)

    def reachMaxComNum(self)->bool:
        """是否到达最大通信限制

        Returns:
            bool: [description]
        """
        if self.getNumOfComNeighbors() < self.links_max:
            return False 
        else:
            return True