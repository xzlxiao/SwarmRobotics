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
        """
        This function returns the best position of the robot based on its type. If the type of the robot is 2D,
        it returns the first two elements of '_mBestPosition'. If the type of the robot is 3D, it returns the 
        first three elements of '_mBestPosition'. If the robot type is unknown or anything else, it returns 
        '_mBestPosition', whole.
        """
        # Check if the type of the robot is 2D
        if self.mRobotType == '2D':
            # Return first two elements of '_mBestPosition'
            return self._mBestPosition[0:2]
        # Check if the type of the robot is 3D
        elif self.mRobotType == '3D':
            # Return the first three elements of '_mBestPosition'
            return self._mBestPosition[0:3]
        # If the robot type is neither 2D nor 3D
        else:
            # Return '_mBestPosition'
            return self._mBestPosition


    # Define a class method as a decorator to be used as a setter.
    @best_pos.setter
    def best_pos(self, value):
        """Method to set the best position of the robot.

        Args:
            value: A list of values representing the new position for the robot.
        
        Returns:
            None.
        """
        # Check if the type of the robot is 2D.
        if self.mRobotType == '2D':
            # If true, update the first two elements of '_mBestPosition' with the given value. 
            self._mBestPosition[0:2] = np.array(value[0:2], dtype=float)
        # Check if the type of the robot is 3D.
        elif self.mRobotType == '3D':
            # If true, update the first three elements of '_mBestPosition' with the given value.
            self._mBestPosition[0:3] = np.array(value[0:3], dtype=float)
        # When the robot type is unknown or anything else,
        else:
            # Update the full '_mBestPosition' with the given value.
            self._mBestPosition = np.array(value, dtype=float)


    def pso(self):
        """Method implementing Particle Swarm Optimization algorithm.
        """    
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
        # Update the speed and position of the robot.
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
        """Method updating the state of the robot by sensing, processing information,
        updating the fitness, PSO optimization and moving.
        """
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
        """Checks if the robot's Particle Swarm Optimization (PSO) speed is very small, indicating that it is about to stop.
        
        Returns:
            A boolean value of True if the PSO speed is very small and False otherwise.
        """        
        if np.linalg.norm(self.mPSO_speed) < 0.0001:
            return True 
        else: 
            return False

        # def move(self):
        #     self.mPos += self.mPSO_speed

    @staticmethod
    def randomTrue(probability=0.5):
        """
        Randomly returns True or False based on a given probability of returning True.
        If the probability parameter is not provided, it defaults to 0.5 (50% chance).
        
        Args:
            probability (float, optional): The probability of returning True. 
        
        Returns:
            A boolean value of either True or False based on the probability argument passed into the function.
        """    
        if np.random.rand() < probability:
            return True
        else:
            return False

    def sense(self):
        """
        Updates the agent's internal state based on sensory information.

        This function makes use of several helper functions to update various internal variables, 
        such as position fitness and best position fitness. Depending on the agent's information state,
        it updates the food source and population information accordingly.

        Args:
            None
        
        Returns:
            None
        """      
        super().sense() # Calls superclass method to update position value
        self.mFitness = self.getPosFit(self.pos) # Gets current position fitness
        self.mBestFitness = self.getPosFit(self.best_pos) # Gets best position fitness
        # Updates food and population variables based on agent's information state
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
            raise # Raises an exception if information state is not recognized
        # 添加更新自身fitness的算法
        # self.mFitness = self.getPosFit(self.pos)
        # self.mFood = self.mProcessedInfo['ComFish'].values()
        # self.mPopulation = self.mProcessedInfo[self.mObjectType]

    def getAgentsInRangeOfPos(self, pos: np.ndarray, r: float):
        """
        Finds agents within a specified range of a given position.

        This function makes use of a KDTree to efficiently search for agents within a radius r of a given position.

        Args:
            pos (np.ndarray): The position around which to search for other agents.
            r (float): The radius within which to search for agents.
        
        Returns:
            A list of tuples, where each tuple contains an agent's id and position.
        """  

        # Extracts positions and keys from the population dictionary
        agents_pos_group = list(self.mPopulation.values())
        agents_keys_group = list(self.mPopulation.keys())
        # Creates a KDTree from the positions
        kd_tree = KDtree(agents_pos_group)
        # Queries the KDTree for all points within radius r of the specified position
        inds, _ = kd_tree.query_radius(pos, r)
        inds = inds[0]
        # Creates a list of tuples containing the agent's key and position for each point in the result
        return [(agents_keys_group[ind], agents_pos_group[ind]) for ind in inds]


    def getPosFit(self, position):
        """
        Calculates the fitness value of a given position.

        This function calculates the fitness value of a given position based on the distance to food positions in the environment.

        Args:
            position (_type_): The position for which to calculate the fitness value.

        Returns:
            A float representing the fitness value of the given position.
        """        
        fitness = 0.0
        
        if len(self.mFood) > 0:
            for food_pos in self.mFood:
                fitness_tmp = 1 - (np.linalg.norm(np.array(position, dtype=float) - food_pos, ord=2) / self.mSenseDistance)
                # fitness_tmp = 1 / (np.linalg.norm(np.array(position, dtype=float) - food_pos, ord=2) + 0.000000000000001)
                
                # Updates the fitness value if the distance-based contribution is greater than the current fitness
                if fitness_tmp > fitness:
                    fitness = fitness_tmp
        # fitness = utils.sigmoid(fitness, 0.5, 0.5)
        return fitness

    def randomSensePosFit(self):
        """
        Calculates the fitness value of a randomly selected position within the agent's sensing distance.

        This function selects a random position within the agent's sensing distance, calculates its fitness using the getPosFit() method,
        and returns both the fitness value and the chosen position.

        Returns:
            A tuple containing a float representing the fitness value, and an array representing the selected position.
        """        

        # Selects a random position within the agent's sensing distance
        pos = self.getRandomSensePos()
        # Calculates the fitness value of the selected position using the getPosFit() method
        fitness = self.getPosFit(pos)
        # Returns a tuple containing the fitness value and the selected position
        return fitness, pos


    def getRandomSensePos(self):
        """Method to get a random position within the robot's sensing range.

        Returns:
            new_pos (float): A random position numpy array.
        """
        
        # Determine the minimum and maximum x, y, z values for the sensing range.
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

        # Generate random x, y, z values within the sensing range.
        while True:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            z = random.uniform(z_min, z_max)
            
            # Create a new position array and calculate angle_in_xy.
            new_pos = np.array([x, y, z], dtype=float)
            angle_in_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='xy') - self.mDirection
            
            # If the robot is 3D, calculate angle_with_xy.
            if self.mRobotType == '3D':
                angle_with_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='o-xy')
            
            # Check that the new position is within the sensing range and angle limits.
            if np.linalg.norm(new_pos - self.pos) < self.mSenseDistance:
                if angle_in_xy >= -self.mSenseAngle and angle_in_xy <= self.mSenseAngle:
                    if self.mRobotType == '3D':
                        if angle_with_xy >= -self.mSenseAngle and angle_with_xy <= self.mSenseAngle:                
                            return new_pos
                    else:
                        return new_pos
