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

    # Define a method called 'update' within a class that takes no parameters.
    def update(self):
        """
        This method is used to update the state of the object.

        It calls the 'sense' and 'processInfo' methods, both of which update information about the environment by looking at the 
        current state of the object.

        If there are other objects present in the environment (that have been sensed), the value of 'mMaxCrowded' instance variable 
        is set as 1 divided by the total number of robot agents present. Otherwise, 'mMaxCrowded' will be set to 0.

        If the robot is not currently communicating with other agents and its stopping criterion is met, it tries to explore 
        new strategies i.e. it examines the outcome of 'swarm' and 'follow' methods and selects the best approach based on the maximum fitness value.
        If neither strategy works, it will execute the 'prey' strategy.

        If the robot is not communicating with any other robots, the data stored in 'mSenseInfo' will be cleared.

        At the end, this method calls the 'move' method on the object, which moves it to the next position based on its strategy choice.
        """
        
        # Call the 'sense' and 'processInfo' methods to update the object's internal state.
        self.sense()
        self.processInfo()

        # Update the value of 'mMaxCrowded' according to the number of robots present.
        if len(self.mPopulation) > 0:
            self.mMaxCrowded = 1 / len(self.mPopulation)
        else:
            self.mMaxCrowded = 0

        if self.isStopping():  # Only if the stopping criterion is met, robot needs to choose a new target.
            
            # Explore new strategies - swarm and follow - to update its status.
            swarm_fitness = self.swarm()
            follow_fitness = self.follow()

            # Compare the results of exploration and choose best approach based on the maximum fitness value.
            if swarm_fitness > self.mFitness or follow_fitness > self.mFitness:
                if swarm_fitness > follow_fitness:
                    self.swarm()
                else:
                    self.follow()
            else:  # If neither strategy works, execute the 'prey' strategy.
                prey_fitness = self.prey()

        # Clear 'mSenseInfo' data if currently not communicating.
        if not self.isCommunicating:
            self.mSenseInfo.clear()

        # Finally, move the robot to the next position based on its current strategy choice.
        self.move()

        
        
    @staticmethod
    def randomTrue(probability=0.5):
        """This function returns True or False based on a given probability.

        Args:
            probability (float, optional): The probability of the result being True. Defaults to 0.5.

        Returns:
            bool: True or False based on the given probability.
        """
        
        # Generate a random number between 0 and 1 using Python's built-in random module. 
        # If it's less than the given probability, return True.
        if random.random() < probability:
            return True
        else:
            # If the generated number is greater than or equal to the given probability, return False.
            return False


    def sense(self):
        """This function calls the sense function of the parent class.

        This function is a method of a child class which inherits from another class that has a sense() method. By calling 
        super().sense(), we can call the sense() method of the parent class.

        Returns:
            None
        """
        super().sense() # Calls the sense() method of the parent class

        # 添加更新自身fitness的算法
        # self.mFitness = self.getPosFit(self.pos)
        # self.mFood = self.mProcessedInfo['ComFish'].values()
        # self.mPopulation = self.mProcessedInfo[self.mObjectType]
    
    def prey(self):
        """This function is responsible for executing the prey operator in the implemented optimization algorithm.

        This function tries to find a prey by searching for a position with better fitness value compared to the current 
        position. If it fails to find a better position after mMaxPreyNum attempts, it returns a random position to avoid 
        getting stuck in local optima.

        Returns:
            float: The fitness value of the new target position.
        """
        # Keep trying to find a better position until we reach mMaxPreyNum attempts or we find a position with better 
        # fitness than the current position.
        for _ in range(self.mMaxPreyNum):
            pos = self.getRandomSensePos()  # Get a random neighboring position
            fitness = self.getPosFit(pos)  # Calculate the fitness at the new position
            if fitness > self.mFitness:  # If the fitness is better than the current position, set it as the new target
                self.target = pos
                return fitness
        
        # If we can't find a better position after mMaxPreyNum attempts, we choose a random position and return its fitness.
        pos = self.getRandomSensePos()
        self.target = pos
        self.mFollowedAgentID = None
        return self.getPosFit(pos)


    def swarm(self):
        """This function is responsible for executing the swarm operator in the implemented optimization algorithm.

        This function calculates the center of the population and its fitness value. If the fitness improves compared 
        to the current position and it's not too crowded around, we set the center as the new target for the agent.
        
        Returns:
            float: The fitness value of the new target position (-1 if no suitable target was found).
        """
        # Calculate the center of the population (only if we have artificial fish agents)
        if len(self.mPopulation) > 0:
            center = np.array([0.0, 0.0, 0.0], dtype=float)
            for agent_pos in self.mPopulation.values():
                center += agent_pos
            center /= len(self.mPopulation)
            
            # Make sure the center doesn't go outside the boundary
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
            
            # Calculate the fitness value at the new center position
            fitness = self.getPosFit(center)
            
            # If the fitness is better than the current position and is not too crowded around, set the center as the 
            # new target
            if fitness > self.mFitness and not self.isCrowded(fitness, self.mFitness, center):
                self.target = center
                return fitness
        
        # If we can't set the center as the new target, reset the followed agent ID and return -1
        self.mFollowedAgentID = None
        return -1


    def getAgentsInRangeOfPos(self, pos: np.ndarray, r: float):
        """Return all agents within a certain range of a given position.

        Args:
            pos (np.ndarray): The center position.
            r (float): The radius around the center to search for agents.

        Returns:
            list: A list of tuples containing the agent IDs and their positions.
        """
        # Get the positions and keys of all agents
        agents_pos_group = list(self.mPopulation.values())
        agents_keys_group = list(self.mPopulation.keys())

        # Create a KD-Tree to find all agents within the specified range
        kd_tree = KDtree(agents_pos_group)
        inds, _ = kd_tree.query_radius(pos, r)
        inds = inds[0]

        # Return the positions and IDs of all agents within range
        return [(agents_keys_group[ind], agents_pos_group[ind]) for ind in inds]

    def follow(self):
        """This function is responsible for executing the follow operator in the implemented optimization algorithm.

        This function finds the fish with the highest fitness value within its perception range.
        If the fish has a higher fitness than the current position and it's not too crowded around, we set the fish as the 
        new target for the agent.
        
        Returns:
            float: The fitness value of the new target position (-1 if no suitable target was found).
        """
        # Find the fish with the highest fitness within the perception range
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
        # If the fish has a higher fitness than the current position and it's not too crowded around, set the fish as 
        # the new target for the agent
        # print("follow_fitness %f"%fitness_max)
        if fitness_max > self.mFitness and agent_max_pos is not None:
            if self.isCrowded(fitness_max, self.mFitness, agent_max_pos):  # 感知该人工鱼的拥挤度，如果拥挤度不大，则向该人工鱼移动一步
                self.target = agent_max_pos
                return fitness_max
        self.mFollowedAgentID = agent_max_id

        return -1

    def isCrowded(self, target_fitness, current_fitness, target_pos):
        """Check whether a certain position is too crowded.

        Args:
            target_fitness (float): The fitness value of the target position.
            current_fitness (float): The fitness value of the current position.
            target_pos (np.ndarray): The target position to check for overcrowding.

        Returns:
            bool: True if the position is too crowded, False otherwise.
        """
        agent_num_in_range = len(self.getAgentsInRangeOfPos(target_pos, mySettings.CS_CROWDEDRANGE))

        if agent_num_in_range == 0:
            return False
        elif (target_fitness / agent_num_in_range) < (self.mMaxCrowded * current_fitness):
            return True
        else:
            return False

    def getPosFit(self, position):
        """Calculate the fitness value at a given position.

        Args:
            position (np.ndarray): The position to calculate the fitness value at.

        Returns:
            float: The fitness value at the given position.
        """
        fitness = 0.0
        
        # Calculate the fitness based on the distance to each food source
        if len(self.mFood) > 0:
            for food_pos in self.mFood:
                fitness_tmp = 1 - (np.linalg.norm(np.array(position, dtype=float) - food_pos, ord=2) / self.mSenseDistance)
                # fitness_tmp = 1 / (np.linalg.norm(np.array(position, dtype=float) - food_pos, ord=2) + 0.000000000000001)
                if fitness_tmp > fitness:
                    fitness = fitness_tmp
        # fitness = utils.sigmoid(fitness, 0.5, 0.5)
        return fitness

    def randomSensePosFit(self):
        """Return the fitness value and position of a randomly chosen point within the agent's perception range.

        Returns:
            tuple: A tuple containing the fitness value and position.
        """
        pos = self.getRandomSensePos()
        fitness = self.getPosFit(pos)
        return fitness, pos

    def getRandomSensePos(self):
        """
        This function returns a random position within the sensing range of a robot.
        
        Args:
            None
        
        Returns:
            new_pos (numpy.ndarray): A 3D numpy array containing x, y, z coordinates of the new position.
            
        """
        
        # Calculate minimum and maximum values for x, y and z coordinates of the random position
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

        # Generate random values for x, y and z coordinates within the calculated ranges until a suitable position is found
        while True:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            z = random.uniform(z_min, z_max)
            new_pos = np.array([x, y, z], dtype=float)
            
            # Calculate angles between the new position and the robot's current position in xy plane
            angle_in_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='xy') - self.mDirection
            
            # If the robot is 3D, also calculate the angle between the new position and the xy plane
            if self.mRobotType == '3D':
                angle_with_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='o-xy')
                
            # Check if the new position is within the sensing range and makes an angle with the robot's direction within a certain range
            if np.linalg.norm(new_pos - self.pos) < self.mSenseDistance:
                if angle_in_xy >= -self.mSenseAngle and angle_in_xy <= self.mSenseAngle:
                    # If the robot is 3D, also check if the new position makes an angle with the xy plane within the same range
                    if self.mRobotType == '3D':
                        if angle_with_xy >= -self.mSenseAngle and angle_with_xy <= self.mSenseAngle:                
                            return new_pos
                    else:
                        return new_pos
