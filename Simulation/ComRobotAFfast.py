# -*- coding: utf-8 -*-


from Simulation.ComRobot import ComRobot 
from Simulation.ComRobotCon import ComRobotCon 
from Simulation.ComObject import ComObject 
import copy
import math
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
AF_MAXPREYNUM = 10     # 每次最大进行觅食尝试的次数
AF_POPULATIONNUM = 20   # 人工鱼数量
AF_FOODSIZE = 5         # 最大食物数量
AF_MAXITERNUM = 1000    # 最大迭代次数
AF_INTERVAL = 0.05      # 两次迭代间隔的时间
AF_SENSEDIST = 500       # 感知距离
AF_MAXCROWDED = 1 / 20  # 拥挤度因子, 分母代表人工鱼的数目
AF_GETFOODDIST = 1      # 找到食物的最小距离



class ComRobotAFfast(ComRobotCon):
    def __init__(self, pos):
        super(ComRobotAFfast, self).__init__(pos)
        self.mPopulation = {}       # 鱼群      感知到的族群及其坐标
        self.mFood = []             # 食物      目标
        self.mFitness = 0.0
        self.mMaxPreyNum = AF_MAXPREYNUM
        self.mMaxCrowded = AF_MAXCROWDED
        self.mFollowedAgentID = None
        self.mObjectType = "ComRobotAF"       # 用于标识当前物体类别
        self.mFoodName = "ComFish"         # 设定用作食物的目标
        self.PosibilityOfNewTarget = 0.1

    def update(self):
        """
        This function updates the state of the robot by performing the following actions:
            1. Checks if the robot has reached its target position, and chooses a new random target if necessary.
            2. Calls the sense() function to get information about nearby robots and targets.
            3. Processes the information obtained in step 2 using the processInfo() function.
            4. Applies the AFfast algorithm to update the self.mDirection variable.
            5. Moves the robot to its new position using the move() function.

        Args:
            None
            
        Returns:
            None
        
        """
        # if (self.pos == self.target).all():
        #     self.chooseRandotarget()

        # Call the sense() function to get information about nearby robots and targets
        self.sense()
        
        # Process the information obtained in step 2 using the processInfo() function
        self.processInfo()
        
        # Apply the AFfast algorithm to update the self.mDirection variable
        self.AFfast()
        
        if self.isPathPlanning:
            if self.getPlanningControl().mTarget is None:
                self.setPlanningTarget(self.mPos)
            self.pathPlanning()

        self.pathFollowing()
        # if self.mId == 0:
        #     print(self.mLineSpeed, self.mRotationSpeed)
        self.move()
    
    def AFfast(self):
        """
        This function implements the AFfast algorithm for the robot to make a decision on its next action.
        The following steps are performed:
            1. Calculate the maximum crowdedness of the environment, which is equal to the inverse of the population size.
            2. If the random number generated is less than the probability of choosing a new target or the robot is stopping, try swarm and follow behaviors.
            3. Compare the fitness values obtained from the swarm and follow behaviors and select the optimal decision.
            4. If neither swarm nor follow are feasible, execute prey behavior instead.
            
        Args:
            None
            
        Returns:
            None
        
        """

        # Calculate the maximum crowdedness of the environment
        if len(self.mPopulation) > 0:
            self.mMaxCrowded = 1
        else:
            self.mMaxCrowded = 0

        # If the random number generated is less than the probability of choosing a new target or the robot is stopping, try swarm and follow behaviors
        if random.random() < self.PosibilityOfNewTarget or self.isStopping():
            
            # Try swarm and follow behaviors and compare their fitness values
            swarm_fitness = self.swarm()
            follow_fitness = self.follow()
            if swarm_fitness < 0 and follow_fitness < 0:
                prey_fitness = self.prey()
            
            # Compare the fitness values obtained from the swarm and follow behaviors and select the optimal decision
            if swarm_fitness > self.mFitness or follow_fitness > self.mFitness:
                if swarm_fitness > follow_fitness:
                    self.swarm()
                else:
                    self.follow()
            else:  # If neither swarm nor follow are feasible, execute prey behavior instead
                prey_fitness = self.prey()

    @staticmethod
    def randomTrue(probability=0.5):
        """
        This function returns a random boolean value based on the given probability of getting True.
        
        Args:
            probability: The probability of returning True, defaults to 0.5.
            
        Returns:
            bool: A random boolean value.
        
        """
        if np.random.rand() < probability:
            return True
        else:
            return False

    def sense(self):
        """
        This function calls the sense() function of the parent class to get information about nearby robots and targets.
        
        Args:
            None
            
        Returns:
            None
        
        """
        super().sense()
        # 添加更新自身fitness的算法
        # self.mFitness = self.getPosFit(self.pos)
        # self.mFood = self.mProcessedInfo['ComFish'].values()
        # self.mPopulation = self.mProcessedInfo[self.mObjectType]
    
    def prey(self):
        """
        This function is called "prey", which is responsible for hunting food. If the maximum number of attempts to search for food has been reached and the fitness of the target is still lower than the current fitness, return a random position.
        
        Returns:
            _type_: _description_
        """ 
        # Loops through a specified range of hunt attempts (self.mMaxPreyNum) 
        for _ in range(self.mMaxPreyNum):
            pos = self.getRandomSensePos()  # Get a random position to search for food
            fitness = self.getPosFit(pos)  # Calculate the fitness of the position
            # If the fitness is greater than the current fitness
            if fitness > self.mFitness:
                # If path planning is enabled, set the planning target to the new position
                if self.isPathPlanning:
                    self.setPlanningTarget(pos)
                else:
                    self.target = pos   # Otherwise, update the target position to the new position
                return fitness  # Return the fitness of the position
        pos = self.getRandomSensePos()  # Get another random position to search for food
        if self.isPathPlanning:
            self.setPlanningTarget(pos)    # If path planning is enabled, set the planning target to the new position
        else:
            self.target = pos   # Otherwise, update the target position to the new position
        self.mFollowedAgentID = None   # Reset followed agent ID to None
        return self.getPosFit(pos)  # Return the fitness of the position


    def swarm(self):
        """
        This function implements the swarm operator, which calculates the center of the population and its fitness value.

        Args:
            None
            
        Returns:
            float: The fitness value at the calculated center of the population.
        
        """
        # Calculate the center of the population
        if len(self.mPopulation) > 0:
            center = np.array([0.0, 0.0, 0.0], dtype=float)
            for agent_pos in self.mPopulation.values():
                if len(agent_pos) < 3:
                    agent_pos_tmp = np.array([0.0, 0.0, 0.0], dtype=float)
                    agent_pos_tmp[0:2] = agent_pos[0:2]
                    agent_pos = agent_pos_tmp
                center += agent_pos
            center /= len(self.mPopulation)
            
            # Ensure that the center is within the bounds of the environment
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
            
            # Calculate the fitness value at the center and return it if it's higher than the current fitness value and
            # not too crowded
            fitness = self.getPosFit(center)
            if fitness > self.mFitness and not self.isCrowded(fitness, self.mFitness, center):
                if self.isPathPlanning:
                    self.setPlanningTarget(center)
                else:
                    self.target = center
                return fitness
        # If there are no agents or the fitness value at the center can't be improved or the center is too crowded,
        # return -1 to indicate failure to find a target.
        self.mFollowedAgentID = None
        return -1

    def getAgentsInRangeOfPos(self, pos: np.ndarray, r: float):
        """
        This function finds all the agents within a given radius of a position.

        Args:
            pos (np.ndarray): The position to check for nearby agents.
            r (float): The radius within which to check for nearby agents.
            
        Returns:
            list: A list of tuples in the format [[id, pos], [id, pos], ...] representing the IDs and positions of 
            all the agents within the given radius.
        
        """
        query_pos = np.array([.0, .0, .0])
        if len(pos) < 3:
            query_pos[0:2] = pos[0:2]
        else:
            query_pos = pos
        agents_pos_group = list(self.mPopulation.values())
        agents_keys_group = list(self.mPopulation.keys())
        kd_tree = KDtree(agents_pos_group)
        inds, _ = kd_tree.query_radius(query_pos, r)
        inds = inds[0]
        return [(agents_keys_group[ind], agents_pos_group[ind]) for ind in inds]


    # Define a method 'follow' for a class
    def follow(self):
        """
        This method is used to calculate the following target and the target's fitness score:
        1. Find the artificial fish with the highest fitness score within the perception range;
        2. If the target's fitness score is larger than that of itself and the area is not crowded, move one step towards the target;
        Returns: The fitness score of the target or -1 if no suitable target found
        """
        # Find the artificial fish with the highest fitness score within the perception range
        agent_max_pos = None
        agent_max_id = None
        fitness_max = 0
        for agent_id, agent_pos in self.mPopulation.items():
            # Calculate the fitness score of the target
            fitness = self.getPosFit(agent_pos)
            if fitness > fitness_max:
                fitness_max = fitness
                agent_max_pos = agent_pos
                agent_max_id = agent_id
        # If the target's fitness score is larger than that of itself
        if fitness_max > self.mFitness and agent_max_pos is not None:
            # Check if the area is crowded and move one step towards the target if it is not too crowded
            if self.isCrowded(fitness_max, self.mFitness, agent_max_pos):  
                self.target = agent_max_pos
                return fitness_max
        self.mFollowedAgentID = agent_max_id
        # Return -1 if there is no suitable target found
        return -1

    def isCrowded(self, target_fitness, current_fitness, target_pos):
        """
        This method checks whether the given position is too crowded or not:
        1. Calculates the number of agents within the specified crowded range around the target position;
        2. If there are no agents within the range, it returns False;
        3. Otherwise, it calculates the ratio of the target fitness and the number of agents in the range to the maximum allowed crowdedness and compares it with the current fitness;
        4. If the ratio is less than the maximum allowed crowdedness, then it returns True; otherwise, it returns False.

        Args:
            target_fitness (float): The fitness score of the target position
            current_fitness (float): The fitness score of the current position
            target_pos (tuple): A tuple specifying the x and y coordinates of the target position

        Returns:
            bool: True if the position is too crowded, otherwise False
        """        
        # Calculate the number of agents within the specified crowded range around the target position
        agent_num_in_range = len(self.getAgentsInRangeOfPos(target_pos, mySettings.CS_CROWDEDRANGE))

        # Return False if there are no agents within the specified range
        if agent_num_in_range == 0:
            return False
        # Calculate the fitness ratio and compare it with the maximum allowed crowdedness
        elif (target_fitness / agent_num_in_range) < (self.mMaxCrowded * current_fitness):
            return True
        else:
            return False


    def getPosFit(self, position):
        """
        This method calculates the fitness score of a given position based on the distance from the food sources:
        1. Converts the two-dimensional position to three-dimensional coordinates;
        2. For each food source in the environment, it calculates the Euclidean distance between the position and the food source and subtracts it from 1 to get a normalized fitness value for that food source;
        3. The final fitness value is the maximum fitness value among all the food sources;
        4. Applies a sigmoid function to normalize and restrict the fitness value to range [0, 1].

        Args:
            position (tuple): A tuple specifying the x and y coordinates of the position

        Returns:
            float: The fitness score of the position
        """        
        # Initialize the fitness score to 0
        fitness = 0.0
        # Convert the 2D position to 3D coordinates
        position = utils.two_dim_to_three_dim(position)
        
        # Calculate the fitness contribution of each food source
        if len(self.mFood) > 0:
            for food_pos in self.mFood:
                food_pos = utils.two_dim_to_three_dim(food_pos)
                distance = np.linalg.norm(np.array(position, dtype=float) - food_pos, ord=2)
                fitness_tmp = 1 - (distance / self.mSenseDistance)
                # In case we want to use inverse distance as fitness value
                #fitness_tmp = 1 / (distance + 0.000000000000001)
                # Update the fitness score with the maximum fitness contribution from all the food sources
                if fitness_tmp > fitness:
                    fitness = fitness_tmp
        
        # Apply a sigmoid function to normalize and restrict the fitness value to range [0, 1]
        fitness = utils.sigmoid(fitness, 0.5, 0.5)
        return fitness


    def randomSensePosFit(self):
        """
        This method calculates the fitness score of a randomly generated position within the sensing range of the agent:
        1. Generates a random position within the sensing range of the agent;
        2. Calculates the fitness score of the random position using the 'getPosFit()' method.

        Returns:
            tuple: A tuple containing the fitness score and the randomly generated position (in that order).
        """        
        # Generate a random position within the sensing range of the agent
        pos = self.getRandomSensePos()
        # Calculate the fitness score of the random position using 'getPosFit()' method
        fitness = self.getPosFit(pos)
        # Return a tuple containing the fitness score and the randomly generated position
        return fitness, pos


    def getRandomSensePos(self):
        """
        This method generates a random position within the sensing range of the agent.

        Returns:
            float: A randomly generated position within the agent's sensing range.
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

        z_min = 0
        z_max = 0
        if self.mRobotType == '3D':
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
            new_pos = None
            if self.mRobotType == '3D':
                new_pos = np.array([x, y, z], dtype=float)
            elif self.mRobotType == '2D':
                new_pos = np.array([x, y], dtype=float)
            angle_in_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='xy') - self.mDirection     # xy平面内的偏航角度
            angle_with_xy = ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='o-xy')   # 与xy平面的夹角
            if angle_in_xy > math.pi * 2:
                angle_in_xy = angle_in_xy % (math.pi * 2)
            elif angle_in_xy < -math.pi *2:
                angle_in_xy = angle_in_xy % (-math.pi * 2)
            # print(ComObject.getAngleBetweenXandVector(new_pos, self.pos, plat='xy'), angle_with_xy, self.mDirection)
            if np.linalg.norm(new_pos - self.pos) < self.mSenseDistance:
                if angle_in_xy >= -self.mSenseAngle and angle_in_xy <= self.mSenseAngle:
                    if angle_with_xy >= -self.mSenseAngle and angle_with_xy <= self.mSenseAngle:                
                        return new_pos