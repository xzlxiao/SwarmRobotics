from Simulation.ComRobot import ComRobot 
from Simulation.ComObject import ComObject 
from Simulation.ComRobotPSO import ComRobotPSO
from Common.utils import *
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
from Simulation.ComObjectCollection import *


special_colors = [
    "cyan",
    'royalblue',
    "lightsteelblue",
    'red',
    "purple",
    'blueviolet',
    
    "yellow",
    "lightgreen",
    "orange",
    'red',
    "white",
    # 'antiquewhite',
    # 'aqua',
    
    # 'red',
    # 'white',
    
]
# R_I = 500   # 小生境范围
R_I = 400   # 小生境范围
C_TH = 300  # 激活新种群的目标距离
# NICHE_aggregation_degree_threshold = 8.0      # 子群聚集度阈值
NICHE_aggregation_degree_threshold = 5.0      # 子群聚集度阈值
# AGG_beta = 1.0      # 聚集度参数
AGG_beta = 0.8      # 聚集度参数
AGG_lambda = -0.4    # 聚集度参数 
SIG_STEEP = 0.002
SIG_MID = 700
target_line_len = 300.0

class ComRobotPSO_niche(ComRobotPSO):
    SpecialPool = []
    PopulationGroup = []      # 子群集合

    def __init__(self, pos):
        super().__init__(pos)
        if len(ComRobotPSO_niche.SpecialPool) == 0:
            ComRobotPSO_niche.SpecialPool = [[] for _ in range(10000)]
        if len(ComRobotPSO_niche.PopulationGroup) == 0:
            ComRobotPSO_niche.PopulationGroup = [{} for _ in range(10000) ]
        self.setColor("blue")
        self.mFoodAll = {}              # 所有食物
        self.mPopulationAll = {}        # 所有个体
        self.mTargetLineLen = -1
        
        self._species = -1
        self.setSpecies(-1)

    def getAllRobotsNum(self):
        '''
        A function that returns the total number of robots.

        Returns:
            int: The total number of robots.
        '''
        # Return the length of the mPopulationAll list.
        return len(self.mPopulationAll)


    def getRobotsNumBySpecies(self, species_id):
        '''
        A function that returns the number of robots for a specific species.

        Args:
            species_id (int): The ID of the species.

        Returns:
            int: The number of robots for the given species ID.
        '''
        # Return the length of the PopulationGroup list for the given species ID.
        return len(ComRobotPSO_niche.PopulationGroup[species_id])


    def sense(self):
        '''
        A function that updates the environment state based on the information state.

        Returns:
            None
        '''

        # Get objects within sight range and update fitness values.
        self.getObjectBySight()
        self.mFitness = self.getPosFit(self.pos)
        self.mBestFitness = self.getPosFit(self.best_pos)

        # Check the information state and update the environment accordingly.
        if self.getInformationState() == 'global':

            # Get all available food objects and store them in a dictionary.
            foods = getObjectByType(self.mFoodName)
            for food in foods:
                self.mFoodAll[food.mId] = food

            # Update the positions of the food objects based on the species ID.
            self.mFood.clear()
            if self._species >= 0:
                self.mFood.append(self.mFoodAll[self._species].pos)

            # Get all available robot objects and store them in a dictionary.
            neighors = getObjectByType(self.mObjectType)
            for agent in neighors:
                self.mPopulationAll[agent.mId] = agent

            # Update the positions of the subset of robots based on their IDs.
            self.mPopulation_agents.clear()
            for subgroup_agent_id in self.mPopulation.keys():
                self.mPopulation[subgroup_agent_id] = self.mPopulationAll[subgroup_agent_id].pos
                self.mPopulation_agents.append(self.mPopulationAll[subgroup_agent_id])

        elif self.getInformationState() == 'local':

            # Store the positions of food and robots based on the processed information.
            self.mFood = self.mProcessedInfo[self.mFoodName].values()
            self.mPopulation = self.mProcessedInfo['Pos']

        elif self.getInformationState() == 'no':

            # Store the positions of food and robots based on the processed information.
            self.mFood = self.mProcessedInfo[self.mFoodName].values()
            self.mPopulation = self.mProcessedInfo[self.mRobotType]

        else:
            raise

        
    def update(self):
        """
        A function that updates the robot's state based on its current information state.

        Returns:
            None
        """
        # return super().update()
        # if self.mId == 0:
        #     print(self._species)
        #     print(self.nicheAggregationDegree())
        #     with open("/media/storage1/Code/data/AFSA/20220602002/log.txt", 'a') as file:
        #         file.write("{} | {}\n".format(self._species, self.nicheAggregationDegree()))

        # if self.nicheAggregationDegree()> 5:
        #     print(self.nicheAggregationDegree())
        if self.mId == 0:
            pass
            # main_pop = len(ComRobotPSO_niche.PopulationGroup[-1])
            # sub_pop0 = len(ComRobotPSO_niche.PopulationGroup[0])
            # sub_pop1 = len(ComRobotPSO_niche.PopulationGroup[1])
            # sub_pop2 = len(ComRobotPSO_niche.PopulationGroup[2])
            # sub_pop3 = len(ComRobotPSO_niche.PopulationGroup[3])
            # sub_pop4 = len(ComRobotPSO_niche.PopulationGroup[4])
            # print("""
            # main pop: {}
            # sub0: {}
            # sub1: {}
            # sub2: {}
            # sub3: {}
            # sub4: {}
            # ---------------------------------
            # """.format(main_pop, sub_pop0, sub_pop1, sub_pop2, sub_pop3, sub_pop4))

        self.sense()
        self.processInfo()
        if self._species == -1 and self.getUpdateCount() > 5:
            for food in self.mFoodAll.values():
                # print(food)
                if not ComRobotPSO_niche.SpecialPool[food.mId]:
                    if self.distance(food.pos, self.pos) < C_TH:
                        print("set species {}".format(food.mId))
                        ComRobotPSO_niche.SpecialPool[food.mId] = True
                        current_population = [(agent_id, agent_pos) for agent_id, agent_pos in ComRobotPSO_niche.PopulationGroup[-1].items()]
                        for agent_id, agent_pos in current_population:
                            if self.distance(agent_pos, self.pos) < R_I:
                                self.mPopulationAll[agent_id].setSpecies(food.mId)
                        break
                else:
                    if self.distance(food.pos, self.pos) < R_I:
                        self.mPopulationAll[self.mId].setSpecies(food.mId)
        else:
            if self.isReturnToMainSwarm():
                print("return main swarm: {}.{} -> main".format(self._species, self.mId))
                # with open("/media/storage1/Code/data/AFSA/20220602003/log1.txt", 'a') as file:
                #     file.write("return main swarm: {}.{} -> main\n".format(self._species, self.mId))
                self.setSpecies(-1)
            # print(len(self.mPopulation))

        if self._species == -1:
            
            self.randomMove()
        else:
            self.pso()
        self.move()

    def isStopping(self):
        """
        A function that determines whether the robot has reached its target location.

        Returns:
            bool: True if the robot has reached its target location, else False.
        """        
        # If the robot belongs to the main swarm, check if it has reached its target location.
        if self._species == -1:
            if (self.pos == self.target).all():
                return True
            else:
                return False
        # If the robot belongs to a sub-swarm, call the super class' isStopping method to determine stopping criteria.
        else:
            return super().isStopping()

    def randomMove(self):
        """
        A function that moves the robot randomly on the grid.
        """        
        # If the robot has reached its target location, choose a new random target location.
        if self.isStopping():
            self.chooseRandomTarget()


  
    def setSpecies(self, species_id):
        """
        A function that sets the species ID of the robot.

        Args:
            species_id (int): The ID of the robot's species.
        """        
        # If the robot belongs to the main swarm, set mTargetLineLen to -1.
        if species_id == -1:
            self.mTargetLineLen = -1
        # Otherwise, set mTargetLineLen to target_line_len.
        else:
            self.mTargetLineLen = target_line_len
        # Set the species ID of the robot.
        self._species = species_id
        # Set the color of the robot based on its species ID.
        self.setColor(special_colors[species_id%len(special_colors)])
        # If the robot ID is in the population dictionary, remove it.
        if self.mId in self.mPopulation.keys():
            self.mPopulation.pop(self.mId)
        # Set the population dictionary based on the robot's species ID.
        self.mPopulation = ComRobotPSO_niche.PopulationGroup[self._species]
        # Add the robot's current position to the population dictionary.
        self.mPopulation[self.mId] = self.pos

    def nicheAggregationDegree(self):
        """
        A function that calculates the degree of aggregation in the sub-swarm.

        Returns:
            float: The degree of aggregation in the sub-swarm.
        """        

        # Initialize the degree of aggregation to 0.
        aggregation_degree = 0
        # If there is at least one food item in the sub-swarm, calculate the degree of aggregation.
        if len(self.mFood) > 0:
            for agent_pos in self.mPopulation.values():
                aggregation_degree += 1 - (self.distance(self.mFood[0], agent_pos) / R_I)**AGG_beta
        # Return the degree of aggregation.
        return aggregation_degree

    def isReturnToMainSwarm(self):
        """
        A function that determines whether the robot should return to the main swarm.

        Returns:
            bool: True if the robot should return to the main swarm, else False. 
        """        

        # Calculate the degree of aggregation in the sub-swarm.
        aggregation_degree = self.nicheAggregationDegree()
        # If the degree of aggregation is greater than the threshold, calculate whether the robot should return to the main swarm.
        if aggregation_degree > NICHE_aggregation_degree_threshold:
            niche_size = len(self.mPopulation)
            current_time = self.getTime()
            rho_i = niche_size * AGG_lambda * (1.0/(1.0+math.e**(current_time*aggregation_degree/niche_size)) - 0.5)

            if rho_i > 1:

                dist = self.distance(self.pos, self.mFood[0])
                for agent_pos in self.mPopulation.values():
                    if self.distance(self.mFood[0], agent_pos) > dist:
                        return False 
                return True 
        # If the degree of aggregation is not greater than the threshold, don't return to the main swarm.
        return False

    def chooseRandomDistancedTarget(self):
        """
        A function that chooses a random target location for the robot based on distance probability.
        """        
        
        # Initialize x, y, z to 0.
        x = 0
        y = 0
        z = 0
        # Choose a random target location until one is found that satisfies the distance probability criteria.
        while True:
            range_x = (-100, 100)
            range_y = (-100, 100)
            range_z = (-100, 100)
            if self.mStage is not None:
                range_x = (-self.mStage.mEnvSize[0], self.mStage.mEnvSize[0])
                range_y = (-self.mStage.mEnvSize[1], self.mStage.mEnvSize[1])
                range_z = (-self.mStage.mEnvSize[2], self.mStage.mEnvSize[2])
            x = random.uniform(range_x[0], range_x[1])
            y = random.uniform(range_y[0], range_y[1])
            z = random.uniform(range_z[0], range_z[1])
            dist = self.distance(self.pos, (x, y, z))
            posibility = sigmoid(dist, SIG_MID, SIG_STEEP)
            if random.random() < posibility:
                break

        # Set the target location to the randomly chosen location.
        self.setTarget((x, y, z))
