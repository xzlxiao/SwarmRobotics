isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random
import copy
import math
from Common.utils import *
from Simulation.ComObjectCollection import *
from Simulation.ComRobotAFfast import ComRobotAFfast

# special_colors = [
#     "red",
#     'red',
#     "red",
#     'red',
#     "red",
#     'red',
    
#     "red",
#     "red",
#     "red",
#     'red',
#     "white",
#     # 'antiquewhite',
#     # 'aqua',
    
#     # 'red',
#     # 'white',
    
# ]

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

R_I = 200   # 小生境范围
C_TH = 600  # 激活新种群的目标距离
NICHE_aggregation_degree_threshold = 8.0      # 子群聚集度阈值
AGG_beta = 1.0      # 聚集度参数
AGG_lambda = -0.4    # 聚集度参数 
SIG_STEEP = 0.002
SIG_MID = 700

class ComRobotAF_niche(ComRobotAFfast):
    SpecialPool = []
    PopulationGroup = []      # 子群集合

    def __init__(self, pos):
        super().__init__(pos)
        if len(ComRobotAF_niche.SpecialPool) == 0:
            ComRobotAF_niche.SpecialPool = [[] for _ in range(10000)]
        if len(ComRobotAF_niche.PopulationGroup) == 0:
            ComRobotAF_niche.PopulationGroup = [{} for _ in range(10000) ]
        self.setColor("blue")
        self.mFoodAll = {}              # 所有食物
        self.mPopulationAll = {}        # 所有个体
        
        self._species = -1
        self.setSpecies(-1)

    # This function returns the total number of robots in the population.
    def getAllRobotsNum(self):
        """
        Returns the total number of robots in the population.

        The total robot count is simply the length of the mPopulationAll list.

        Returns:
            int: The total number of robots in the population.
        """        

        return len(self.mPopulationAll)


    def getRobotsNumBySpecies(self, species_id):
        """
        Returns the number of robots in a specific species.

        Args:
            species_id (int): The ID of the species to get the robot count for.

        Returns:
            int: The number of robots in the specified species.
        """
        
        # Get the number of robots in a specific species
        # Retrieve the length of the list associated with the given species_id key in the PopulationGroup dictionary
        return len(ComRobotAF_niche.PopulationGroup[species_id])


    def getSpecies(self):
        """
        Returns the species associated with this object.

        Returns:
            str: The species value of the object.
        """        
        # This function returns the species value of the object calling it
        # The 'self._species' is a property variable that returns the species value of the object
        return self._species

        
    # Define a method update
    def update(self):
        """
        Updates the robot and prints the count of the population groups.

        Returns:
            None
        """
        
        # If it's the first robot in the swarm, print the species and niche aggregation degree to the console and log it to file.
        if self.mId == 0:
            main_pop = len(ComRobotAF_niche.PopulationGroup[-1])
            sub_pop0 = len(ComRobotAF_niche.PopulationGroup[0])
            sub_pop1 = len(ComRobotAF_niche.PopulationGroup[1])
            sub_pop2 = len(ComRobotAF_niche.PopulationGroup[2])
            sub_pop3 = len(ComRobotAF_niche.PopulationGroup[3])
            sub_pop4 = len(ComRobotAF_niche.PopulationGroup[4])
            print("""
            main pop: {}
            sub0: {}
            sub1: {}
            sub2: {}
            sub3: {}
            sub4: {}
            ---------------------------------
            """.format(main_pop, sub_pop0, sub_pop1, sub_pop2, sub_pop3, sub_pop4))
            
            # with open("/media/storage1/Code/data/AFSA/20220602003/log2.txt", 'a') as file:
            #     file.write("""
            #         main pop: {}
            #         sub0: {}
            #         sub1: {}
            #         sub2: {}
            #         sub3: {}
            #         sub4: {}
            #         ---------------------------------
            #         """.format(main_pop, sub_pop0, sub_pop1, sub_pop2, sub_pop3, sub_pop4))
        
        # Call the methods sense() and processInfo() from the class ComRobotAF.
        self.sense()
        self.processInfo()
        
        # If the robot's species is -1, it searches for food and sets its own species and those of nearby agents.
        if self._species == -1:
            for food in self.mFoodAll.values():
                if not ComRobotAF_niche.SpecialPool[food.mId]:
                    if self.distance(food.pos, self.pos) < C_TH:
                        print("set species {}".format(food.mId))
                        ComRobotAF_niche.SpecialPool[food.mId] = True
                        current_population = [(agent_id, agent_pos) for agent_id, agent_pos in ComRobotAF_niche.PopulationGroup[-1].items()]
                        for agent_id, agent_pos in current_population:
                            if self.distance(agent_pos, self.pos) < R_I:
                                self.mPopulationAll[agent_id].setSpecies(food.mId)
                        break
                else:
                    if self.distance(food.pos, self.pos) < R_I:
                        self.mPopulationAll[self.mId].setSpecies(food.mId)
        else:
            # If the robot has a defined species, check whether it needs to return to the main swarm or stay and furthur search for food.
            if self.isReturnToMainSwarm():
                print("return main swarm: {}.{} -> main".format(self._species, self.mId))
                # Write to the log file that the robot has returned to the main swarm.

                #     file.write("return main swarm: {}.{} -> main\n".format(self._species, self.mId))
                self.setSpecies(-1)
                
        # If the robot's species is -1, perform a random move. Otherwise, run the AFfast method.
        if self._species == -1:
            self.randomMove()
        else:
            self.AFfast()
        
        # Move the robot.
        self.move()


    # Define a method randomMove
    def randomMove(self):
        """
        Makes the robot move randomly if it is stopped.

        Returns:
            None
        """        
        # If the robot is stopped, choose a random target.
        if self.isStopping():
            self.chooseRandomTarget()

    # Define a method sense
    def sense(self):
        """
        Calls the sense method of the superclass ComRobotAF.

        Returns:
            None
        """        
        # Call the sense method from the superclass ComRobotAF.
        super().sense()


        # # 获取所有食物
        # foods = getObjectByType(self.mFoodName)
        
        # for food in foods:
            
        #     self.mFoodAll[food.mId] = food
        
        # # 根据种群类别获取食物,更新食物位置
        # self.mFood.clear()
        # if self._species >= 0:
        #     self.mFood.append(self.mFoodAll[self._species].pos)

        # # 获取所有机器人
        # neighors = getObjectByType(self.mObjectType)
        # for agent in neighors:
        #     self.mPopulationAll[agent.mId] = agent
        
        # # 更新子群中机器人的坐标
        # for subgroup_agent_id in self.mPopulation.keys():
        #     self.mPopulation[subgroup_agent_id] = self.mPopulationAll[subgroup_agent_id].pos

        # self.mFitness = self.getPosFit(self.pos)

    # Define a method setSpecies
    def setSpecies(self, species_id):
        """
        Sets the species of the robot and updates its color.
        
        Args:
            species_id (int): The id of the new species.

        Returns:
            None
        """    
        # Set the _species attribute to the new species id.
        self._species = species_id
        # Update the color of the robot based on its species using special_colors list.
        self.setColor(special_colors[species_id % len(special_colors)])
        
        # If the current robot is in the population dictionary, remove it.
        if self.mId in self.mPopulation.keys():
            self.mPopulation.pop(self.mId)
            
        # Get the new population group for the robot's species and add it to the dictionary.
        self.mPopulation = ComRobotAF_niche.PopulationGroup[self._species]
        self.mPopulation[self.mId] = self.pos
        
    def nicheAggregationDegree(self):
        """
        Calculates the degree of aggregation for the robots in a population group.

        Returns:
            float: The degree of aggregation.
        """
        aggregation_degree = 0
        if len(self.mFood) > 0:
            for agent_pos in self.mPopulation.values():
                aggregation_degree += 1 - (self.distance(self.mFood[0], agent_pos) / R_I)**AGG_beta
        return aggregation_degree

# Define a method isReturnToMainSwarm
def isReturnToMainSwarm(self):
    """
    Determines whether or not the robot should return to the main swarm based on the aggregation degree of the sub-swarm.

    Returns:
        bool: True if the robot should return to the main swarm, False otherwise.
    """
    # Calculate the aggregation degree of the sub-swarm.
    aggregation_degree = self.nicheAggregationDegree()
    
    # If the aggregation degree is higher than the threshold, calculate probability of returning to main swarm.
    if aggregation_degree > NICHE_aggregation_degree_threshold:
        # Calculate rho_i using niche size, current time, and aggregation degree.
        niche_size = len(self.mPopulation)
        current_time = self.getTime()
        rho_i = niche_size * AGG_lambda * (1.0/(1.0+math.e**(current_time*aggregation_degree/niche_size)) - 0.5)
        
        # If rho_i > 1, check if all robots in the sub-swarm are closer to food than the current robot.
        if rho_i > 1:
            dist = self.distance(self.pos, self.mFood[0])
            for agent_pos in self.mPopulation.values():
                if self.distance(self.mFood[0], agent_pos) > dist:
                    return False 
            return True 
    # Return False if not above threshold or if conditions are not met.
    return False

# Define a method chooseRandomDistancedTarget
def chooseRandomDistancedTarget(self):
    """
    Chooses a random target position, with more probability given to farther positions.

    Returns:
        None
    """
    # While loop to keep trying to find a valid target position.
    while True:
        # Set ranges for x, y, and z based on environment size if available.
        range_x = (-100, 100)
        range_y = (-100, 100)
        range_z = (-100, 100)
        if self.mStage is not None:
            range_x = (-self.mStage.mEnvSize[0], self.mStage.mEnvSize[0])
            range_y = (-self.mStage.mEnvSize[1], self.mStage.mEnvSize[1])
            range_z = (-self.mStage.mEnvSize[2], self.mStage.mEnvSize[2])
        
        # Choose a random position within the ranges. 
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        z = random.uniform(range_z[0], range_z[1])

        # Calculate the distance between the current robot and the new target position.
        dist = self.distance(self.pos, (x, y, z))
        # Calculate the possibility of selecting the new target position using sigmoid function and SIG_MID and SIG_STEEP constants.
        posibility = sigmoid(dist, SIG_MID, SIG_STEEP)
        
        # If the random probability generated is less than the calculated possibility, stop the while loop and set the new target position.
        if random.random() < posibility:
            break
        
    self.setTarget((x, y, z))