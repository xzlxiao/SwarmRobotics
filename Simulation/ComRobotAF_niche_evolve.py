"""进化版小生态人工鱼群算法
"""
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from ast import Num
import random
import copy
import math
from Common.utils import *
from Common import settings

from Simulation.ComObjectCollection import *
from Simulation.ComRobotAF_niche import *



class ComRobotAF_niche_evolve(ComRobotAF_niche):
    
    def __init__(self, pos):
        super().__init__(pos)
        self.isDistancedTravel = False # 子群突变会主群，进行远距离移动，该值为True，到达目的地后变为False
        self.mKeepSpeciesTime = 0.0    # 维持种群的时间
    def update(self):
        """Update robot status and actions."""
        
        # Increment time since last species change
        self.mKeepSpeciesTime += settings.CS_INTERVAL
        
        # If this is robot 0, print population information
        if self.mId == 0:
            main_pop = len(ComRobotAF_niche.PopulationGroup[-1])
            sub_pop0 = len(ComRobotAF_niche.PopulationGroup[0])
            sub_pop1 = len(ComRobotAF_niche.PopulationGroup[1])
            sub_pop2 = len(ComRobotAF_niche.PopulationGroup[2])
            sub_pop3 = len(ComRobotAF_niche.PopulationGroup[3])
            sub_pop4 = len(ComRobotAF_niche.PopulationGroup[4])
            sub_pop5 = len(ComRobotAF_niche.PopulationGroup[5])
            sub_pop6 = len(ComRobotAF_niche.PopulationGroup[6])
            sub_pop7 = len(ComRobotAF_niche.PopulationGroup[7])
            sub_pop8 = len(ComRobotAF_niche.PopulationGroup[8])
            sub_pop9 = len(ComRobotAF_niche.PopulationGroup[9])
            
            # Print population counts for each subgroup
            print("""
            main pop: {}
            sub0: {}
            sub1: {}
            sub2: {}
            sub3: {}
            sub4: {}
            sub5: {}
            sub6: {}
            sub7: {}
            sub8: {}
            sub9: {}
            ---------------------------------
            """.format(main_pop, sub_pop0, sub_pop1, sub_pop2, sub_pop3, sub_pop4, sub_pop5, sub_pop6, sub_pop7, sub_pop8, sub_pop9))
        
        # Update perception information
        self.sense()
        
        # Process information to update internal state
        self.processInfo()
        
        # If robot is not currently in a subgroup, attempt to find a new subgroup based on nearby food objects
        if self._species == -1:
            for food in self.mFoodAll.values():
                if not ComRobotAF_niche.SpecialPool[food.mId]:
                    if self.distance(food.pos, self.pos) < C_TH:
                        print("set species {}".format(food.mId))
                        
                        # Set this robot's subgroup to the one associated with the nearest food object
                        ComRobotAF_niche.SpecialPool[food.mId] = True
                        current_population = [(agent_id, agent_pos) for agent_id, agent_pos in ComRobotAF_niche.PopulationGroup[-1].items()]
                        for agent_id, agent_pos in current_population:
                            if self.distance(agent_pos, self.pos) < R_I:
                                self.mPopulationAll[agent_id].setSpecies(food.mId)
                        break
                        
                else:
                    if not self.isDistancedTravel:
                        if self.distance(food.pos, self.pos) < R_I:
                            # If robot is within range of a food source and already in a subgroup, update its position and fitness
                            self.mPopulationAll[self.mId].setSpecies(food.mId)
                        else:
                            # If robot is too far from food source, there is a chance it will mutate into a new subgroup
                            new_species_id = self.mutateToSubspecies()
                            if new_species_id != -1:
                                self.setSpecies(new_species_id)
        
        else:
            # If the robot is already in a subgroup...
            
            if self.isReturnToMainSwarm():
                # If robot is too far from other robots in its subgroup, return to main swarm
                print("return main swarm: {}.{} -> main".format(self._species, self.mId))
                self.randomDistancedMove()
                self.setSpecies(-1)
            
            if self._species == -1:
                # If robot is not in a subgroup, move randomly
                self.randomMove()
            else:
                # Otherwise, use AF algorithm to move towards other robots in the same subgroup
                self.AFfast()
            
            # Move the robot based on its updated position and orientation
            self.move()

    # This is a method called setSpecies and it takes a parameter called species_id
    def setSpecies(self, species_id):
        """
        This method sets the value of species_id for an instance of a class.
        
        Args:
            species_id (int): The id of the species being set
        
        Returns:
            None
        """
        # The super() function is used to call a method from a parent class
        super().setSpecies(species_id)
        # This line of code initializes an instance variable mKeepSpeciesTime to 0.0
        self.mKeepSpeciesTime = 0.0


    def isReturnToMainSwarm(self):
        """
        Determines whether or not a species will return to the main swarm.
        
        Returns:
            bool: True if the species mutates into the main swarm, False otherwise.
        """     
        if random.random() < self.getMainspeciesMutationChance():
            return True
        # aggregation_degree = self.nicheAggregationDegree()
        # if aggregation_degree > NICHE_aggregation_degree_threshold:
        #     # print(aggregation_degree)
        #     niche_size = len(self.mPopulation)
        #     current_time = self.getTime()
        #     rho_i = niche_size * AGG_lambda * (1.0/(1.0+math.e**(current_time*aggregation_degree/niche_size)) - 0.5)
            
        #     if rho_i > 1:
                
        #         dist = self.distance(self.mPos, self.mFood[0])
        #         for agent_pos in self.mPopulation.values():
        #             if self.distance(self.mFood[0], agent_pos) > dist:
        #                 return False 
        #         return True 
        return False

    # This is a method called mutateToSubspecies that returns an integer value
    def mutateToSubspecies(self) -> int:
        """
        Mutates an agent to a new subspecies based on a mutation chance and existing species data.
        
        Returns:
            int: The ID of the new subspecies or -1 if no mutations occurred.
        """        
        # Enumerate loops over an iterable and returns tuples containing the index and value of each element in the iterable
        # SpecialPool is a list of booleans indicating whether or not a species has been initialized
        for species_id, is_species_init in enumerate(ComRobotAF_niche.SpecialPool):
            if is_species_init:
                # random.random() returns a random float between 0 and 1
                # self.getSubspeciesMutationChance() retrieves a mutation chance based on the given population group from an instance variable of the class
                # ComRobotAF_niche.PopulationGroup[species_id] retrieves the population group of the given species from a class constant
                # If the random float is less than the mutation chance, the method returns the species ID
                if random.random() < self.getSubspeciesMutationChance(ComRobotAF_niche.PopulationGroup[species_id]):
                    return species_id
        # If no mutations occurred, the method returns -1
        return -1


    # This is a method that calculates the probability of an agent mutating to a certain subspecies based on distance and population size
    def getSubspeciesMutationChance(self, population) -> Num:
        """
        Calculates the probability of an agent mutating to a certain subspecies based on distance and population size.
        
        Args:
            population: The ID of the population group.
        
        Returns:
            float: The mutation chance.
        """
        # len() returns the number of elements in a list or string
        # self.mPopulation refers to the current population ID
        # len(self.mPopulation) finds the number of agents in the current population
        # len(self.mPopulationAll) finds the total number of agents across all populations
        # settings.CS_INTERVAL is a constant value
        return 1.0 * len(self.mPopulation)/len(self.mPopulationAll) * settings.CS_INTERVAL

    # This is a method that calculates the probability of an agent mutating to the main species based on population size
    def getMainspeciesMutationChance(self) -> Num:
        """
        Calculates the probability of an agent mutating to the main species based on population size.
        
        Returns:
            float: The mutation chance.
        """
        return 1.0 * len(self.mPopulation)/len(self.mPopulationAll) * settings.CS_INTERVAL

    # This is a method that randomly moves the agent based on distance
    def randomDistancedMove(self):
        """
        Randomly moves the agent based on distance.
        """
        # The chooseRandomDistancedTarget() method chooses a new target for the agent
        self.chooseRandomDistancedTarget()
        self.isDistancedTravel = True

    

    # Define a method named `move` for the current class instance (self)
    def move(self):
        """
        This method invokes super().move() and sets self.isDistancedTravel = False if self.isStopping() is True.
        """
        # Calls the superclass method `move()`
        super().move()
        
        # Checks whether the current object is stopping, and updates the value of self.isDistancedTravel accordingly
        if self.isStopping():
            self.isDistancedTravel = False


    def getPosFit(self, position):
        """
        Calculates the fitness of the agent's current position based on food locations.
        
        Args:
            position: The position of the agent in the environment.
        
        Returns:
            float: The fitness value.
        """
        fitness = 0.0
        
        if len(self.mFood) > 0:
            for food_pos in self.mFood:
                fitness_tmp = 1 - (np.linalg.norm(np.array(position, dtype=np.float32) - food_pos, ord=2) / self.mSenseDistance)
                # fitness_tmp = 1 / (np.linalg.norm(np.array(position, dtype=np.float32) - food_pos, ord=2) + 0.000000000000001)
                if fitness_tmp > fitness:
                    fitness = fitness_tmp
        # fitness = sigmoid(fitness, 0.5, 1)
        return fitness