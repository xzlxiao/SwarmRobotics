"""标准版小生态人工鱼群算法
"""
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random
import copy

from Simulation.ComObjectCollection import *
from Simulation.ComRobotAF_niche_markov import ComRobotAF_niche_markov

special_colors = [
    "orange",
    "yellow",
    "green",
    "cyan",
    "blue",
    "purple",
    "red",
]

R_I = 200   # 小生境范围
C_TH = 300  # 激活新种群的目标距离

class ComRobotAF_niche_markov_Global(ComRobotAF_niche_markov):
    SpecialPool = [
        False,
        False,
        False,
        False,
        False,
    ]
    def __init__(self, pos):
        super().__init__(pos)

    # Define a method named `sense` for the current class instance (self)
    def sense(self):
        """
        This method invokes super().sense() and updates internal state variables to reflect changes in the environment.
        It also calculates the fitness of the current position based on its distance from food sources and other agents.
        """
        # Calls the superclass method `sense()`
        super().sense()
        
        # Commented-out code: updates the location of the food source(s) in the environment
        # self.mFood = getPosByType(self.mFoodName)
        
        # Gets all objects of type 'mFoodName' from the environment, and adds each to a dictionary
        foods = getObjectByType(self.mFoodName)
        for food in foods:
            self.mFoodAll[food.mId] = food
            
        # Updates the list of food sources based on the current object's "species" value
        # If the species is not set, then the list remains empty
        # If the species is set, then only the food source corresponding to that species is added to the list
        self.mFood.clear()
        if self._species >= 0:
            self.mFood.append(self.mFoodAll[self._species].pos)

        # Gets all objects of type mObjectType in the environment, and adds each to a dictionary
        neighors = getObjectByType(self.mObjectType)
        for agent in neighors:
            self.mPopulationAll[agent.mId] = agent
            
        # Updates the positions of population subgroup members
        for subgroup_agent_id in self.mPopulation.keys():
            self.mPopulation[subgroup_agent_id] = self.mPopulationAll[subgroup_agent_id].pos

        # Calculates the fitness of the current position based on distances to food sources and other population subgroup members
        self.mFitness = self.getPosFit(self.pos)



        
