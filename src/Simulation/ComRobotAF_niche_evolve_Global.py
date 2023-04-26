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
from Simulation.ComRobotAF_niche import ComRobotAF_niche
from Simulation.ComRobotAF_niche_evolve import ComRobotAF_niche_evolve

special_colors = [
    "orange",
    "yellow",
    "green",
    "cyan",
    "blue",
    "purple",
    "red",
]

R_I = 500   # 小生境范围
C_TH = 300  # 激活新种群的目标距离

class ComRobotAF_niche_evolve_Global(ComRobotAF_niche_evolve):
    def __init__(self, pos):
        super().__init__(pos)




    def sense(self):
        """Update perception information for the robot."""
        
        # Call parent class's sense method to get basic sensor information (e.g. position, orientation)
        super().sense()
        
        # Add algorithm to update self fitness
        
        # Get all food objects
        foods = getObjectByType(self.mFoodName)
        
        # Update list of all available food objects (indexed by ID)
        for food in foods:
            self.mFoodAll[food.mId] = food
        
        # Based on species type, update list of available food and its positions
        self.mFood.clear()
        if self._species >= 0:
            self.mFood.append(self.mFoodAll[self._species].pos)

        # Get all neighbor robots
        neighors = getObjectByType(self.mObjectType)
        for agent in neighors:
            self.mPopulationAll[agent.mId] = agent
        
        # Update positions of robots in this robot's subgroup
        for subgroup_agent_id in self.mPopulation.keys():
            self.mPopulation[subgroup_agent_id] = self.mPopulationAll[subgroup_agent_id].pos
        
        # Update self fitness based on current position
        self.mFitness = self.getPosFit(self.pos)

        
