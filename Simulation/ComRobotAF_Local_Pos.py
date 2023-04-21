# -*- coding: utf-8 -*-


isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random
from Simulation.ComRobotAF import ComRobotAF
from Simulation.ComRobotAFfast import ComRobotAFfast


class ComRobotAF_Local_Pos(ComRobotAFfast):
    def __init__(self, pos):
        super().__init__(pos)

    def sense(self):
        """
        Updates the agent's fitness, food, and population information based on its position and environment objects.
        """        
        super().sense()
        # Add an algorithm to update your fitness
        self.mFitness = self.getPosFit(self.pos)

        # Retrieve all 'ComFish' food items in the environment and store them in the agent's `mFood` attribute
        self.mFood = self.mProcessedInfo['ComFish'].values()
        
        # Retrieve information about other agents in the environment and store it in the agent's `mPopulation` attribute
        # The retrieved information is stored as a dictionary where the keys are the agents' names and the values are their positions
        self.mPopulation = self.mProcessedInfo['Pos']