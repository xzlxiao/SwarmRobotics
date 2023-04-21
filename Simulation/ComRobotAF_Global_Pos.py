# -*- coding: utf-8 -*-


from networkx.algorithms.assortativity import neighbor_degree
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random
from Simulation.ComObject import ComObject 
import math
import copy
from Simulation.ComObjectCollection import *
from Simulation.ComRobotAF import ComRobotAF
from Simulation.ComRobotAFfast import ComRobotAFfast


class ComRobotAF_Global_Pos(ComRobotAFfast):
    def __init__(self, pos):
        super().__init__(pos)

    def sense(self):
        """Update agent's fitness based on its position and environment objects.
        
        This method updates the fitness value of the current instance using a certain algorithm,
        gets the positions of all objects with a certain name from the environment,
        gets a list of neighbor agents with a certain object type from the environment,
        and sets each agent's position in the population dictionary to its own position.
        
        Returns:
            None.
        """
        # This calls the 'sense' method of the superclass of the current class.
        super().sense()

        # This updates the fitness value of the current instance using a certain algorithm.
        self.mFitness = self.getPosFit(self.pos)

        # This gets the positions of all objects with a certain name from the environment.
        self.mFood = getPosByType(self.mFoodName)

        # This gets a list of neighbor agents with a certain object type from the environment.
        neighors = getObjectByType(self.mObjectType)

        # This loop sets each agent's position in the population dictionary to its own position.
        for agent in neighors:
            self.mPopulation[agent.mId] = agent.pos