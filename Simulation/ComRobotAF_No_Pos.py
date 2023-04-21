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


class ComRobotAF_No_Pos(ComRobotAFfast):
    def __init__(self, pos):
        super().__init__(pos)
        self.isCommunicating = False

    # Define a method called 'sense' within a class that takes no parameters.
    def sense(self):
        """
        This method is used to update the fitness of the object itself by calling the 'getPosFit' method on its position.
        It also stores information about food and population in instance variables.
        """ 
        # Call the 'sense' method of the parent class using 'super()' function.
        super().sense()
        
        # Update the fitness value of the object itself based on its position.
        self.mFitness = self.getPosFit(self.pos)
        
        # Store the value of available food in an instance variable named 'mFood'.
        self.mFood = self.mProcessedInfo['ComFish'].values()
        
        # Store the value of population in an instance variable named 'mPopulation'.
        self.mPopulation = self.mProcessedInfo['ComRobotAF']
