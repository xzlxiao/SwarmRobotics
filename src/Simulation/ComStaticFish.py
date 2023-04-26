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
from Simulation.ComFish import ComFish 
import math
import copy


class ComStaticFish(ComFish):
    def __init__(self, pos):
        super(ComStaticFish, self).__init__(pos)
        self.mSpeed = 0

    def update(self):
        """
        This function updates the simulation by calling the super class's update function.
        """        
        super().update()

    def move(self):
        """
        This function moves the robot by calling the super class's move function.
        """        
        super().move()
