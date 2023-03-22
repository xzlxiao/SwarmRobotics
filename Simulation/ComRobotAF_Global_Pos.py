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
        super().sense()
        # 添加更新自身fitness的算法
        self.mFitness = self.getPosFit(self.pos)
        self.mFood = getPosByType(self.mFoodName)
        neighors = getObjectByType(self.mObjectType)
        for agent in neighors:
            self.mPopulation[agent.mId] = agent.pos