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
        super().sense()
        # 添加更新自身fitness的算法
        self.mFitness = self.getPosFit(self.pos)
        self.mFood = self.mProcessedInfo['ComFish'].values()
        self.mPopulation = self.mProcessedInfo['Pos']