from Simulation.ComRobot import ComRobot 
from Simulation.ComObject import ComObject 
import copy
import math
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Common import settings
from Simulation.ComRobotCon import ComRobotCon
from Common.DrKDtree import KDtree
import Common.settings as mySettings 
import random
from Common import utils
from Simulation.ComPathPlanning import ComPathPlanning
from Simulation.ComPathPlanning3D import ComPathPlanning3D
import Simulation.ComObjectCollection as ComCol
import matplotlib.path as mpath

star = mpath.Path.unit_regular_star(6)
circle = mpath.Path.unit_circle()
# concatenate the circle with an internal cutout of the star
cut_star = mpath.Path(
    vertices=np.concatenate([circle.vertices, star.vertices[::-1, ...]]),
    codes=np.concatenate([circle.codes, star.codes]))


class ComRobotCommunicationDepict(ComRobotCon):
    def __init__(self, pos):
        super(ComRobotCommunicationDepict, self).__init__(pos)
        self.mFood = []             # 食物      目标
        self.mFoodName = "ComFish"         # 设定用作食物的目标

    def sense(self):
        super().sense()
        self.mFood = list(self.mProcessedInfo['ComFish'].values())

    def draw(self, ax):
        super().draw(ax)
        if len(self.mFood) > 0:
            x, y = self.mFood[0][0], self.mFood[0][1]
            self.setMessage('%.2f,%.2f'%(x, y))
            ax.plot(x, y, marker=cut_star, markersize=20, markerfacecolor=self.mTrailLineColor, alpha=0.3)

    def setInitInfo(self, object_type, object_id, value):
        if object_type not in self.mProcessedInfo:
            self.mProcessedInfo[object_type] = {}
        self.mProcessedInfo[object_type][object_id] = np.array(value)

    def update(self):
        self.sense()
        self.processInfo()
        target = self.mFood[0]
        self.getPlanningControl().setTarget(target)
        if self.isPathPlanning:
            if self.getPlanningControl().mTarget is None:
                self.setPlanningTarget(self.mPos)
            self.pathPlanning()


        self.pathFollowing()
        self.move()

    