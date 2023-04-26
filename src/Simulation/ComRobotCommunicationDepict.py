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
        """
        This method updates the state of the agent based on its sensory input.
        It sets the mFood instance variable to the positions of all nearby 'ComFish'.

        Returns:
            None
        """
        super().sense()
        self.mFood = list(self.mProcessedInfo['ComFish'].values())

    def draw(self, ax):
        """
        This method draws the representation of the agent on the provided axis object.
        If there is food available, it plots a cut_star marker at the position of the nearest food.

        Args:
            ax: The matplotlib axis object to plot on.

        Returns:
            None
        """
        super().draw(ax)
        if len(self.mFood) > 0:
            x, y = self.mFood[0][0], self.mFood[0][1]
            self.setMessage('%.2f,%.2f'%(x, y))
            ax.plot(x, y, marker=cut_star, markersize=20, markerfacecolor=self.mTrailLineColor, alpha=0.3)

    def setInitInfo(self, object_type, object_id, value):
        """
        This method sets the initial information about an object in the agent's environment.

        Args:
            object_type: A string representing the type of object being initialized.
            object_id: An int representing the ID of the object being initialized.
            value: A numpy array representing the initial value of the object.

        Returns:
            None
        """
        if object_type not in self.mProcessedInfo:
            self.mProcessedInfo[object_type] = {}
        self.mProcessedInfo[object_type][object_id] = np.array(value)

    def update(self):
        """
        This method updates the state of the agent and performs its actions for this time step.
        It senses its environment, processes the information received,
        sets its planning control to move towards the nearest food and moves towards it.

        Returns:
            None
        """
        # Sense the environment and process information
        self.sense()
        self.processInfo()

        # Set the target for the planner as the nearest food and perform path following and movement
        target = self.mFood[0]
        self.getPlanningControl().setTarget(target)
        if self.isPathPlanning:
            if self.getPlanningControl().mTarget is None:
                self.setPlanningTarget(self.mPos)
            self.pathPlanning()
        self.pathFollowing()
        self.move()