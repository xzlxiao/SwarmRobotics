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



class ComRobotSenseDepict(ComRobotCon):
    def __init__(self, pos):
        super(ComRobotSenseDepict, self).__init__(pos)
        self.mUndetected_obj_type = 'ComFish'
        self.mDetected_obj_type = 'ComFish_detected'
        self.mDetectedColor = (0, 0, 0.5, 1)

    def setUndetectedObjType(self, obj_type):
        self.mUndetected_obj_type = obj_type
        if obj_type not in self.mProcessedInfo:
            self.mProcessedInfo[obj_type] = {}

    def setDetectedObjType(self, obj_type):
        self.mDetected_obj_type = obj_type
        if obj_type not in self.mProcessedInfo:
            self.mProcessedInfo[obj_type] = {}

    def sense(self):
        super().sense()
        obj_sensed = self.getObjectBySight()
        for obj in obj_sensed:
            if obj.mObjectType == self.mUndetected_obj_type:
                obj.mObjectType = self.mDetected_obj_type
                obj.setColor(self.mDetectedColor)
                obj.setRadius(obj.mRadius+5)

    
    def update(self):
        self.sense()
        self.processInfo()

        if self.isPathPlanning:
            if self.getPlanningControl().mTarget is None:
                self.setPlanningTarget(self.mPos)
            self.pathPlanning()


        self.pathFollowing()

        self.move()

    