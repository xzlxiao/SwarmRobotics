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
        """Method to set the type of undetected object.

        Args:
            obj_type (type): The type of object.
        """
        self.mUndetected_obj_type = obj_type
        
        # Create an empty dictionary for processed info if it doesn't exist.
        if obj_type not in self.mProcessedInfo:
            self.mProcessedInfo[obj_type] = {}

    def setDetectedObjType(self, obj_type):
        """Method to set the type of detected object.

        Args:
            obj_type (type): The type of object.
        """
        self.mDetected_obj_type = obj_type
        
        # Create an empty dictionary for processed info if it doesn't exist.
        if obj_type not in self.mProcessedInfo:
            self.mProcessedInfo[obj_type] = {}

    def sense(self):
        """Method to sense objects in the robot's environment.
        """
        # Call the parent class sense method.
        super().sense()
        
        # Get objects sensed by the robot.
        obj_sensed = self.getObjectBySight()
        
        # Iterate over sensed objects.
        for obj in obj_sensed:
            # Check if the object is of the undetected type.
            if obj.mObjectType == self.mUndetected_obj_type:
                # Change the object's type to the detected type.
                obj.mObjectType = self.mDetected_obj_type
                obj.setColor(self.mDetectedColor)
                obj.setRadius(obj.mRadius+5)

    def update(self):
        """Method to update the robot's state.
        """
        self.sense() # Sense the environment.
        self.processInfo() # Process environmental data.
        
        # If path planning is enabled, plan and follow a path to the target.
        if self.isPathPlanning:
            if self.getPlanningControl().mTarget is None:
                self.setPlanningTarget(self.mPos)
            self.pathPlanning()
        self.pathFollowing() # Follow the planned path.
        self.move() # Move the robot to its destination.


    