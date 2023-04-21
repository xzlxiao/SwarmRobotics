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


class ComFish(ComObject):
    _fish_count = 0
    def __init__(self, pos):
        """
        This constructor initializes a ComFish object with the given position.
        
        Args:
            pos: A list or tuple representing the 3D coordinates of the fish's position.
        
        Returns:
            None
        """
        
        # Call superclass's constructor to initialize object
        super(ComFish, self).__init__()

        # Set initial values for properties
        self.mPos = np.array(pos, dtype=np.float32)
        self.mTarget = np.array(self.mPos, dtype=np.float32)
        self.mId = ComFish._fish_count
        self.mSenseDistance = 300
        self.isShowSenseRange = False
        ComFish._fish_count += 1
        self.setShape('fish')
        self.mColor = (0.0, 0.5, 0.0, 0.8)
        self.mRadius = 200
        self.setColor(self.mColor)
        self.mObjectType = "ComFish"       # Used to identify the type of object
        self.isVisible = True 
        self.delayVisibleCount = 0
        self.isPlotTargetLine = False


    @property
    def fish_count(self):
        """
        This method returns the total number of ComFish objects created so far.
        
        Args:
            None
        
        Returns:
            An integer representing the count of all ComFish objects created so far.
        """
        
        return ComFish._fish_count


    def update(self):
        """
        This method updates the position of the ComFish object in the game.
        
        If the fish has reached its target position, it chooses a new random target to swim towards.
        
        Args:
            None
        
        Returns:
            None
        """
        
        if (self.pos == self.target).all():
            # If current position equals target position, choose a new random target to swim towards
            self.chooseRandomTarget()
            
        # Call super class's update method to update the fish's position and movement
        super().update()


    def move(self):
        """
        This method moves the character horizontally across the screen.
        
        If the character is currently invisible, it checks if a certain time delay has passed before becoming visible again.
        
        Args:
            None
        
        Returns:
            None
        """
        
        if not self.isVisible:
            # If the character is currently invisible...
            
            if self.count >= self.delayVisibleCount:
                # ...check if a certain number of update cycles have passed (delayVisibleCount)...
                
                self.isVisible = True  # ...and make the character visible again
                
        # Call the superclass's move method to move the character horizontally
        super().move()
