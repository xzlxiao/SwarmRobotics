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
from Simulation.ComFish import ComFish
import math
import copy


class ComFish_range(ComFish):
    def __init__(self, pos):
        """
        This method initializes an instance of the ComFish_range class.

        Args:
            pos: A tuple representing position coordinates on the map.

        Returns:
            None
        """

        # Call superclass's __init__() method and set initial attributes
        super(ComFish_range, self).__init__(pos)
        self.count = 0              # Counter for range calculations
        self.setShape('circle')     # Set shape of object to circle
        self.mColor = (0.4, 0.0, 0.5, 0.8)   # Set base color of object
        self.mRadius = 20           # Set radius of object
        self.setColor(self.mColor)  # Set final color of object

        # Set additional attributes for range visualization
        self.mRange = 300               # Set maximum range of visibility
        self.mRangeColor = "blueviolet" # Set color of range visualization
        self.mRangeAlpha = 0.03         # Set transparency level of range visualization
        self.mRangeType = 0             # Set type of range visualization, 0: surface 1: wireframe
        self.mWireframeRstride = 5      # Set r-stride of wireframe-type range visualization
        self.mWireframeCstride = 5      # Set c-stride of wireframe-type range visualization
        self.isDrawRange = True         # Flag to determine whether range should be drawn or not

        

    def update(self):
        """This method updates the state of the ComFish_range object.

        Args:
            None

        Returns:
            None
        """

        # Call superclass's update() method to update position
        super().update()

        # Increment the count attribute by 1
        self.count += 1


    def move(self):
        """
        This method moves the ComFish_range object.

        Args:
            None

        Returns:
            None
        """

        # Call superclass's move() method to move the object
        super().move()


    def draw(self, ax):
        """
        This method draws the ComFish_range object on a 3D plot.

        Args:
            ax: Matplotlib Axes3D object that represents the axes of the 3D plot.

        Returns:
            None
        """

        # Check if object is visible before drawing it
        if self.isVisible:

            # Call superclass's draw() method to draw the object
            super().draw(ax)

            # Draw range surface or wireframe, depending on range type
            if self.isDrawRange:

                # Generate coordinates for the surface or wireframe
                u = np.linspace(0, 2 * np.pi, 100)
                v = np.linspace(0, np.pi, 100)
                x = self.mRange * np.outer(np.cos(u), np.sin(v)) + self.mPos[0]
                y = self.mRange * np.outer(np.sin(u), np.sin(v)) + self.mPos[1]
                z = self.mRange * np.outer(np.ones(np.size(u)), np.cos(v)) + self.mPos[2]

                # Check if range type is surface, and draw accordingly
                if self.mRangeType == 0:
                    ax.plot_surface(x, y, z, color=self.mRangeColor, alpha=self.mRangeAlpha)

                # Check if range type is wireframe, and draw accordingly
                elif self.mRangeType == 1:
                    ax.plot_wireframe(x, y, z, color=self.mRangeColor, alpha=self.mRangeAlpha, rstride=self.mWireframeRstride, cstride=self.mWireframeCstride)
