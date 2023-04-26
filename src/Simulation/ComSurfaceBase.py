from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False

from enum import Enum

class PlotType:
    type_contour = 0
    type_contourf = 1
    

class ComSurfaceBase:
    def __init__(self, ax = None) -> None:
        self.mX = None 
        self.mY = None 
        self.mData = None
        self.mCMap = cm.coolwarm
        self.mOffset = .0
        self.mZDir = "z"
        self.mAx = ax 
        self.mAlpha = 0.5
        self.mX, self.mY, self.mData = axes3d.get_test_data(0.05)
        self.isShowAgentMark = True
        self.mAgentMarkXs = []
        self.mAgentMarkYs = []
        self.mPopulation = None 
        self.isPlotBoundingBox = True
        self.mPlotType = PlotType.type_contour

    def setAx(self, ax):
        """
        This function sets the axis object for the plot. If the given axis object is not None,
        it sets that as the new axis object for the plot, otherwise, it raises an exception.
        """
        if ax is not None:
            self.mAx = ax
        else:
            raise Exception("Axis object cannot be None")

    def setX(self, x: np.ndarray):
        """
        This function sets the X-axis data for the plot.
        """
        self.mX = x

    def setY(self, y: np.ndarray):
        """
        This function sets the Y-axis data for the plot.
        """
        self.mY = y

    def setData(self, data: np.ndarray):
        """
        This function sets the data to be plotted. The data should contain information about
        the values to be plotted and their corresponding coordinates on the X and Y axes.
        """
        self.mData = data 

    def setCMap(self, cmap):
        """
        This function sets the colormap to be used for the plot. Colormaps are used to assign
        colors to different data points based on their values.
        """
        self.mCMap = cmap 

    def setOffset(self, offset: float):
        """
        This function sets the offset for the data. The offset is a value that shifts the data
        along the Z-axis of the plot.
        """
        self.mOffset = offset

    def setZDir(self, zdir: str):
        """
        This function sets the direction of the Z-axis. The options are 'x', 'y' or 'z'.
        """
        self.mZDir = zdir

    def setAlpha(self, alpha):
        """
        This function sets the transparency level (alpha) for the data points in the plot.
        """
        self.mAlpha = alpha

    def setPopulation(self, population: list):
        """
        This function sets the population data (if applicable) for the plot. The population
        data should contain information about the total size of the population and other
        relevant statistics.
        """
        self.mPopulation = population


    def draw(self):
        """
        This function draws the plot. It first checks if an axis object has been set, and if so,
        it creates a grid of X and Y coordinates based on the data points, sets the colormap and other
        attributes, and then plots the data using contour or contourf plots.
        If the Z-direction is not '2D', it also shows the agents on the plot by drawing lines between their
        positions and the offset point along the Z-axis.
        If the plot bounding box flag is set to true, it also plots the bounding box around the data.
        """
        if self.mAx is not None:
            x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
            y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
            data = self.mData
            
            # Check if data is in cupy format (used for GPU acceleration)
            if isCupy:
                x_mat = x_mat.get()
                y_mat = y_mat.get()
                data = data.get()
            
            # 2D plot without markers
            if self.mZDir == '2D':
                if self.mPlotType == PlotType.type_contour:
                    self.mAx.contour(x_mat, y_mat, data, cmap=self.mCMap, alpha=self.mAlpha) 
                elif self.mPlotType == PlotType.type_contourf:
                    self.mAx.contourf(x_mat, y_mat, data, vmax=100, cmap=self.mCMap, alpha=self.mAlpha)
            else:
                # Plot with markers
                self.mAx.contourf(x_mat, y_mat, data, zdir=self.mZDir, offset=self.mOffset, cmap=self.mCMap,
                                alpha=self.mAlpha)
                
                if self.isShowAgentMark:
                    offset = self.mOffset
                    if self.mZDir == 'y':
                        offset = -offset
                    
                    # Add markers for each agent
                    for agent in self.mPopulation:
                        x = [agent.mPos[0]]
                        y = [agent.mPos[1]]
                        z = [agent.mPos[2]]
                        if self.mZDir == 'z':
                            x.append(agent.mPos[0])
                            y.append(agent.mPos[1])
                            z.append(self.mOffset)
                        elif self.mZDir == 'y':
                            x.append(agent.mPos[0])
                            y.append(-self.mOffset)
                            z.append(agent.mPos[2])
                        elif self.mZDir == 'x':
                            x.append(self.mOffset)
                            y.append(agent.mPos[1])
                            z.append(agent.mPos[2])
                            
                        self.mAx.plot(x, y, z, 'k', alpha=1, linewidth=1.5)
                    
                    # Add red markers for the specified coordinates
                    self.mAx.scatter(self.mAgentMarkXs, self.mAgentMarkYs, zs=offset, zdir=self.mZDir, color="red",
                                    marker='o', alpha=1)
                    
                # Plot bounding box
                if self.isPlotBoundingBox:
                    if self.mZDir == 'z':
                        self.mAx.plot((self.mX[0], self.mX[-1]), (self.mY[0], self.mY[0]), (self.mOffset, self.mOffset),
                                        'cyan', linewidth=2, alpha=1)
                        self.mAx.plot((self.mX[-1], self.mX[-1]), (self.mY[0], self.mY[-1]), (self.mOffset, self.mOffset),
                                        'cyan', linewidth=2, alpha=1)
                        self.mAx.plot((self.mX[0], self.mX[-1]), (self.mY[-1], self.mY[-1]), (self.mOffset, self.mOffset),
                                        'cyan', linewidth=2, alpha=1)
                        self.mAx.plot((self.mX[0], self.mX[0]), (self.mY[0], self.mY[-1]), (self.mOffset, self.mOffset),
                                        'cyan', linewidth=2, alpha=1)
                    elif self.mZDir == 'y':
                        pass
                    elif self.mZDir == 'x':
                        pass

    def update(self):
        """This method updates the positions of agents in the plot based on their coordinates and the value of self.mZDir.
        """
        if self.mZDir == 'z' or self.mZDir == '2D':  # If the z direction is 'z' or '2D'
            if len(self.mPopulation) > 0 and self.isShowAgentMark:  # If there's at least one agent and self.isShowAgentMark is True
                self.mAgentMarkXs = [agent.mPos[0] for agent in self.mPopulation]  # Extract the x position of each agent from its mPos attribute and store it in the list mAgentMarkXs
                self.mAgentMarkYs = [agent.mPos[1] for agent in self.mPopulation]  # Extract the y position of each agent from its mPos attribute and store it in the list mAgentMarkYs

        elif self.mZDir == 'y':  # If the z direction is 'y'
            if len(self.mPopulation) > 0 and self.isShowAgentMark:  # If there's at least one agent and self.isShowAgentMark is True
                self.mAgentMarkXs = [agent.mPos[0] for agent in self.mPopulation]  # Extract the x position of each agent from its mPos attribute and store it in the list mAgentMarkXs
                self.mAgentMarkYs = [agent.mPos[2] for agent in self.mPopulation]  # Extract the z position of each agent from its mPos attribute and store it in the list mAgentMarkYs

        elif self.mZDir == 'x':  # If the z direction is 'x'
            if len(self.mPopulation) > 0 and self.isShowAgentMark:  # If there's at least one agent and self.isShowAgentMark is True
                self.mAgentMarkXs = [agent.mPos[1] for agent in self.mPopulation]  # Extract the y position of each agent from its mPos attribute and store it in the list mAgentMarkYs
                self.mAgentMarkYs = [agent.mPos[2] for agent in self.mPopulation]  # Extract the z position of each agent from its mPos attribute and store it in the list mAgentMarkYs
