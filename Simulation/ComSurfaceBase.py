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
        if ax is not None:
            self.mAx = ax
        else:
            raise

    def setX(self, x: np.ndarray):
        self.mX = x

    def setY(self, y: np.ndarray):
        self.mY = y

    def setData(self, data: np.ndarray):
        self.mData = data 

    def setCMap(self, cmap):
        self.mCMap = cmap 

    def setOffset(self, offset: float):
        self.mOffset = offset

    def setZDir(self, zdir: str):
        self.mZDir = zdir

    def setAlpha(self, alpha):
        self.mAlpha = alpha
    
    def setPopulation(self, population: list):
        self.mPopulation = population

    def draw(self):
        if self.mAx is not None:
            # print(self.mData.shape)
            x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
            y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
            data = self.mData
            if isCupy:
                x_mat = x_mat.get()
                y_mat = y_mat.get()
                data = data.get()
            if self.mZDir == '2D':
                # print(self.mPlotType)
                if self.mPlotType == PlotType.type_contour:
                    self.mAx.contour(x_mat, y_mat, data, cmap=self.mCMap, alpha=self.mAlpha) 
                elif self.mPlotType == PlotType.type_contourf:
                    # print(len(self.mX), len(self.mY), data.shape)
                    self.mAx.contourf(x_mat, y_mat, data, vmax=100, cmap=self.mCMap, alpha=self.mAlpha)
            else:
                self.mAx.contourf(x_mat, y_mat, data, zdir=self.mZDir, offset=self.mOffset, cmap=self.mCMap, alpha=self.mAlpha)

                if self.isShowAgentMark:
                    offset = self.mOffset
                    if self.mZDir == 'y':
                        offset = -offset

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
                    self.mAx.scatter(self.mAgentMarkXs, self.mAgentMarkYs, zs=offset, zdir=self.mZDir, color="red", marker='o', alpha=1)
                if self.isPlotBoundingBox:
                    if self.mZDir == 'z':
                        self.mAx.plot((self.mX[0], self.mX[-1]), (self.mY[0], self.mY[0]), (self.mOffset, self.mOffset), 'cyan', linewidth=2, alpha=1)
                        self.mAx.plot((self.mX[-1], self.mX[-1]), (self.mY[0], self.mY[-1]), (self.mOffset, self.mOffset), 'cyan', linewidth=2, alpha=1)
                        self.mAx.plot((self.mX[0], self.mX[-1]), (self.mY[-1], self.mY[-1]), (self.mOffset, self.mOffset), 'cyan', linewidth=2, alpha=1)
                        self.mAx.plot((self.mX[0], self.mX[0]), (self.mY[0], self.mY[-1]), (self.mOffset, self.mOffset), 'cyan', linewidth=2, alpha=1)
                    elif self.mZDir == 'y':
                        pass

                    elif self.mZDir == 'x':
                        pass

    def update(self):
        if self.mZDir == 'z' or self.mZDir == '2D':
            if len(self.mPopulation) > 0 and self.isShowAgentMark:
                self.mAgentMarkXs = [agent.mPos[0] for agent in self.mPopulation]
                self.mAgentMarkYs = [agent.mPos[1] for agent in self.mPopulation]

        elif self.mZDir == 'y':
            if len(self.mPopulation) > 0 and self.isShowAgentMark:
                self.mAgentMarkXs = [agent.mPos[0] for agent in self.mPopulation]
                self.mAgentMarkYs = [agent.mPos[2] for agent in self.mPopulation]

        elif self.mZDir == 'x':
            if len(self.mPopulation) > 0 and self.isShowAgentMark:
                self.mAgentMarkXs = [agent.mPos[1] for agent in self.mPopulation]
                self.mAgentMarkYs = [agent.mPos[2] for agent in self.mPopulation]
        