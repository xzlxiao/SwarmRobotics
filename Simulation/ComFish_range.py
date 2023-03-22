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
        super(ComFish_range, self).__init__(pos)
        self.count = 0
        self.setShape('circle')
        self.mColor = (0.4, 0.0, 0.5, 0.8)
        self.mRadius = 20
        self.setColor(self.mColor)

        self.mRange = 300
        self.mRangeColor = "blueviolet"
        self.mRangeAlpha = 0.03
        self.mRangeType = 0         # 0: surface 1: wireframe
        self.mWireframeRstride = 5
        self.mWireframeCstride = 5
        self.isDrawRange = True 
        

    def update(self):
        
        super().update()
        self.count += 1

    def move(self):
        super().move()

    def draw(self, ax):
        if self.isVisible:
            super().draw(ax)
            # self.drawOnFigure(ax)
            if self.isDrawRange:
                u = np.linspace(0, 2 * np.pi, 100)
                v = np.linspace(0, np.pi, 100)
                x = self.mRange * np.outer(np.cos(u), np.sin(v)) + self.mPos[0]
                y = self.mRange * np.outer(np.sin(u), np.sin(v)) + self.mPos[1]
                z = self.mRange * np.outer(np.ones(np.size(u)), np.cos(v)) + self.mPos[2]
                if self.mRangeType == 0:
                    ax.plot_surface(x, y, z, color=self.mRangeColor, alpha=self.mRangeAlpha)
                elif self.mRangeType == 1:
                    ax.plot_wireframe(x, y, z, color=self.mRangeColor, alpha=self.mRangeAlpha, rstride=self.mWireframeRstride, cstride=self.mWireframeCstride)