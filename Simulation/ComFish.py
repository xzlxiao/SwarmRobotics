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
        super(ComFish, self).__init__()
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
        self.mObjectType = "ComFish"       # 用于标识当前物体类别
        self.isVisible = True 
        self.delayVisibleCount = 0

    @property
    def fish_count(self):
        return ComFish._fish_count

    def update(self):
        if (self.pos == self.target).all():
            self.chooseRandomTarget()
        super().update()

    def move(self):
        if not self.isVisible:
            if self.count >= self.delayVisibleCount:
                self.isVisible = True
        super().move()