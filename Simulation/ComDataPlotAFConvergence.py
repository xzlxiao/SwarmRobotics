try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False

from Simulation.ComDataPlotBase import ComDataPlotBase
from Common.utils import distance

class ComDataPlotAFConvergence(ComDataPlotBase):
    def __init__(self) -> None:
        super().__init__()
        self.mDataX = []
        self.mTime = None
        self.mPopulation = None 
        self.mTarget = None 
        self.mCount = 0

    def setTimeRef(self, time: list):
        self.mTime = time

    def setPopulation(self, population):
        self.mPopulation = population
    
    def setTarget(self, target):
        self.mTarget = target

    def update(self):
        super().update()
        if self.mDataY is None: 
            self.mDataY = np.array([i*self.mTime[1]/20 for i in range(20)])
            for _ in range(self.mDataY.shape[0]):
                self.mDataX.append(np.array([]))
            
        if self.mPopulation is not None and self.mTarget is not None and self.mTime is not None:
            if self.mTime[0] >= self.mCount * self.mTime[1] / 20:
                agent_pos = np.array([agent.mPos for agent in self.mPopulation])
                data = distance(agent_pos, self.mTarget.mPos)
                self.mDataX[self.mCount] = data
                self.mCount += 1
                
                # self.mDataX.append(data)
                
    def draw(self, ax):
        super().draw(ax)
        # ax.set_xlim(0, int(self.mTime[1]))
        ax.boxplot(self.mDataX,labels=self.mDataY,patch_artist=True)
        # ax.boxplot(self.mDataX,patch_artist=True)
        