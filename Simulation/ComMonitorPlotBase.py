from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False


class ComMonitorPlotBase:
    def __init__(self) -> None:
        self.mPopulation = None 
        self.mData = None
        self.mX = None 
        self.mY = None 
        self.mCMap = cm.coolwarm
        self.mAlpha = 0.5

    def update(self):
        pass

    def draw(self, ax_group):
        for ind, agent_id in enumerate(self.mRandomInds):
            agentMarkXs = [agent_pos[0] for agent_pos in self.mPopulation[agent_id].mPopulation.values()]
            agentMarkYs = [agent_pos[1] for agent_pos in self.mPopulation[agent_id].mPopulation.values()]
            ax_group[ind].scatter(agentMarkXs, agentMarkYs, color="red", marker='o', alpha=1)

    def setX(self, x: np.ndarray):
        self.mX = x

    def setY(self, y: np.ndarray):
        self.mY = y

    def setCMap(self, cmap):
        self.mCMap = cmap 

    def setAlpha(self, alpha):
        self.mAlpha = alpha

    def setPopulation(self, population: list):
        self.mPopulation = population