import random
import numpy as np
import sys
sys.path.append('./')
from common.utils import distance

class Agent:
    def __init__(self, pos=None) -> None:
        self.pos = pos 
        self.trail = []
        if pos:
            self.trail.append(pos)
        self.info = []
    
    def setPos(self, pos):
        self.pos = pos
        
    def update(self):
        self.trail.append(self.pos)


def initGroup(num):
    return [Agent(random.random() * 100) for i in range(num)]

def getAllAgentPos(group):
    return [agent.pos for agent in group]

def getAveragePos(group):
    pos_group = np.array(getAllAgentPos(group), dtype=float)
    return np.sum(pos_group)/len(pos_group)

def getStd(group):
    """得到坐标的标准差

    Args:
        group (list): Agent Group
    """
    pos_group = np.array(getAllAgentPos(group), dtype=float)
    return np.std(pos_group)

def get3DStd(group):
    """得到坐标的标准差

    Args:
        group (list): Agent Group
    """
    pos_group = getAllAgentPos(group)
    center_pos = getAveragePos(group)
    ret = []
    for pos in pos_group:
        ret.append(distance(pos, center_pos))
    return ret



def remove_nan_data(k1, k2, k3, k4):
    """[summary]

    Args:
        k1 ([type]): [description]
        k2 ([type]): [description]
        k3 ([type]): [description]
        k4 ([type]): [description]
    """
    k_group = [k1, k2, k3, k4]

    for k_i in range(len(k_group)):
        k_nan_ind = np.argwhere(np.isnan(k_group[k_i]))
        for k_j in range(len(k_group)):
            k_group[k_j] = np.delete(k_group[k_j], k_nan_ind)

    return tuple(k_group)
    