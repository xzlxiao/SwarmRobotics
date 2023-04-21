"""
File name: ExperimentUtils.py
Author: 肖镇龙（Zhenlong Xiao）
Description: Experimental tool.
"""

import random
import numpy as np
import sys
sys.path.append('./')
from Common.utils import distance


class Agent:
    def __init__(self, pos=None) -> None:
        """
        Initialize a new instance of the class.

        Args:
            pos (tuple, optional): A tuple representing the position. Defaults to None.
        """
         # Store the given position and an empty trail list and info list
        self.pos = pos 
        self.trail = []
        
        # If a position is provided, add it to the trail list
        if pos:
            self.trail.append(pos)
        self.info = []
    
    def setPos(self, pos):
        """
        Sets the position of the object.

        Args:
            pos (_type_): The new position of the object.
        """        
        self.pos = pos
        
    def update(self):
        """
        adds the current position to a "trail" list

        """        
        self.trail.append(self.pos)


def initGroup(num):
    """
    Returns a list of Agent objects with length equal to the input argument

    Args:
        num (int): _description_

    Returns:
        _type_: a list of Agent objects with length equal to the input argument
    """    
    return [Agent(random.random() * 100) for i in range(num)]

def getAllAgentPos(group):
    """
    return the position of every agent that is in the group

    Args:
        group (_type_): agent group

    Returns:
        list: a list of positions
    """    
    return [agent.pos for agent in group]

def getAveragePos(group):
    """
    A function that calculates the average position of agents in a given group.

    Args:
        group (type): The group for which to calculate the average position.

    Returns:
        type: The average position of the agents in the group.
    """    
    pos_group = np.array(getAllAgentPos(group), dtype=float)
    return np.sum(pos_group)/len(pos_group)

def getStd(group):
    """
    A function that calculates the standard deviation of positions for a given group of agents.

    Args:
        group (list): The group for which to calculate the standard deviation of positions.

    Returns:
        float: The standard deviation of positions in the group.
    """
    pos_group = np.array(getAllAgentPos(group), dtype=float)
    return np.std(pos_group)

def get3DStd(group):
    """
    A function that returns the standard deviation of the distances from each agent's position to the center position of a given group.

    Args:
        group (list): The group for which to calculate the standard deviation of distances.

    Returns:
        list: A list of distances from each agent's position to the center position of the group.
    """
    pos_group = getAllAgentPos(group)
    center_pos = getAveragePos(group)
    ret = []
    for pos in pos_group:
        ret.append(distance(pos, center_pos))
    return ret



def remove_nan_data(k1, k2, k3, k4):
    """
    Removes NaN (Not a Number) data from multiple lists and returns the modified lists as a tuple.

    Args:
        k1 (list): Input list 1
        k2 (list): Input list 2
        k3 (list): Input list 3
        k4 (list): Input list 4

    Returns:
        tuple: A tuple of lists with NaN values removed.
    """
    k_group = [k1, k2, k3, k4]

    for k_i in range(len(k_group)):
        k_nan_ind = np.argwhere(np.isnan(k_group[k_i]))
        for k_j in range(len(k_group)):
            k_group[k_j] = np.delete(k_group[k_j], k_nan_ind)

    return tuple(k_group)
    