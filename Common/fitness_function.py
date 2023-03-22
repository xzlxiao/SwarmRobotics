import sys
sys.path.append('./')
import numpy as np
import matplotlib.pyplot as plt
from common.utils import *
from matplotlib import cm
import math


def getXYMatData(ax_min=-1000, ax_max=1000, ax_step=1): 
    """获得x轴和y轴数据

    Args:
        ax_min (float, optional): 坐标轴最小值. Defaults to -1000.
        ax_max (float, optional): 坐标轴最大值. Defaults to 1000.
        ax_step (float, optional): 数据步长. Defaults to 1.

    Returns:
        numpy.ndarray: x_data(一维), y_data(一维), x_data(二维), y_data(二维)
    """    
    x = np.arange(ax_min, ax_max, ax_step)
    y = np.arange(ax_min, ax_max, ax_step)
    x_mat = np.repeat(np.array([x]), len(y), axis=0).astype(float)
    y_mat = np.repeat(np.array([y]), len(x), axis=0).T.astype(float)
    return x, y, x_mat, y_mat