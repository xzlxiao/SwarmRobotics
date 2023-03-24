from matplotlib import cm, markers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from Common import utils
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Simulation.ComSurfaceBase import ComSurfaceBase
from Simulation.ComObjectCollection import *
from Common.DrKDtree import KDtree
import Common.settings as mySettings 


class ComSurfaceCrowded(ComSurfaceBase):
    def __init__(self, ax=None) -> None:
        super().__init__(ax=ax)
        self.mFood = None
        self.mSenseDistance = None 
        # self.mCMap = cm.ocean
        self.mCMap = cm.Blues

    def setFood(self, food: list):
        self.mFood = food

    def setSenseDistance(self, distance: float):
        self.mSenseDistance = distance

    def draw(self):
        super().draw()

    def update(self):
        super().update()
        food_pos_group = [food.mPos for food in self.mFood]
        agent_pos_group = np.array([[agent.mPos[0], agent.mPos[1]] for agent in self.mPopulation])
        x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
        y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
        self.mData = np.zeros_like(x_mat, dtype=np.float)
        fitness_mat = np.zeros_like(x_mat, dtype=np.float)
        data_tmp = np.zeros_like(x_mat, dtype=np.float)
        food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)
        food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)
        MaxCrowded = 1 / len(self.mPopulation) # 有些问题，看是不是倒数
        agent_in_range_mat = np.zeros_like(x_mat, dtype=np.float)
        if len(self.mFood) > 0:
            for food_pos_tmp in food_pos_group:
                food_pos_x_mat[:] = food_pos_tmp[0]     # 把x坐标赋予x矩阵
                food_pos_y_mat[:] = food_pos_tmp[1]     # 把y坐标赋予y矩阵
                data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                ind_bool = data_tmp > fitness_mat
                fitness_mat[ind_bool] = data_tmp[ind_bool]
        # print(agent_in_range_mat.shape)
        size1, size2 = agent_in_range_mat.shape
        z_mat = np.zeros_like(x_mat.reshape(size1*size2, 1))
        pt_group = np.hstack((x_mat.reshape(size1*size2, 1), y_mat.reshape(size1*size2, 1), z_mat))
        kd_tree = KDtree(agent_pos_group)
        count_mat = kd_tree.query_radius_count2(pt_group, mySettings.CS_CROWDEDRANGE)
        count_mat = count_mat.reshape(size1, size2)
        fitness_mat = utils.sigmoid(fitness_mat, 0.5, 0.5)
        self.mData = -count_mat / (fitness_mat * MaxCrowded + 0.0000000000001)
        # self.mData = -utils.sigmoid(self.mData, 0.5, 0.5)
        # self.mData = (fitness_mat * MaxCrowded ) / (count_mat + 0.0000000000001)