from matplotlib import cm, markers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Simulation.ComSurfaceBase import ComSurfaceBase


class ComSurfaceFitness(ComSurfaceBase):
    def __init__(self, ax=None) -> None:
        super().__init__(ax=ax)
        self.mFood = None
        self.mSenseDistance = None 
        self.mCMap = cm.winter
        # self.mCMap = cm.ocean
        

    def setFood(self, food: list):
        self.mFood = food

    def setSenseDistance(self, distance: float):
        self.mSenseDistance = distance

    def update(self):
        """
        适应度计算
        :param position:
        :return: 适应度
        """
        
        food_pos_group = [food.mPos for food in self.mFood]
        if self.mZDir == 'z' or self.mZDir == '2D':
            x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
            y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
            self.mData = np.zeros_like(x_mat, dtype=np.float)
            # print(x_mat.shape, y_mat.shape, self.mData.shape)
            data_tmp = np.zeros_like(x_mat, dtype=np.float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[0]     # 把x坐标赋予x矩阵
                    food_pos_y_mat[:] = food_pos_tmp[1]     # 把y坐标赋予y矩阵
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]

        elif self.mZDir == 'y':
            x_mat = np.repeat([self.mX], len(self.mY), axis=0).astype(float)
            y_mat = np.repeat([self.mY], len(self.mX), axis=0).T.astype(float)
            self.mData = np.zeros_like(x_mat, dtype=np.float)
            data_tmp = np.zeros_like(x_mat, dtype=np.float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[0]     # 把x坐标赋予x矩阵
                    food_pos_y_mat[:] = food_pos_tmp[2]     # 把z坐标赋予y矩阵
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]

        elif self.mZDir == 'x':
            x_mat = np.repeat([self.mX], len(self.mY), axis=0).astype(float)
            y_mat = np.repeat([self.mY], len(self.mX), axis=0).T.astype(float)
            self.mData = np.zeros_like(x_mat, dtype=np.float)
            data_tmp = np.zeros_like(x_mat, dtype=np.float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[1]     # 把x坐标赋予y矩阵
                    food_pos_y_mat[:] = food_pos_tmp[2]     # 把z坐标赋予z矩阵
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]
        
        super().update()

    def draw(self):
        super().draw()
        
