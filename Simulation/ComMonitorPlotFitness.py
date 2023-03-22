try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random

from Simulation.ComMonitorPlotBase import ComMonitorPlotBase


class ComMonitorPlotFitness(ComMonitorPlotBase):
    def __init__(self) -> None:
        super().__init__()
        self.mRandomInds = []

    def update(self):
        super().update()
        
        # x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
        # y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
        if len(self.mRandomInds) == 0:
            for i in range(len(self.mPopulation)):
                self.mRandomInds.append(i)
            if len(self.mPopulation) >= 0:
                self.mRandomInds = random.sample(self.mRandomInds, 8)
                
        x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
        x_mat = np.repeat(np.array([x_mat]), len(self.mRandomInds), axis=0).astype(float)
        y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
        y_mat = np.repeat(np.array([y_mat]), len(self.mRandomInds), axis=0).astype(float)

        self.mData = np.zeros_like(x_mat, dtype=float)
        data_tmp = np.zeros_like(self.mData, dtype=float)
        food_pos_x_mat = np.zeros_like(self.mData, dtype=float)
        food_pos_y_mat = np.zeros_like(self.mData, dtype=float)
        # print(self.mData.shape)
        for ind, agent_ind in enumerate(self.mRandomInds):
            food_pos_group = [food for food in self.mPopulation[agent_ind].mFood]
            if len(food_pos_group) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[ind, :, :] = food_pos_tmp[0]     # 把x坐标赋予x矩阵
                    food_pos_y_mat[ind, :, :] = food_pos_tmp[1]     # 把y坐标赋予y矩阵
        data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mPopulation[agent_ind].mSenseDistance
        ind_bool = data_tmp > self.mData
        self.mData[ind_bool] = data_tmp[ind_bool]

    def draw(self, ax_group):
        for ind, _ in enumerate(self.mRandomInds):
            x_mat = self.mX
            y_mat = self.mY
            data = self.mData[ind, :, :]
            if isCupy:
                x_mat = x_mat.get()
                y_mat = y_mat.get()
                data = data.get()
            ax_group[ind].contourf(x_mat, y_mat, data, cmap=self.mCMap, alpha=self.mAlpha)
        super().draw(ax_group)