from matplotlib import cm, markers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from common import utils as myUtils
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Simulation.ComSurfaceBase import ComSurfaceBase, PlotType
from Simulation.ComPathPlanning import calc_potential_field2



class ComSurfacePotentialField(ComSurfaceBase):
    def __init__(self, ax=None) -> None:
        super().__init__(ax=ax)
        self.mObstacleList = []
        self.mTarget = None
        self.mCMap = cm.Blues
        self.mRobotRadius = 20      
        self.mPlotType = PlotType.type_contourf
        # self.mCMap = cm.ocean
        
    def setRobotRadius(self, radius):
        self.mRobotRadius = radius

    def setTarget(self, target: tuple):
        self.mTarget = target

    def setObstacleList(self, obstacle_list: list):
        self.mObstacleList = obstacle_list

    def update(self):
        """
        人工势场计算
        """
        
        obstacle_pos_group = [obstacle.mPos for obstacle in self.mObstacleList]
        # x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
        # y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
        # self.mData = np.zeros_like(x_mat, dtype=np.float)
        # data_tmp = np.zeros_like(x_mat, dtype=np.float)

        if self.mZDir == 'z' or self.mZDir == '2D':
            # print(x_mat.shape, y_mat.shape, self.mData.shape)
            if len(self.mObstacleList) > 0:
                obstacle_pos_x_list = [i[0] for i in obstacle_pos_group]
                obstacle_pos_y_list = [i[1] for i in obstacle_pos_group]
                gx, gy = None, None
                ox = obstacle_pos_x_list
                oy = obstacle_pos_y_list
                reso= (np.max(self.mX) - np.min(self.mX))/len(self.mX)
                rr = self.mRobotRadius
                # print(rr)
                map_size=(np.min(self.mX), np.min(self.mY), np.max(self.mX), np.max(self.mY))
                if self.mTarget is not None:
                    gx, gy = self.mTarget[0:2]
                data, _, _ = calc_potential_field2(gx, gy, ox, oy, rr, reso, map_size)
                self.mData = np.array(data).T
                # for i in range(len(ox)):
                    # print(self.mData[int(oy[i]/reso), int(ox[i]/reso)])
                # print(np.max(self.mData), np.min(self.mData))
                # data_tmp = 1 - np.sqrt(np.power(x_mat - obstacle_pos_x_list, 2) + np.power(y_mat - obstacle_pos_y_list, 2)) / self.mSenseDistance
                # self.mData = data_tmp

        elif self.mZDir == 'y':
            pass 
            # if len(self.mObstacleList) > 0:
            #     for obstacle_pos_tmp in obstacle_pos_group:
            #         obstacle_pos_x_list[:] = obstacle_pos_tmp[0]     # 把x坐标赋予x矩阵
            #         obstacle_pos_y_list[:] = obstacle_pos_tmp[2]     # 把z坐标赋予y矩阵
            #         data_tmp = 1 - np.sqrt(np.power(x_mat - obstacle_pos_x_list, 2) + np.power(y_mat - obstacle_pos_y_list, 2)) / self.mSenseDistance
            #         ind_bool = data_tmp > self.mData
            #         self.mData[ind_bool] = data_tmp[ind_bool]

        elif self.mZDir == 'x':
            pass 
            # if len(self.mObstacleList) > 0:
            #     for obstacle_pos_tmp in obstacle_pos_group:
            #         obstacle_pos_x_list[:] = obstacle_pos_tmp[1]     # 把x坐标赋予y矩阵
            #         obstacle_pos_y_list[:] = obstacle_pos_tmp[2]     # 把z坐标赋予z矩阵
            #         data_tmp = 1 - np.sqrt(np.power(x_mat - obstacle_pos_x_list, 2) + np.power(y_mat - obstacle_pos_y_list, 2)) / self.mSenseDistance
            #         ind_bool = data_tmp > self.mData
            #         self.mData[ind_bool] = data_tmp[ind_bool]
        
        super().update()

    def draw(self):
        super().draw()
        
