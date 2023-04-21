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
        """Sets the food list for an object.

        Args:
            food (list): A list of food items that the object can eat.
        """
        self.mFood = food

    def setSenseDistance(self, distance: float):
        """Sets the sensing distance for an object.

        Args:
            distance (float): The maximum distance at which the object can detect other objects.
        """
        self.mSenseDistance = distance


    def update(self):
        """Update the object's data.

        Calculates the object's fitness or 'adaptability' by constructing a matrix of distances between the object and nearby food
        sources. The method then updates the object's fitness level and calls its parent class's update method.
        """
        
        # Construct a list of positions for nearby food sources
        food_pos_group = [food.mPos for food in self.mFood]
        
        # Calculate distance matrix based on directionality
        if self.mZDir == 'z' or self.mZDir == '2D':
            x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
            y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
            self.mData = np.zeros_like(x_mat, dtype=float)
            data_tmp = np.zeros_like(x_mat, dtype=float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=float)
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[0]     # Assign x-coordinate to x-matrix
                    food_pos_y_mat[:] = food_pos_tmp[1]     # Assign y-coordinate to y-matrix
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]

        elif self.mZDir == 'y':
            x_mat = np.repeat([self.mX], len(self.mY), axis=0).astype(float)
            y_mat = np.repeat([self.mY], len(self.mX), axis=0).T.astype(float)
            self.mData = np.zeros_like(x_mat, dtype=float)
            data_tmp = np.zeros_like(x_mat, dtype=float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=float)
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[0]     # Assign x-coordinate to x-matrix
                    food_pos_y_mat[:] = food_pos_tmp[2]     # Assign z-coordinate to y-matrix
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]

        elif self.mZDir == 'x':
            x_mat = np.repeat([self.mX], len(self.mY), axis=0).astype(float)
            y_mat = np.repeat([self.mY], len(self.mX), axis=0).T.astype(float)
            self.mData = np.zeros_like(x_mat, dtype=float)
            data_tmp = np.zeros_like(x_mat, dtype=float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=float)
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[1]     # Assign y-coordinate to x-matrix
                    food_pos_y_mat[:] = food_pos_tmp[2]     # Assign z-coordinate to z-matrix
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]
        
        super().update()   # Call the parent class's update method


    def draw(self):
        """Draws the object."""
        super().draw()
        
