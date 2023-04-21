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
        """This method sets the mFood attribute of the environment to a given list of food items.

        Args:
            food (list): A list of food items to be added to the environment.

        Returns:
            None.
        """
        self.mFood = food

    def setSenseDistance(self, distance: float):
        """This method sets the mSenseDistance attribute of the environment to a given value.

        Args:
            distance (float): The value to set as the sensing distance.

        Returns:
            None.
        """
        self.mSenseDistance = distance

    def update(self):
        """This method calculates the fitness value of a given position.
        It computes the distance of each food item from the position and returns the maximum 
        distance as the fitness value. The distance is computed in two dimensions or three dimensions 
        depending on the value of mZDir attribute.

        Args:
            None.

        Returns:
            None.
        """
        
        # Get the positions of all food items.
        food_pos_group = [food.mPos for food in self.mFood]

        # Compute the fitness value in 2D if mZDir is 'z' or '2D'.
        if self.mZDir == 'z' or self.mZDir == '2D':

            # Set up x_mat and y_mat to store the X and Y coordinates of the current position.
            x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
            y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)

            # Initialize mData to zeros.
            self.mData = np.zeros_like(x_mat, dtype=np.float)
            
            # Set up data_tmp, food_pos_x_mat, and food_pos_y_mat to store temporary values.
            data_tmp = np.zeros_like(x_mat, dtype=np.float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)

            # Calculate the fitness value based on the distance from each food item.
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[0]     # Assign x coordinate to x matrix.
                    food_pos_y_mat[:] = food_pos_tmp[1]     # Assign y coordinate to y matrix.
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]

        # Compute the fitness value in 3D if mZDir is 'y'.
        elif self.mZDir == 'y':

            # Set up x_mat and y_mat to store the X and Z coordinates of the current position.
            x_mat = np.repeat([self.mX], len(self.mY), axis=0).astype(float)
            y_mat = np.repeat([self.mY], len(self.mX), axis=0).T.astype(float)

            # Initialize mData to zeros.
            self.mData = np.zeros_like(x_mat, dtype=np.float)
            
            # Set up data_tmp, food_pos_x_mat, and food_pos_y_mat to store temporary values.
            data_tmp = np.zeros_like(x_mat, dtype=np.float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)

            # Calculate the fitness value based on the distance from each food item.
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[0]     # Assign x coordinate to x matrix.
                    food_pos_y_mat[:] = food_pos_tmp[2]     # Assign z coordinate to y matrix.
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]

        # Compute the fitness value in 3D if mZDir is 'x'.
        elif self.mZDir == 'x':

            # Set up x_mat and y_mat to store the Y and Z coordinates of the current position.
            x_mat = np.repeat([self.mX], len(self.mY), axis=0).astype(float)
            y_mat = np.repeat([self.mY], len(self.mX), axis=0).T.astype(float)

            # Initialize mData to zeros.
            self.mData = np.zeros_like(x_mat, dtype=np.float)
            
            # Set up data_tmp, food_pos_x_mat, and food_pos_y_mat to store temporary values.
            data_tmp = np.zeros_like(x_mat, dtype=np.float)
            food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)
            food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)

            # Calculate the fitness value based on the distance from each food item.
            if len(self.mFood) > 0:
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[:] = food_pos_tmp[1]     # Assign y coordinate to x matrix.
                    food_pos_y_mat[:] = food_pos_tmp[2]     # Assign z coordinate to y matrix.
                    data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance
                    ind_bool = data_tmp > self.mData
                    self.mData[ind_bool] = data_tmp[ind_bool]
                    
        super().update()   # Call the update method of the superclass.

    def draw(self):
        """Draws the object."""
        super().draw()
        
