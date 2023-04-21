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
        """This method sets the food items.

        Args:
            food (list): A list of food items.
        """
        self.mFood = food  # Set self.mFood attribute to the input food list

    def setSenseDistance(self, distance: float):
        """This method sets the sensing distance.

        Args:
            distance (float): The sensing distance.
        """
        self.mSenseDistance = distance  # Set self.mSenseDistance attribute to the input distance

    def draw(self):
        """This method calls the draw method of the parent class to draw the environment.
        """
        super().draw()  # Call the draw method of the parent class to draw the environment


    def update(self):
        """This method updates the state of the environment and agents.

        It calculates the fitness of the food available in the environment and uses it to determine how crowded 
        the environment is. It then calculates the fitness of each agent based on their proximity to other agents 
        and the amount of food they have consumed.

        Args:
            None.

        Returns:
            None.
        """
        super().update()  # Call the update method of the parent class to update the environment
        food_pos_group = [food.mPos for food in self.mFood]  # Get the position of all food items in a list
        agent_pos_group = np.array([[agent.mPos[0], agent.mPos[1]] for agent in self.mPopulation])  # Get the position of all agents in a numpy array
        x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)  # Create a matrix of x-coordinates
        y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)  # Create a matrix of y-coordinates
        self.mData = np.zeros_like(x_mat, dtype=np.float)  # Initialize the self.mData attribute to a matrix of zeros
        fitness_mat = np.zeros_like(x_mat, dtype=np.float)  # Initialize the fitness_mat variable to a matrix of zeros
        data_tmp = np.zeros_like(x_mat, dtype=np.float)  # Initialize the data_tmp variable to a matrix of zeros
        food_pos_x_mat = np.zeros_like(x_mat, dtype=np.float)  # Initialize the food_pos_x_mat variable to a matrix of zeros
        food_pos_y_mat = np.zeros_like(x_mat, dtype=np.float)  # Initialize the food_pos_y_mat variable to a matrix of zeros
        MaxCrowded = 1 / len(self.mPopulation) # Calculate the maximum amount of crowding (inverse proportion of agents)
        agent_in_range_mat = np.zeros_like(x_mat, dtype=np.float)  # Initialize the agent_in_range_mat variable to a matrix of zeros
        if len(self.mFood) > 0:
            for food_pos_tmp in food_pos_group:  # Iterate through all food positions
                food_pos_x_mat[:] = food_pos_tmp[0]     # Assign the x-coordinates of the current food item to food_pos_x_mat
                food_pos_y_mat[:] = food_pos_tmp[1]     # Assign the y-coordinates of the current food item to food_pos_y_mat
                data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mSenseDistance  # Calculate the fitness of the current food item
                ind_bool = data_tmp > fitness_mat
                fitness_mat[ind_bool] = data_tmp[ind_bool]  # Update the fitness_mat variable with the calculated fitness values
        size1, size2 = agent_in_range_mat.shape
        z_mat = np.zeros_like(x_mat.reshape(size1*size2, 1))
        pt_group = np.hstack((x_mat.reshape(size1*size2, 1), y_mat.reshape(size1*size2, 1), z_mat))
        kd_tree = KDtree(agent_pos_group)  # Construct a KDTree object with the positions of all agents
        count_mat = kd_tree.query_radius_count2(pt_group, mySettings.CS_CROWDEDRANGE)  # Count the number of agents within a certain radius of each position
        count_mat = count_mat.reshape(size1, size2)  # Reshape the count_mat variable to have the same shape as agent_in_range_mat
        fitness_mat = utils.sigmoid(fitness_mat, 0.5, 0.5)  # Calculate the sigmoid of the fitness matrix
        self.mData = -count_mat / (fitness_mat * MaxCrowded + 0.0000000000001)  # Update the self.mData attribute based on the calculated fitness and crowding values

        # self.mData = -utils.sigmoid(self.mData, 0.5, 0.5)
        # self.mData = (fitness_mat * MaxCrowded ) / (count_mat + 0.0000000000001)