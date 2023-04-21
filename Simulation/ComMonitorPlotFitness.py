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
        """Constructor method for the class.

        Initializes an empty list to store random indexes.
        """
        super().__init__()  # Call constructor of parent class
        self.mRandomInds = []  # Initialize empty list to store random indexes


    def update(self):
        """Update the grid of food sources sensed by the agents.

        No parameters are taken. The method updates the mData array such that the value of each element corresponds to
        the maximum level of food detected by all of the agents at that point. The mData array is initialized and manipulated
        using NumPy methods that enable operations on multi-dimensional arrays.

        Returns nothing, but updates the self.mData attribute with new values based on the location of food sources and the 
        sense distance of agents.
        """
        super().update()
        
        # x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
        # y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
        
        # Check if mRandomInds is empty or not
        if len(self.mRandomInds) == 0:
            for i in range(len(self.mPopulation)):
                self.mRandomInds.append(i)
            if len(self.mPopulation) >= 0:
                # Randomly sample 8 individuals from mRandomInds, assuming there are at least 8 individuals in the population
                self.mRandomInds = random.sample(self.mRandomInds, 8)

        # Create and initialize numpy arrays using np.repeat and np.array methods        
        x_mat = np.repeat(np.array([self.mX]), len(self.mY), axis=0).astype(float)
        x_mat = np.repeat(np.array([x_mat]), len(self.mRandomInds), axis=0).astype(float)
        y_mat = np.repeat(np.array([self.mY]), len(self.mX), axis=0).T.astype(float)
        y_mat = np.repeat(np.array([y_mat]), len(self.mRandomInds), axis=0).astype(float)

        # Initialize numpy arrays used later in the function
        self.mData = np.zeros_like(x_mat, dtype=float)
        data_tmp = np.zeros_like(self.mData, dtype=float)
        food_pos_x_mat = np.zeros_like(self.mData, dtype=float)
        food_pos_y_mat = np.zeros_like(self.mData, dtype=float)

        # Loop through each index and agent from mRandomInds
        for ind, agent_ind in enumerate(self.mRandomInds):
            # Get a list of all food sources at the current agent's position
            food_pos_group = [food for food in self.mPopulation[agent_ind].mFood]
            if len(food_pos_group) > 0:
                 # If the list is not empty, loop through each food source and assign its x- and y-coordinates to the corresponding array
                for food_pos_tmp in food_pos_group:
                    food_pos_x_mat[ind, :, :] = food_pos_tmp[0]     # Assign x-coordinates to food_pos_x_mat
                    food_pos_y_mat[ind, :, :] = food_pos_tmp[1]     # Assign y-coordinates to food_pos_y_mat

        # Calculate data_tmp using the distance between agents and food sources
        data_tmp = 1 - np.sqrt(np.power(x_mat - food_pos_x_mat, 2) + np.power(y_mat - food_pos_y_mat, 2)) / self.mPopulation[agent_ind].mSenseDistance
        
        # Update mData where data_tmp is greater than the current value of mData
        ind_bool = data_tmp > self.mData
        self.mData[ind_bool] = data_tmp[ind_bool]

    def draw(self, *args):
        """
        This function is used to draw the heatmap of food sources in each agent's sensing range on the Matplotlib Axes object.
        
        Args:
            - *args: A variable-length argument list of Matplotlib Axes objects representing the subplots of the figure where
                    the heatmaps will be drawn.

        Returns:
            None
        
        """
        # Iterate over random indices and extract the data for each heatmap
        for ind, _ in enumerate(self.mRandomInds):
            
            # Extract X and Y coordinates and data array from the object attributes
            x_mat = self.mX
            y_mat = self.mY
            data = self.mData[ind, :, :]
            
            # Convert Cupy arrays to NumPy arrays if needed
            if isCupy:
                x_mat = x_mat.get()
                y_mat = y_mat.get()
                data = data.get()
                
            # Plot the contour plot of the heatmap using Matplotlib's contourf function
            args[ind].contourf(x_mat, y_mat, data, cmap=self.mCMap, alpha=self.mAlpha)
            
        # Call the 'draw' method of the parent class to finish drawing the figure
        super().draw(*args)
