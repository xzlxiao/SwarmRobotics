from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False


class ComMonitorPlotBase:
    def __init__(self) -> None:
        """Initialize the class variables.

        This function sets the initial values for several class variables, including mPopulation,
        mData, mX, mY, mCMap, and mAlpha. These variables are used throughout the class to store
        data and configure visualizations. 

        Args:
            None.

        Returns:
            None.
        """
        self.mPopulation = None    # Set mPopulation to None so we can populate it later with data
        self.mData = None          # Set mData to None so we can populate it later with data
        self.mX = None             # Set mX to None so we can populate it later with data
        self.mY = None             # Set mY to None so we can populate it later with data
        self.mCMap = cm.coolwarm   # Set the default colormap to coolwarm for visualizations
        self.mAlpha = 0.5          # Set the default transparency level to 0.5 for visualizations


    def update(self):
        pass

    def draw(self, ax_group):
        """Draw the agent positions on a group of axes.

        This function takes in a group of axes and uses them to plot the positions of selected agents
        from the simulation. For each agent, it retrieves their position data and plots it as a scatter 
        plot with red circles. 

        Args:
            ax_group (list): A list of matplotlib axes to plot on. The length of the list should be equal
                            to the number of agents being plotted. 

        Returns:
            None.
        """
        for ind, agent_id in enumerate(self.mRandomInds):
            # Get the x and y positions of the agent's population
            agentMarkXs = [agent_pos[0] for agent_pos in self.mPopulation[agent_id].mPopulation.values()]
            agentMarkYs = [agent_pos[1] for agent_pos in self.mPopulation[agent_id].mPopulation.values()]

            # Plot the agent's positions on the corresponding axis
            ax_group[ind].scatter(agentMarkXs, agentMarkYs, color="red", marker='o', alpha=1)


    def setX(self, x: np.ndarray):
        """Set the value of the 'mX' attribute.

        Args:
            x (np.ndarray): A numpy array with the new value for 'mX'.

        Returns:
            None.
        """
        self.mX = x

    def setY(self, y: np.ndarray):
        """Set the value of the 'mY' attribute.

        Args:
            y (np.ndarray): A numpy array with the new value for 'mY'.

        Returns:
            None.
        """
        self.mY = y

    def setCMap(self, cmap):
        """Set the value of the 'mCMap' attribute.

        Args:
            cmap: A matplotlib color map object with the new value for 'mCMap'.

        Returns:
            None.
        """
        self.mCMap = cmap 

    def setAlpha(self, alpha):
        """Set the value of the 'mAlpha' attribute.

        Args:
            alpha: A float or int with the new value for 'mAlpha'.

        Returns:
            None.
        """
        self.mAlpha = alpha
        
    def setPopulation(self, population: list):
        """Set the value of the 'mPopulation' attribute.

        Args:
            population (list): A list with the new value for 'mPopulation'.

        Returns:
            None.
        """
        self.mPopulation = population