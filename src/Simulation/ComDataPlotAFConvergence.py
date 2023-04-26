try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False

from Simulation.ComDataPlotBase import ComDataPlotBase
from Common.utils import distance

class ComDataPlotAFConvergence(ComDataPlotBase):
    def __init__(self) -> None:
        super().__init__()
        self.mDataX = []
        self.mTime = None
        self.mPopulation = None 
        self.mTarget = None 
        self.mCount = 0

    def setTimeRef(self, time: list):
        """
        Args:
            time (list): A list representing the time value that needs to be set.
        """
        # Assigning the value of 'time' to the class variable 'self.mTime'
        self.mTime = time

    def setPopulation(self, population):
        """
        This method sets the value of self.mPopulation to the given 'population' parameter.

        Args:
            population (any): The population to assign to 'mPopulation'
        """
        self.mPopulation = population

    
    def setTarget(self, target):
        """
        Assigns the given 'target' parameter to self.mTarget.

        Args:
            target (any): The value that will be assigned to 'mTarget'.
        """
        self.mTarget = target

    def update(self):
        """
        This function updates the data for visualization.
        It first calls the base class to perform some actions,
        then checks if mDataY is None and creates it if so.
        It then checks if all necessary data has been initialized
        and if enough time has passed to add a new data point.
        If conditions are met, it calculates the distance between agents
        and the target, saves it in mDataX and increments mCount.
        """

        # Call base class update method
        super().update()

        # If mDataY not yet created, create it
        if self.mDataY is None: 
            self.mDataY = np.array([i*self.mTime[1]/20 for i in range(20)])
            for _ in range(self.mDataY.shape[0]):
                self.mDataX.append(np.array([]))
                
        # Check if necessary data is available and it's time to add new data
        if self.mPopulation is not None and self.mTarget is not None and self.mTime is not None:
            if self.mTime[0] >= self.mCount * self.mTime[1] / 20:
                agent_pos = np.array([agent.mPos for agent in self.mPopulation])
                data = distance(agent_pos, self.mTarget.mPos)
                self.mDataX[self.mCount] = data
                self.mCount += 1

                    
                # self.mDataX.append(data)
                
    def draw(self, ax):
        """
        This function draws a boxplot of the data stored in mDataX on a specified axis object.

        Args:
            ax (matplotlib.axes._subplots.AxesSubplot): The Axis Subplot onto which this Boxplot should be drawn.
        """

        # Call base class draw method
        super().draw(ax)

        # Draw a boxplot of the data stored in mDataX with labels from mDataY.
        ax.boxplot(self.mDataX, labels=self.mDataY, patch_artist=True)

        