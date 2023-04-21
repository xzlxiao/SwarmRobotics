try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False

from Simulation.ComDataPlotBase import ComDataPlotBase
from Common.utils import distance

class ComDataPlotPosPredict(ComDataPlotBase):
    def __init__(self) -> None:
        """
        This is the constructor of a class.

        Upon object creation, this calls the superclass constructor and initializes 
        `__ys`, `mMonitorRobot` and `mPredictRobot` member variables to None.
        """

        # Call the constructor of the superclass 
        super().__init__()

        # Initialize the private member variable __ys to an empty list
        self.__ys = []

        # Initialize mMonitorRobot to None
        self.mMonitorRobot = None 

        # Initialize mPredictRobot to None
        self.mPredictRobot = None


    def setRobots(self, monitor_robot, predict_robot):
        """
        This function sets the values of member variables `mMonitorRobot` and `mPredictRobot`.

        Args:
            monitor_robot: An object representing the monitoring robot.
            predict_robot: An object representing the prediction robot.

        Returns:
            None
        """
        # Set member variable mMonitorRobot to the value of `monitor_robot`
        self.mMonitorRobot = monitor_robot
        # Set member variable mPredictRobot to the value of `predict_robot`
        self.mPredictRobot = predict_robot

    def update(self):
        """
        This method updates the object's data by calling the update method of the parent class (superclass) 
        and then calling another method within the current class to update some additional plot data.

        """
        super().update() # Calls the update method of the parent/superclass
        self.getPositionPlotData() # Updates some additional plot data in the current class


    def draw(self, ax):
        """
        This method draws the data on a given axes object by calling the draw method of the parent class (superclass) 
        and then creating a new plot using some internal data variables.
        
        Args:
            ax (matplotlib.axes.Axes): The axes object onto which to draw the plot.
        """
        super().draw(ax) # Calls the draw method of the parent/superclass to draw any additional data 
                        # that may have been added in a subclass
        ax.plot(self.mDataX, self.mDataY) # Plots the current object's X and Y data on the given axes object


    def getPositionPlotData(self):
        """This method gets data for a plot of predicted and monitored robot positions.

        Returns:
            None
        """
        
        # Check if there are valid monitor and prediction robot objects to get data from
        if self.mMonitorRobot is not None and self.mPredictRobot is not None:
            
            dict_list = []                  # Create a list for storing position information
            
            predict_robot_id = self.mPredictRobot.mId  # Get ID of predicted robot object
            
            # Loop through each monitored robot's processed info recorder and append the Pos dictionary to dict_list
            for item in self.mMonitorRobot.mProcessedInfoRecorder: 
                dict_list.append(item['Pos'])
                
            # If there exists previously recorded monitor info...
            if len(self.mMonitorRobot.mProcessedInfoRecorder) > 0:
                
                # Get latest monitored robot position information
                info_item = self.mMonitorRobot.mProcessedInfoRecorder[-1]
                
                # If the monitored robot has its own Pos dictionary...
                if 1 in info_item['Pos'].keys():
                    
                    # Calculate Euclidean distance between predicted robot and real monitored robot positions, negating it
                    dist_between_predict_real = distance(info_item['Pos'][predict_robot_id], self.mPredictRobot.mPos)
                    self.__ys.append(-dist_between_predict_real)
                
                else:   # Otherwise default to -1000 as the distance
                    self.__ys.append(-1000)
            
            # Set attributes for plot data
            self.mDataX = np.arange(len(self.__ys))
            self.mDataY = np.array(self.__ys)
            
            # Check if using Cupy library before converting to numpy arrays
            if isCupy:
                self.mDataX = self.mDataX.get()
                self.mDataY = self.mDataY.get()
