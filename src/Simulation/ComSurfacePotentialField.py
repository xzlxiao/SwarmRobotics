from matplotlib import cm, markers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from Common import utils as myUtils
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Simulation.ComSurfaceBase import ComSurfaceBase, PlotType
from Simulation.ComPathPlanning import calc_potential_field2
from Simulation.ComPathPlanning3D import potential_field_planning, calc_potential_field_mat


class ComSurfacePotentialField(ComSurfaceBase):
    def __init__(self, ax=None) -> None:
        super().__init__(ax=ax)
        self.mObstacleList = []
        self.mTarget = None
        self.mCMap = cm.Blues
        self.mRobotRadius = 20      
        self.mPlotType = PlotType.type_contourf
        self.mBindingRobot = None
        # self.mCMap = cm.ocean

    def setBindingRobot(self, robot):
        """Sets the binding robot for this object.

        Args:
            robot (object): The robot object to bind.
        """
        self.mBindingRobot = robot
        
    def setRobotRadius(self, radius):
        """Sets the radius of this object's associated robot.

        Args:
            radius (float): The radius to set for the robot.
        """
        self.mRobotRadius = radius
        
    def setTarget(self, target: tuple):
        """Sets the target location for this object's associated robot.

        Args:
            target (tuple): A tuple representing the x and y coordinates of the target location.
        """
        self.mTarget = target
        
    def setObstacleList(self, obstacle_list: list):
        """Sets the list of obstacle locations for this object's associated robot.

        Args:
            obstacle_list (list): A list of tuples representing the x and y coordinates of each obstacle.
        """
        self.mObstacleList = obstacle_list


    def update(self):
        """Updates the potential field based on current parameters."""

        """
        人工势场计算 (Translation: Artificial potential field calculation)
        """

        # Create a list of obstacle positions
        obstacle_pos_group = [obstacle.mPos for obstacle in self.mObstacleList]

        # If there is a bound robot, set its offset
        if self.mBindingRobot is not None:
            self.setOffset(self.mBindingRobot.mPos[2])

        # If the Z direction is 'z' or '2D'
        if self.mZDir == 'z' or self.mZDir == '2D':

            # If there are obstacles
            if len(self.mObstacleList) > 0:

                # Create lists of X and Y coordinates from each obstacle position
                obstacle_pos_x_list = [i[0] for i in obstacle_pos_group]
                obstacle_pos_y_list = [i[1] for i in obstacle_pos_group]

                # Calculate the potential field
                gx, gy = None, None
                ox = obstacle_pos_x_list
                oy = obstacle_pos_y_list
                reso= (np.max(self.mX) - np.min(self.mX))/len(self.mX)
                rr = self.mRobotRadius
                map_size=(np.min(self.mX), np.min(self.mY), np.max(self.mX), np.max(self.mY))

                # If there is a target, set the target coordinates
                if self.mTarget is not None:
                    gx, gy = self.mTarget[0:2]

                # Calculate the potential field data and store it in mData
                data, _, _ = calc_potential_field2(gx, gy, ox, oy, rr, reso, map_size)
                self.mData = np.array(data).T

        # If the Z direction is '3D'
        elif self.mZDir == '3D':

            # If there are obstacles
            if len(self.mObstacleList) > 0:

                # Create lists of X, Y, and Z coordinates from each obstacle position
                obstacle_pos_x_list = [i[0] for i in obstacle_pos_group]
                obstacle_pos_y_list = [i[1] for i in obstacle_pos_group]
                obstacle_pos_z_list = [i[2] for i in obstacle_pos_group]

                gx, gy, gz = None, None, None
                ox = obstacle_pos_x_list
                oy = obstacle_pos_y_list
                oz = obstacle_pos_z_list
                reso= (np.max(self.mX) - np.min(self.mX))/len(self.mX)
                rr = self.mRobotRadius
                map_size=(np.min(self.mX), np.min(self.mY), np.max(self.mX), np.max(self.mY))

                # If there is a target, set the target coordinates
                if self.mTarget is not None:
                    gx, gy, gz = self.mTarget[:]

                # Calculate the potential field data and store it in mData
                data, minx, miny, minz = calc_potential_field_mat(gx, gy, gz, ox, oy, oz, rr, reso=10, map_size=(-1000, -1000, -1000, 1000, 1000, 1000))
                offset_z = int((self.mOffset-minz)/reso)
                self.mData = data[:,:,offset_z]

        # If the Z direction is 'y'
        elif self.mZDir == 'y':
            pass 

        # If the Z direction is 'x'
        elif self.mZDir == 'x':
            pass 

        # Call the update method of the parent class
        super().update()


    def draw(self):
        """Draws the object."""
        super().draw()
        

