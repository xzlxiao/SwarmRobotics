from PyQt5.QtCore import pyqtSignal, QObject, QEvent
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.pyplot as plt
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import matplotlib.patches as mpathes
from PyQt5.QtWidgets import QWidget
from Common.utils import svg_parse, loadSVG, unitVector
from matplotlib.path import Path
import math
import random

path_list = []
translate_list = []
path_list.append("M60.0,120.0C28.0,120,.0,88.0,.0,60.0C.0,28.0,26.0,.0,60.0,.0L60.0,120.0z")
translate_list.append((-3,49.13))
path_list.append("M327.9,120.0L327.9,.0L.0,.0L.0,120.0L327.9,120.0z")
translate_list.append((57.64,49.13))
path_list.append("M.0,24.6C.0,11.0,11.0,.0,24.6,.0C38.1,.0,49.1,11.0,49.1,24.6C49.1,38.1,38.1,49.1,24.6,49.1C11.0,49.1,.0,38.1,.0,24.6z")
translate_list.append((185.20,0.00))
path_list.append("M.0,24.6C.0,11.0,11.0,.0,24.6,.0C38.1,.0,49.1,11.0,49.1,24.6C49.1,38.1,38.1,49.1,24.6,49.1C11.0,49.1,.0,38.1,.0,24.6z")
translate_list.append((185.20,168.19))
path_list.append("M25.5,49.1L25.5,.0L.0,.0L.0,49.1L25.5,49.1z")
translate_list.append((348.43,0.00))
path_list.append("M25.5,49.1L25.5,.0L.0,.0L.0,49.1L25.5,49.1z")
translate_list.append((348.43,168.19))

codes, verts = loadSVG(path_list, translate_list)

class ComObject:
    update_count = 0

    # This method initializes the object's attributes when it is created.
    def __init__(self) -> None:
        """
        Constructor for ComObject class.
        
        Initializes all object attributes with their default values.
        """
        self.mId = -1   # Object ID
        self.mPos = np.array([0, 0, 0], dtype=np.float32)   # Current location (x, y, z)
        self.mOrientation = np.zeros((3, 3), dtype=np.float)   # Current orientation matrix
        self.mTarget = np.array(self.mPos, dtype=np.float32)   # Target location (x, y, z)
        self.mTargetOrientation = np.array(self.mOrientation, dtype=np.float)   # Target orientation matrix
        self.mInterval = []   # List of time intervals between iterations
        self.mSpeed = 300   # Moving speed in unit per second
        self.mColor = 'red'   # Object color
        self.mShape = 'circle_line'   # Object shape
        self.mRadius = 20   # Radius of circle or cylinder
        self.image = None   # Image for texture mapping
        self.setRadius(self.mRadius)   # Set radius
        self.setColor(self.mColor)   # Set color
        self.mWindow = None   # Parent window
        self.mAlpha = 1.0   # Transparency value (1.0 means opaque, 0.0 means transparent)
        self.mPopulation = None   # Associated population object (if any)
        self.mObstacle = None   # Associated obstacle object (if any)
        self.mAx = None   # Axes handle
        self.mDirection = 0.0   # Radians (current direction)
        self.mTargetDirection = None   # Radians (target direction)
        self.isPlotTargetLine = True   # Whether or not to plot a line from the object to its target
        self.mStage = None   # Stage name (if any)
        self.mTrail = [[], [], []]   # Object trail data (x, y, z)
        self.isPlotTrail = False   # Whether or not to plot the object's trail
        self.mObjectType = "ComObject"   # Marks the type of object
        self.mIterations = 0   # Number of iterations
        self.mRobotType = '3D'   # Robot type ('2D', '3D', or 'other')
        self.mTargetLineLen = -1   # Target line length. If < 0, connects to destination; otherwise, only displays a section
        self.mTrailLineColor = (0, 0, 1, 0.1)   # Color of plotted trail lines
        self.mTrailLineAlpha = 0.5   # Alpha value (transparency) for plotted trail lines
        self.mMessage = ''   # Special information for saving records



    # This method is called getMessage() and belongs to a class.
    # The purpose of the method is to return the value of mMessage.

    def getMessage(self):
        """
        Returns the value of mMessage.

        Public method of the class that returns the user-defined message stored in the object's mMessage attribute.
        This message can be used to store intermediate data during simulation.

        Returns:
            string: The current instance's mMessage string
        """
        return self.mMessage 



    def setMessage(self, message):
        """
        Sets the value of mMessage.

        Public method of the class that sets the user-defined message stored in the object's mMessage attribute.
        This message can be used to store intermediate data during simulation.

        Args:
            message (string): The new string value to set for the current instance's mMessage attribute
        """
        # Set the value of the object's mMessage attribute to the passed message string value
        self.mMessage = message




    def setRobotType(self, robot_type='3D'):
        """
        Sets the value of mRobotType.

        Public method of the class that sets the type of robot as defined by the user via the robot_type argument.
        If no robot_type argument is provided, then the default robot type '3D' will be set.

        Args:
            robot_type (string): The new string value to set for the current instance's mRobotType attribute. Default: '3D'.
        """
        # Set the value of the object's mRobotType attribute to the passed robot type string value, or the default '3D' if none provided.
        self.mRobotType = robot_type


    # This method is called setTrailLineColor() and belongs to a class.
    # The purpose of the method is to set the color of the robot's motion trail.
    def setTrailLineColor(self, color=(0, 0, 1, 0.1)):
        """
        Set the color used for drawing the robot's movement trail.

        Args:
            color (tuple, optional): A tuple representing the RGBA color value to use for the trail. 
                Defaults to (0, 0, 1, 0.1).
        """        
        # Set the instance variable 'mTrailLineColor' to the passed color argument
        self.mTrailLineColor = color 
        
        # If the color argument is a tuple and its length is at least 4, set the alpha value of the trail line
        if type(color) == tuple and len(color)>=4:
            self.mTrailLineAlpha = color[3]

    
    def setTrailLineAlpha(self, alpha):
        """
        Sets the value of mTrailLineAlpha.

        Public method of the class that sets the alpha value for the robot trail line as defined by the user via the alpha argument.

        Args:
            alpha (float): The new float value to set for the current instance's mTrailLineAlpha attribute.
        """
        # Set the value of the object's mTrailLineAlpha attribute to the passed alpha float value.
        self.mTrailLineAlpha = alpha


    def getUpdateCount(self):
        """
        Gets the update count from the ComObject.

        Public method of the class that returns the update_count attribute value of ComObject.

        Returns:
            int: The integer value of the update_count attribute in the ComObject.
        """
        # Access the static attribute update_count of the ComObject and return its value.
        return ComObject.update_count

        
    def setTargetDirection(self, direction):
        """Set the target direction for this object

        Args:
            direction (str): A string representing the new target direction

        Returns:
            None
        """
        self.mTargetDirection = direction

        
    def setTarget(self, target):
        '''
            Set the target for this object

            Args:
                target (list): A list of 3 elements representing the new target coordinates in space

            Returns:
                None
        '''
        self.target = target
        # self.mTarget = np.array(target[0:3], dtype=np.float32)

    def setStage(self, stage):
        '''
            Set the current stage to a new value

            Args:
                stage (int): An integer representing the new stage value

            Returns:
                None
        '''
        self.mStage = stage


    def setDirection(self, direction):
        '''
            Set the current direction to a new value

            Args:
                direction (str): A string representing the new direction value

            Returns:
                None
        '''
        self.mDirection = direction
        
        # If there is no target direction, set it to be the same as the current direction
        if self.mTargetDirection is None:
            self.mTargetDirection = direction

            
    @staticmethod
    def getAngleBetweenXandVector(pt1, pt2, plat='xy'):
        '''
            Calculate angle between two points and x-axis.
            
            Args:
                pt1 (tuple): Starting point coordinates as a tuple (x, y, z).
                pt2 (tuple): Ending point coordinates as a tuple (x, y, z).
                plat (str): Plane on which the angle is to be calculated. Default is 'xy'.
                
            Returns:
                float: The angle between the two points and the x-axis in radians.
                    If an invalid plane is specified, return None.
        '''
        
        # Calculate change in X, Y and Z coordinates.
        deltaX, deltaY, deltaZ = pt2[0] - pt1[0], pt2[1] - pt1[1], 0
        
        # Modify deltaZ if third component is present in both points.
        if len(pt1) == 3 and len(pt2) == 3:
            deltaZ = pt2[2] - pt1[2]
        
        # Define a dictionary to map input strings to their corresponding mathematical functions.
        # This makes the code more concise by eliminating multiple if-else blocks.
        planes = {'xy':(deltaY, deltaX), 'xz':(deltaZ, deltaX), 'yz':(deltaZ, deltaY), 'o-xy':(math.sqrt(deltaX**2 + deltaY**2), deltaZ)}
        
        try:
            # Calculate angle using atan2() function from math library.
            return math.atan2(*planes[plat])
        except KeyError:  # Invalid plane parameter provided.
            return None



    def setColor(self, color=(1.0, .0, .0, 1.0)):
        """
            Set the color of a shape.

            Args:
                color (tuple): A tuple of four floats representing RGBA values in the range [0.0, 1.0]. Default is red (1.0, 0.0, 0.0, 1.0).

            Returns:
                None
        """

        # Update the color instance variable.
        self.mColor = color
        
        # Update the shape using setShape() method.
        self.setShape(self.mShape)

    
    def setRadius(self, radius=5):
        """
            Set the radius of a shape.

            Args:
                radius (int or float): The radius of the shape. Default is 5.

            Returns:
                None
        """

        # Update the radius instance variable.
        self.mRadius = radius
        
        # Update the shape using setShape() method.
        self.setShape(self.mShape)


    def setInterval(self, interval: list):
        """
            Set the interval of a function.

            Args:
                interval (list): A list of two values representing the start and end of the interval.

            Returns:
                None
        """

        # Update the interval instance variable.
        self.mInterval = interval


    @property
    def pos(self):
        """
            Get the position of the robot.

            Returns:
                A list of either two or three values representing the x, y, and possibly z coordinates of the robot.

            Note: The returned list will always contain the first two values of mPos if robot type is 2D. If robot type is 3D, it should contain all three values of mPos. Otherwise, the function returns the entire list mPos.
        """

        # Check the robot type and return the appropriate number of coordinates from the beginning of mPos.
        if self.mRobotType == '2D':
            return self.mPos[0:2]
        elif self.mRobotType == '3D':
            return self.mPos[0:3]
        else:
            return self.mPos

    @pos.setter
    def pos(self, value):
        """
            Set the position of the robot.

            Args:
                value (list): A list of either two or three numerical values representing the x, y, and possibly z coordinates of the robot.

            Note: If the robot type is 2D, the first two elements of value will be used to update mPos. If the robot type is 3D, all three elements of value are used. Otherwise, the entire value list is used.
        """

        # Check the robot type to determine how many coordinates to update from the beginning of mPos.
        if self.mRobotType == '2D':
            self.mPos[0:2] = np.array(value[0:2], dtype=np.float32)
        elif self.mRobotType == '3D':
            self.mPos[0:3] = np.array(value[0:3], dtype=np.float32)
        else:
            # For other robot types, update all coordinates in mPos.
            # This assumes that value has the same number of elements as mPos.
            self.mPos = np.array(value, dtype=np.float32)

    
    @property
    def target(self):
        """
            Get the target position of the robot.

            Returns:
                A list containing either two or three numerical values representing the x, y, and possibly z coordinates of the robot's target.
        """

        # Check the robot type to determine how many coordinates to return from the beginning of mTarget.
        if self.mRobotType == '2D':
            return self.mTarget[0:2]
        elif self.mRobotType == '3D':
            return self.mTarget[0:3]
        else:
            # For other robot types, return the entire mTarget list.
            return self.mTarget


    @target.setter
    def target(self, value):
        """
            Set the target position of the robot.

            Args:
                value: A list containing either two or three numerical values representing the x, y, and possibly z coordinates of the robot's target.
        """

        # Check the robot type to determine how many coordinates to set in mTarget.
        if self.mRobotType == '2D':
            self.mTarget[0:2] = np.array(value[0:2], dtype=np.float32)
        elif self.mRobotType == '3D':
            self.mTarget[0:3] = np.array(value[0:3], dtype=np.float32)
        else:
            # For other robot types, set the entire mTarget list to the given value.
            self.mTarget = np.array(value, dtype=np.float32)

    def getPos2d(self):
        """
            Get the position of the robot in 2D space.

            Returns:
                A list containing the x and y coordinates of the robot's current position.
        """
        return self.mPos[0:2]


    @staticmethod
    def getRotationMat(angle: float):
        """
        This function takes an angle value in radians and returns a 2x2 numpy array representing the corresponding 
        rotation matrix for rotating a 2D point by this angle.
        
        Args:
        angle (float): The angle (in radians) by which we want to rotate our 2D point
        
        Returns:
        A 2x2 numpy array representing the rotation matrix
        """
        return np.array([[math.cos(angle), math.sin(angle)],[-math.sin(angle), math.cos(angle)]])

    def setShape(self, shape='circle'):
        """This function sets the shape of an image to either circle, circle_line, square, square_line, ship, or fish.
        
        Args:
            shape (str, optional): determines the shape of the image. Defaults to 'circle'.
            
        Raises:
            RuntimeError: if an invalid shape is inputted.
        """
        # Sets mShape equal to the chosen shape
        self.mShape = shape
        
        # Calculates position based on radius and sets image based on chosen shape
        pos = self.mPos[0:2] - self.mRadius
        
        if shape=='circle':     # Creates a solid circle
            self.image = mpathes.Circle(pos, radius=self.mRadius, color=self.mColor, fill=True)
        elif shape=='circle_line':      # Creates a circle border
            self.image = mpathes.Circle(pos, radius=self.mRadius, color=self.mColor,fill=False)
        elif shape=='square':     # Creates a solid square
            pos = self.mPos[0:2] - self.mRadius
            self.image = mpathes.Rectangle(pos, self.mRadius*2, self.mRadius*2, color=self.mColor, fill=True)
        elif shape=='square_line':    # Creates a square border
            self.image = mpathes.Rectangle(pos, self.mRadius*2, self.mRadius*2, color=self.mColor, fill=False)
        elif shape=='ship':    # Creates a ship shape
            verts_real = verts.copy()
            verts_real[:, 0] = -(verts[:, 0]/3) * (self.mRadius/20)
            verts_real[:, 1] = verts[:, 1]/3 * (self.mRadius/20)
            rotation_mat = ComObject.getRotationMat(self.mDirection)
            verts_real[:, 0:2] = np.matmul(verts_real[:, 0:2], rotation_mat)
            verts_real[:, 0] += self.mPos[0]
            verts_real[:, 1] += self.mPos[1]
                
            if isCupy:
                path = Path(verts_real.get(), codes.get())
            else:
                path = Path(verts_real, codes)
                
            # Creates the ship shape
            self.image = mpathes.PathPatch(path, facecolor=self.mColor, edgecolor='k', lw=1)
        elif shape=='fish':     # Creates a fish shape
            face_color = self.mColor
            edge_color = (0, 0, 0)
                
            angle = self.getDirectionByDegree()
            
            if isCupy:
                pos = self.mPos[0:2].get()
            else:
                pos = self.mPos[0:2]
                    
            # Creates the fish shape
            self.image = mpathes.Wedge(pos, self.mRadius, theta1=angle-15+180, theta2=angle+15+180, edgecolor=edge_color, facecolor=face_color)
        else:
            # Raises an error if an invalid shape was inputted
            raise RuntimeError('没有该预定义形状')


    def getTime(self):
        """_summary_

        Returns:
            float: The total time elapsed in the simulation
        """
        # Calculate the total simulated time by multiplying 
        # the number of iterations with the time interval between iterations.
        return self.mIterations * self.mInterval[0]

        
    def sense(self):
        """
        Observe the surroundings in a circular sensing area 
        and return the positions of obstacles and the robot.

        """
        pass 


    def distance(self, pt1, pt2):
        """
        Calculate the Euclidean distance between two points in either 2D or 3D space.

        Args:
            pt1: list or np.array. A list or Numpy array containing the coordinates of the first point.
            pt2: list or np.array. A list or Numpy array containing the coordinates of the second point.

        Returns:
            float. The Euclidean distance between the two points.

        The function first determines the dimensionality based on the robot type.
        It then takes the first `dim` coordinates from `pt1` and `pt2`,
        converts them to numpy arrays, and calculates their difference vector.
        The norm (magnitude) of this vector is calculated using the `numpy.linalg.norm`
        function to obtain the Euclidean distance between the two points.    
        """
        # Original code remains unchanged
        if self.mRobotType == '3D':
            dim = 3
        elif self.mRobotType == '2D':
            dim = 2
        
        return np.linalg.norm(np.array(pt2[:dim]) - np.array(pt1[:dim]))



    def move(self):
        """ Move robot towards target 
        """
        # Increment iteration count
        self.mIterations += 1

        # Copy the current position of robot
        pos_last = self.mPos.copy()

        # Get the current position of robot and the target position
        robot_pos = self.pos
        target_pos = self.target

        # If robot is not stopping, set its direction towards the target
        if not self.isStopping():
            angle = ComObject.getAngleBetweenXandVector(robot_pos, target_pos)
            self.setDirection(angle)

            # Calculate distance to target, move in that direction if still far from it
            if np.linalg.norm(target_pos - robot_pos, ord=2) > self.mSpeed * self.mInterval[0]:
                self.pos = robot_pos + \
                        self.mSpeed * self.mInterval[0] * (target_pos - robot_pos) / \
                        np.linalg.norm(target_pos - robot_pos, ord=2)

            # Otherwise, set robot's position to the target
            else:
                self.pos = self.target

        # Set the robot's visual shape to the appropriate value
        self.setShape(shape=self.mShape)

        # If the robot moved, add its new position to trail history for visualization purposes
        if not (self.mPos == pos_last).all():
            self.mTrail[0].append(self.mPos[0])
            self.mTrail[1].append(self.mPos[1])
            self.mTrail[2].append(self.mPos[2])


    def isStopping(self):
        """Check if the robot has stopped.

        Returns:
            bool: True if the robot has reached its target and stopped, False otherwise.
        """        

        # If the current position of the robot is equal to its target position, then it has stopped
        if (self.pos == self.target).all():
            return True

        # Otherwise, the robot is still in motion
        else:
            return False

    def update(self):
        """
        Update the robot's position by calling the move() method.
        No arguments are passed to this method.
        """        
        self.move()

    def setAx(self, ax: float) -> None:
        """
        Setter method for the robot's mAx attribute. Assigns the input parameter,
        ax, to the mAx attribute of the current instance of the class.

        Args:
        - ax (float): The new axis value to be assigned.

        Returns:
        - None
        """
        self.mAx = ax

    def drawOnFigure(self, ax):
        """Draws the image on the specified axis and adds target lines/trails.

        Args:
            ax (_type_): The matplotlib axis to draw the image on.

        """
        # Add the image patch to the axis
        ax.add_patch(self.image)
        
        # Get the z-position of the image patch
        z_pos = self.mPos[2]
        
        # If using Cupy, get the value of the z-position
        if isCupy:
            z_pos = z_pos.get()
            
        # If the stage type is 3D, convert the image patch to a 3D path patch
        if self.mStage.mStageType == '3D':
            art3d.pathpatch_2d_to_3d(self.image, z=z_pos, zdir='z')
            
        # If target line should be plotted, calculate its coordinates
        if self.isPlotTargetLine:
            x, y, z = 0, 0, 0
            
            # If the target line length is negative, set the end point as the target position
            if self.mTargetLineLen < 0:
                x = np.array([self.mPos[0], self.mTarget[0]])
                y = np.array([self.mPos[1], self.mTarget[1]])
                z = np.array([self.mPos[2], self.mTarget[2]])
            # Otherwise, calculate the unit vector of the target direction and extend it to the desired length
            else:
                target_vector = self.mTarget - self.mPos
                unit_target = self.mPos + unitVector(target_vector)*self.mTargetLineLen
                x = np.array([self.mPos[0], unit_target[0]])
                y = np.array([self.mPos[1], unit_target[1]])
                z = np.array([self.mPos[2], unit_target[2]])
                
            # If using Cupy, get the values of the target line coordinates
            if isCupy:
                x = x.get()
                y = y.get()
                z = z.get()
            
            # If the stage type is 3D, plot the target line in 3D
            if self.mStage.mStageType == '3D':
                ax.plot(x, y, z, 'b--')
            # Otherwise, plot the target line in 2D
            else:
                ax.plot(x, y, 'b--')
        
        # If plot trail is enabled, plot the trail of the image
        if self.isPlotTrail:
            # If the stage type is 3D, plot the trail in 3D
            if self.mStage.mStageType == '3D':
                ax.plot(self.mTrail[0], self.mTrail[1], self.mTrail[2], color=self.mTrailLineColor, alpha=self.mTrailLineAlpha)
            # Otherwise, plot the trail in 2D
            else:
                ax.plot(self.mTrail[0], self.mTrail[1], color=self.mTrailLineColor, alpha=self.mTrailLineAlpha)

        
    def draw(self, ax):
        """Draws the image on the specified axis.

        Args:
            ax (_type_): The matplotlib axis to draw the image on.

        """
        # Call the drawOnFigure method to draw the image on the specified axis
        self.drawOnFigure(ax)

    def chooseRandomTarget(self):
        """Chooses a random target location and sets it as the agent's new target.
        If the stage is in 3D, the target will also have a random z coordinate.

        """
        # Set the default range of coordinates for the target
        range_x = (-100, 100)
        range_y = (-100, 100)
        range_z = (-100, 100)
        
        # If the agent has a specific stage to operate on...
        if self.mStage is not None:
            # ...use the environment size from the stage instead of the default ranges
            range_x = (-self.mStage.mEnvSize[0], self.mStage.mEnvSize[0])
            range_y = (-self.mStage.mEnvSize[1], self.mStage.mEnvSize[1])
            # If the stage is in 3D, also set the z-coordinate range based on the environment size
            if self.mStage.mStageType == '3D':
                range_z = (-self.mStage.mEnvSize[2], self.mStage.mEnvSize[2])
        
        # Choose random x and y coordinates within the specified range
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        
        # Set the z coordinate to 0 by default (for 2D stages), and randomly choose it if the stage is 3D
        z = 0
        if self.mStage.mStageType == '3D':
            z = random.uniform(range_z[0], range_z[1])
        
        # Set the newly chosen target as the agent's new target position
        self.setTarget((x, y, z))


    def getDirectionByDegree(self):
        """Returns the agent's current direction in degrees."""
        
        # Convert the agent's direction from radians to degrees and return it
        return math.degrees(self.mDirection)
    
        # return self.mDirection*180/math.pi 
        