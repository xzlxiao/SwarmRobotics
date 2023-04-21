import numpy as np
import Common.utils as myUtils
import math
import time 
import copy 
from collections import deque
from Simulation import ComObjectCollection
import random

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

def potential_field_planning(sx, sy, gx, gy, ox, oy, rr, reso=1, map_size=(-1000, -1000, 1000, 1000)):
    """
    A function that plans a path using potential fields.

    Args:
        sx (int): Starting x position [mm]
        sy (int): Starting y position [mm]
        gx (int): Goal x position [mm]
        gy (int): Goal y position [mm]
        ox (list[int]): Obstacle x positions list [mm]
        oy (list[int]): Obstacle y positions list [mm]
        rr (int): Robot radius [mm]
        reso (int, optional): Grid resolution per unit. Defaults to 1.
        map_size (tuple, optional): Map size defined by lower-left and upper-right corners. Defaults to (-1000, -1000, 1000, 1000).

    Returns:
        tuple: The planned path represented as x and y position lists in millimeters.
    """    

    # Calculate potential field 
    pmap, minx, miny = calc_potential_field2(gx, gy, ox, oy, rr, reso, map_size)
    
    # Search for path
    d = np.hypot(sx - gx, sy - gy) # Euclidean distance between starting point and goal
    ix = round((sx - minx) / reso) # X index of starting position inside potential field map
    iy = round((sy - miny) / reso) # Y index of starting position inside potential field map

    rx, ry = [sx], [sy] # list to hold the planned path (x and y positions)
    
    motion = get_motion_model() # Get motion model defined as discrete set of movements
    previous_ids = deque() # deque to hold previous ids for oscillation detection
    
    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny]
                
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
                
        ix = minix
        iy = miniy
        
        xp = ix * reso + minx
        yp = iy * reso + miny
        
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)
        
        if (oscillations_detection(previous_ids, ix, iy)): # Detect oscillations during path planning
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break
            
    return rx, ry # Return planned path


# Defines a function named potential_field_planning2, which takes the parameters sx, sy, gx, gy, rr, step_size with default value 2
def potential_field_planning2(sx, sy, gx, gy, rr, step_size=2):
    """
    This function calculates the potential field for given input parameters and returns the final path

    Args:
        sx (float): The x-coordinate of the start position in millimeters
        sy (float): The y-coordinate of the start position in millimeters
        gx (float): The x-coordinate of the goal position in millimeters
        gy (float): The y-coordinate of the goal position in millimeters
        rr (float): The radius of the robot in millimeters
        step_size (int, optional): The size of each step taken during path searching. Defaults to 2.

    Returns:
        A tuple containing two lists - rx and ry. These are the x-coordinates and y-coordinates of the complete path.
    """

    # Calculates the distance between start position and goal position using Euclidean Distance formula
    d = np.hypot(sx - gx, sy - gy)

    # Initialises the current position as start position
    ix = sx
    iy = sy

    # Intialises the list storing coordinates of the complete path with starting coordinates
    rx, ry = [sx], [sy]

    # Defines a motion model which is used to generate different motion moves
    motion = get_motion_model()

    # Keeps track of previous IDs to detect oscillations in the path
    previous_ids = deque()

    # The loop runs until the distance to the goal position is greater than or equal to twice of the step_size
    while d >= step_size*2:

        # Initializes minimum potential value and the corresponding x and y positions
        minp = float("inf")
        minix, miniy = -1, -1

        # Iterates through all possible motions to find the next potential minimum position
        for i, _ in enumerate(motion):
            inx = ix + motion[i][0] * step_size
            iny = iy + motion[i][1] * step_size
            
            # Gets the coordinates of the nearest obstacle from the current position
            obs_pos = ComObjectCollection.getNearestObstacle((inx, iny, 0))
            
            # If an obstacle is present, get its X and Y coordinate values
            if obs_pos is not None:
                ox, oy, _ = obs_pos[0][0]
            else: 
                ox = oy = None
                
            # Calculates the potential value for the new position
            p = calc_potential_field3(inx, iny, gx, gy, ox, oy, rr)
            
            # Updates the position with minimum potential value
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        
        # Shifts the position to the new minimum potential value position
        ix = minix
        iy = miniy

        # Adds the new position to the path list
        rx.append(ix)
        ry.append(iy)

        # Detects oscillations in the path to avoid redundancy
        if (oscillations_detection(previous_ids, ix, iy)):
            break

    # Returns the final path as a tuple of lists containing x and y coordinates respectively
    return rx, ry


def calc_potential_field(gx, gy, ox, oy, rr, reso=1, map_size=(-1000, -1000, 1000, 1000)):
    """
    Calculates potential field for given parameters.

    Args:
        gx (float): Goal x position [mm].
        gy (float): Goal y position [mm].
        ox (list): List of obstacle x positions [mm].
        oy (list): List of obstacle y positions [mm].
        rr (float): Robot radius [mm].
        reso (float): Size of each unit grid in mm. Defaults to 1.
        map_size (tuple): Map size dimensions as a tuple of minx, miny, maxx, maxy in mm. Defaults to (-1000, -1000, 1000, 1000).

    Returns:
        pmap (list): 2D array representing the potential field.
        minx (float): Minimum x value of the map.
        miny (float): Minimum y value of the map.
    """

    # Get the boundaries of the map
    minx, miny, maxx, maxy = map_size

    # Calculate width and height of the map
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # Initialize the potential field with default values
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    # Iterate over all x and y coordinates of the map
    for ix in range(xw):
        x = ix * reso + minx
        for iy in range(yw):
            y = iy * reso + miny

            # Check if goal x and goal y coordinates are not None
            if gx is not None and gy is not None:
                # Calculate attractive potential and repulsive potential if goal x and goal y are provided
                ug = calc_attractive_potential(x, y, gx, gy)
                uo = calc_repulsive_potential(x, y, ox, oy, rr)
                uf = ug + uo
            else:
                # Calculate only repulsive potential if goal x and goal y are None
                uo = calc_repulsive_potential(x, y, ox, oy, rr)
                uf = uo

            # Assign the calculated potential value to the respective coordinate of the potential field
            pmap[ix][iy] = uf
    
    # Return the calculated potential field and starting x and y position
    return pmap, minx, miny




def calc_potential_field2(gx, gy, ox, oy, rr, reso=1, map_size=(-1000, -1000, 1000, 1000)):
    """
    Calculate the potential field for a given goal and obstacle positions

    Args:
        gx (float): Goal x position in millimeters
        gy (float): Goal y position in millimeters
        ox (List[float]): List of obstacle x positions in millimeters
        oy (List[float]): List of obstacle y positions in millimeters
        rr (float): Robot radius in millimeters
        reso (float): Grid size for each unit in millimeters (Default value = 1)
        map_size (Tuple[int,int,int,int]): Tuple containing the minimum and maximum x and y values of the map in millimeters 
                                           (Default value = (-1000, -1000, 1000, 1000))

    Returns:
        pmap (List[List[float]]): 2D list of potential values representing the potential field
        minx (float): Minimum x value of the map in millimeters
        miny (float): Minimum y value of the map in millimeters
    """

    # Get the boundaries of the map
    minx, miny, maxx, maxy = map_size

    # Calculate width and height of the map
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # Initialize the potential field with default values
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    # Create a 2D numpy array representing x and y coordinates
    pmap = np.array(pmap)
    x_mat = [[i for i in range(xw)]]
    x_mat_tmp = copy.copy(x_mat)
    y_mat = [[i] for i in range(yw)]
    y_mat_tmp = copy.copy(y_mat)

    # Fill the 2D numpy array with x and y coordinate values
    for _ in range(yw-1):
        x_mat = np.concatenate((x_mat, x_mat_tmp), axis=0)
    for _ in range(xw-1):
        y_mat = np.concatenate((y_mat, y_mat_tmp), axis=1)
    x_mat = x_mat * reso + minx
    y_mat = y_mat * reso + miny

    # Calculate potential values based on goal and obstacle positions
    if gx is not None or gy is not None:
        # Calculate attractive potential
        ug = calc_attractive_potential(x_mat, y_mat, gx, gy)

        # Calculate repulsive potential
        uo = calc_repulsive_potential2(x_mat, y_mat, ox, oy, rr)

        # Combine attractive and repulsive potentials
        uf = ug + uo
    else:
        # If no goal is specified, only calculate repulsive potential
        uo = calc_repulsive_potential2(x_mat, y_mat, ox, oy, rr)
        uf = uo

    # Convert the calculated 2D numpy array to a list of lists representing the potential field
    pmap = uf.T.tolist()

    # Return the calculated potential field and starting x and y position
    return pmap, minx, miny


def calc_potential_field3(x, y, gx, gy, ox, oy, rr):
    '''
    Calculates potential field based on given start point, goal point, and obstacle locations.

    Args:
        x (float): x-coordinate of start point, in millimeters (mm)
        y (float): y-coordinate of start point, in millimeters (mm)
        gx (float): x-coordinate of goal point, in millimeters (mm). If no goal point is present, set to None.
        gy (float): y-coordinate of goal point, in millimeters (mm). If no goal point is present, set to None.
        ox (List[float]): list of x-coordinates of obstacles, in millimeters (mm).
        oy (List[float]): list of y-coordinates of obstacles, in millimeters (mm).
        rr (float): robot radius, in millimeters (mm).
         
    Returns:
        p (List[float]): 1D list representing the potential field.
    '''

    # If a goal point is present, calculate the sum of attractive potential from start to goal and repulsive potential from obstacles.
    if gx is not None or gy is not None:
        ug = calc_attractive_potential(x, y, gx, gy) # calculates attractive potential
        uo = calc_repulsive_potential3(x, y, ox, oy, rr) # calculates repulsive potential from obstacles
        uf = ug + uo # calculates total potential
    else: # If no goal point is present, only calculate repulsive potential from obstacles.
        uo = calc_repulsive_potential3(x, y, ox, oy, rr)
        uf = uo

    p = uf # assign the total potential to variable p

    return p # returns the potential field represented as a 1D list.



def calc_attractive_potential(x, y, gx, gy):
    '''
    Calculates attractive potential between start point (x, y) and goal point (gx, gy).

    Args:
        x (float): x-coordinate of start point, in millimeters (mm).
        y (float): y-coordinate of start point, in millimeters (mm).
        gx (float): x-coordinate of goal point, in millimeters (mm).
        gy (float): y-coordinate of goal point, in millimeters (mm).

    Returns:
        ret (float): attractive potential.
    '''

    ret = 0.5 * KP * np.hypot(x - gx, y - gy) # calculates attractive potential
    return ret # returns the attractive potential


def calc_repulsive_potential(x, y, ox, oy, rr):
    '''
    Calculates repulsive potential from obstacles.

    Args:
        x (float): x-coordinate of point to calculate potential for, in millimeters (mm).
        y (float): y-coordinate of point to calculate potential for, in millimeters (mm).
        ox (List[float]): list of x-coordinates of obstacles, in millimeters (mm).
        oy (List[float]): list of y-coordinates of obstacles, in millimeters (mm).
        rr (float): robot radius, in millimeters (mm).

    Returns:
        ret (float): repulsive potential.
    '''
    
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox): # iterate through obstacles and find the one closest to (x,y)
        d = np.hypot(x - ox[i], y - oy[i]) # calculate Euclidean distance between (x,y) and obstacle
        if dmin >= d: # if this obstacle is closer than the previous closest one, update minid and dmin
            dmin = d
            minid = i
    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid]) # calculate Euclidean distance between (x,y) and nearest obstacle

    if dq <= rr:
        # print(dq)
        if dq <= 0.1:
            dq = 0.1
        ret = 3 * ETA * (1.0 / dq - 1.0 / rr) ** 0.3 # calculates repulsive potential using given formula when robot is within the obstacle radius
        return ret
    else: # If robot is outside the obstacle radius, its potential is zero.
        return 0.0


def calc_repulsive_potential2(x, y, ox, oy, rr):
    '''
    Vectorized version of calc_repulsive_potential function.

    Args:
        x (float): Array of x-coordinates of points to calculate potential for, in millimeters (mm).
        y (float): Array of y-coordinates of points to calculate potential for, in millimeters (mm).
        ox (List[float]): list of x-coordinates of obstacles, in millimeters (mm).
        oy (List[float]): list of y-coordinates of obstacles, in millimeters (mm).
        rr (float): robot radius, in millimeters (mm).

    Returns:
        ret (ndarray): Array of repulsive potentials.
    '''

    ret = np.zeros_like(x, dtype=float) # initialize output array with zeros
    if len(ox) ==  0 or len(oy) == 0: # if there are no obstacles, return an array of zeros
        return ret

    # search nearest obstacle
    minid = np.zeros_like(x, dtype=int)
    minid[:] = -1
    dmin = np.zeros_like(x, dtype=float)
    dmin[:] = float("inf")
    for i, _ in enumerate(ox): # iterate through obstacles and find the one closest to each (x,y) pair
        d = np.hypot(x - ox[i], y - oy[i]) # calculate Euclidean distance between each (x,y) pair and obstacle
        minid[dmin>d] = i # update minid only if this obstacle is closer than the previous closest one
        dmin[dmin>d] = d[dmin>d]

    # calc repulsive potential
    ox_min_mat = np.array(ox)[minid]
    oy_min_mat = np.array(oy)[minid]
    dq = np.hypot(x - ox_min_mat, y - oy_min_mat) # calculate Euclidean distance between each (x,y) pair and its corresponding nearest obstacle
    
    dq[dq<=0.1] = 0.1 # set minimum distance to 0.1 mm
    ret[dq<=rr] = 3 * ETA * (1.0 / dq[dq<=rr] - 1.0 / rr) ** 0.3 # calculates repulsive potential using given formula when robot is within the obstacle radius
    return ret # returns an array of repulsive potentials 


def calc_repulsive_potential3(x, y, ox, oy, rr):
    """Calculate repulsive potential.

    Args:
        x (float): x-coordinate of point to calculate potential for.
        y (float): y-coordinate of point to calculate potential for.
        ox (float): x-coordinate of nearest obstacle.
        oy (float): y-coordinate of nearest obstacle.
        rr (float): robot radius.

    Returns:
        float: Repulsive potential.
    """
    if ox is None or oy is None: # If there are no obstacles, the potential is zero.
        return 0.0
    
    # Calculate repulsive potential using given formula when robot is within the obstacle radius.
    dq = np.hypot(x - ox, y - oy) # calculate Euclidean distance between (x,y) and nearest obstacle.
    
    if dq <= rr:
        # print(dq)
        if dq <= 0.1:
            dq = 0.1
        ret = 3 * ETA * (1.0 / dq - 1.0 / rr) ** 0.1
        return ret
    else:
        return 0.0

def get_motion_model():
    '''
    Returns a motion model for the robot.

    Returns:
        List[List[float]]: List of possible motions for robot [dx, dy].
    '''
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def oscillations_detection(previous_ids, ix, iy):
    """Detect oscillations in position.

    Args:
        previous_ids (deque): A queue containing previous positions.
        ix (int): The current x position.
        iy (int): The current y position.

    Returns:
        bool: True if we have gone through a loop and are oscillating. False otherwise.
    """    
    # Add the current position to the queue of previous positions
    previous_ids.append((ix, iy))

    # If the queue is longer than a fixed length,
    # remove the oldest element from the left (i.e. the first in the list)
    if len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH:
        previous_ids.popleft()

    # Check if there are any duplicate positions in the queue
    # by copying the queue into a set and comparing lengths
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            # If the current position is already in the set,
            # it means we have gone through a loop and are oscillating
            return True
        else:
            # Otherwise, add the position to the set and continue
            previous_ids_set.add(index)
    
    # If we've made it through the whole loop without finding a duplicate,
    # it means we are not oscillating
    return False


class ComPathPlanning:
    def __init__(self) -> None:
        # Initialize class variables
        self.mTarget = None
        self.mPos = None 
        self.mRobotRadius = 20  
        self.mPathPtList_x = None 
        self.mPathPtList_y = None 
        self.mEnvSize = None
        self.mStride = 10   # Set the step size of the search in gradient descent. A larger value can better jump out of the local extreme value, but the path planning is less precise
        self.isRandomLeapOn = True   # Whether it can jump out of local extreme values by random movement, default yes
        self.mRandomLeapThreshold = 3  # Step multiple that jumps out of the extreme value judgment
        self.mTargetBackup = None
        self.mRandomMoveRange = 30      # The multiple of the step size of the random range of movement
    
    def setPos(self, pos):
        """
        Sets the current position of the robot.

        Args:
            pos: A tuple representing the new position of the robot.

        Returns:
            None.
        """
        # Set the current position of the robot
        self.mPos = pos 
        
    def setEnvSize(self, _size):
        """
        Sets the size of the environment that the robot is moving in.

        Args:
            _size: A tuple representing the size of the environment.

        Returns:
            None.
        """

        # Set the size of the environment this robot is moving in
        self.mEnvSize = _size

    def setTarget(self, target: tuple):
        """
        Sets the coordinates for the robot to move towards.

        Args:
            target: A tuple representing the target coordinates.

        Returns:
            None.
        """       
        # Set the target coordinates for the robot to move towards
        self.mTarget = target

    def setRobotRadius(self, radius):
        """
        Sets the radius of the robot used for path planning.

        Args:
            radius: The new radius of the robot.

        Returns:
            None.
        """
        # Set the radius of the robot, used for path planning
        self.mRobotRadius = radius

    def setStride(self, stride_len):
        """
        Sets the stride length of the gradient descent algorithm used in path planning.

        Args:
            stride_len: An integer representing the new stride length.

        Returns:
            None.
        """
        # Set the stride length of the gradient descent algorithm used for path planning
        self.mStride = stride_len

    def isRandomMove(self):
        """
        Determines whether random movement is needed to escape local minima.

        Returns:
            A boolean indicating whether random movement is required.
        """
        # Returns a boolean indicating whether random movement is required to escape local minima
        if not self.isRandomLeapOn:
            return False 
        
        x = self.mPathPtList_x[-1]
        y = self.mPathPtList_y[-1]
        if myUtils.distance((x, y), self.mTarget[0:2]) > self.mStride*self.mRandomLeapThreshold and self.mTargetBackup is None:
            return True 
        else:
            return False
    
    def randomMove(self):
        """Randomly moves the robot within a given range to escape local minima.
        """
        # Define variables for x and y coordinates of the random move
        x = 0
        y = 0

        # Calculate the stride limit for the random move
        multi_stride = self.mStride * self.mRandomMoveRange

        # Define the ranges for x and y coordinates based on the stride limit
        range_x = [self.mPos[0]-multi_stride, self.mPos[0]+multi_stride]
        range_y = [self.mPos[1]-multi_stride, self.mPos[1]+multi_stride]

        # Check whether the environment size is defined
        if self.mEnvSize is not None:
            # Limit the robot's movement based on the environment size
            minx = -self.mEnvSize[0]
            maxx = self.mEnvSize[0]
            miny = -self.mEnvSize[1]
            maxy = self.mEnvSize[1]
            
            # Check and adjust x and y coordinates if they go beyond the environment boundaries
            if range_x[0] < minx:
                range_x[0] = minx 
            if range_x[1] > maxx:
                range_x[1] = maxx 
            if range_y[0] < miny:
                range_y[0] = miny
            if range_y[1] > maxy:
                range_y[1] = maxy
                
        # Generate a random target coordinate within the defined ranges for x and y
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        
        # Set generated target as the robot's new target
        self.setTarget((x, y, 0))


    def update(self):
        """Updates the path from the robot's current position to the target position using potential field path planning

        Args:
            None

        Returns:
            None
        """
        # Extract the x, y coordinates from the target position and the robot radius from the class variables
        gx, gy = self.mTarget[0:2]
        rr = self.mRobotRadius
        # Extract the x, y coordinates from the robot's current position from the class variables
        sx, sy = self.mPos[0:2]
        # Use potential field path planning function to get a path from the current position to the target position
        # The resulting path points are stored in mPathPtList_x and mPathPtList_y class variables.
        self.mPathPtList_x, self.mPathPtList_y = potential_field_planning2(
            sx, 
            sy, 
            gx, 
            gy,
            rr,
            self.mStride)

        
    def getNextDest(self):
        """Get the next destination for the robot to move towards.

        Args:
            None

        Returns:
            A tuple containing the x-coordinate and y-coordinate of the next destination, as well as the angle at which
            the robot should move towards that destination.
        """
        # Select the second point in the path as the next destination, calculate the angle to reach it, and return these values
        pt_num = 2
        if len(self.mPathPtList_x) > 5:
            x1, x2 = self.mPathPtList_x[pt_num:pt_num+2]
            y1, y2 = self.mPathPtList_y[pt_num:pt_num+2]
            v = (x2-x1, y2-y1)
            angle = myUtils.angle_with_x_axis(v)
            return self.mPathPtList_x[pt_num], self.mPathPtList_y[pt_num], angle
        # If there are no more points in the path, randomly move the robot or reset its destination
        elif self.isRandomMove():
            if self.mTargetBackup is None:
                self.mTargetBackup = copy.copy(self.mTarget)
            self.randomMove()
            return None, None, None
        elif self.mTargetBackup is not None:
            self.mTarget =  copy.copy(self.mTargetBackup)
            self.mTargetBackup = None
            return None, None, None
        # If there are no more points in the path and no more destinations to randomize, return the target coordinates
        else:
            return self.mTarget[0], self.mTarget[1], None

