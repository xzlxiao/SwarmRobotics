import numpy as np
import Common.utils as myUtils
import math
import time 
import copy 
from collections import deque
from Simulation.ComPathPlanning import ComPathPlanning
from Simulation import ComObjectCollection





# Parameters
# KP = 5.0  # attractive potential gain
KP = 5  # attractive potential gain
KP_MAT = 5#0.3
# ETA = 100.0  # repulsive potential gain
ETA = 500000.0  # repulsive potential gain
ETA_MAT = 6000.0  # repulsive potential gain 5000
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

def potential_field_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, rr, reso=1, map_size=(-1000, -1000, -1000, 1000, 1000, 1000)):
    """Plan the path for a robot to navigate from a start position to a goal position using a potential field algorithm.
    
    Args:
        sx: The x-coordinate of the robot's starting position in millimeters.
        sy: The y-coordinate of the robot's starting position in millimeters.
        sz: The z-coordinate of the robot's starting position in millimeters.
        gx: The x-coordinate of the robot's goal position in millimeters.
        gy: The y-coordinate of the robot's goal position in millimeters.
        gz: The z-coordinate of the robot's goal position in millimeters.
        ox: A list of the x-coordinates of all obstacles in millimeters.
        oy: A list of the y-coordinates of all obstacles in millimeters.
        oz: A list of the z-coordinates of all obstacles in millimeters.
        rr: The radius of the robot in millimeters.
        reso: The size of each unit grid in millimeters. Defaults to 1.
        map_size: A tuple containing the minimum and maximum x, y, and z values for the map. Defaults to (-1000, -1000, -1000, 1000, 1000, 1000).
        
    Returns:
        A tuple of lists containing the x-coordinates, y-coordinates, and z-coordinates of the path, respectively.
    """
    # Initialize variables and arrays for path planning
    minx, miny, minz, maxx, maxy, maxz = map_size
    d = np.sqrt((sx - gx)**2+(sy - gy)**2+(sz - gz)**2)

    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    iz = round((sz - minz) / reso)

    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))
    zw = int(round((maxz - minz) / reso))

    pmap = [ [ [0.0 for i in range(zw)] for i in range(yw)] for i in range(xw)]
    
    rx, ry, rz = [sx], [sy], [sz]
    motion_step = 1
    motion = np.array(get_motion_model()) * motion_step
    motion = motion.tolist()
    previous_ids = deque()
    
    # Iteratively calculate potential field values and select the next movement direction based on the minimum value
    while d >= reso:
        minp = float("inf")
        minix, miniy, miniz = -1, -1, -1
        mini = 0
        for i, _ in enumerate(motion):
            # Calculate the position of the robot after taking this motion step
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            inz = int(iz + motion[i][2])
            x = inx * reso + minx
            y = iny * reso + miny
            z = inz * reso + minz
            
            # If the position is outside the map, set its potential to infinity
            if inx >= len(pmap) or iny >= len(pmap[0]) or inz >= len(pmap[0][0]) or inx < 0 or iny < 0 or inz < 0:
                p = float("inf")  
                print("outside potential!")
            else:
                # Calculate the attractive and repulsive potentials at this position, and combine them to get the total potential
                if gx is not None or gy is not None:
                    ug = calc_attractive_potential(x, y, z, gx, gy, gz)
                    uo = calc_repulsive_potential(x, y, z, ox, oy, oz, rr)
                    uf = ug + uo
                else:
                    uo = calc_repulsive_potential(x, y, z, ox, oy, oz, rr)
                    uf = uo

                pmap[inx][iny][inz] = uf
                p = pmap[inx][iny][inz]
            
            # If the potential at this point is lower than the minimum recorded value so far, update the minimum values
            if minp > p:
                minp = p
                mini = i
                minix = inx
                miniy = iny
                miniz = inz
        
        # Update the robot's position to the location corresponding to the minimum potential
        ix = minix
        iy = miniy
        iz = miniz
        xp = ix * reso + minx
        yp = iy * reso + miny
        zp = iz * reso + minz
        d = np.sqrt((gx - xp)**2+(gy - yp)**2+(gz - zp)**2)
        rx.append(xp)
        ry.append(yp)
        rz.append(zp)
        
        # If the robot has become trapped in an oscillation, return the path so far
        if (oscillations_detection(previous_ids, xp, yp, zp)):
            print("Oscillation detected at ({},{},{})!".format(xp, yp, zp))
            break
    return rx, ry, rz


def potential_field_planning2(sx, sy, sz, gx, gy, gz, rr, step_size=2):
    """
    This function uses a potential field planning algorithm to generate a path from the starting position (sx,sy,sz) 
    to the goal position (gx,gy,gz). Other inputs include robot radius (rr) and step size (2 by default).
    The function returns a list of x,y,z coordinates of the path.

    Args:
        sx (float): Starting position x coordinate.
        sy (float): Starting position y coordinate.
        sz (float): Starting position z coordinate.
        gx (float): Goal position x coordinate.
        gy (float): Goal position y coordinate.
        gz (float): Goal position z coordinate.
        rr (float): Robot radius.
        step_size (int, optional): Step size for generating the path. Defaults to 2.

    Returns:
        tuple: A tuple of lists containing the x,y,z coordinates of the path from start to goal.
    """
    
    # calculate distance between the starting and goal positions
    d = np.hypot(sx - gx, sy - gy)
    
    # initialize variables for starting position
    ix = sx
    iy = sy
    iz = sz

    # create empty lists to store positions visited in the path
    rx, ry, rz = [sx], [sy], [sz]
    
    # create motion model using get_motion_model() function
    motion = get_motion_model()
    
    # create a deque to store previous ids
    previous_ids = deque()
    
    # run loop until distance to goal is greater than or equal to twice the step size
    while d >= step_size*2:
        
        # initialize minimum potential value and position
        minp = float("inf")
        minix, miniy, miniz = -1, -1, -1
        
        # iterate through each possible direction in the motion model
        for i, _ in enumerate(motion):
            
            # calculate next position
            inx = ix + motion[i][0] * step_size
            iny = iy + motion[i][1] * step_size
            inz = iz + motion[i][2] * step_size
            
            # get nearest obstacle's position
            obs_pos = ComObjectCollection.getNearestObstacle((inx, iny, inz))
            
            # if an obstacle is present at nearest position, assign its coordinates to ox, oy, oz. Otherwise, 
            # set them as None.
            if obs_pos is not None:
                ox, oy, oz = obs_pos[0][0]
            else: 
                ox = oy = oz = None
                
            # calculate potential field at the new position using calc_potential_field3 function
            p = calc_potential_field3(inx, iny, inz, gx, gy, gz, ox, oy, oz, rr)
            
            # choose direction with the minimum potential value as the next position
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
                miniz = inz

        # update current position to the chosen next position
        ix = minix
        iy = miniy
        iz = miniz
        
        # append new position to the path list
        rx.append(ix)
        ry.append(iy)
        rz.append(iz)

        # break out of loop if oscillation detected
        if (oscillations_detection(previous_ids, ix, iy, iz)):
            break
            
        # update distance to goal
        d = np.hypot(gx - ix, gy - iy)
    
    # return the list of x, y and z coordinates of the path from start to goal
    return rx, ry, rz


import numpy as np # Importing numpy for array operations

def calc_potential_field_mat(gx, gy, gz, ox, oy, oz, rr, reso=1, map_size=(-1000, -1000, -1000, 1000, 1000, 1000)):
    """This function calculates the potential field map for given goal and obstacle positions.

    Args:
        gx (float): Goal x position [mm].
        gy (float): Goal y position [mm].
        gz (float): Goal z position [mm].
        ox (list): List of obstacle x positions [mm].
        oy (list): List of obstacle y positions [mm].
        oz (list): List of obstacle z positions [mm].
        rr (float): Robot radius [mm].
        reso (float, optional): Grid size in mm. Defaults to 1.
        map_size (tuple, optional): Map size in mm. Defaults to (-1000, -1000, -1000, 1000, 1000, 1000).

    Returns:
        tuple: Tuple containing the potential field map, and minimum x, y, and z values.
    """

    # Extracting minimum and maximum values from the provided map size
    minx, miny, minz, maxx, maxy, maxz = map_size

    # Calculating the number of cells in each dimension
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))
    zw = int(round((maxz - minz) / reso))

    # Creating an empty 3D numpy array with dimensions (yw, xw, zw)
    pmap = np.zeros((yw, xw, zw), dtype=float)

    # Creating mesh grid of all x, y and z positions in the potential field
    x_mat, y_mat, z_mat = np.meshgrid(np.arange(xw), np.arange(yw), np.arange(zw))
    x_mat = x_mat * reso + minx # Converting x position from cell coordinate to world coordinate
    y_mat = y_mat * reso + miny # Converting y position from cell coordinate to world coordinate
    z_mat = z_mat * reso + minz # Converting z position from cell coordinate to world coordinate

    # Calculating attractive potential if a goal is provided
    if gx is not None or gy is not None:
        ug = calc_attractive_potential_mat(x_mat, y_mat, z_mat, gx, gy, gz)
        uo = calc_repulsive_potential_mat(x_mat, y_mat, z_mat, ox, oy, oz, rr) # Calculate repulsive potential for obstacles
        uf = ug + uo # Adding up both potentials to get the final potential field
    else:
        uo = calc_repulsive_potential_mat(x_mat, y_mat, z_mat, ox, oy, oz, rr) # Calculate repulsive potential for obstacles
        uf = uo # No goal provided, hence final potential field = repulsive potential map

    pmap = uf # Assigning the final potential field to the empty potential field array

    return pmap, minx, miny, minz


def calc_potential_field3(x, y, z, gx, gy, gz, ox, oy, oz, rr):
    """
    This function calculates the potential field for a given geometry of goal position, obstacle positions, and
    robot radius.

    Args:
        x (float): The current x position of the robot
        y (float): The current y position of the robot
        z (float): The current z position of the robot
        gx (float): The x position of the goal point
        gy (float): The y position of the goal point
        gz (float): The z position of the goal point
        ox (list): A list of x positions for all obstacles
        oy (list): A list of y positions for all obstacles
        oz (list): A list of z positions for all obstacles
        rr (float): Radius of the robot

    Returns:
        p (float): The final potential field for the given configuration
    """    

    # If goal position provided, calculate attractive potential
    if gx is not None and gy is not None and gz is not None:
        ug = calc_attractive_potential(x, y, z, gx, gy, gz) # Calculate attractive potential

        uo = calc_repulsive_potential3(x, y, z, ox, oy, oz, rr) # Calculate repulsive potential
        uf = ug + uo # Adding both potentials to get the final potential
    else:
        uo = calc_repulsive_potential3(x, y, z, ox, oy, oz, rr) # Calculate repulsive potential
        uf = uo # No goal provided, hence final potential field = repulsive potential map

    p = uf # Assigning the final potential field to variable 'p'

    return p



def calc_attractive_potential(x, y, z, gx, gy, gz):
    """
    This function calculates the attractive potential for a given geometry of robot and goal positions.

    Args:
        x (float): The current x position of the robot
        y (float): The current y position of the robot
        z (float): The current z position of the robot
        gx (float): The x position of the goal point
        gy (float): The y position of the goal point
        gz (float): The z position of the goal point

    Returns:
        (float): The calculated attractive potential
    """
    return 0.5 * KP * math.sqrt((x - gx)**2+(y - gy)**2+(z - gz)**2) # Calculating and returning attractive potential

def calc_attractive_potential_mat(x, y, z, gx, gy, gz):
    """
    This function calculates the attractive potential for a given geometry of robot and goal positions using numpy arrays.

    Args:
        x (numpy.ndarray): An array containing current x positions of the robots
        y (numpy.ndarray): An array containing current y positions of the robots
        z (numpy.ndarray): An array containing current z positions of the robots
        gx (float): The x position of the goal point
        gy (float): The y position of the goal point
        gz (float): The z position of the goal point

    Returns:
        (numpy.ndarray): An array containing the calculated attractive potentials for each robot
    """
    return 0.5 * KP_MAT * np.sqrt((x - gx)**2+(y - gy)**2+(z - gz)**2) # Calculating and returning attractive potential using numpy arrays

def calc_repulsive_potential(x, y, z, ox, oy, oz, rr):
    """
    This function calculates the repulsive potential for a given geometry of robot, obstacle positions and robot radius.

    Args:
        x (float): The current x position of the robot
        y (float): The current y position of the robot
        z (float): The current z position of the robot
        ox (list): A list of x positions for all obstacles
        oy (list): A list of y positions for all obstacles
        oz (list): A list of z positions for all obstacles
        rr (float): Radius of the robot

    Returns:
        (float): The calculated repulsive potential
    """
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox): # Looping through all obstacles to find nearest one
        d = math.sqrt((x - ox[i])**2+(y - oy[i])**2+(z - oz[i])**2)
        if dmin >= d:
            dmin = d
            minid = i   
    dq = math.sqrt((x - ox[minid])**2+(y - oy[minid])**2+(z - oz[minid])**2) # Distance between robot and nearest obstacle
    
    if dq <= rr:
        if dq <= 0.1: # To avoid division by zero error
            dq = 0.1
        return 3 * ETA * (1.0 / dq - 1.0 / rr) ** 1 # Calculating and returning repulsive potential
    else:
        return 0.0 

def calc_repulsive_potential_mat(x, y, z, ox, oy, oz, rr):
    """
    This function calculates the repulsive potential for a given geometry of robot, obstacle positions and robot radius using numpy arrays.

    Args:
        x (numpy.ndarray): An array containing current x positions of the robots
        y (numpy.ndarray): An array containing current y positions of the robots
        z (numpy.ndarray): An array containing current z positions of the robots
        ox (list): A list of x positions for all obstacles
        oy (list): A list of y positions for all obstacles
        oz (list): A list of z positions for all obstacles
        rr (float): Radius of the robot

    Returns:
        (numpy.ndarray): An array containing the calculated repulsive potentials for each robot
    """
    ret = np.zeros_like(x, dtype=float)
    if len(ox) ==  0 or len(oy) == 0 or len(oz): # If there are no obstacles, return zero potential
        return ret
    
    # search nearest obstacle
    minid = np.zeros_like(x, dtype=int)
    minid[:] = -1
    dmin = np.zeros_like(x, dtype=float)
    dmin[:] = float("inf")
    for i, _ in enumerate(ox): # Looping through all obstacles to find nearest one
        d = np.sqrt((x - ox[i])**2+(y - oy[i])**2+(z - oz[i])**2)
        minid[dmin>d] = i
        dmin[dmin>d] = d[dmin>d]

    # calc repulsive potential
    ox_min_mat = np.array(ox)[minid]
    oy_min_mat = np.array(oy)[minid]
    oz_min_mat = np.array(oz)[minid]

    dq = np.sqrt((x - ox_min_mat)**2+(y - oy_min_mat)**2+(z - oz_min_mat)**2)
    dq[dq<=1] = 4#0.1

    ret[dq<=rr] = 1 * ETA_MAT * (1.0 / dq[dq<=rr] - 1.0 / rr) ** 0.3#3 0.1 # Calculating and returning repulsive potential using numpy arrays
    return ret




def calc_repulsive_potential3(x, y, z, ox, oy, oz, rr):
    """
    Calculate repulsive potential

    Args:
        x (float): Robot's x coordinate
        y (float): Robot's y coordinate 
        z (float): Robot's z coordinate
        ox (float): Closest obstacle's x coordinate
        oy (float): Closest obstacle's y coordinate
        oz (float): Closest obstacle's z coordinate
        rr (float): Maximum range of repulsive potential

    Returns:
        float: Repulsive potential value
    """
    # Return 0 if there's no obstacle nearby
    if ox is None or oy is None or oz is None:
        return 0.0
    
    # Calculate the distance between robot and the nearest obstacle
    dq = math.sqrt((x - ox)**2+(y - oy)**2+(z - oz)**2)
    
    # Calculate repulsive potential if the obstacle is within range
    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1
        ret = 3 * ETA_MAT * (1.0 / dq - 1.0 / rr) ** 0.3
        return ret
    else:
        return 0.0


def get_motion_model():
    """Returns a list of possible motions in three-dimensional space.

    Returns:
        List: A list containing all possible directional movements along three dimensions 
              along with their associated costs.
    """    
    # dx, dy
    motion = [[1, 0, 1],               
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, 1],
              [-1, 1, 1],
              [1, -1, 1],
              [1, 1, 1],
              [0, 0, 1],

              [1, 0, 0],
              [0, 1, 0],
              [-1, 0, 0],
              [0, -1, 0],
              [-1, -1, 0],
              [-1, 1, 0],
              [1, -1, 0],
              [1, 1, 0], 
              
              [1, 0, -1],
              [0, 1, -1],
              [-1, 0, -1],
              [0, -1, -1],
              [-1, -1, -1],
              [-1, 1, -1],
              [1, -1, -1],
              [1, 1, -1],
              [0, 0, -1],]

    return motion



# def oscillations_detection(previous_ids, ix, iy, iz, minx, miny, minz, reso, path=[]):
#     previous_ids.append((ix * reso + minx, iy * reso +miny, iz * reso + minz))
#     path.append((ix * reso + minx, iy * reso +miny, iz * reso + minz))
def oscillations_detection(previous_ids, xp, yp, zp, path=[]):
    """Detects oscillations in the robot path and returns True if oscillation is detected.

    Args:
        previous_ids (deque): A deque containing previous positions of the robot.
        xp (int): The x-coordinate of the robot's current position.
        yp (int): The y-coordinate of the robot's current position.
        zp (int): The z-coordinate of the robot's current position.
        path (list): A list to keep track of the robot's current path. Defaults to an empty list.

    Returns:
        bool: True if the current position has already been visited before, False otherwise.
    """

    # Append current position to previous_ids and path.
    previous_ids.append((xp, yp, zp))
    path.append((xp, yp, zp))

    # If previous_ids exceeds a certain limit, remove oldest element from previous_ids.
    if len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH:
        previous_ids.popleft()

    # Check if set contains any duplicates
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False


class ComPathPlanning3D(ComPathPlanning):
    def __init__(self) -> None:
        super(ComPathPlanning3D, self).__init__()
        # self.mTarget = None
        # self.mPos = None 
        # self.mObstacleList = None
        # self.mRobotRadius =  20 
        # self.mPathPtList_x = None 
        # self.mPathPtList_y = None 
        self.mStride = 15   # 设置梯度下降时搜索的步长，较大的值可以更好的跳出局部极值，但路径规划越不精细
        self.mPathPtList_z = None 
        # self.mEnvSize = None
    
    # def setPos(self, pos):
    #     self.mPos = pos 
        
    # def setEnvSize(self, _size):
    #     self.mEnvSize = _size

    # def setTarget(self, target: tuple):
    #     self.mTarget = target

    # def setRobotRadius(self, radius):
    #     self.mRobotRadius = radius

    # def setObstacleList(self, obstacle_list: list):
    #     self.mObstacleList = obstacle_list

    def update(self):
        """Updates the position of the robot using the potential field path planning algorithm.

        The function calls the `potential_field_planning2()` function to generate a path from the current position of the
        robot to its target position. The generated path is set as `mPathPtList_x`, `mPathPtList_y`, and `mPathPtList_z`
        respectively.

        """
        
        # Get target coordinates
        gx, gy, gz = self.mTarget[0:3]
        # Get robot radius
        rr = self.mRobotRadius
        # Get start co-ordinates
        sx, sy, sz = self.mPos[0:3]

        # Call potential_field_planning2() to generate path based on start and end positions
        self.mPathPtList_x, self.mPathPtList_y, self.mPathPtList_z = potential_field_planning2(
            sx,
            sy,
            sz,
            gx,
            gy,
            gz,
            rr,
            self.mStride
        )

    def getNextDest(self):
        """Returns the next destination coordinate for the robot to move towards and the angle at with the robot should travel.

        The function gets the next two x,y points in the robot's planned path from mPathPtList_x and mPathPtList_y. It calculates
        the angle of the vector between those two points with respect to the x-axis and returns the coordinates along with 
        the calculated angle. If the robot has not yet reached its target, it selects the next destination based on the 
        calculated angle. If the robot has already reached its target, it returns the target coordinates and None for the angle. 

        Returns:
            tuple (float, float, float, float/None): a tuple containing the next destination x,y,z coordinates and the angle with respect to the x-axis or None if the robot is already at its target.
        """
        
        pt_num = 2
        if len(self.mPathPtList_x) > 3:
            # get the next two x,y points in the robot's planned path
            x1, x2 = self.mPathPtList_x[pt_num:pt_num+2] 
            y1, y2 = self.mPathPtList_y[pt_num:pt_num+2]
            
            v = (x2-x1, y2-y1)
            # calculate angle of the vector between the two points with respect to the x-axis
            angle = myUtils.angle_with_x_axis(v)
            # return the next destination coordinate and angle
            return self.mPathPtList_x[pt_num], self.mPathPtList_y[pt_num], self.mPathPtList_z[pt_num], angle
        else:
            # if the robot has reached its target, return the target coordinates and None for angle
            return self.mTarget[0], self.mTarget[1], self.mTarget[2], None

