'''
@namespace Common
File name: utils.py
Author: 肖镇龙（Zhenlong Xiao）
Description: This is a collection of commonly used function utilities in this project.
'''

isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from matplotlib.path import Path
import matplotlib.pyplot as plt
import re
import bezier
import networkx as nx
import numpy as np
from matplotlib.collections import LineCollection
import random
import os
import math
from draw import ImageProc
import cv2
import time
import copy
import time
from skimage import morphology

__colors = [
    "cyan",
    'royalblue',
    "lightsteelblue",
    'red',
    "purple",
    'blueviolet',
    "yellow",
    "lightgreen",
    "orange",
    'red',
    # 'antiquewhite',
    # 'aqua',
    # 'red',
    # 'white',
]
def getColor(num:int):
    """
    Returns the color associated with a given number.

    Args:
        num (int): The input number to retrieve the color for.

    Returns:
        str: The color corresponding to the input number.
    """
    return __colors[num%len(__colors)]

def readPointFromFile(dir: str):
    """
    Reads and parses a file containing coordinate points in the format 'x,y', and returns them as a list of tuples.

    Args:
        dir (str): The directory location of the file to be read.

    Returns:
        list: A list of tuples, with each tuple representing a (x, y) coordinate point.
    """
    ret = []
    with open(dir, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip()[0:-1].split(',')
            ret.append((float(line[0]), float(line[1])))
    return ret


def distance(pt1, pt2):
    """
    Calculates the Euclidean distance between two points in n-dimensional space.

    Args:
        pt1 (list or np.array): A list or array of float or int values representing the coordinates for the first point.
        pt2 (list or np.array): A list or array of float or int values representing the coordinates for the second point.

    Returns:
        float: The Euclidean distance between the two points.
    """
    pt1 = np.array(pt1, dtype=float)
    pt2 = np.array(pt2, dtype=float)
    return np.linalg.norm(pt2 - pt1, ord=2)


def calcLost(base_point_list: list, robot_point_list: list):
    """
    Calculates the deviation of each point in robot_point_list from the closest point in base_point_list.

    Args:
        base_point_list (list): A list of points to be used as the baseline.
        robot_point_list (list): A list of points to be evaluated.

    Returns:
        list: A list of distances representing the minimum distance between each robot point and the baseline.
    """
    ret = []
    for pt_robot in robot_point_list:
        dist_min = None
        for pt_base in base_point_list:
            dist = distance(pt_base, pt_robot)
            if dist_min is None:
                dist_min = dist
            elif dist < dist_min:
                dist_min = dist
        ret.append(dist_min)
    return ret


def svg_parse(path):
    """
    This function parses an SVG path string and converts it into a sequence of Path vertices.
    
    Args:
        path : str : A string containing SVG path commands
        
    Returns:
        numpy.ndarray : An array of vertices and their respective codes
    """
    commands = {'M': (Path.MOVETO,),
                'L': (Path.LINETO,),
                'Q': (Path.CURVE3,)*2,
                'C': (Path.CURVE4,)*3,
                'Z': (Path.CLOSEPOLY,)}
    vertices = []
    codes = []
    cmd_values = re.split("([A-Za-z])", path)[1:]  # Split over commands.
    for cmd, values in zip(cmd_values[::2], cmd_values[1::2]):
        # Numbers are separated either by commas, or by +/- signs (but not at
        # the beginning of the string).
        try:
            points = ([*map(float, re.split(",|(?<!^)(?=[+-])", values))] if values
                    else [(0., 0.)])  # Only for "z/Z" (CLOSEPOLY).
        except:
            continue
        points = np.reshape(np.array(points), (-1, 2))
        if cmd.islower():
            points += vertices[-1][-1]
        codes.extend(commands[cmd.upper()])
        vertices.append(points)
    return np.array(codes), np.concatenate(vertices)


def loadSVG(path_list:list, translate_list:list):
    """used for batch loading and processing of SVG files

    Args:
        path_list (list): A list of paths to SVG files.
        translate_list (list): A list of tuples representing the x,y values to translate each SVG file.

    Returns:
        tuple: Returns a tuple containing two numpy arrays. The first array contains codes and the second array contains vertices.
    """   
    codes = None
    vertices = None 
    for index, item in enumerate(path_list):
        code, vertice = svg_parse(item)
        vertice[:, 0] += translate_list[index][0]
        vertice[:, 1] += translate_list[index][1]
        if codes is None:
            codes = code
            vertices = vertice
        else:
            codes = np.concatenate([codes, code])
            vertices = np.concatenate([vertices, vertice])
    vertices[:, 0] -= np.max(vertices[:, 0])/2
    vertices[:, 1] -= np.max(vertices[:, 1])/2
    return codes, vertices


def getAllKeysInDict(dict_list: list):
    """
    Returns a list containing all unique keys in a list of dictionaries.

    Args:
        dict_list (list): A list of dictionaries. Each dictionary should contain key-value pairs representing items and their quantities.

    Returns:
        list: A list containing all unique keys found in the input list of dictionaries.
    """   
    keys_list = []
    for dict_item in dict_list:
        dict_item: dict
        for key_item in dict_item.keys():
            if key_item not in keys_list:
                keys_list.append(key_item)
    return keys_list


def distance(pt1, pt2):
    """
    Calculate the distance between two points

    Args:
        pt1 (list or np.array): A list or array containing the coordinates of the first point (x,y,z).
        pt2 (list or np.array): A list or array containing the coordinates of the second point (x,y,z).

    Returns:
        float: The euclidean distance between the two points.
    """
    if np.array(pt1, dtype=float).ndim == 1:
        pt1 = np.array(pt1[0:3], dtype=float)
        pt2 = np.array(pt2[0:3], dtype=float)
        return np.linalg.norm(pt2 - pt1, ord=2)
    elif np.array(pt1, dtype=float).ndim == 2 and np.array(pt2, dtype=float).ndim == 1:
        pt1 = np.array(pt1, dtype=float)
        pt2 = np.array(pt2, dtype=float)
        return np.sqrt(np.power(pt2[0] - pt1[:, 0], 2) + np.power(pt2[1] - pt1[:, 1], 2))
    else:
        print("unsupported dim in  distance(pt1, pt2)")



def sigmoid(x, z=0, k=1):
    """
    sigmoid 函数 (Sigmoid function)

    Args:
        x (float): The input value to the sigmoid function.
        z (float): The midpoint of the sigmoid function. Default is 0.
        k (float): The steepness of the sigmoid function. Default is 1.

    Returns:
        float: The output of the sigmoid function given the input value x.
    """
    return 1 / (1 + np.exp((-k) * (x - z)))

def curved_line(x0, y0, x1, y1, miner_dist, x_center=0, y_center=0, pointn=20, mode=0, eps=0.2):
    """
    Returns a list of points representing an arc between two given points.

    Args:
        x0 (int): The x-coordinate of the first point.
        y0 (int): The y-coordinate of the first point.
        x1 (int): The x-coordinate of the second point.
        y1 (int): The y-coordinate of the second point.
        miner_dist ([type]): The minimum distance between the two points.
        x_center (int, optional): The x-coordinate of the center point around which to draw the arc. Defaults to 0.
        y_center (int, optional): The y-coordinate of the center point around which to draw the arc. Defaults to 0.
        pointn (int, optional): The number of points to return. Defaults to 20.
        mode (int, optional): The type of arc to draw. 0 curves towards the center, 1 curves in a random direction. Defaults to 0.
        eps (float, optional): The magnitude of the curve. Higher values result in more pronounced curvature. Defaults to 0.2.

    Returns:
        segments (list): A list of points along the arc.
    """    
    x2 = 0
    y2 = 0

    # Determine the position of the control point based on the specified mode
    if mode == 0:
        pt2_dist = distance((x1, y1), (x0, y0))     # Calculate the Euclidean distance between the two points
        if pt2_dist < 1.5 * miner_dist:
            x2 = 0.5 * (x1 + x0)
            y2 = 0.5 * (y1 + y0)
        elif pt2_dist < 2.5 * miner_dist:
            x2 = 0.35 * (x1 + x0)
            y2 = 0.35 * (y1 + y0)
        elif pt2_dist < 3.5 * miner_dist:
            x2 = 0.30 * (x1 + x0)
            y2 = 0.30 * (y1 + y0)
        else:
            x2 = 0.25 * (x1 + x0)
            y2 = 0.25 * (y1 + y0)
    elif mode == 1:
        x2 = (x0+x1)/2.0 + 0.1 ** (eps+abs(x0-x1)) * (-1) ** (random.randint(1,4))
        y2 = (y0+y1)/2.0 + 0.1 ** (eps+abs(y0-y1)) * (-1) ** (random.randint(1,4))

    # Initialize a Bezier curve using the control point and the given endpoints
    nodes = np.asfortranarray([
        [x0, x2, x1],
        [y0, y2, y1]
    ])
    curve = bezier.Curve(nodes,
                         degree=2)

    # Generate the specified number of points along the curve using the given values for t
    s_vals = np.linspace(0.0, 1.0, pointn)
    data=curve.evaluate_multi(s_vals)
    x=data[0]
    y=data[1]

    # Convert the list of x- and y-coordinates to a list of segments
    segments =[]
    for index in range(0,len(x)):
        segments.append([x[index],y[index]])
    segments = [segments]

    return  segments


def curved_graph(_graph, ax, pos=None, linewidth=1, color='k', mode=0, eps=0.2, vmax=.0, vmin=1.0, cmap='winter'):
    """
    This function adds curved edges to a NetworkX graph visualization using Matplotlib.

    Args:
        _graph: A NetworkX graph object.
        ax: A variable representing the axis obtained through `gca()` of Matplotlib.
        pos (optional): Optional parameter for node coordinates. Default is None.
        linewidth (optional): Optional parameter for line width. Default is 1.
        color (optional): Optional parameter for line color. Default is 'k' (black).
        mode (optional): Optional parameter for the curvature mode. Default is 0 (center-bend) and 1 for random direction bend.
        eps (optional): Optional parameter for line curve intensity. Default is 0.2.
        vmax (optional): Optional parameter for maximum value in the normalization feature of line color. Default is .0.
        vmin (optional): Optional parameter for minimum value in the normalization feature of line color. Default is 1.0.
        cmap (optional): Optional parameter for choosing a specific colour map if `color` is a list. Default is `'winter'`.

    If node coordinates are not provided in `pos`, it generates a spring layout of `_graph`.
    It then calculates the minimum distance between two nodes in the layout using the `distance()` function.
    For each edge in the graph, it generates a curved line between the positions of its source and target nodes
    with the specified mode, eps and miner_dist.

    The line's color depends on whether `color` is a string or a list.
    If it is a string, it sets the line color to the value of `color`.
    If it is a list, it uses Matplotlib's color map feature to generate a color map from the given color list.
    The color map is then normalized using the `vmin` and `vmax` parameters.

    Finally, the LineCollection of each segment is added to the axis using `add_collection()`
    and then returned.
    """
    
    if pos == None:
        # Generate spring layout if node positions are not provided
        pos = nx.spring_layout(_graph)
    
    # Calculate minimum distance between two nodes in the layout
    miner_dist = distance(pos[0], pos[1])

    count = 0
    for u,v in _graph.edges():
        # Get position coordinates of source and target nodes
        x0, y0 = pos[u]
        x1, y1 = pos[v]
        # Generate curved line segments between source and target nodes
        segs = curved_line(x0,y0,x1,y1,miner_dist, mode=mode, eps=eps)
        # Set line color based on whether it's a string or list
        if type(color) == str:
            lc = LineCollection(segs, color=color, linewidth=linewidth)
        elif type(color) == list:
            # Generate colormap from given color list and normalize it
            color_map = plt.get_cmap(cmap)
            norm = plt.Normalize(vmin=vmin, vmax=vmax)
            # Set the LineCollection color to a value retrieved from the normalized colormap
            lc = LineCollection(segs, color=color_map(norm(color[count])), linewidth=linewidth)
        # Add LineCollection segment to Matplotlib axis object
        ax.add_collection(lc)
        count += 1



def mkdir(dir):
    """
    Create a directory with the given name if it does not already exist.

    Args:
        dir (str): The name of the directory to create.

    Returns:
        bool: True if a new directory was created, False otherwise.

    """
    if os.path.exists(dir):
        return False
    else:
        os.makedirs(dir)
        return True

def two_dim_to_three_dim(pos):
    """
    Converts a 2D position to a 3D position by appending a 0 to the z-coordinate.

    Args:
        pos (numpy.ndarray): A 2D position represented as a numpy array with shape (2,).

    Returns:
        numpy.ndarray: A 3D position represented as a numpy array with shape (3,).

    """
    if len(pos) < 3:
        pos_tmp = np.array([0.0, 0.0, 0.0])
        pos_tmp[0:2] = pos[0:2]
        pos = pos_tmp
    return pos 

def unitVector(vector:np.ndarray):
    """
    Takes a numpy array representing a vector and returns a normalized (unit) version of the vector.

    Args:
        vector (numpy.ndarray): A numpy array representing a vector.

    Returns:
        numpy.ndarray: A numpy array representing the normalized version of the input vector.

    """
    vector_norm = np.linalg.norm(vector)
    if vector_norm:
        return vector/vector_norm
    else:
        return vector

def getNormalPoint(x_range=(-1, 1), y_range=(-1, 1), z_range=(-1, 1), dim=3, loc=(.0, .0, .0), scale=0.5):
    """
    Generates a random point with normally distributed coordinates in the specified ranges.

    Args:
        x_range (tuple): A tuple specifying the minimum and maximum values for the x-coordinate.
        y_range (tuple): A tuple specifying the minimum and maximum values for the y-coordinate.
        z_range (tuple): A tuple specifying the minimum and maximum values for the z-coordinate.
        dim (int): An integer indicating the number of dimensions of the generated point.
        loc (tuple): A tuple specifying the mean value for each coordinate.
        scale (float): A float specifying the standard deviation for each coordinate.

    Returns:
        tuple: A tuple representing a randomly generated point with normally distributed coordinates within the specified ranges,
        or None if no such point can be generated within 100 attempts.
    """
    x = None
    y = None 
    z = None
    if dim == 1:
        for _ in range(100):
            tmp = np.random.normal(loc=loc[0], scale=scale, size=None)
            if tmp > x_range[0] and tmp < x_range[1]:
                x = tmp 
                break 
        return x

    elif dim == 2:
        for _ in range(100):
            tmp = np.random.normal(loc=loc[0], scale=scale, size=None)
            if tmp > x_range[0] and tmp < x_range[1]:
                x = tmp 
                break 

        for _ in range(100):
            tmp = np.random.normal(loc=loc[1], scale=scale, size=None)
            if tmp > y_range[0] and tmp < y_range[1]:
                y = tmp 
                break 

        if x is None or y is None:
            return None 
        else:
            return (x, y)
    elif dim == 3:
        for _ in range(100):
            tmp = np.random.normal(loc=loc[0], scale=scale, size=None)
            if tmp > x_range[0] and tmp < x_range[1]:
                x = tmp 
                break 

        for _ in range(100):
            tmp = np.random.normal(loc=loc[1], scale=scale, size=None)
            if tmp > y_range[0] and tmp < y_range[1]:
                y = tmp 
                break 
        
        for _ in range(100):
            tmp = np.random.normal(loc=loc[2], scale=scale, size=None)
            if tmp > z_range[0] and tmp < z_range[1]:
                z = tmp 
                break 
        if x is None or y is None or z is None:
            return None 
        else:
            return (x, y, z)





def getCircleCoordinate(degree, r=5, center=(0, 0)):
    '''
    Calculates the coordinates of a point on a circle.

    Args:
        degree (float): The angle in degrees of the desired point on the circle. 
            Measured clockwise from the positive x-axis.
        r (float): The radius of the circle.
        center (tuple): The (x, y) coordinates of the center of the circle.

    Returns:
        tuple: The (x, y) coordinates of the point on the circle at the given angle.
    '''
    return center[0] + r * math.cos(degree * math.pi/180), center[1] + r * math.sin(degree * math.pi/180)


def angle_with_x_axis(v):
    """
    This is a Python function that calculates the angle between a 2D vector and the positive x-axis. 
    The 2D vector is represented as a tuple or list containing two numbers.
    
    Parameters:
    v -- 2D vector represented as a tuple or list containing two numbers
    
    Returns:
    The angle between the vector and the positive x-axis in radians
    """
    
    # Extract the components of the vector
    x, y = v
    
    # Return the angle between the vector and the positive x-axis in radians using atan2(y,x) function from math library
    return math.atan2(y, x)


def getMeshGridMat(x_start, x_end, y_start, y_end, z_start=None, z_end=None, x_step=1,y_step=1,z_step=1):
    """This function returns a meshgrid of coordinates for 2D or 3D plots.

    Args:
        x_start (float): Starting x-coordinate value.
        x_end (float): Ending x-coordinate value.
        y_start (float): Starting y-coordinate value.
        y_end (float): Ending y-coordinate value.
        z_start (float, optional): Starting z-coordinate value. Defaults to None.
        z_end (float, optional): Ending z-coordinate value. Defaults to None.
        x_step (float, optional): Step size in the x direction. Defaults to 1.
        y_step (float, optional): Step size in the y direction. Defaults to 1.
        z_step (float, optional): Step size in the z direction. Defaults to 1.

    Returns:
        tuple: Returns a tuple of arrays (x, y) if two dimensional and (x, y, z) if three dimensional
    """
    
    # Create a 1D array of evenly spaced values between start and end with a step value of x_step
    x = np.arange(x_start, x_end, x_step)
    
    # Create a 1D array of evenly spaced values between start and end with a step value of y_step
    y = np.arange(y_start, y_end, y_step)
    
    # If z_end and z_start is not specified, only the 2D mesh grid is needed
    if z_start is None or z_end is None:
        return np.meshgrid(x, y) # Return a tuple of coordinate matrices (x, y)
    else:
        # Create a 1D array of evenly spaced values between z_start and z_end with a step value of z_step
        z = np.arange(z_start, z_end, z_step)
        
        return np.meshgrid(x,y,z) # Return a tuple of coordinate matrices (x, y, z)

# Define a function to extract coordinates of thinned line points from an image
def getLinePointFromImage(image_dir:str, resize=None):
    """
    This function will thin out black lines in the image and extract normalized point coordinates.
    
    Args:
        image_dir (str): The directory of the input image.
        resize (tuple, optional): A tuple with normalized range to expand (x1,x2,y1,y2). Defaults to None.
        
    Returns:
        A tuple containing lists of x and y coordinates of extracted line points.
    """  
    # Read the input image and convert to grayscale
    image = cv2.imread(image_dir, cv2.IMREAD_GRAYSCALE)
    # Thinning-out the black lines in the image using skeletonization technique
    skeleton_image = morphology.skeletonize(~image.astype(bool))
    # Extract the point coordinates from the thinned image 
    pts = np.argwhere(skeleton_image)
    # Normalize the coordinates and resize if required
    y, x = pts[:,0], pts[:,1]
    y = y / np.max(y)
    x = x / np.max(x)
    if resize is not None:
        diff_resize_x = resize[1] - resize[0]
        diff_resize_y = resize[3] - resize[2]
        x = x * diff_resize_x + resize[0]
        y = y * diff_resize_y + resize[2]
    # Return the extracted list of x and y coordinates 
    return x.tolist(), y.tolist()


def getCircleCoordinate(degree, r=5, center=(0, 0)):
    '''
    A function that calculates the coordinates of a point on a circle given the degree of rotation.

    Arguments:
        degree (float): The angle of rotation in degrees. The angle is measured from the positive x-axis and increases clockwise.
        r (float, optional): The radius of the circle. Defaults to 5 if not specified.
        center (tuple, optional): The coordinates of the center of the circle. Defaults to (0, 0) if not specified.

    Returns:
        tuple: A tuple containing the x and y coordinates of the point on the circle.
    '''

    # Calculate the x and y coordinates of the point on the circle using the given formulae
    x = center[0] + r * math.cos(degree * math.pi/180)
    y = center[1] + r * math.sin(degree * math.pi/180)

    # Return the tuple of coordinates
    return x, y



def how_much_time(func):
    """
    A decorator function that calculates and prints the execution time of a given function.

    Args:
        func (function): The function to be decorated.

    Returns:
        inner (function): The decorated function.
    """

    # Define a new function that calculates the execution time of the input function 'func'.
    def inner(*args, **kwargs):

        # Get the current time before calling the input function.
        t_start = time.time()

        # Call the input function with all the given arguments.
        result = func(*args, **kwargs)

        # Get the current time after the input function has executed.
        t_end = time.time()

        # Calculate and print the time taken by the input function.
        print("The function '{0}' took {1:.6f} seconds to execute".format(func.__name__, t_end - t_start))

        # Return the result returned by the input function.
        return result

    # Return the new function that calculates execution time.
    return inner



if __name__ == "__main__":
    for i in range(100):
        print(i)
        ret = getNormalPoint((-1000, 1000), (-1000, 1000), (-1000, 1000), dim=3, loc=(0, 0, 0), scale=500)
        print(ret)