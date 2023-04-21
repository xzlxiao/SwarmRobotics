isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Common.DrKDtree import KDtree
from Common import utils


object_collection = []
obstacle_kdtree = None 

def updateObstacle_kdtree(ObstacleTypeList:list):
    """
    Updates the global variable 'obstacle_kdtree' to a new kd-tree generated from positions associated with obstacles listed in ObstacleTypeList.

    Args:
        ObstacleTypeList (list): A list of obstacle types (strings) specifying which obstacles to include in the kd-tree.
    """    
    global obstacle_kdtree   # Declare that we're referencing the global variable 'obstacle_kdtree'.

    if len(ObstacleTypeList) > 0:   # If there are items in our 'ObstacleTypeList'...
        pos_list = []    # Create an empty list to store positions of those obstacles.
        for obstacle_name in ObstacleTypeList:   # For each obstacle type in the list...
            obs_list = getPosByType(obstacle_name)   # Get a list of positions associated with that obstacle type.
            pos_list.extend(obs_list)   # Append those positions to our list of obstacle positions.
        if len(pos_list) > 0:   # If the list of obstacles is non-empty...
            pos_list = np.array(pos_list)   # Convert the position list to a numpy array for use in the KDTree constructor.
            obstacle_kdtree = KDtree(pos_list)  # Construct a new KDTree from the obstacle positions and save it to our global variable.


def getNearestObstacle(pos):
    """
    Returns the position of the nearest obstacle to the specified position, as stored in the global variable 'obstacle_kdtree'.

    Args:
        pos (numpy.ndarray): A 3-element numpy array representing the position we want to find the nearest obstacle to.

    Returns:
        numpy.ndarray or None: Returns a 3-element numpy array representing the position of the nearest obstacle to 'pos', or None if there is no obstacle in the global KDTree.
    """ 
    global obstacle_kdtree   # Declare that we're referencing the global variable 'obstacle_kdtree'.
    if obstacle_kdtree is not None:   # If the KDTree is not empty...
        _, inds = obstacle_kdtree.query(pos, 1)   # Find the index of the point in the KDTree nearest to our specified position.
        return obstacle_kdtree.getPoints(inds)   # Return the position of the obstacle corresponding to that index.
    else:   # If the KDTree is empty...
        return None   # Return None since there is no obstacle to return.


def ObjectAppend(obj):
    """
    Appends the specified object to the global 'object_collection' list.

    Args:
        obj : An object to be appended to the 'object_collection' list.
    """
    object_collection.append(obj)

def clearObject():
    """
    Clears the global 'object_collection' list, removing all objects from it.
    """
    object_collection.clear()

def getAllPos():
    """
    Gets a list of positions of all objects in the global 'object_collection' list.

    Returns:
        List[np.ndarray]: A list of numpy arrays representing the positions of all objects in 'object_collection'.
    """
    return [obj.mPos for obj in object_collection]

def getObjectInRange(pos: np.ndarray, r: float):
    """
    Finds all objects in the global 'object_collection' list within a certain range of a specified position.

    Args:
        pos (np.ndarray): A numpy array representing the position we want to find objects around.
        r (float): The maximum radial distance from 'pos' at which to consider objects.

    Returns:
        List[object]: A list of objects in 'object_collection' that are within 'r' units of 'pos'.
    """
    kd_tree = KDtree(getAllPos())   # Create a KDTree from the positions of all objects in 'object_collection'.
    inds, _ = kd_tree.query_radius(pos, r)   # Use the KDTree to find the indices of all points within radius 'r' of 'pos'.
    inds = inds[0]   # We only care about the first element of these arrays since we're only querying one point.
    return [object_collection[ind] for ind in inds]   # Return the objects corresponding to the indices found above.

def getObjectCountMatInRangeByType(type_name: str, pos_group: np.ndarray, r: float):
    """
    Finds the number of objects of a certain type within a certain range of each position in a specified group.

    Args:
        type_name (str): A string representing the type of objects we want to count.
        pos_group (np.ndarray): A numpy array where every row represents a position we want to check around.
        r (float): The maximum radial distance from each position at which to consider objects.

    Returns:
        Tuple[np.ndarray, np.ndarray]: A tuple containing two numpy arrays - the first contains the number of objects of 'type_name' within radius 'r' of each position in 'pos_group', and the second is an array of indices corresponding to the positions in 'pos_group'.
    """
    kd_tree = KDtree(getPosByType(type_name))   # Create a KDTree from the positions of all objects of 'type_name'.
    return kd_tree.query_radius(pos_group, r)   # Use the KDTree to count the number of objects within 'r' units of each point in 'pos_group'.

def getPosByType(type_name: str):
    """
    Gets a list of positions of all objects in 'object_collection' that have a certain type.

    Args:
        type_name (str): A string representing the type of objects we want to find.

    Returns:
        List[np.ndarray]: A list of numpy arrays representing the positions of all objects in 'object_collection' that have type 'type_name'.
    """
    return [obj.pos.tolist() for obj in object_collection if obj.mObjectType == type_name]

def getObjectByType(type_name: str):
    """
    Gets a list of all objects in 'object_collection' that have a certain type.

    Args:
        type_name (str): A string representing the type of objects we want to find.

    Returns:
        List[object]: A list of objects in 'object_collection' that have type 'type_name'.
    """
    return [obj for obj in object_collection if obj.mObjectType == type_name]
