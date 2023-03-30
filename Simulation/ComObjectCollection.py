isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Common.DrKDtree import KDtree


object_collection = []
obstacle_kdtree = None 

def updateObstacle_kdtree(ObstacleTypeList:list):
    global obstacle_kdtree
    if len(ObstacleTypeList) > 0:
        pos_list = []
        for obstacle_name in ObstacleTypeList:
            obs_list = getPosByType(obstacle_name)
            pos_list.extend(obs_list)
        if len(pos_list) > 0:
            pos_list = np.array(pos_list)
            # print(pos_list)
            obstacle_kdtree = KDtree(pos_list)

def getNearestObstacle(pos):
    global obstacle_kdtree
    if obstacle_kdtree is not None:
        _, inds = obstacle_kdtree.query(pos, 1)
        return obstacle_kdtree.getPoints(inds)
    else:
        return None

def ObjectAppend(obj):
    object_collection.append(obj)

def clearObject():
    object_collection.clear()

def getAllPos():
    return [obj.mPos for obj in object_collection]

def getObjectInRange(pos: np.ndarray, r: float):
    kd_tree = KDtree(getAllPos())
    inds, _ = kd_tree.query_radius(pos, r)
    inds = inds[0]
    return [object_collection[ind] for ind in inds]

def getObjectCountMatInRangeByType(type_name: str, pos_group: np.ndarray, r: float):
    kd_tree = KDtree(getPosByType(type_name))
    return kd_tree.query_radius(pos_group, r)

def getPosByType(type_name: str):
    return [obj.pos.tolist() for obj in object_collection if obj.mObjectType == type_name]

def getObjectByType(type_name: str):
    return [obj for obj in object_collection if obj.mObjectType == type_name]