import numpy as np
import Common.utils as myUtils
import math
import time 
import copy 
from collections import deque

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 30.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3

def potential_field_planning(sx, sy, gx, gy, ox, oy, rr, reso=1, map_size=(-1000, -1000, 1000, 1000)):
    '''
    gx: goal x position [mm]
    gy: goal y position [mm]
    ox: obstacle x position list [mm]
    oy: obstacle y position list [mm]
    reso: 每单位网格的尺寸，默认为1就可以 [mm]
    rr: robot radius [mm]
    sx:  start x position [mm]
    sy: start y positon [mm]
    '''
    # calc potential field
    # pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, rr, reso, map_size)
    pmap, minx, miny = calc_potential_field2(gx, gy, ox, oy, rr, reso, map_size)
    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()
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
        # print(ix, iy, xp, yp)
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break
    return rx, ry

def calc_potential_field(gx, gy, ox, oy, rr, reso=1, map_size=(-1000, -1000, 1000, 1000)):
    '''
    gx: goal x position [mm]
    gy: goal y position [mm]
    ox: obstacle x position list [mm]
    oy: obstacle y position list [mm]
    reso: 每单位网格的尺寸，默认为1就可以 [mm]
    rr: robot radius [mm]
    sx:  start x position [mm]
    sy: start y positon [mm]
    map_size: 地图尺寸 [mm]
    '''
    minx, miny, maxx, maxy = map_size
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]


    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            if gx is not None or gy is not None:
                ug = calc_attractive_potential(x, y, gx, gy)
                uo = calc_repulsive_potential(x, y, ox, oy, rr)
                uf = ug + uo
            else:
                uo = calc_repulsive_potential(x, y, ox, oy, rr)
                uf = uo
            pmap[ix][iy] = uf

    return pmap, minx, miny



def calc_potential_field2(gx, gy, ox, oy, rr, reso=1, map_size=(-1000, -1000, 1000, 1000)):
    '''
    gx: goal x position [mm]
    gy: goal y position [mm]
    ox: obstacle x position list [mm]
    oy: obstacle y position list [mm]
    reso: 每单位网格的尺寸，默认为1就可以 [mm]
    rr: robot radius [mm]
    sx:  start x position [mm]
    sy: start y positon [mm]
    map_size: 地图尺寸 [mm]
    '''
    minx, miny, maxx, maxy = map_size
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]


    pmap = np.array(pmap)
    x_mat = [[i for i in range(xw)]]
    x_mat_tmp = copy.copy(x_mat)
    y_mat = [[i] for i in range(yw)]
    y_mat_tmp = copy.copy(y_mat)
    for _ in range(yw-1):
        x_mat = np.concatenate((x_mat, x_mat_tmp), axis=0)
    for _ in range(xw-1):
        y_mat = np.concatenate((y_mat, y_mat_tmp), axis=1)
    x_mat = x_mat * reso + minx
    y_mat = y_mat * reso + miny
    if gx is not None or gy is not None:
        ug = calc_attractive_potential(x_mat, y_mat, gx, gy)
        uo = calc_repulsive_potential2(x_mat, y_mat, ox, oy, rr)
        uf = ug + uo
    else:
        uo = calc_repulsive_potential2(x_mat, y_mat, ox, oy, rr)
        uf = uo
    pmap = uf.T.tolist()

    # for ix in range(xw):
    #     x = ix * reso + minx

    #     for iy in range(yw):
    #         y = iy * reso + miny
    #         if gx is not None or gy is not None:
    #             ug = calc_attractive_potential(x, y, gx, gy)
    #             uo = calc_repulsive_potential(x, y, ox, oy, rr)
    #             uf = ug + uo
    #         else:
    #             uo = calc_repulsive_potential(x, y, ox, oy, rr)
    #             uf = uo
    #         pmap[ix][iy] = uf

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i
    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        # print(dq)
        if dq <= 0.1:
            dq = 0.1
        ret = 3 * ETA * (1.0 / dq - 1.0 / rr) ** 0.3
        return ret
    else:
        return 0.0


def calc_repulsive_potential2(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = np.zeros_like(x, dtype=int)
    minid[:] = -1
    dmin = np.zeros_like(x, dtype=float)
    dmin[:] = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        minid[dmin>d] = i
        dmin[dmin>d] = d[dmin>d]
        
    # print(dmin)
    # print(minid.shape)

    # calc repulsive potential
    ox_min_mat = np.array(ox)[minid]
    oy_min_mat = np.array(oy)[minid]
    dq = np.hypot(x - ox_min_mat, y - oy_min_mat)
    ret = np.zeros_like(dq, dtype=float)
    dq[dq<=0.1] = 0.1
    ret[dq<=rr] = 3 * ETA * (1.0 / dq[dq<=rr] - 1.0 / rr) ** 0.3
    return ret



def get_motion_model():
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
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False

class ComPathPlanning:
    def __init__(self) -> None:
        self.mTarget = None
        self.mPos = None 
        self.mObstacleList = None
        self.mRobotRadius = 20  
        self.mPathPtList_x = None 
        self.mPathPtList_y = None 
        self.mEnvSize = None
    
    def setPos(self, pos):
        self.mPos = pos 
        
    def setEnvSize(self, _size):
        self.mEnvSize = _size

    def setTarget(self, target: tuple):
        self.mTarget = target

    def setRobotRadius(self, radius):
        self.mRobotRadius = radius

    def setObstacleList(self, obstacle_list: list):
        self.mObstacleList = obstacle_list

    def update(self):
        obstacle_pos_group = [obstacle.mPos for obstacle in self.mObstacleList]
        obstacle_pos_x_list = [i[0] for i in obstacle_pos_group]
        obstacle_pos_y_list = [i[1] for i in obstacle_pos_group]
        gx, gy = self.mTarget[0:2]
        ox = obstacle_pos_x_list
        oy = obstacle_pos_y_list
        reso = 10
        rr = self.mRobotRadius
        sx, sy = self.mPos[0:2]
        self.mPathPtList_x, self.mPathPtList_y = potential_field_planning(
            sx, 
            sy, 
            gx, 
            gy,
            ox, 
            oy, 
            rr, 
            reso)
        
    def getNextDest(self):
        '''
        获取下一个目的地
        x, y, angle
        '''
        pt_num = 2
        if len(self.mPathPtList_x) > 3:
            x1, x2 = self.mPathPtList_x[pt_num:pt_num+2]
            y1, y2 = self.mPathPtList_y[pt_num:pt_num+2]
            v = (x2-x1, y2-y1)
            angle = myUtils.angle_with_x_axis(v)
            return self.mPathPtList_x[pt_num], self.mPathPtList_y[pt_num], angle
        else:
            return self.mTarget[0], self.mTarget[1], None