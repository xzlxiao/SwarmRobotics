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

def potential_field_planning2(sx, sy, gx, gy, rr, step_size=2):
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
    # nearest_obs_pos = ComObjectCollection.getNearestObstacle()
    # pmap, minx, miny = calc_potential_field2(gx, gy, ox, oy, rr, reso, map_size)
    
    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = sx
    iy = sy

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()
    while d >= step_size*2:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = ix + motion[i][0] * step_size
            iny = iy + motion[i][1] * step_size
            obs_pos = ComObjectCollection.getNearestObstacle((inx, iny, 0))
            if obs_pos is not None:
                ox, oy, _ = obs_pos[0][0]
            else: 
                ox = oy = None
            # print(a)
            # ox, oy, _ = a[0][0]
            p = calc_potential_field3(inx, iny, gx, gy, ox, oy, rr)
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        # print(ix, iy, xp, yp)
        # d = np.hypot(gx - ix, gy - iy)
        rx.append(ix)
        ry.append(iy)

        if (oscillations_detection(previous_ids, ix, iy)):
            # print("Oscillation detected at ({},{})!".format(ix, iy))
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
            if gx is not None and gy is not None:
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

    return pmap, minx, miny

def calc_potential_field3(x, y, gx, gy, ox, oy, rr):
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
    # print(x, y, gx, gy, ox, oy)
    if gx is not None or gy is not None:
        ug = calc_attractive_potential(x, y, gx, gy)
        uo = calc_repulsive_potential3(x, y, ox, oy, rr)
        uf = ug + uo
        # if uo > 0.01:
        #     print(ug, uo)
    else:
        uo = calc_repulsive_potential3(x, y, ox, oy, rr)
        uf = uo
    p = uf

    return p


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
    ret = np.zeros_like(x, dtype=float)
    if len(ox) ==  0 or len(oy) == 0:
        return ret

    # search nearest obstacle
    minid = np.zeros_like(x, dtype=int)
    minid[:] = -1
    dmin = np.zeros_like(x, dtype=float)
    dmin[:] = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        minid[dmin>d] = i
        dmin[dmin>d] = d[dmin>d]

    # calc repulsive potential
    ox_min_mat = np.array(ox)[minid]
    oy_min_mat = np.array(oy)[minid]
    dq = np.hypot(x - ox_min_mat, y - oy_min_mat)
    
    dq[dq<=0.1] = 0.1
    ret[dq<=rr] = 3 * ETA * (1.0 / dq[dq<=rr] - 1.0 / rr) ** 0.3
    return ret

def calc_repulsive_potential3(x, y, ox, oy, rr):
    """计算斥力场

    Args:
        x (_type_): _description_
        y (_type_): _description_
        ox (_type_): 最近障碍物的x坐标
        oy (_type_): 最近障碍物的y坐标
        rr (_type_): _description_

    Returns:
        _type_: _description_
    """
    if ox is None or oy is None:
        return 0.0
    # calc repulsive potential
    dq = np.hypot(x - ox, y - oy)
    
    if dq <= rr:
        # print(dq)
        if dq <= 0.1:
            dq = 0.1
        ret = 3 * ETA * (1.0 / dq - 1.0 / rr) ** 0.1
        return ret
    else:
        return 0.0

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
        self.mRobotRadius = 20  
        self.mPathPtList_x = None 
        self.mPathPtList_y = None 
        self.mEnvSize = None
        self.mStride = 10   # 设置梯度下降时搜索的步长，较大的值可以更好的跳出局部极值，但路径规划越不精细
        self.isRandomLeapOn = True   # 是否可以通过随机移动跳出局部极值，默认可以
        self.mRandomLeapThreshold = 3  # 跳出极值判断的步长倍数
        self.mTargetBackup = None
        self.mRandomMoveRange = 30      # 随机移动范围的步长倍数
    
    def setPos(self, pos):
        self.mPos = pos 
        
    def setEnvSize(self, _size):
        self.mEnvSize = _size

    def setTarget(self, target: tuple):
        self.mTarget = target

    def setRobotRadius(self, radius):
        self.mRobotRadius = radius

    def setStride(self, stride_len):
        self.mStride = stride_len

    def isRandomMove(self):
        '''
        是否需要通过随机移动跳出局部极值
        '''
        if not self.isRandomLeapOn:
            return False 
        
        x = self.mPathPtList_x[-1]
        y = self.mPathPtList_y[-1]
        if myUtils.distance((x, y), self.mTarget[0:2]) > self.mStride*self.mRandomLeapThreshold and self.mTargetBackup is None:
            return True 
        else:
            return False
    
    def randomMove(self):
        x = 0
        y = 0
        multi_stride = self.mStride * self.mRandomMoveRange
        range_x = [self.mPos[0]-multi_stride, self.mPos[0]+multi_stride]
        range_y = [self.mPos[1]-multi_stride, self.mPos[1]+multi_stride]
        if self.mEnvSize is not None:
            # 边界判断
            minx = -self.mEnvSize[0]
            maxx = self.mEnvSize[0]
            miny = -self.mEnvSize[1]
            maxy = self.mEnvSize[1]
            if range_x[0] < minx:
                range_x[0] = minx 
            if range_x[1] > maxx:
                range_x[1] = maxx 
            if range_y[0] < miny:
                range_y[0] = miny
            if range_y[1] > maxy:
                range_y[1] = maxy
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        self.setTarget((x, y, 0))

    def update(self):
        # obstacle_pos_group = [obstacle.mPos for obstacle in self.mObstacleList]
        # obstacle_pos_x_list = [i[0] for i in obstacle_pos_group]
        # obstacle_pos_y_list = [i[1] for i in obstacle_pos_group]
        gx, gy = self.mTarget[0:2]
        # ox = obstacle_pos_x_list
        # oy = obstacle_pos_y_list
        rr = self.mRobotRadius
        sx, sy = self.mPos[0:2]
        self.mPathPtList_x, self.mPathPtList_y = potential_field_planning2(
            sx, 
            sy, 
            gx, 
            gy,
            rr,
            self.mStride)
        
    def getNextDest(self):
        '''
        获取下一个目的地
        x, y, angle
        '''
        pt_num = 2
        if len(self.mPathPtList_x) > 5:
            x1, x2 = self.mPathPtList_x[pt_num:pt_num+2]
            y1, y2 = self.mPathPtList_y[pt_num:pt_num+2]
            v = (x2-x1, y2-y1)
            angle = myUtils.angle_with_x_axis(v)
            return self.mPathPtList_x[pt_num], self.mPathPtList_y[pt_num], angle
        elif self.isRandomMove():
            if self.mTargetBackup is None:
                self.mTargetBackup = copy.copy(self.mTarget)
            self.randomMove()
            return None, None, None
        elif self.mTargetBackup is not None:
            self.mTarget =  copy.copy(self.mTargetBackup)
            self.mTargetBackup = None
            return None, None, None
        else:
            return self.mTarget[0], self.mTarget[1], None