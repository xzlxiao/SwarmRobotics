import numpy as np
import Common.utils as myUtils
import math
import time 
import copy 
from collections import deque
from Simulation.ComPathPlanning import ComPathPlanning

import sys 
print(sys.path)
sys.path.append('D:\桌面\Epuck\GRN')



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
    # pmap, minx, miny, minz = calc_potential_field(gx, gy, gz, ox, oy, oz, rr, reso, map_size)
    # search path
    minx, miny, minz, maxx, maxy, maxz = map_size
    d = np.sqrt((sx - gx)**2+(sy - gy)**2+(sz - gz)**2)

    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    iz = round((sz - minz) / reso)

    xw = int(round((maxx - minx) / reso))#2000
    yw = int(round((maxy - miny) / reso))
    zw = int(round((maxz - minz) / reso))

    pmap = [ [ [0.0 for i in range(zw)] for i in range(yw)] for i in range(xw)]#2000*2000*2000
    
    rx, ry, rz = [sx], [sy], [sz]
    motion_step = 1#2
    motion = np.array(get_motion_model()) * motion_step
    motion = motion.tolist()
    previous_ids = deque()
    while d >= reso:
        minp = float("inf")
        minix, miniy, miniz = -1, -1, -1
        mini = 0
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            inz = int(iz + motion[i][2])

            x = inx * reso + minx#real position of next move
            y = iny * reso + miny
            z = inz * reso + minz
            if inx >= len(pmap) or iny >= len(pmap[0]) or inz >= len(pmap[0][0]) or inx < 0 or iny < 0 or inz < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                if gx is not None or gy is not None:
                    ug = calc_attractive_potential(x, y, z, gx, gy, gz)
                    uo = calc_repulsive_potential(x, y, z, ox, oy, oz, rr)
                    uf = ug + uo
                else:
                    uo = calc_repulsive_potential(x, y, z, ox, oy, oz, rr)
                    uf = uo

                pmap[inx][iny][inz] = uf
                p = pmap[inx][iny][inz]

                # print('Potential', (motion[i][0], motion[i][1], motion[i][2]), ': ',  p, 'ug: ', ug, 'uo: ', uo)
            if minp > p:
                minp = p
                mini = i#print decision movement direction
                minix = inx
                miniy = iny
                miniz = inz
        ix = minix
        iy = miniy
        iz = miniz
        xp = ix * reso + minx
        yp = iy * reso + miny
        zp = iz * reso + minz
        # print(zp)
        # print(ix, iy, xp, yp)
        d = np.sqrt((gx - xp)**2+(gy - yp)**2+(gz - zp)**2)
        # print(d, xp, yp, zp)
        rx.append(xp)
        ry.append(yp)
        rz.append(zp)
        # print('X_path: ', rx)
        # print('Z_path: ', rz)
        # print('Minimum Potential', (motion[mini][0], motion[mini][1], motion[mini][2]), ': ', minp, '\n\n')
        # if (oscillations_detection(previous_ids, ix, iy, iz, minx, miny, minz, reso)):
        if (oscillations_detection(previous_ids, xp, yp, zp)):
            # print("Oscillation detected at ({},{},{})!".format(ix * reso + minx, iy * reso + miny, iz * reso + minz))
            print("Oscillation detected at ({},{},{})!".format(xp, yp, zp))
            break
    return rx, ry, rz


def calc_potential_field_mat(gx, gy, gz, ox, oy, oz, rr, reso=1, map_size=(-1000, -1000, -1000, 1000, 1000, 1000)):
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
    minx, miny, minz, maxx, maxy, maxz = map_size
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))
    zw = int(round((maxz - minz) / reso))

    # calc each potential
    # pmap = [[[0.0 for _ in range(zw)] for _ in range(yw)] for _ in range(xw)]
    pmap = np.zeros((yw, xw, zw), dtype=float)

    x_mat, y_mat, z_mat = np.meshgrid(np.arange(xw), np.arange(yw), np.arange(zw))
    x_mat = x_mat * reso + minx
    y_mat = y_mat * reso + miny
    z_mat = z_mat * reso + minz

    
    if gx is not None or gy is not None:
        ug = calc_attractive_potential_mat(x_mat, y_mat, z_mat, gx, gy, gz)
        uo = calc_repulsive_potential_mat(x_mat, y_mat, z_mat, ox, oy, oz, rr)
        uf = ug + uo
    else:
        uo = calc_repulsive_potential_mat(x_mat, y_mat, z_mat, ox, oy, oz, rr)
        uf = uo
    pmap = uf

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

    return pmap, minx, miny, minz


def calc_attractive_potential(x, y, z, gx, gy, gz):
    return 0.5 * KP * math.sqrt((x - gx)**2+(y - gy)**2+(z - gz)**2)

def calc_attractive_potential_mat(x, y, z, gx, gy, gz):
    return 0.5 * KP_MAT * np.sqrt((x - gx)**2+(y - gy)**2+(z - gz)**2)
    # return 0

def calc_repulsive_potential(x, y, z, ox, oy, oz, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = math.sqrt((x - ox[i])**2+(y - oy[i])**2+(z - oz[i])**2)
        if dmin >= d:
            dmin = d
            minid = i   

    dq = math.sqrt((x - ox[minid])**2+(y - oy[minid])**2+(z - oz[minid])**2)
    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1
            
        return 3 * ETA * (1.0 / dq - 1.0 / rr) ** 1
    else:
        return 0.0 

def calc_repulsive_potential_mat(x, y, z, ox, oy, oz, rr):
    # search nearest obstacle
    minid = np.zeros_like(x, dtype=int)
    minid[:] = -1
    dmin = np.zeros_like(x, dtype=float)
    dmin[:] = float("inf")
    for i, _ in enumerate(ox):
        d = np.sqrt((x - ox[i])**2+(y - oy[i])**2+(z - oz[i])**2)
        minid[dmin>d] = i
        dmin[dmin>d] = d[dmin>d]
        
    # print(dmin)
    # print(minid.shape)

    # calc repulsive potential
    ox_min_mat = np.array(ox)[minid]
    oy_min_mat = np.array(oy)[minid]
    oz_min_mat = np.array(oz)[minid]

    dq = np.sqrt((x - ox_min_mat)**2+(y - oy_min_mat)**2+(z - oz_min_mat)**2)
    ret = np.zeros_like(dq, dtype=float)
    dq[dq<=1] = 4#0.1

    # print(dq[100,100,100])
    ret[dq<=rr] = 1 * ETA_MAT * (1.0 / dq[dq<=rr] - 1.0 / rr) ** 0.3#3 0.1
    # print(ret[95:105,95:105,100])
    return ret


def get_motion_model():
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
    previous_ids.append(( xp, yp, zp))
    path.append(( xp, yp, zp))
    # print('Path: ', path, '\n\n')

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            print('Path: ', path, '\n\n')
            return True
        else:
            previous_ids_set.add(index)
    return False

class ComPathPlanning3D(ComPathPlanning):
    def __init__(self) -> None:
        self.mTarget = None
        self.mPos = None 
        self.mObstacleList = None
        self.mRobotRadius =  20 
        self.mPathPtList_x = None 
        self.mPathPtList_y = None 
        self.mPathPtList_z = None 
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
        obstacle_pos_z_list = [i[2] for i in obstacle_pos_group]
        gx, gy, gz = self.mTarget[0:3]
        ox = obstacle_pos_x_list
        oy = obstacle_pos_y_list
        oz = obstacle_pos_z_list
        reso = 20
        rr = self.mRobotRadius
        sx, sy, sz = self.mPos[0:3]
        self.mPathPtList_x, self.mPathPtList_y, self.mPathPtList_z = potential_field_planning(
            sx, 
            sy, 
            sz,
            gx, 
            gy,
            gz,
            ox, 
            oy, 
            oz,
            rr, 
            reso)
        
    def getNextDest(self):
        '''
        获取下一个目的地
        x, y, z, angle
        '''
        pt_num = 2
        if len(self.mPathPtList_x) > 3:
            x1, x2 = self.mPathPtList_x[pt_num:pt_num+2]
            y1, y2 = self.mPathPtList_y[pt_num:pt_num+2]

            v = (x2-x1, y2-y1)
            angle = myUtils.angle_with_x_axis(v)
            return self.mPathPtList_x[pt_num], self.mPathPtList_y[pt_num], self.mPathPtList_z[pt_num], angle
        else:
            return self.mTarget[0], self.mTarget[1], self.mTarget[2], None