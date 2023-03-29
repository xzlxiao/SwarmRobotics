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
    return __colors[num%len(__colors)]

def readPointFromFile(dir: str):
    ret = []
    with open(dir, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip()[0:-1].split(',')
            ret.append((float(line[0]), float(line[1])))
    return ret


def distance(pt1, pt2):
    """
    计算两点间的距离
    :param pt1: list or np.array
    :param pt2: list or np.array
    :return:
    """
    pt1 = np.array(pt1, dtype=np.float32)
    pt2 = np.array(pt2, dtype=np.float32)
    return np.linalg.norm(pt2 - pt1, ord=2)


def calcLost(base_point_list: list, robot_point_list: list):
    """
    计算每个点的偏差
    :param base_point_list: 作为基准的点集
    :param robot_point_list: 需要评估的点集
    :return: 偏差列表
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
    keys_list = []
    for dict_item in dict_list:
        dict_item: dict
        for key_item in dict_item.keys():
            if key_item not in keys_list:
                keys_list.append(key_item)
    return keys_list


def distance(pt1, pt2):
    """
    计算两点间的距离
    :param pt1: list or np.array
    :param pt2: list or np.array
    :return:
    """
    if np.array(pt1, dtype=np.float32).ndim == 1:
        pt1 = np.array(pt1[0:3], dtype=np.float32)
        pt2 = np.array(pt2[0:3], dtype=np.float32)
        return np.linalg.norm(pt2 - pt1, ord=2)
    elif np.array(pt1, dtype=np.float32).ndim == 2 and np.array(pt2, dtype=np.float32).ndim == 1:
        pt1 = np.array(pt1, dtype=np.float32)
        pt2 = np.array(pt2, dtype=np.float32)
        return np.sqrt(np.power(pt2[0] - pt1[:, 0], 2) + np.power(pt2[1] - pt1[:, 1], 2))
    else:
        print("unsupported dim in  distance(pt1, pt2)")



def sigmoid(x, z=0, k=1):
    """
    sigmoid 函数
    :param x:
    :param z: 中值
    :param k: 陡峭度
    :return:
    """
    return 1 / (1 + np.exp((-k) * (x - z)))

def curved_line(x0, y0, x1, y1, miner_dist, x_center=0, y_center=0, pointn=20, mode=0, eps=0.2):
    """获得点pt0 和pt1之间的弧线
        控制点位置由黄吉实现
    Args:
        x0 (int): pt0
        y0 (int): pt0
        x1 (int): pt1
        y1 (int): pt1
        miner_dist ([type]): 两点最小距离
        x_center (int, optional): 中心点x坐标
        y_center (int, optional): 中心点y坐标
        pointn (int, optional): 点的数量. Defaults to 20.
        mode (int, optional): 弧线模式，默认为0，即向中心弯曲，1：随机方向弯曲
    Returns:
        [list]: 弧线
    """    
    # x2 = (x0+x1)/2.0 + 0.1 ** (eps+abs(x0-x1)) * (-1) ** (random.randint(1,4))
    # y2 = (y0+y1)/2.0 + 0.1 ** (eps+abs(y0-y1)) * (-1) ** (random.randint(1,4))
    x2 = 0
    y2 = 0
    if mode == 0:
        pt2_dist = distance((x1, y1), (x0, y0))     # 两点之间的距离
        if pt2_dist < 1.5 * miner_dist:
            # 如果是最近的两个点之一
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

    nodes = np.asfortranarray([
        [x0, x2, x1],
        [y0, y2, y1]
    ])
    curve = bezier.Curve(nodes,
                         degree=2)
    s_vals = np.linspace(0.0, 1.0, pointn)
    data=curve.evaluate_multi(s_vals)
    x=data[0]
    y=data[1]
    segments =[]
    for index in range(0,len(x)):
        segments.append([x[index],y[index]])
    segments = [segments]
    return  segments

def curved_graph(_graph, ax, pos = None, linewidth=1, color='k', mode=0, eps=0.2, vmax=.0, vmin=1.0, cmap='winter'):
    """
    为networkx位置弧线型的边

    Args:
        _graph ([type]): networkx的图对象
        ax ([type]): matplotlib通过gca()获得的轴的变量
        pos ([type], optional): 节点的坐标位置. Defaults to None.
        linewidth (int, optional): 线宽. Defaults to 1.
        color (str or list, optional): 线的颜色. Defaults to 'k'.
        mode (int, optional): 弧线模式，默认为0，即向中心弯曲，1：随机方向弯曲
    """    
    if pos == None:
        pos = nx.spring_layout(_graph)
    
    miner_dist = distance(pos[0], pos[1])

    count = 0
    for u,v in _graph.edges():
        x0, y0 = pos[u]
        x1, y1 = pos[v]
        segs = curved_line(x0,y0,x1,y1,miner_dist, mode=mode, eps=eps)
        if type(color) == str:
            lc = LineCollection(segs, color=color, linewidth=linewidth)
        elif type(color) == list:
            color_map = plt.get_cmap(cmap)
            norm = plt.Normalize(vmin=vmin, vmax=vmax)
            lc = LineCollection(segs, color=color_map(norm(color[count])), linewidth=linewidth)
        ax.add_collection(lc)
        count += 1


def mkdir(dir):
    if os.path.exists(dir):
        return False
    else:
        os.makedirs(dir)
        return True

def two_dim_to_three_dim(pos):
    if len(pos) < 3:
        pos_tmp = np.array([0.0, 0.0, 0.0])
        pos_tmp[0:2] = pos[0:2]
        pos = pos_tmp
    return pos 

def unitVector(vector:np.ndarray):
    # 求模长
    vector_norm = np.linalg.norm(vector)
    if vector_norm:
        return vector/vector_norm
    else:
        return vector

def getNormalPoint(x_range=(-1, 1), y_range=(-1, 1), z_range=(-1, 1), dim=3, loc=(.0, .0, .0), scale=0.5):
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
    计算圆上的坐标
    degree: 以x轴正向为0度顺时针旋转
    r: 圆的半径
    center: 圆中心点的坐标
    '''
    return center[0] + r * math.cos(degree * math.pi/180), center[1] + r * math.sin(degree * math.pi/180)


def angle_with_x_axis(v):
    """
    计算二维向量与 x 轴正向的夹角的函数
    
    参数：
    v -- 二维向量，以一个包含两个数字的列表或元组表示
    
    返回值：
    与 x 轴正向的夹角，以弧度表示
    """
    x, y = v
    return math.atan2(y, x)

def getMeshGridMat(x_start, x_end, y_start, y_end, z_start=None, z_end=None, x_step=1,y_step=1,z_step=1):
    x = np.arange(x_start, x_end, x_step)
    y = np.arange(y_start, y_end, y_step)
    if z_start is None or z_end is None:
        return np.meshgrid(x, y)
    else:
        z = np.arange(z_start, z_end, z_step)
        return np.meshgrid(x,y,z)
    
def getLinePointFromImage(image_dir:str, resize=None):
    """将图片的的黑色线条细化，并提取点归一化的坐标

    Args:
        image_dir (str): _description_
        resize (_type_, optional): 归一化后扩大的范围，（x1, x2, y1, y2). Defaults to None.
    """    
    image = ImageProc.readImage(image_dir)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    binary_image = ImageProc.binaryFilter(image)
    binary_image = 255 - binary_image
    binary_image = binary_image.astype(np.bool)
    skeleton_image = ImageProc.skeletonize(binary_image)
    pts = ImageProc.getPointFromImage(skeleton_image)
    pts = pts.tolist()
    y = np.array([i[0] for i in pts])
    y = y / np.max(y)
    x = np.array([i[1] for i in pts])
    x = x / np.max(x)
    if resize is not None:
        diff_resize_x = resize[0] - resize[1]
        diff_resize_y = resize[2] - resize[3]
        x = x * diff_resize_x + resize[0]
        y = y * diff_resize_y + resize[2]
    
    return x.tolist(), y.tolist()

def getCircleCoordinate(degree, r=5, center=(0, 0)):
    '''
    计算圆上的坐标
    degree: 以x轴正向为0度顺时针旋转
    r: 圆的半径
    center: 圆中心点的坐标
    '''
    return center[0] + r * math.cos(degree * math.pi/180), center[1] + r * math.sin(degree * math.pi/180)

if __name__ == "__main__":
    for i in range(100):
        print(i)
        ret = getNormalPoint((-1000, 1000), (-1000, 1000), (-1000, 1000), dim=3, loc=(0, 0, 0), scale=500)
        print(ret)