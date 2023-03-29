# -*- coding: utf-8 -*-

"""
       .==.        .==.
      //`^\\      //^`\\
     // ^ ^\(\__/)/^ ^^\\
    //^ ^^ ^/+  0\ ^^ ^ \\
   //^ ^^ ^/( >< )\^ ^ ^ \\
  // ^^ ^/\| v''v |/\^ ^ ^\\
 // ^^/\/ /  `~~`  \ \/\^ ^\\
 ----------------------------
BE CAREFULL! THERE IS A DRAGON.

功能：图像处理
备注：
案例：

模块：
(c) 肖镇龙 2019
依赖：
pip3 install mkl
pip3 install -U scikit-image
pip3 install --upgrade scikit-image
"""

import numpy as np
import cv2
from skimage import morphology
from Common.DrKDtree import KDtree
from geomdl import NURBS
from geomdl import BSpline
from geomdl import utilities
from geomdl import exchange
from geomdl import operations

def skeletonize(image: np.ndarray):
    return morphology.skeletonize(image)

def binaryFilter(image: np.ndarray):
    """
    二值化
    :param image:
    :return:
    """
    gray = image*255
    gray = gray.astype(np.uint8)
    _, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    return dst

def getPointFromImage(skeleton_image, pForcolor=True):
    """
    从图像中获得线条上所有的点的坐标
    :param pForcolor: 前景的颜色
    :param skeleton_image:
    :return: 点集 list [[x1, y1], [x2, y2] ...]
    """
    return np.argwhere(skeleton_image == pForcolor).astype(np.float)

def getBSplineCurve(Points: np.ndarray):
    """
    通过nurbs模型，用Points点集创建BSpline
    :param Points:
    :return: NURBS.Curve
    """
    point_list = Points.tolist()
    kdtree = KDtree(point_list)
    pt_list = kdtree.sort()
    curve = NURBS.Curve()

    # Set up the curve
    curve.degree = 5
    curve.ctrlpts = pt_list

    # Auto-generate knot vector
    curve.knotvector = utilities.generate_knot_vector(curve.degree, len(curve.ctrlpts))

    # Set evaluation delta
    curve.delta = 0.01

    return curve

def getNurbsValue(curve: NURBS.Curve, value: float):
    """
    根据nurbs曲线，获得值对应的点
    :param curve: nurbs曲线
    :param value: 0~1的数值
    :return: point
    """
    curve.evaluate(start=value, stop=value)
    return [curve.evalpts[0][1], curve.evalpts[0][0]]


def concentration(x: np.ndarray, coefficients, Q, time: float):
    '''
    形态发生素扩散浓度计算
    :param x: 到释放源的距离
    :param coefficients: 扩散系数
    :param Q: 释放的分子两
    :param time: 扩散时间
    :return:
    '''
    return (Q / (2 * np.sqrt(np.pi * coefficients * time))) * np.exp(-np.power(x, 2)/(4 * coefficients * time))

def laplacian(image: np.ndarray):
    image_x = np.zeros_like(image)
    image_y = np.zeros_like(image)
    image_x2 = np.zeros_like(image)
    image_y2 = np.zeros_like(image)
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            if row-1 < 0:
                image_y[row, col] = image[row+1, col]-image[row, col]
            elif row+1 >= image.shape[0]:
                image_y[row, col] = image[row, col]-image[row-1, col]
            else:
                image_y[row, col] = (image[row+1, col]-image[row-1, col])/2
            if col-1 < 0:
                image_x[row, col] = image[row, col+1]-image[row, col]
            elif col+1 >= image.shape[1]:
                image_x[row, col] = image[row, col]-image[row, col-1]
            else:
                image_x[row, col] = (image[row, col+1]-image[row, col-1])/2
    for row in range(image.shape[0]):
        for col in range(image.shape[1]):
            if row-1 < 0:
                image_y2[row, col] = image_y[row+1, col]-image_y[row, col]
            elif row+1 >= image.shape[0]:
                image_y2[row, col] = image_y[row, col]-image_y[row-1, col]
            else:
                image_y2[row, col] = (image_y[row+1, col]-image_y[row-1, col])/2
            if col-1 < 0:
                image_x2[row, col] = image_x[row, col+1]-image_x[row, col]
            elif col+1 >= image.shape[1]:
                image_x2[row, col] = image_x[row, col]-image_x[row, col-1]
            else:
                image_x2[row, col] = (image_x[row, col+1]-image_x[row, col-1])/2
    return image_x2 + image_y2

def laplacian1D(image: np.ndarray):
    '''
    扩散滤波, 扩散公式，C = u + D \nabla^2 u
    :param image:
    :param diffusion_coefficient: 扩散系数
    :param core_size:
    :return:
    '''
    image_x = np.zeros_like(image)
    image_x2 = np.zeros_like(image)
    for col in range(image.shape[0]):
        if col - 1 < 0:
            image_x[col] = image[col + 1] - image[col]
        elif col + 1 >= image.shape[0]:
            image_x[col] = image[col] - image[col - 1]
        else:
            image_x[col] = (image[col+1] - image[col - 1])/2
    for col in range(image.shape[0]):
        if col - 1 < 0:
            image_x2[col] = image_x[col + 1] - image_x[col]
        elif col + 1 >= image.shape[0]:
            image_x2[col] = image_x[col] - image_x[col - 1]
        else:
            image_x2[col] = (image_x[col+1] - image_x[col - 1])/2
    return image_x2


def rotateImage(image: np.ndarray, degree: float):
    """
    图片旋转
    :param image:
    :param degree: 逆时针旋转的度数
    :return:
    """
    rows = image.shape[0]
    cols = image.shape[1]
    M = cv2.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), degree, 1)
    dst = cv2.warpAffine(image, M, (cols, rows))
    return dst

def readImage(dir):
    image = cv2.imread(dir)
    return image


def resizeImage(image, sideLen=100):
    return cv2.resize(image, (sideLen, sideLen))


def getPatternFromImage(dir, sideLen=100):
    image = readImage(dir)
    image = resizeImage(image, sideLen)
    pattern = np.zeros((sideLen, sideLen), dtype=np.uint8)
    mask = np.zeros((sideLen, sideLen), dtype=np.uint8)
    for row in range(sideLen):
        for col in range(sideLen):
            if image[row, col, 0] < 1 and image[row, col, 1] < 1 and image[row, col, 2] < 1:
                pattern[row, col] = 1
    return pattern, mask
