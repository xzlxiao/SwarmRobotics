# -*- coding: utf-8 -*-

from sklearn.neighbors import KDTree as skl_kdtree
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import copy
import math


class KDtree():
    def __init__(self, points):
        kd_points = np.array(points).tolist()
        # print(kd_points[0])
        if len(kd_points[0]) < 3:
            kd_points = [[point[0], point[1], 0.0] for point in kd_points]
            
        # print(kd_points[0])
        if isCupy:
            kd_points = kd_points.get()
        self.tree = skl_kdtree(kd_points)
        self.mPoints = points
        self.mLen = len(points)

    def getPoints(self, inds):
        return np.array([self.mPoints[i] for i in inds])
    
    def formatPoint(self, point):
        pt = point.copy()
        if pt.ndim == 1:
            if pt.size < 3:
                pt_tmp = np.array([.0, .0, .0])
                pt_tmp[0:2] = pt[0:2]
                pt = pt_tmp
            pt = pt.reshape(1, -1)
        elif pt.ndim == 2:
            if pt.size < 3:
                pt_tmp = np.array([[.0, .0, .0]])
                pt_tmp[0, 0:2] = pt[0, 0:2]
                pt = pt_tmp
        return pt

    def query(self, point, k=3):
        """
        查询与点point最近的k个点
        :param point:
        :param k:
        :return: dist ind
        """
        point = np.array(point)
        point = self.formatPoint(point)

        return self.tree.query(point, k)
    
    # def queryMat(self, point_mat):
    #     """
    #     查询与point_mat点集中每个点最近的1个点，返回对应的点集
    #     :param point_mat: 必须是n x 3的矩阵
    #     :return: dist ind
    #     """
    #     return self.tree.query(point_mat, 1)

    def query_radius(self, point, r=1.0):
        """
        查询点point半径r范围内的所有点
        :param point:
        :param r:
        :return: ind
        """
        pt = self.formatPoint(point)
        if isCupy:
            pt = pt.get()
        return self.tree.query_radius(pt, r, return_distance=True, sort_results=True)
    
    def query_radius_count(self, point, r=1.0):

        pt = self.formatPoint(point)
        if isCupy:
            pt = pt.get()
        return self.tree.query_radius(pt, r, count_only=True)

    def query_radius_count2(self, points, r=1.0):
        if isCupy:
            points = points.get()
        return self.tree.query_radius(points, r, count_only=True)

    def sort(self):
        """
        按照距离进行排序
        :return:
        """
        ret = [self.mPoints[0]]
        for i in range(1, self.mLen):
            k = 3
            isFind = False
            while(True):
                _, ind = self.query([ret[-1]], k)
                for j in ind[0]:
                    if self.mPoints[j] not in ret:
                        ret.append(self.mPoints[j])
                        isFind = True
                        break
                if isFind:
                    break
                else:
                    k += 1
        # 多添加1个点作为控制点，以形成完整的圆
        ret.append(ret[0])
        return ret

    def sort2(self, ori_pt, ori_direction):
        """
        通过极坐标排序，先将所有点转化为以ori_pt为中心的极坐标系，取离极坐标系角度最小的点为起始点
        :param ori_pt: 中心点
        :param ori_direction: 方向
        :return:
        """
        angle_list, dist_list = KDtree.polarCoordinates(ori_pt, ori_direction, self.mPoints, self.mLen)
        angle_min_pt_index = angle_list.index(min(angle_list))
        ret = [self.mPoints[angle_min_pt_index]]
        for i in range(1, self.mLen):
            k = 3
            isFind = False
            while (True):
                _, ind = self.query([ret[-1]], k)
                for j in ind[0]:
                    if self.mPoints[j] not in ret:
                        ret.append(self.mPoints[j])
                        isFind = True
                        break
                if isFind:
                    break
                else:
                    k += 1
        # 多添加1个点作为控制点，以形成完整的圆
        ret.append(ret[angle_min_pt_index])
        return ret

    @staticmethod
    def polarCoordinates(ori_pt: np.ndarray, ori_direction: float, point_list: np.ndarray, pt_num: int):
        """
        计算极坐标
        :param ori_pt: 极坐标系原点
        :param ori_direction: 极坐标系方向
        :param point_list: 要转换的点集
        :param pt_num: 点集的数量
        :return: 角度列表，距离列表
        """
        angle_list = []
        dist_list = []
        ori_pt_tmp = np.array([ori_pt[0], ori_pt[1]], dtype=np.float32)
        for i in range(pt_num):
            pt = point_list[i] - ori_pt_tmp
            dist = math.sqrt(math.pow(pt[1], 2) + math.pow(pt[0], 2))
            angle1 = math.asin(pt[1]/dist)
            if angle1 > 0 and pt[0] < 0:
                angle1 = math.pi - angle1
            elif angle1 < 0 and pt[0] > 0:
                angle1 = 2 * math.pi + angle1
            elif angle1 < 0 and pt[0] < 0:
                angle1 = math.pi - angle1
            angle1 -= ori_direction
            if angle1 < 0:
                angle1 += 2 * math.pi
            elif angle1 > 2 * math.pi:
                angle1 -= 2 * math.pi
            angle_list.append(angle1)
            dist_list.append(dist)
        return angle_list, dist_list
