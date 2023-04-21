"""
File name: DrKDtree.py
Author: 肖镇龙（Zhenlong Xiao）
Description: This is KDtree module.
"""

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
        '''
        Constructs a KDTree object using the provided input points.

        Args:
            points (list): A list of points to construct the KDTree from.
        '''
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
        """
        Get the points from self.mPoints at the given indices.
        
        Args:
            inds (list): A list of indices for which to retrieve the points.
            
        Returns:
            np.array: An np.array containing the points from self.mPoints at the given indices.
        """    
        return np.array([self.mPoints[i] for i in inds])
    
    def formatPoint(self, point):
        """
        Formats the provided point into a numpy array with up to 3 dimensions.

        Args:
            point (np.array): The point to be formatted.

        Returns:
            np.array: The formatted point. Will have a shape of (1, 3) if it has only two non-zero values.
                    Otherwise will have the same shape as the input.
        """
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
        Query the k nearest neighbors to a given point.

        Args:
            point (tuple): A tuple representing the point to search the nearest neighbors for.
            k (int, optional): The number of nearest neighbors to return. Defaults to 3.

        Returns:
            dist: An array of distances to the k neighbors.
            ind:  The indices of the k neighbors in the original data.
        """
        # Convert the point to a numpy array and format it
        point = np.array(point)
        point = self.formatPoint(point)

        # Use the k-D tree to query the k nearest neighbors
        return self.tree.query(point, k)

    
    # def queryMat(self, point_mat):
    #     """
    #     查询与point_mat点集中每个点最近的1个点，返回对应的点集
    #     :param point_mat: 必须是n x 3的矩阵
    #     :return: dist ind
    #     """
    #     return self.tree.query(point_mat, 1)

    def query_radius(self, point, r=1.0):
        '''
        Returns the number of points within a given radius of a given point.

        Args:
            point (list): A list containing the x,y,z coordinates of the query point.
            r (float): The radius around the query point in which to search for other points.

        Returns:
            int: The number of points within the specified radius of the query point.
        '''
        pt = self.formatPoint(point)
        if isCupy:
            pt = pt.get()
        return self.tree.query_radius(pt, r, return_distance=True, sort_results=True)
    
    def query_radius_count(self, point, r=1.0):
        '''
        Returns the number of points within a given radius of a given point.

        Args:
            point (list): A list containing the x,y,z coordinates of the query point.
            r (float): The radius around the query point in which to search for other points.
            
        Returns:
            int: The number of points within the specified radius of the query point.
        '''   

        pt = self.formatPoint(point)
        if isCupy:
            pt = pt.get()
        return self.tree.query_radius(pt, r, count_only=True)

    def query_radius_count2(self, points, r=1.0):
        '''
        Returns the number of points within a given radius of multiple query points.

        Args:
            points (array-like): A 2D array or list containing the x,y,z coordinates of the query points.
            r (float): The radius around each query point in which to search for other points.
            
        Returns:
            int: The total number of points within the specified radius of all query points combined.
        '''
        if isCupy:
            points = points.get()
        return self.tree.query_radius(points, r, count_only=True)

    def sort(self):
        """
        Sort the points according to proximity.

        Returns:
            ret: A list of points, sorted by their proximity to each other.
        """
        # Create a list containing only the first point
        ret = [self.mPoints[0]]

        # Iterate over each point in the dataset
        for i in range(1, self.mLen):
            k = 3
            isFind = False

            # Keep searching for neighbors until we find one that is not already in the list
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

        # Add an extra point to make the contour complete
        ret.append(ret[0])

        return ret


    def sort2(self, ori_pt, ori_direction):
        """
        Sort points by polar coordinates, with respect to a specified center point and direction.

        Args:
            ori_pt: The center point around which to sort the other points.
            ori_direction: The direction in which to sort the other points.

        Returns:
            ret: A list of points, sorted by their polar coordinates with respect to the center point and direction.
        """
        # Convert all points to polar coordinates
        angle_list, dist_list = KDtree.polarCoordinates(ori_pt, ori_direction, self.mPoints, self.mLen)

        # Find the index of the point with the smallest angle
        angle_min_pt_index = angle_list.index(min(angle_list))

        # Add this point to the beginning of the sorted list
        ret = [self.mPoints[angle_min_pt_index]]

        # Iterate over each remaining point in the dataset
        for i in range(1, self.mLen):
            k = 3
            isFind = False

            # Keep searching for neighbors until we find one that is not already in the list
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

        # Add an extra point to make the contour complete
        ret.append(ret[angle_min_pt_index])

        return ret


    @staticmethod
    def polarCoordinates(ori_pt: np.ndarray, ori_direction: float, point_list: np.ndarray, pt_num: int):
        """
        Compute polar coordinates for a set of points relative to an origin point in 2D space.

        Args:
            ori_pt (numpy array): The center point of the polar coordinate system.
            ori_direction (float): The direction in radians that the polar coordinate system is facing.
            point_list (numpy array): The set of points to convert to polar coordinates.
            pt_num (int): The number of points in the set.

        Returns:
            tuple: A tuple containing two lists, the angle list and distance list of all points in the set relative to the polar coordinate system.
        """
        # Initialize empty lists to store the resulting polar coordinates
        angle_list = []
        dist_list = []

        # Convert the origin point to a numpy array if it's not already
        ori_pt_tmp = np.array([ori_pt[0], ori_pt[1]], dtype=np.float32)

        # Loop through each point in the point_list to convert to polar coordinates
        for i in range(pt_num):
            # Get the x,y components of the point relative to the origin
            pt = point_list[i] - ori_pt_tmp

            # Compute the distance from the point to the origin using the Pythagorean theorem
            dist = math.sqrt(math.pow(pt[1], 2) + math.pow(pt[0], 2))

            # Compute the angle of the point relative to the positive y-axis (in radians)
            angle1 = math.asin(pt[1]/dist)

            # Adjust angles based on placement of point around the origin; ensure all angles are within [0, 2pi)
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

        # Return the resulting angle and distance lists as a tuple
        return angle_list, dist_list

