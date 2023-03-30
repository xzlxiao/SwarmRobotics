import sys 
sys.path.append('./')
import matplotlib.pyplot as plt
from draw import ImageProc
import cv2
import numpy as np
import random
from Common import utils, DrKDtree


image_dir = 'Resource/simple_rooms.png'


# x, y = utils.getLinePointFromImage(image_dir, (-300, 800, -1000, 1000))
x, y = utils.getLinePointFromImage(image_dir)
# target = [(60, 60)]
# image_p = np.zeros((300, 300), dtype=np.float)
# pts = [[x[i], y[i], 0] for i in range(len(x))]
# pts = np.array(pts)
# pt = np.array([0.1, 0.1, 0])
# pt2 = np.array([[0.1, 0.1, 0], [0.9, 0.9, 0]])
# kdtree = DrKDtree.KDtree(pts)
# _, a = kdtree.query(pt, k=3)
# _, b = kdtree.query(pt2, k=3)
# print(kdtree.getPoints(b)[0])
plt.scatter(x, y, s=1)
plt.show()