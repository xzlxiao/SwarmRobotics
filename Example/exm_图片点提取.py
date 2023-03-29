import sys 
sys.path.append('./')
import matplotlib.pyplot as plt
from draw import ImageProc
import cv2
import numpy as np
import random
from Common import utils


image_dir = 'Resource/simple_rooms.png'


x, y = utils.getLinePointFromImage(image_dir, (-300, 800, -1000, 1000))
plt.scatter(x, y, s=1)
plt.show()