from PyQt5.QtCore import pyqtSignal, QObject, QEvent
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.pyplot as plt
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import matplotlib.patches as mpathes
from PyQt5.QtWidgets import QWidget
from Common.utils import svg_parse, loadSVG, unitVector
from matplotlib.path import Path
import math
import random

path_list = []
translate_list = []
path_list.append("M60.0,120.0C28.0,120,.0,88.0,.0,60.0C.0,28.0,26.0,.0,60.0,.0L60.0,120.0z")
translate_list.append((-3,49.13))
path_list.append("M327.9,120.0L327.9,.0L.0,.0L.0,120.0L327.9,120.0z")
translate_list.append((57.64,49.13))
path_list.append("M.0,24.6C.0,11.0,11.0,.0,24.6,.0C38.1,.0,49.1,11.0,49.1,24.6C49.1,38.1,38.1,49.1,24.6,49.1C11.0,49.1,.0,38.1,.0,24.6z")
translate_list.append((185.20,0.00))
path_list.append("M.0,24.6C.0,11.0,11.0,.0,24.6,.0C38.1,.0,49.1,11.0,49.1,24.6C49.1,38.1,38.1,49.1,24.6,49.1C11.0,49.1,.0,38.1,.0,24.6z")
translate_list.append((185.20,168.19))
path_list.append("M25.5,49.1L25.5,.0L.0,.0L.0,49.1L25.5,49.1z")
translate_list.append((348.43,0.00))
path_list.append("M25.5,49.1L25.5,.0L.0,.0L.0,49.1L25.5,49.1z")
translate_list.append((348.43,168.19))

codes, verts = loadSVG(path_list, translate_list)

class ComObject:
    update_count = 0

    # This method initializes the object's attributes when it is created.
    def __init__(self) -> None:
        self.mId = -1
        self.mPos = np.array([0, 0, 0], dtype=np.float32)
        self.mOrientation = np.zeros((3, 3), dtype=np.float)
        self.mTarget = np.array(self.mPos, dtype=np.float32)
        self.mTargetOrientation = np.array(self.mOrientation, dtype=np.float)
        self.mInterval = []  # List of time intervals between iterations
        self.mSpeed = 300
        self.mColor = 'red'
        self.mShape = 'circle_line'
        self.mRadius = 20
        self.image = None
        self.setRadius(self.mRadius)
        self.setColor(self.mColor)
        self.mWindow = None
        self.mAlpha = 1.0
        self.mPopulation = None 
        self.mObstacle = None 
        self.mAx = None
        self.mDirection = 0.0     # Radians
        self.mTargetDirection = None
        self.isPlotTargetLine = True
        self.mStage = None
        self.mTrail = [[], [], []]
        self.isPlotTrail = False
        self.mObjectType = "ComObject"       # Marks the type of object
        self.mIterations = 0        # Number of iterations
        self.mRobotType = '3D'
        self.mTargetLineLen = -1  # Target line length. If < 0, connects to destination; otherwise, only displays a section
        self.mTrailLineColor = (0, 0, 1, 0.1)
        self.mTrailLineAlpha = 0.5
        self.mMessage = ''      # Special information for saving records


    # This method is called getMessage() and belongs to a class.
    # The purpose of the method is to return the value of mMessage.

    def getMessage(self):
        """
        Public method of the class, returns the value of mMessage.
        mMessage is a user-defined message that can be used to store intermediate data during simulation.

        Returns:
            string: The current instance's mMessage string
        """
        return self.mMessage 


    def setMessage(self, message):
        """
        Public method of the class, used to set the value of mMessage.

        Args:
            message (string): message is a user-defined message that can be used to store intermediate data during simulation.
        """        
        # Set the value of the instance variable mMessage to the passed message argument.
        self.mMessage = message



    def setRobotType(self, robot_type='3D'):
        self.mRobotType = robot_type

    # This method is called setTrailLineColor() and belongs to a class.
    # The purpose of the method is to set the color of the robot's motion trail.
    def setTrailLineColor(self, color=(0, 0, 1, 0.1)):
        """
        Set the color used for drawing the robot's movement trail.

        Args:
            color (tuple, optional): A tuple representing the RGBA color value to use for the trail. 
                Defaults to (0, 0, 1, 0.1).
            """        
        # Set the instance variable 'mTrailLineColor' to the passed color argument
        self.mTrailLineColor = color 
        
        # If the color argument is a tuple and its length is at least 4, set the alpha value of the trail line
        if type(color) == tuple and len(color)>=4:
            self.mTrailLineAlpha = color[3]

    
    def setTrailLineAlpha(self, alpha):
        """Set the alpha value of the trail line
        
        Args:
            alpha (float): The desired alpha value for the trail line.
        """        
        # Set the trail line's alpha value to the input argument
        self.mTrailLineAlpha = alpha


    def getUpdateCount(self):
        return ComObject.update_count
        
    def setTargetDirection(self, direction):
        self.mTargetDirection = direction
        
    def setTarget(self, target):
        '''
            target: 1 x 3, x y z
        '''
        self.target = target
        # self.mTarget = np.array(target[0:3], dtype=np.float32)

    def setStage(self, stage):
        self.mStage = stage

    def setDirection(self, direction):
        if direction > math.pi*2:
            self.mDirection = direction%(math.pi*2)
        elif direction < -math.pi*2:
            self.mDirection = direction%(-math.pi*2)
        else:
            self.mDirection = direction
        if self.mTargetDirection is None:
            self.mTargetDirection = self.mDirection
            
    @staticmethod
    def getAngleBetweenXandVector(pt1, pt2, plat='xy'):
        '''
        @brief: Calculate angle between two points and x-axis.
        @param pt1: Starting point.
        @param pt2: Ending point.
        @param plat: Plane on which the angle is to be calculated.
        '''

        deltaX, deltaY, deltaZ = pt2[0] - pt1[0], pt2[1] - pt1[1], 0
        
        if len(pt1) == 3 and len(pt2) == 3:
            deltaZ = pt2[2] - pt1[2]
        
        # Define a dictionary to map input strings to their corresponding mathematical functions. 
        # As there were multiple if-else conditions in the original code, it can be made more concise by using a dictionary.
        planes = {'xy':(deltaY, deltaX), 'xz':(deltaZ, deltaX), 'yz':(deltaZ, deltaY), 'o-xy':(deltaZ, math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY, 2)))}
        
        try:
            return math.atan2(*planes[plat])
        except KeyError:  # Invalid 'plat' parameter.
            return None
        
    # @staticmethod
    # def getAngleBetweenXandVector(pt1, pt2, plat='xy'):
    #     '''
    #     @brief 获得两点形成的向量与x轴之间的夹角（二维）
    #     @param pt1: 起始点
    #     @param pt2: 终止点
    #     '''

    #     deltaX = pt2[0] - pt1[0]
    #     deltaY = pt2[1] - pt1[1]
    #     deltaZ = 0
    #     if len(pt1) == 3 and len(pt2) == 3:
    #         deltaZ = pt2[2] - pt1[2]
        
    #     if plat=='xy':  # xy平面内与x轴的夹角
    #         return math.atan2(deltaY, deltaX)
    #     elif plat=='xz': # xz平面内与x轴的夹角
    #         return math.atan2(deltaZ, deltaX)
    #     elif plat=='yz': # yz平面内与y轴的夹角
    #         return math.atan2(deltaZ, deltaY) 
    #     elif plat=='o-xy': # 与xy平面的夹角
    #         deltaXY = math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY, 2))
    #         return math.atan2(deltaZ, deltaXY) 
    #     else:
    #         return None


    def setColor(self, color=(1.0, .0, .0, 1.0)):
        self.mColor = color
        self.setShape(self.mShape)
    
    def setRadius(self, radius=5):
        self.mRadius = radius
        self.setShape(self.mShape)

    def setInterval(self, interval: list):
        self.mInterval = interval

    @property
    def pos(self):
        if self.mRobotType == '2D':
            return self.mPos[0:2]
        elif self.mRobotType == '3D':
            return self.mPos[0:3]
        else:
            return self.mPos

    @pos.setter
    def pos(self, value):
        if self.mRobotType == '2D':
            self.mPos[0:2] = np.array(value[0:2], dtype=np.float32)
        elif self.mRobotType == '3D':
            self.mPos[0:3] = np.array(value[0:3], dtype=np.float32)
        else:
            self.mPos = np.array(value, dtype=np.float32)
    
    @property
    def target(self):
        if self.mRobotType == '2D':
            return self.mTarget[0:2]
        elif self.mRobotType == '3D':
            return self.mTarget[0:3]
        else:
            return self.mTarget

    @target.setter
    def target(self, value):
        if self.mRobotType == '2D':
            self.mTarget[0:2] = np.array(value[0:2], dtype=np.float32)
        elif self.mRobotType == '3D':
            self.mTarget[0:3] = np.array(value[0:3], dtype=np.float32)
        else:
            self.mTarget = np.array(value, dtype=np.float32)

    def getPos2d(self):
        return self.mPos[0:2]

    @staticmethod
    def getRotationMat(angle: float):
        return np.array([[math.cos(angle), math.sin(angle)],[-math.sin(angle), math.cos(angle)]])

    def setShape(self, shape='circle'):
        self.mShape = shape
        pos = self.mPos[0:2] - self.mRadius
        if shape=='circle':     # 实心圆
            self.image = mpathes.Circle(pos, radius=self.mRadius, color=self.mColor, fill=True)
        elif shape=='circle_line':      # 圆圈
            self.image = mpathes.Circle(pos, radius=self.mRadius, color=self.mColor,fill=False)
        elif shape=='square':     # 实心方块
            pos = self.mPos[0:2] - self.mRadius
            self.image = mpathes.Rectangle(pos, self.mRadius*2, self.mRadius*2, color=self.mColor, fill=True)
        elif shape=='square_line':    # 空心方块
            self.image = mpathes.Rectangle(pos, self.mRadius*2, self.mRadius*2, color=self.mColor, fill=False)
        elif shape=='ship':
            verts_real = verts.copy()
            verts_real[:, 0] = -(verts[:, 0]/3) * (self.mRadius/20)
            verts_real[:, 1] = verts[:, 1]/3 * (self.mRadius/20)
            rotation_mat = ComObject.getRotationMat(self.mDirection)
            verts_real[:, 0:2] = np.matmul(verts_real[:, 0:2], rotation_mat)
            verts_real[:, 0] += self.mPos[0]
            verts_real[:, 1] += self.mPos[1]
            
            if isCupy:
                path = Path(verts_real.get(), codes.get())
            else:
                path = Path(verts_real, codes)
            
            self.image = mpathes.PathPatch(path, facecolor=self.mColor, edgecolor='k', lw=1)
        elif shape=='fish':
            face_color = self.mColor
            # edge_color = self.mColor[0:3]
            edge_color = (0, 0, 0)
            
            angle = self.getDirectionByDegree()
            # self.image = mpathes.Circle(self.mPos[0:2], radius=self.mRadius, color=self.mColor, fill=True)
            if isCupy:
                pos = self.mPos[0:2].get()
            else:
                pos = self.mPos[0:2]
            self.image = mpathes.Wedge(pos, self.mRadius, theta1=angle-15+180, theta2=angle+15+180, edgecolor=edge_color, facecolor=face_color)
        else:
            raise RuntimeError('没有该预定义形状')

    def getTime(self):
        return self.mIterations * self.mInterval[0]
        
    def sense(self):
        """
        观察周围，感知区域为圆形
        :return: 障碍物位置 机器人位置
        """
        pass 


    def distance(self, pt1, pt2):
        """
        计算两点间的距离
        :param pt1: list or np.array
        :param pt2: list or np.array
        :return:
        """
        if self.mRobotType == '3D':
            dim = 3
        elif self.mRobotType == '2D':
            dim = 2

        return np.linalg.norm(np.array(pt2[:dim]) - np.array(pt1[:dim]))


    def move(self):
        """
        向目标移动一步
        :return:
        """
        self.mIterations += 1
        pos_last = self.mPos.copy()
        robot_pos = self.pos
        target_pos = self.target

        if not self.isStopping():
            angle = ComObject.getAngleBetweenXandVector(robot_pos, target_pos)
            self.setDirection(angle)
            
            if np.linalg.norm(target_pos - robot_pos, ord=2) > self.mSpeed * self.mInterval[0]:
                
                self.pos = robot_pos + \
                            self.mSpeed * self.mInterval[0] * (target_pos - robot_pos) / \
                            np.linalg.norm(target_pos - robot_pos, ord=2)
                # if self.mId == 0:
                #     print(pos_last, self.mPos)
                #     print(self.distance(pos_last, self.mPos))
            else:
                self.pos = self.target
        self.setShape(shape=self.mShape)
        if not (self.mPos == pos_last).all():
            self.mTrail[0].append(self.mPos[0])
            self.mTrail[1].append(self.mPos[1])
            self.mTrail[2].append(self.mPos[2])

    def isStopping(self):
        """
        当前机器人是处于运动还是停止状态
        :return:
        """
        
        if (self.pos == self.target).all():
            return True
        else:
            return False

    def update(self):
        """

        :return:
        """
        self.move()

    def setAx(self, ax):
        self.mAx = ax

    def drawOnFigure(self, ax):
        ax.add_patch(self.image)
        z_pos = self.mPos[2]
        if isCupy:
            z_pos = z_pos.get()
        if self.mStage.mStageType == '3D':
            art3d.pathpatch_2d_to_3d(self.image, z=z_pos, zdir='z')
        if self.isPlotTargetLine:
            x, y, z = 0, 0, 0
            if self.mTargetLineLen < 0:
                x = np.array([self.mPos[0], self.mTarget[0]])
                y = np.array([self.mPos[1], self.mTarget[1]])
                z = np.array([self.mPos[2], self.mTarget[2]])
            else:
                target_vector = self.mTarget - self.mPos
                unit_target = self.mPos + unitVector(target_vector)*self.mTargetLineLen
                x = np.array([self.mPos[0], unit_target[0]])
                y = np.array([self.mPos[1], unit_target[1]])
                z = np.array([self.mPos[2], unit_target[2]])
            if isCupy:
                x = x.get()
                y = y.get()
                z = z.get()
            if self.mStage.mStageType == '3D':
                ax.plot(x, y, z, 'b--')
            else:
                ax.plot(x, y, 'b--')
        if self.isPlotTrail:
            if self.mStage.mStageType == '3D':
                ax.plot(self.mTrail[0], self.mTrail[1], self.mTrail[2], color=self.mTrailLineColor, alpha=self.mTrailLineAlpha)
            else:
                ax.plot(self.mTrail[0], self.mTrail[1], color=self.mTrailLineColor, alpha=self.mTrailLineAlpha)
        
    def draw(self, ax):
        self.drawOnFigure(ax)

    def chooseRandomTarget(self):
        """_summary_
        """        
        range_x = (-100, 100)
        range_y = (-100, 100)
        range_z = (-100, 100)
        if self.mStage is not None:
            range_x = (-self.mStage.mEnvSize[0], self.mStage.mEnvSize[0])
            range_y = (-self.mStage.mEnvSize[1], self.mStage.mEnvSize[1])
            if self.mStage.mStageType == '3D':
                range_z = (-self.mStage.mEnvSize[2], self.mStage.mEnvSize[2])
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        z = 0
        if self.mStage.mStageType == '3D':
            z = random.uniform(range_z[0], range_z[1])
        self.setTarget((x, y, z))

    def getDirectionByDegree(self):
        return math.degrees(self.mDirection)
        # return self.mDirection*180/math.pi 
        