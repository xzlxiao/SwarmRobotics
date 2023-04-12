'''
我们使用图片绘图来实现复杂的障碍物环境
'''
import sys
sys.path.append('./')
from Simulation.ComApi import *
import random
from Common import settings
import math
from Simulation.ComSurfacePotentialField import ComSurfacePotentialField
import Simulation.ComObjectCollection as ComCol
import numpy as np
from Common import utils, DrKDtree

target = (0, 0, 0)
robot_location = [
    (800, -800, 0),
    (800, 800, 0),
    (-800, 800, 0),
    (-800, -800, 0),
]
if __name__ == "__main__":
    
    stage = ComStage2D()
    # stage.enableFigSave('/Volumes/disk3/实验数据/test7')
    # stage.enableInfoSave('/Volumes/disk3/实验数据/test7')
    # stage.enablePosSave('/Volumes/disk3/实验数据/test7')
    stage.mRuningTime = 1000000
    stage.isPlotGraph = False
    stage.isShowCommunicated = True
    stage.setEnvSize((1000, 1000, 1000))
    stage.setFigSize((8, 8))
    for i in range(4):
        
        range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
        range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        robot = ComRobotCommunicationDepict(robot_location[i])
        robot.setColor((0, 0.5, 0.5, 1))
        robot.mRobotType = '2D'
        robot.isCommunicating = True
        robot.mCommunicationRange = 800
        robot.mSenseDistance = 800
        robot.isShowSenseRange = True
        robot.isDrawCommunicationRange = False 
        robot.isPlotTargetLine = False
        robot.isPlotOrientationLine = True
        robot.isPlotTrail = True
        robot.isPathPlanning = True
        robot.setDirection(math.pi/2)
        robot.getPlanningControl().setTarget((x, y, 0))
        robot.setInitInfo('ComFish', 0, (x, y, 0))
        robot.getPlanningControl().setStride(10)
        robot.getPlanningControl().mRandomMoveRange = 60
        robot.setRadius(15)
        robot.setTrailLineColor(utils.getColor(i))
        if i > 0:
            robot.setSensable(False)
        stage.addRobot(robot)
    
    dest = ComFish(target)
    dest.setRadius(100)
    dest.mSpeed = 50
    dest.isPlotTargetLine = False 
    stage.addStuff(dest)


    CommunicationDist = 800
    stage.run()