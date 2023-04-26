'''
我们使用图片绘图来实现复杂的障碍物环境
'''
import sys
sys.path.append('./')
sys.path.append('./src')
from Simulation.ComApi import *
import random
from Common import settings
import math
from Simulation.ComSurfacePotentialField import ComSurfacePotentialField
import Simulation.ComObjectCollection as ComCol
import numpy as np
from Common import utils, DrKDtree

image_dir = 'Resource/simple_rooms.png'
target = (0, 800, 0)
if __name__ == "__main__":
    
    stage = ComStage2D()
    # stage.enableFigSave('/Volumes/disk3/实验数据/test6')
    stage.mRuningTime = 1000000
    stage.isPlotGraph = False
    stage.isShowCommunicated = False
    stage.setEnvSize((1000, 1000, 1000))
    stage.setFigSize((8, 8))
    stage.addObstacleType('obstacle')
    for i in range(1):
        robot = ComRobotCon((800, -800, 0))
        robot.setColor((0, 0.5, 0.5, 1))
        robot.mRobotType = '2D'
        robot.mCommunicationRange = 800
        robot.isShowSenseRange = False
        robot.isDrawCommunicationRange = False 
        robot.isPlotTargetLine = False
        robot.isPlotOrientationLine = True
        robot.isPlotTrail = True
        robot.isPathPlanning = True
        robot.setDirection(math.pi/2)
        robot.mPathPlanningControl.setTarget(target)
        robot.getPlanningControl().setStride(20)
        robot.getPlanningControl().mRandomMoveRange = 60
        robot.setRadius(15)
        robot.setTrailLineColor((1, 0, 1, 0.5))
        stage.addRobot(robot)
        
    

    xs, ys = utils.getLinePointFromImage(image_dir, (-1000, 1000, -1000, 1000))
    z = 0
    for ind, _ in enumerate(xs):
        x = xs[ind]
        y = ys[ind]
        obstacle = ComFish((x, y, z))
        obstacle.mObjectType = 'obstacle'
        obstacle.isPlotTargetLine = False
        obstacle.isPlotTrail = False
        obstacle.isShowSenseRange = False
        obstacle.mSpeed = 0
        obstacle.setShape('circle')
        obstacle.setColor((0.3, 0.3, 0.3, 0.5))
        obstacle.setRadius(5)
        stage.addStuff(obstacle)
    
    dest = ComFish(np.array(target)+5)
    dest.setShape('circle')
    dest.setColor('red')
    dest.setRadius(10)
    dest.mSpeed = 0
    stage.addStuff(dest)


    CommunicationDist = 800

    stage.run()
