
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

if __name__ == "__main__":
    stage = ComStage2D()
    # stage.enableFigSave('/Volumes/disk3/实验数据/test6')
    stage.mRuningTime = 1000000
    stage.isPlotGraph = False
    stage.isShowCommunicated = False
    stage.setEnvSize((1000, 1000, 1000))
    stage.setFigSize((8, 8))
    stage.addObstacleType('ComFish')
    for i in range(1):
        # range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
        # range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
        # range_z = (-stage.mEnvSize[2], stage.mEnvSize[2])
        # x = random.uniform(range_x[0], range_x[1])
        # y = random.uniform(range_y[0], range_y[1])
        # z = random.uniform(range_z[0], range_z[1])
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
        robot.mPathPlanningControl.setTarget((-800, 800, 0))
        # robot.setTargetDirection(0)
        robot.getPlanningControl().setStride(15)
        robot.setRadius(15)
        robot.setTrailLineColor((1, 0, 1, 0.5))
        stage.addRobot(robot)
        
    

    obstacles_pos = [
        (-200, 0, 0),
        # (-160, 0, 0),
        (-120, 0, 0),
        # (-80, 0, 0),
        (-40, 0, 0),
        (0, 0, 0),
        (40, 0, 0),
        # (80, 0, 0),
        (120, 0, 0),
        # (160, 0, 0),
        (200, 0, 0),
    ]
    for obj in obstacles_pos:
        x = obj[0]
        y = obj[1]
        z = obj[2]
        fish = ComFish((x, y, z))
        fish.isPlotTargetLine = False
        fish.isPlotTrail = False
        fish.isShowSenseRange = True
        fish.mSpeed = 0
        fish.setShape('square')
        fish.setColor((0.3, 0.3, 0.3, 0.5))
        fish.setRadius(40)
        stage.addStuff(fish)
    
    food = ComCol.getObjectByType('ComFish')
    population = ComCol.getObjectByType('ComRobot')

    CommunicationDist = 800
    surfz = ComSurfacePotentialField()
    surfz.setZDir('2D')
    surfz.setX(np.arange(-1000, 1000, 10))
    surfz.setY(np.arange(-1000, 1000, 10))
    surfz.setAlpha(1.0)
    surfz.setObstacleList(food)
    surfz.setPopulation(population)
    surfz.isShowAgentMark = True
    surfz.setRobotRadius(75)
    # surfz.setCMap(plt.cm.hot)
    stage.addSurface(surfz)
    stage.run()
    # stage.run_once()