
import sys
sys.path.append('./')
import os
from Simulation.ComApi import *
import random
from Common import settings
import math
from Simulation.ComSurfacePotentialField import ComSurfacePotentialField
import Simulation.ComObjectCollection as ComCol
import numpy as np


target = (-800, 800, 800)
potential_obstacle_range = 100
if __name__ == "__main__":
    stage = ComStage()
    # stage.enableFigSave('/Volumes/disk3/实验数据/test6')
    stage.mRuningTime = 1000000
    stage.isPlotGraph = False
    stage.isShowCommunicated = False
    stage.isRotating = True
    stage.setEnvSize((1000, 1000, 1000))
    stage.setFigSize((8, 8))

    binding_robot = None
    for i in range(1):
        # range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
        # range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
        # range_z = (-stage.mEnvSize[2], stage.mEnvSize[2])
        # x = random.uniform(range_x[0], range_x[1])
        # y = random.uniform(range_y[0], range_y[1])
        # z = random.uniform(range_z[0], range_z[1])
        robot = ComRobotCon((800, -800, -800))#调用potential_3D(800, -800, -800)
        robot.setColor((0, 0.5, 0.5, 1))
        robot.mRobotType = '3D'
        robot.mCommunicationRange = 800
        robot.isShowSenseRange = False
        robot.isDrawCommunicationRange = False 
        robot.isPlotTargetLine = False
        robot.isPlotOrientationLine = True
        robot.isPlotTrail = True
        robot.isPathPlanning = True
        robot.setDirection(math.pi/2)
        robot.mPathPlanningControl3D.setTarget(target)
        # robot.setTargetDirection(0)
        robot.setRadius(15)
        robot.setTrailLineColor((1, 0, 1, 0.5))
        binding_robot = robot
        stage.addRobot(robot)
        
    

    # obstacles_pos = [(i, j, k) for k in range(-2, 3) for j in range(-2, 3) for i in range(-2, 3)]
    obstacles_pos =[
                    (0, 0, 0),
                    (0, 0, -100),
                    (0, 0, 135),
                    (300, 0, 0),
                    (-300, 0, 0),
                    (450, 120, 200),
                    (-100, 500,430),
                    (500, -100, -200),
                    (700, -400, 800),
                    (700, -350,-350),
                    (-600, 800, -500),
                    (0, 210, 310),
                    (0, 350, 580),
                    (-920, 650, 950),
                    (950, -700, -700),
                    (800, -800, -800),
                    (600, -500, -400)
                    ]
    for obj in obstacles_pos:
        x = obj[0]
        y = obj[1]
        z = obj[2]
        fish = ComFish_range((x, y, z))
        fish.isPlotTargetLine = False
        fish.mRangeType = 1
        fish.mRangeAlpha = 0.2
        fish.isPlotTrail = False
        fish.isShowSenseRange = True
        fish.mSpeed = 0
        fish.mRange = potential_obstacle_range
        fish.setShape('circle_line')
        fish.setColor((0.3, 0.3, 0.3, 0.5))
        fish.setRadius(2)
        stage.addStuff(fish) 
    
    food = ComCol.getObjectByType('ComFish')
    population = ComCol.getObjectByType('ComRobot')

    CommunicationDist = 800
    surfz = ComSurfacePotentialField()
    surfz.setBindingRobot(binding_robot)
    surfz.setZDir('3D')
    surfz.setX(np.arange(-1000, 1000, 10))
    surfz.setY(np.arange(-1000, 1000, 10))
    surfz.setAlpha(0.5)
    surfz.setObstacleList(food)
    surfz.setPopulation(population)
    surfz.isShowAgentMark = True
    surfz.setOffset(0)
    surfz.setRobotRadius(potential_obstacle_range)
    surfz.setTarget(target)
    # surfz.setCMap(plt.cm.hot)
    stage.addSurface(surfz)
    stage.run()
    # stage.run_once()