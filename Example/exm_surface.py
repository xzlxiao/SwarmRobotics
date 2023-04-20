import sys
sys.path.append('./')
from Simulation.ComSurfaceFitness import ComSurfaceFitness
from Simulation.ComSurfaceCrowdness import ComSurfaceCrowded
from Simulation.ComApi import *
import Simulation.ComObjectCollection as ComCol
import random
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
CommunicationDist = 800
if __name__ == "__main__":
    stage = ComStage()
    stage.isPlotGraph = False
    stage.mRuningTime = 1000
    stage.enableFigSave('/Volumes/disk3/实验数据/test6')
    
    for i in range(5):
        range_x = (-stage.mEnvSize[0]+200, stage.mEnvSize[0]-200)
        range_y = (-stage.mEnvSize[1]+200, stage.mEnvSize[1]-200)
        range_z = (-stage.mEnvSize[2]+500, stage.mEnvSize[2]-200)
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        z = random.uniform(range_z[0], range_z[1])
        robot = ComRobotAF_Global_Pos((x, y, z))
        robot.mCommunicationRange = CommunicationDist
        robot.isShowSenseRange = True
        robot.isDrawCommunicationRange = True 
        robot.isPlotTargetLine = True
        robot.isPlotTrail = True
        robot.isPathPlanning = False
        stage.addRobot(robot)

    for i in range(1):
        range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
        range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
        range_z = (-stage.mEnvSize[2], stage.mEnvSize[2])
        # x = random.uniform(range_x[0], range_x[1])
        # y = random.uniform(range_y[0], range_y[1])
        # z = random.uniform(range_z[0], range_z[1])
        x = 0
        y = 0
        z = 500
        fish = ComFish((x, y, z))
        fish.isPlotTargetLine = False
        fish.isPlotTrail = False
        fish.isShowSenseRange = True
        fish.mSpeed = 100
        stage.addStuff(fish)
    
    food = ComCol.getObjectByType('ComFish')
    population = ComCol.getObjectByType('ComRobotAF')

    surfz = ComSurfaceFitness()
    surfz.setOffset(-1000)
    surfz.setZDir('z')
    surfz.setX(np.arange(-1000, 1000, 20))
    surfz.setY(np.arange(-1000, 1000, 20))
    surfz.setAlpha(0.5)
    surfz.setFood(food)
    surfz.setPopulation(population)
    surfz.isShowAgentMark = True
    surfz.setSenseDistance(CommunicationDist)
    stage.addSurface(surfz)

    surf_crowd = ComSurfaceCrowded()
    surf_crowd.setOffset(-1200)
    surf_crowd.setX(np.arange(-1000, 1000, 20))
    surf_crowd.setY(np.arange(-1000, 1000, 20))
    surf_crowd.setAlpha(0.5)
    surf_crowd.setFood(food)
    surf_crowd.setPopulation(population)
    surf_crowd.isShowAgentMark = False
    surf_crowd.setSenseDistance(CommunicationDist)
    stage.addSurface(surf_crowd)

    stage.run()
    # stage.run_once()