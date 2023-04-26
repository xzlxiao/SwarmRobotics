import sys
sys.path.append('./')
sys.path.append('./src')
from Simulation.ComSurfaceFitness import ComSurfaceFitness
from Simulation.ComSurfaceCrowdness import ComSurfaceCrowded
from Simulation.ComDataPlotPosPredict import ComDataPlotPosPredict
from Simulation.ComDataPlotAFConvergence import ComDataPlotAFConvergence
from Simulation.ComMonitorPlotFitness import ComMonitorPlotFitness
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
    stage.mRuningTime = 100
    stage.setNaturalComNet()
    # stage.enableFigSave('/Users/xiaozhenlong/Desktop/tmp/test')
    robot0 = None 
    robot1 = None
    population = []
    target = None
    for i in range(20):
        range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
        range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
        range_z = (-stage.mEnvSize[2], stage.mEnvSize[2])
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        z = random.uniform(range_z[0], range_z[1])
        robot = ComRobotAF_Global_Pos((x, y, z))
        robot.mCommunicationRange = CommunicationDist
        robot.isShowSenseRange = True
        robot.isDrawCommunicationRange = True 
        robot.isPlotTargetLine = True
        robot.isPlotTrail = True
        robot.isPathPlanning = True
        stage.addRobot(robot)
        population.append(robot)
        if i == 0:
            robot0 = robot 
        elif i == 1:
            robot1 = robot

    for i in range(1):
        range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
        range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
        range_z = (-stage.mEnvSize[2], stage.mEnvSize[2])
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        z = random.uniform(range_z[0], range_z[1])
        fish = ComFish((x, y, z))
        fish.isPlotTargetLine = False
        fish.isPlotTrail = False
        fish.isShowSenseRange = True
        fish.mSpeed = 100
        stage.addStuff(fish)
        target = fish
    
    food = ComCol.getObjectByType('ComFish')
    population = ComCol.getObjectByType('ComRobotAF')
  

    surf_crowd = ComSurfaceCrowded()
    surf_crowd.setOffset(-1200)
    surf_crowd.setX(np.arange(-1000, 1000, 20))
    surf_crowd.setY(np.arange(-1000, 1000, 20))
    surf_crowd.setAlpha(0.8)
    surf_crowd.setFood(food)
    surf_crowd.setPopulation(population)
    surf_crowd.isShowAgentMark = False
    surf_crowd.setSenseDistance(CommunicationDist)
    stage.addSurface(surf_crowd)

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
    
    data_plot = ComDataPlotAFConvergence()
    data_plot.setPopulation(population)
    data_plot.setTarget(target)
    data_plot.setTimeRef(stage.mCurrentTime)

    monitor = ComMonitorPlotFitness()
    monitor.setX(np.arange(-1000, 1000, 20))
    monitor.setY(np.arange(-1000, 1000, 20))
    monitor.setPopulation(population)
    monitor.setAlpha(1)
    stage.setMonitorPlot(monitor)

    stage.setDataPlot(data_plot)
    stage.run()