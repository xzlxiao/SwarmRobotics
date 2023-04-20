
import sys
sys.path.append('./')
from Simulation.ComApi import *
import random
from Common import settings
from Simulation.ComSurfaceFitness import ComSurfaceFitness
from Simulation.ComSurfaceCrowdness import ComSurfaceCrowded
import numpy as np

if __name__ == "__main__":
    stage = ComStage()
    stage.mRuningTime = 1000000
    stage.isPlotGraph = False
    for i in range(10):
        range_x = (-stage.mEnvSize[0], stage.mEnvSize[0])
        range_y = (-stage.mEnvSize[1], stage.mEnvSize[1])
        range_z = (-stage.mEnvSize[2], stage.mEnvSize[2])
        x = random.uniform(range_x[0], range_x[1])
        y = random.uniform(range_y[0], range_y[1])
        z = random.uniform(range_z[0], range_z[1])
        robot = ComRobotAF_Global_Pos((x, y, z))
        robot.mCommunicationRange = 800
        robot.isShowSenseRange = True
        robot.isDrawCommunicationRange = True 
        robot.isPlotTargetLine = True
        robot.isPlotTrail = True
        robot.isPathPlanning = True
        stage.addRobot(robot)

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
    
    stage.run()
    # stage.run_once()