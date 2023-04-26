import sys
sys.path.append('./')
sys.path.append('./src')
from Simulation.ComApi import *
import math

target = (0, 0, 0)
target_direction = math.pi/2

stage = ComStage2D()
stage.mRuningTime = 100
stage.setEnvSize((1000, 1000, 0))
stage.setFigSize((8, 8))
robot = ComRobotCon((500, 500, 0))
robot.mRobotType = '2D'
robot.isShowSenseRange = False
robot.isDrawCommunicationRange = False 
robot.isPlotTargetLine = False
robot.isPlotOrientationLine = True
robot.isPlotTrail = True
robot.setDirection(math.pi/2)
robot.setTarget(target)
robot.setTargetDirection(target_direction)
stage.addRobot(robot)

stage.run()