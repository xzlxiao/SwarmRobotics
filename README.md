# SwarmRobotics

The project is a Swarm Robotics simulation platform implemented using Python and Matplotlib.It allows users to create custom Swarm Robotics algorithms and test and evaluate them in a simulated environment.

[English](./README.md)|[中文](./readme_chinese.md)

## 1. Configure

To install the project, you need to perform the following steps:

*   Clone this project:


```bash
git clone https://github.com/xzlxiao/SwarmRobotics.git
```
 - Install Dependencies:


```bash
pip install -r requirements.txt
```


*   Enter the project directory:


```bash
cd SwarmRobotics
```


## 2. How to use

To run the Swarm Robotics simulation platform, you can execute the following command:

*   3D Path Planning (Potential Field Path Planning Algorithm)
```bash
python Example/exm_路径规划-3D.py
```


Replace Python with your compiler version.In this script, you can modify various parameters of the algorithm.

The following example image is a gif and may not autoplay in some browsers.

![](Resource/path_planning_3d.gif)

*   2D Path Planning 1

![](Resource/path_planning_2d.gif)

*   2D Path Planning 2

![](Resource/path_planning_2d_2.gif)

*   3D Path Following

![](Resource/path_following_3d.gif)

*   2D Path Following

![](Resource/path_following_2d.gif)

*   multi-objective search algorithm

![](Resource/multi_target_search.gif)

*   Subgroup dynamic segmentation algorithm

![](Resource/subgroup_split.gif)

*   AFSA multi-robot target search, perception limited

![](Resource/AFSA.gif)

*   Local Extreme Value Problem of Potential Field Class Path Planning Method

![](Resource/local_extremum1.gif)

*   Random Bounce Local Extreme Method

![](Resource/local_extremum_leap1.gif)

*   Path Planning with Perceptual Restrictions

![](Resource/sense_limited.gif)

## 3. Example

*   Here's a simple example that uses a path-following computer to move the robot from its current position to its target position and posture (target, target\_direction):


```python
import sys
sys.path.append('./')
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
```


## 4. Author

*   [肖镇龙](https://github.com/xzlxiao)

## 5. Contribution

Contributors:

*   [肖镇龙](https://github.com/xzlxiao)
*   [李昌浩](https://github.com/Pekachiu)
*   黄吉
*   If you want to contribute to the project, see contributing

[CONTRIBUTING.md]()

## 6. License

It is licensed under the GPL (GNU General Public License). For more information, see license.

[LICENSE](./License)
