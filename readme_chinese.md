#  群体机器人（SwarmRobotics）

该项目是一个使用Python和Matplotlib实现的Swarm Robotics仿真平台。它允许用户创建自定义的Swarm Robotics算法，并在仿真环境中进行测试和评估。


[English](./README.md)|[中文](./readme_chinese.md)

## 1. 安装
要安装该项目，您需要执行以下步骤：

- 克隆该项目：

```bash
git clone https://github.com/xzlxiao/SwarmRobotics.git
```
- 安装依赖项：

```bash
pip install -r requirements.txt
```

- 进入项目目录：

```bash
cd SwarmRobotics
```



## 2. 使用方法
要运行Swarm Robotics仿真平台，可以执行如下以下命令：

- 在根目录下运行`Example`文件夹中的脚本
```bash
python Example/exm_AFSA_search.py
python Example/exm_path_planning-2D.py
python Example/exm_path_planning-3D.py
python Example/exm_path_planning2-2D.py
python Example/exm_path_planning3_sense_limited-2D.py
python Example/exm_path_planning4_communication_limited-2D.py
python Example/exm_surface.py
```

把python换成你对应的编译器版本。
在该脚本里，你能修改算法的各样参数。

以下示例图是gif图片，部分浏览器可能无法自动播放。

![](Resource/path_planning_3d.gif)

- 2D路径规划1

![](Resource/path_planning_2d.gif)

- 2D路径规划2

![](Resource/path_planning_2d_2.gif)

- 3D路径跟随

![](Resource/path_following_3d.gif)

- 2D路径跟随

![](Resource/path_following_2d.gif)

- 多目标搜索算法

![](Resource/multi_target_search.gif)

- 子群动态分割算法

![](Resource/subgroup_split.gif)

- AFSA多机器人目标搜索，感知受限

![](Resource/AFSA.gif)

- 势场类路径规划方法的局部极值问题

![](Resource/local_extremum1.gif)

- 随机跳出局部极值方法

![](Resource/local_extremum_leap1.gif)

- 感知受限的路径规划

![](Resource/sense_limited.gif)

## 3. 示例
- 下面是一个简单的示例，它使用路径跟随算使机器人从当前位置移动的目标位置和姿态（target， target_direction）：

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

## 4. 作者
- [肖镇龙](https://github.com/xzlxiao)

## 5. 贡献
贡献者们：

- [肖镇龙](https://github.com/xzlxiao)
- [李昌浩](https://github.com/Pekachiu)
- 黄吉
- 如果您想为该项目做出贡献，请参阅CONTRIBUTING

[CONTRIBUTING.md]()

## 6. Todo List
- [Todo list](./ToDoList.md)

## 7. 许可证
该项目基于GPL(GNU General Public License)许可证。有关详细信息，请参阅LICENSE。

[LICENSE.md](./LICENSE)

