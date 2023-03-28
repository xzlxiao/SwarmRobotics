#  群体机器人（SwarmRobotics）

该项目是一个使用Python和Matplotlib实现的Swarm Robotics仿真平台。它允许用户创建自定义的Swarm Robotics算法，并在仿真环境中进行测试和评估。
英文版是机器翻译，中文版readme更为可读，建议主要参考中文版。
自动翻译脚本为：

```bash
python Tools/translate_readme.py
```

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

- 3D路径规划（势场路径规划算法）
```bash
python Example/exm_路径规划-3D.py
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


## 3. 示例
- 下面是一个简单的示例，它使用随机游走算法模拟一个Swarm Robotics系统：

```python
from swarm import Swarm
from algorithms import RandomWalk
swarm = Swarm(algorithm=RandomWalk)
swarm.run()
swarm.animate()
```

## 4. 作者
- [肖镇龙](https://github.com/xzlxiao)

## 5. 贡献
贡献者们：

- [肖镇龙](https://github.com/xzlxiao)
- [李昌浩](https://github.com/Pekachiu)
- 如果您想为该项目做出贡献，请参阅CONTRIBUTING

[CONTRIBUTING.md]()

## 6. 许可证
该项目基于GPL(GNU General Public License)许可证。有关详细信息，请参阅LICENSE。

[LICENSE.md](./LICENSE)