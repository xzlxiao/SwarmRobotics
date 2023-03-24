# SwarmRobotics

This project is a Swarm Robotics simulation platform implemented using Python and Matplotlib. It allows users to create custom Swarm Robotics algorithms and test and evaluate them in a simulated environment.

[English](./README.md)|[中文](./readme_chinese.md)

## Target Platform

- Linux
- OSX
- windows10

## Installation

To install this project, you need to follow these steps:

1. Clone the project:

```bash
git clone https://github.com/[your-username]/[your-project].git
```

2. Navigate to the project directory:
```bash
cd [your-project]
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage
To run the Swarm Robotics simulation platform, execute the following command:

```bash
python main.py
```

In the GUI, you can select the algorithm, environment, and simulation parameters and then launch the simulation. During the simulation, you can observe the behavior of the Swarm Robotics system and make real-time adjustments to the algorithm.

## Example
Here's a simple example that uses a random walk algorithm to simulate a Swarm Robotics system:

```python
from swarm import Swarm
from algorithms import RandomWalk

swarm = Swarm(algorithm=RandomWalk)
swarm.run()
swarm.animate()
```

## Authors
- [肖镇龙](https://github.com/username)

## Contributing
Contributors:

- [Contributor Name](https://github.com/contributor-username)
If you want to contribute to this project, please see[CONTRIBUTING.md]().

## License
This project is licensed under the MIT License. For more information, see[LICENSE]().