from Simulation.ComRobot import ComRobot 


class ComRobotRandomMove(ComRobot):
    def __init__(self, pos):
        super(ComRobotRandomMove, self).__init__(pos)
        self.mObjectType = "ComRobotRandomMove"       # 用于标识当前物体类别

    def update(self):
        if (self.pos == self.target).all():
            self.chooseRandomTarget()
        super().update()

    