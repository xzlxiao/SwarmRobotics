from Simulation.ComRobot import ComRobot 


class ComRobotRandomMove(ComRobot):
    def __init__(self, pos):
        super(ComRobotRandomMove, self).__init__(pos)
        self.mObjectType = "ComRobotRandomMove"       # 用于标识当前物体类别

    def update(self):
        """Method to update the robot's state.

        If the robot has reached its target position, choose a new random target.
        Call the parent class update method.
        """
        
        # Check if the robot has reached its target position.
        if (self.pos == self.target).all():
            self.chooseRandomTarget()
        
        # Call the superclass update method.
        super().update()


    