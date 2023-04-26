

class ComModuleBase:
    def __init__(self, robot=None) -> None:
        """
        Constructor for the RobotController class.
        
        Args:
            robot (Robot): an instance of the Robot class. Defaults to None.
        
        Returns:
            None
        """
        
        self.mRobot = robot


    def update(self):
        pass