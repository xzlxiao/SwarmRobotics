
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Simulation.ComModuleBase import ComModuleBase
from Simulation.ComObjectCollection import *
import random
from Common.utils import distance

class ComModuleDWorld(ComModuleBase):
    def __init__(self, robot=None) -> None:
        """
        Constructor for the RobotController class.
        
        Args:
            robot (Robot): an instance of the Robot class. Defaults to None.
        
        Returns:
            None
        """
        
        super().__init__(robot=robot)
        # number of times to retry a communication request if it fails
        self.query_num = 10
        # probability of initiating a new communication request
        self.query_posibility = 0.01
        # average number of links per node
        self.k = 6
        # maximum number of links per node
        self.links_max = 10
        # minimum number of links per node
        self.links_min = 2
        # target decay time
        self.T1 = 100.0
        # distance influence weight
        self.d_i = 10.0
        # time interval between steps
        self.epsilon = 0.03
        # reconnection probability
        self.alpha = 0.1
        # dictionary holding communication stability values for each connected robot
        self.mCommunicationSigma = {}
        # dictionary holding last reset time for communication stability values for each connected robot
        self.mCommunicationTime = {}
        # reference to currently available robot for communication
        self.availableRobot = None
        # list to hold edge values of the graph
        self.edges_value = []

    def setRobot(self, robot):
        """
        Sets the robot that owns this module.

        Args:
            robot (Robot): the robot object that owns this module.

        Returns:
            None
        """
        self.mRobot = robot


    def setAvailableComRobots(self, robots: np.ndarray):
        """
        Sets the robots available within communication range.

        Args:
            robots (np.ndarray): numpy array containing the robots available in the communication range.

        Returns:
            None
        """
        self.availableRobot = robots

    
    # This function updates the communication objects and parameters of a robot

    def update(self):
        """
        This function updates the communication objects of a robot and determines their communication capabilities.

        Args:
            None

        Returns:
           list: a list of sigma values for each neighbor
        """
        # Calls the parent class's update function
        super().update()
        
        # Initializes variables new_robot_partner_id and new_robot to None
        new_robot_partner_id = None
        new_robot = None
        
        # Removes communication objects that are not in range
        self.removeCommunicatonObjectNotInRange()
        
        # If the number of communicating neighbors is less than the average, 
        # then check whether a request can be sent based on the probability or minimum number of links required
        if self.getNumOfComNeighbors() < self.k:
            if random.random() < self.query_posibility or self.getNumOfComNeighbors() <= self.links_min:
                # Try several times to find an available robot
                for i in range(self.query_num):
                    if self.availableRobot is not None:
                        availableRobotList = self.availableRobot.tolist()
                        if len(availableRobotList) > 0:
                            # Choose a random robot partner id that is not already among the neighbors
                            new_robot_partner_id = random.sample(self.availableRobot.tolist(), k=1)[0]
                            if new_robot_partner_id != self.mRobot.mId and new_robot_partner_id not in self.getComNeighborsId():
                                # Add a communication object for the chosen robot partner
                                new_robot = self.mRobot.getRobotById(new_robot_partner_id)
                                if new_robot.mNetworkModule is None:
                                    self.addCommunicatonObject(new_robot_partner_id)
                                    break
                                else:
                                    if not new_robot.mNetworkModule.reachMaxComNum():
                                        self.addCommunicatonObject(new_robot_partner_id)
                                        break
            
            # Calculates the communication time, sigma values, and removes the communication object with a low sigma value
            robot_ids = self.getComNeighborsId()
            for robot_id in robot_ids:
                self.mCommunicationTime[robot_id] += self.epsilon
                t = self.mCommunicationTime[robot_id]
                decay_robot = self.mRobot.getRobotById(robot_id)
                
                # Calculates the distance between the two robots and sets the communication range T based on the distance
                p1_p2_dist = distance(self.mRobot.mPos,  decay_robot.mPos)
                T = self.T1 / (1 + (p1_p2_dist * self.d_i))
                self.mCommunicationSigma[robot_id] = 0.5**(t/T)
                
                # Removes the communication object with a low sigma value
                # Resets communication time parameters
                if random.random() < self.alpha * (1 - self.mCommunicationSigma[robot_id]) * self.epsilon:
                    for robot_id_iter in self.mCommunicationTime.keys():
                        self.mCommunicationTime[robot_id_iter] = 0
                    for robot_id_iter in decay_robot.mNetworkModule.mCommunicationTime.keys():
                        decay_robot.mNetworkModule.mCommunicationTime[robot_id_iter] = 0
                    self.removeCommunicatonObject(robot_id)

        # Returns the list of sigma values for each neighbor
        return self.mCommunicationSigma


    def getNumOfComNeighbors(self) -> int:
        """
        Returns the number of currently communicating neighbors.

        This function retrieves the count of neighbors with whom communication is currently active,
        and returns the integer value representing it.

        Returns:
            int: The number of neighbors who are currently communicating.
        """
        return len(self.mCommunicationSigma)


    def getComNeighborsId(self):
        """
        Return a list of IDs of neighbors currently being communicated with.

        This function retrieves the keys (neighbor IDs) of the current communication sigma,
        which is a dictionary containing information about all neighbors currently being
        communicated with. It then converts these keys to a list and returns the result.

        Returns:
            list: A list of IDs (keys) of neighbors being communicated with.
        """
        return list(self.mCommunicationSigma.keys())


    def addCommunicatonObject(self, robot_id):
        """
        Add a new robot to the list of communication objects.

        This function adds a new robot to the communication sigma and time dictionaries,
        which keep track of information about each robot currently being communicated with.
        The robot ID is used as a key, and a default value is assigned to each dictionary.

        Args:
            robot_id (int): The ID of the robot to add to the communication objects.
        """
        self.mCommunicationSigma[robot_id] = 1.0   # Set the initial sigma value for the new robot to 1.0
        self.mCommunicationTime[robot_id] = 0.0    # Set the initial communication time for the new robot to 0.0

    
    def removeCommunicatonObject(self, robot_id):
        """Remove a robot from the list of communication objects.

        This function removes a robot from the communication sigma and time dictionaries,
        using the provided robot ID as a key. If the ID is not found in the dictionaries,
        a KeyError will be raised.

        Args:
            robot_id (int): The ID of the robot to remove from the communication objects.
        """
        self.mCommunicationSigma.pop(robot_id)   # Remove the specified robot ID from the communication sigma dictionary
        self.mCommunicationTime.pop(robot_id)    # Remove the specified robot ID from the communication time dictionary


    def removeCommunicatonObjectNotInRange(self):
        """Remove robots that are not within range from the list of communication objects.

        This function iterates through the list of robot IDs in the communication sigma dictionary,
        and checks if each ID is also present in the availableRobot list. If the ID is not in the 
        availableRobot list, it means the robot is out of range and should be removed from the 
        communication objects. The function calls removeCommunicatonObject(robot_id) to remove
        the robot ID from both the communication sigma and time dictionaries.

        Args:
            None.
        """
        robot_ids = list(self.mCommunicationSigma.keys())  # Get a list of all robot IDs in the communication sigma dictionary
        for robot_id in robot_ids:                         # Iterate through each robot ID
            if robot_id not in self.availableRobot:        # Check if the robot ID is not in the availableRobot list
                self.removeCommunicatonObject(robot_id)    # If the robot is out of range, remove its ID from the communication objects


    def reachMaxComNum(self) -> bool:
        """Check if the number of communication neighbors has reached the maximum limit.

        This function calls getNumOfComNeighbors() to determine the number of communication neighbors
        the robot currently has. If this number is less than the maximum allowed by links_max, the 
        function returns False to indicate that the maximum limit has not been reached. Otherwise,
        the function returns True to indicate that the maximum limit has been reached.

        Args:
            None.

        Returns:
            bool: True if the number of communication neighbors has reached the max limit, 
                False otherwise.
        """
        if self.getNumOfComNeighbors() < self.links_max:  # Check if the current number of communication neighbors is less than the max limit
            return False                                  # Return False to indicate that the max limit has not been reached
        else:
            return True                                   # Return True to indicate that the max limit has been reached
