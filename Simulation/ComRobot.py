# -*- coding: utf-8 -*-

import enum
from networkx.algorithms.assortativity import neighbor_degree
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random
from Simulation.ComObject import ComObject 
import math
import copy
from Simulation.ComObjectCollection import *
from Common import utils


class CommunicateMethods(enum.Enum):
    AllCom = 1      # 与通信范围内所有机器人通信
    WS_net = 2      # Watts and Strogatz小世界网络模型
    D_world = 3
    NearestCom = 4  # 优先与最近的机器人通信

class ObjectInfo:
    def __init__(self) -> None:
        self.type = None 
        self.id = None 
        self.pos = None 

class ComRobot(ComObject):
    _robot_count = 0
    _robot_list = []
    def __init__(self, pos):
        super(ComRobot, self).__init__()
        self.mPos = np.array(pos, dtype=float)
        self.mTarget = np.array(self.mPos, dtype=float)
        self.mId = ComRobot._robot_count
        self._mInformationState = 'global'       # 通信方式是global, local 或 no
        self.mCommunicationRange = 800
        self.mCommunicationRangeColor = "grey"
        self.mCommunicationRangeAlpha = 0.03
        self.mCommunicationRangeType = 0         # 0: surface 1: wireframe
        self.mNetworkModule = None
        self.mWireframeRstride = 5
        self.mWireframeCstride = 5
        self.isDrawCommunicationRange = True 
        self.mSenseDistance = 800
        self.mSenseAngle = math.pi/2
        self.mMaxComNum = 6         # 最大通信数量
        self.isRandomCom = True    # 如果通信范围内机器人数量大于mMaxComNum，是随机mMaxComNum个机器人通信，还是最近的mMaxComNum个机器人通信
        self.mComMethod = CommunicateMethods.NearestCom
        self.isShowSenseRange = True
        ComRobot._robot_count += 1
        ComRobot._robot_list.append(self)
        self.setShape('ship')
        self.mInfo = []             # 接收到的信息
        self.mProcessedInfo = {
            'Pos': {self.mId: self.pos},
            'ComFish': {},
            'ComRobot': {},
            'ComObject': {},
            'ComRobotAF': {},
            'ComRobotRandomMove': {},

            "ComRobotAF_MAC": {}
        }    # 处理后的信息
        self.mSenseInfo = []
        self.mProcessedInfoRecorder = []
        self.mObjectType = "ComRobot"       # 用于标识当前物体类别
        self.isCommunicating = True

        self.isSensable = True  # 是否具有感知功能

    def setSensable(self, state=True):
        """Sets the sensibility of the robot to perceive its environment.

        Args:
            state (bool): True if the robot is sensable, False otherwise.
        """
        self.isSensable = state

    def setRobotType(self, robot_type='3D'):
        """Sets the type of the robot.

        Args:
            robot_type (str, optional): The type of the robot. Defaults to '3D'.
        """
        self.mRobotType = robot_type
        self.mProcessedInfo['Pos'] = {self.mId: self.pos}
    
    def setInformationState(self, state:str):
        """Sets the communication state of the robot.

        Args:
            state (str): communication state - "global", "local" or "no"
        """        
        self._mInformationState = state 

    def getInformationState(self):
        """Gets the communication state of the robot.

        Returns:
            str: communication state - "global", "local" or "no"
        """
        return self._mInformationState

    @property
    def robot_count(self):
        """Gets the count of robots created.

        Returns:
            int: the total number of robots created.
        """
        return ComRobot._robot_count

    def setMaxComNum(self, num):
        """Sets the maximum number of robots that can establish communication with this robot.

        Args:
            num (int): Maximum number of robots that can communicate with this robot.
        """
        self.mMaxComNum = num

    def update(self):
        """Updates robotics object after a certain interval.
        """
        self.sense()
        self.processInfo()

        self.move()

    def move(self):
        """Moves the robot to a new position by calling the superclass method.
        """    
        super().move()

    def communicateWith(self, robot):
        """Enables communication between two robots.

        Args:
            robot (Robot): Another Robot object to communicate with.
        """
        if self.isCommunicating:
            robot.mInfo.append(self.mProcessedInfo)

    def getGlobalPopulations(self):
        """Gets all available global populations.

        Returns:
            list: A list of all available global populations.
        """ 
        return getObjectByType(self.mObjectType)

    def processInfo(self):
        '''
        Communication information processing, such as consensus.
        '''
        
        ######## Start of algorithm for communication consensus and other communication details.
        for robot_type in self.mProcessedInfo.keys():
            dict_list = []
            
            # If there is no information available, update processed info with sensed information and return.
            if len(self.mInfo) == 0:
                for sense_obj in self.mSenseInfo:
                    if sense_obj.type not in self.mProcessedInfo.keys():
                        self.mProcessedInfo[sense_obj.type] = {}
                    self.mProcessedInfo[sense_obj.type][sense_obj.id] = copy.deepcopy(sense_obj.pos)
                self.mSenseInfo.clear()
                return
            
            # Create a list of dictionaries containing positional data from other robots and the current robot.
            for item in self.mInfo:
                dict_list.append(copy.deepcopy(item[robot_type]))
            dict_list.append(copy.deepcopy(self.mProcessedInfo[robot_type]))
            
            # Get a list of available IDs for neighbor robots that have information about this robot.
            neighbor_robot_available_id = ComRobot.getAllKeysInDict(dict_list)
            
            # If no available IDs are found, move to next robot type.
            if len(neighbor_robot_available_id) == 0:
                continue

            # Calculate the average position of the different neighbors based on their individual positions.
            for robot_id in neighbor_robot_available_id:
                varPosInfo = 0
                count = 0
                for info_item in self.mInfo:
                    if robot_id in info_item[robot_type].keys():
                        count += 1
                        if robot_id in self.mProcessedInfo[robot_type].keys():
                            varPosInfo += utils.two_dim_to_three_dim(info_item[robot_type][robot_id]) - utils.two_dim_to_three_dim(self.mProcessedInfo[robot_type][robot_id])
                        else:
                            info_pos = utils.two_dim_to_three_dim(info_item[robot_type][robot_id])
                            varPosInfo += info_pos
                if count != 0:
                    if robot_id in self.mProcessedInfo[robot_type].keys():
                        self.mProcessedInfo[robot_type][robot_id] = utils.two_dim_to_three_dim(self.mProcessedInfo[robot_type][robot_id])
                        self.mProcessedInfo[robot_type][robot_id] += varPosInfo / count
                    else:
                        self.mProcessedInfo[robot_type][robot_id] = varPosInfo / count
        ######## end
        
        # Update the current robot's position in processed information.
        self.mProcessedInfo['Pos'][self.mId] = copy.deepcopy(self.pos)
        
        # Update processed information with recently sensed information.
        for sense_obj in self.mSenseInfo:
            self.mProcessedInfo[sense_obj.type][sense_obj.id] = copy.deepcopy(sense_obj.pos)
        
        # Clear list of sensed information and add updated processed info to log.
        self.mSenseInfo.clear()
        self.mProcessedInfoRecorder.append(copy.deepcopy(self.mProcessedInfo))
        self.mInfo.clear()
        

    @staticmethod
    def getAllKeysInDict(dict_list: list):
        """
        Return a list containing all the unique keys in a list of dictionaries.
        
        Args:
        dict_list (list): List of dictionaries
        
        Returns:
        list : List of unique keys from dictionaries
        """
        keys_list = []
        for dict_item in dict_list:
            dict_item: dict
            
            # Iterate over each key in the dictionary 
            for key_item in dict_item.keys():
                
                # Append the key to the list if it's not already present
                if key_item not in keys_list:
                    keys_list.append(key_item)
        return keys_list
    
    def draw(self, ax):
        """
        Draw this object on the given matplotlib Axes.

        Args:
            ax (Axes): The matplotlib Axes to draw on.
        """

        # Call the superclass's draw method.
        super().draw(ax)

        # If communication range needs to be drawn and this object is communicating, draw it.
        if self.isDrawCommunicationRange and self.isCommunicating:
            
            # Create an array of angles around a circle.
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)

            # Create x, y, and z coordinates for the surface.
            x = self.mCommunicationRange * np.outer(np.cos(u), np.sin(v)) + self.mPos[0]
            y = self.mCommunicationRange * np.outer(np.sin(u), np.sin(v)) + self.mPos[1]
            z = self.mCommunicationRange * np.outer(np.ones(np.size(u)), np.cos(v)) + self.mPos[2]

            # Draw the communication range either as a surface or wireframe, depending on communication range type.
            if self.mCommunicationRangeType == 0:
                ax.plot_surface(x, y, z, color=self.mCommunicationRangeColor, alpha=self.mCommunicationRangeAlpha)
            elif self.mCommunicationRangeType == 1:
                ax.plot_wireframe(x, y, z, color=self.mCommunicationRangeColor, alpha=self.mCommunicationRangeAlpha, rstride=self.mWireframeRstride, cstride=self.mWireframeCstride)

        # If sense range needs to be shown and this object can sense something, show it.
        if self.isShowSenseRange and self.isSensable:
            
            # Get minimum and maximum angles of the sense range cone.
            angle_min = self.mDirection - self.mSenseAngle/2
            angle_max = self.mDirection + self.mSenseAngle/2

            # Divide the cone into equal parts and calculate their positions in space.
            angle_tmp = angle_min
            angles = []
            while angle_tmp < angle_max+0.1:
                angles.append(angle_tmp)
                angle_tmp += (angle_max - angle_min) / 5
            pt = np.array([self.mSenseDistance, 0.0])
            pts = []
            x = []
            y = []
            z = []
            for ang in angles:
                rotation_mat = ComObject.getRotationMat(ang)
                pts.append(np.matmul(pt, rotation_mat)+self.mPos[0:2])
                x.append(pts[-1][0])
                y.append(pts[-1][1])
                z.append(self.mPos[2])

            # Draw the sense range either as lines or a wireframe, depending on the dimensions of the stage.
            if self.mStage.mStageType == '3D':
                ax.plot((self.pos[0], pts[0][0]), (self.pos[1], pts[0][1]), (self.pos[2], self.pos[2]), 'r:')
                ax.plot((self.pos[0], pts[-1][0]), (self.pos[1], pts[-1][1]), (self.pos[2], self.pos[2]), 'r:')
                ax.plot(x, y, z, 'r:')
            else: 
                ax.plot((self.pos[0], pts[0][0]), (self.pos[1], pts[0][1]), 'r:')
                ax.plot((self.pos[0], pts[-1][0]), (self.pos[1], pts[-1][1]), 'r:')
                ax.plot(x, y, 'r:')
            
    # Define a method named "getObjectBySight" that belongs to an object
    def getObjectBySight(self):
        """
        This method returns the objects perceived within the robot's sensing range.

        Returns:
            A list containing the objects perceived by the robot.
        """

        obj_in_sense_length = None
        obj_in_sense_length = getObjectInRange(self.mPos, self.mSenseDistance)      # Get robots within perception range
        ret = []

        # Filter robots within the perception angle, i.e., within positive or negative self.mSenseAngle
        # with respect to the xy plane and within positive or negative self.mSenseAngle with respect
        # to the angle with the xy plane.
        for robot in obj_in_sense_length:
            angle_in_xy = ComObject.getAngleBetweenXandVector(self.pos, robot.pos, plat='xy') - self.mDirection     # The bearing angle of the robot in the xy plane
            angle_with_xy = ComObject.getAngleBetweenXandVector(self.pos, robot.pos, plat='o-xy')   # The angle with respect to the xy plane

            # Make sure the bearing angle is between -pi and pi
            if angle_in_xy > math.pi:
                angle_in_xy = 2*math.pi - angle_in_xy
            if angle_in_xy < -math.pi:
                angle_in_xy = 2*math.pi + angle_in_xy

            # Make sure the angle with respect to the xy plane is between -pi and pi
            if angle_with_xy > math.pi:
                angle_with_xy = 2*math.pi - angle_with_xy
            if angle_with_xy < -math.pi:
                angle_with_xy = 2*math.pi + angle_with_xy

            # Determine whether the robot is sensed
            is_robot_sensed = False
            if self.mRobotType == '3D':
                if angle_in_xy >= -self.mSenseAngle/2 and angle_in_xy <= self.mSenseAngle/2:
                    if angle_with_xy >= -self.mSenseAngle/2 and angle_with_xy <= self.mSenseAngle/2:
                        is_robot_sensed = True
            elif self.mRobotType == '2D':
                if angle_in_xy >= -self.mSenseAngle/2 and angle_in_xy <= self.mSenseAngle/2:
                    is_robot_sensed = True 

            # If the robot is sensed, add it to the response list
            if is_robot_sensed:
                sense_obj = ObjectInfo()
                sense_obj.id = robot.mId
                sense_obj.type = robot.mObjectType
                sense_obj.pos = robot.pos
                self.mSenseInfo.append(sense_obj)
                ret.append(robot)

        # Return the list of robots that were sensed
        return ret
    


    # Define a method named "sense" that belongs to an object.
    def sense(self):
        """
        This method is used to make the robot sense the environment around it. 

        Returns:
        None
        """
        # If the robot can sense, call the parent class's sense method 
        # and then get the objects perceived by the robot.
        if self.isSensable:
            super().sense()
            self.getObjectBySight()
            

    # Define a method named "setSenseDistance" that sets the sensing distance of the robot.
    def setSenseDistance(self, dist):
        """
        This method is used to set the sensing distance for the robot.

        Args:
        dist (float): The desired sensing distance for the robot.

        Returns:
        None
        """
        self.mSenseDistance = dist

    # Define a method named "setCommunicationMethod" that sets the communication 
    # method of the robot using an enum type named "CommunicateMethods".
    def setCommuntcationMethod(self, com_method: CommunicateMethods):
        """
        This method is used to set the communication method for the robot.

        Args:
        com_method (enum): A member of the CommunicateMethods enum.

        Returns:
        None
        """
        self.isRandomCom = False
        self.mComMethod = com_method

    # Define a method named "getRobotById" that returns the robot with the specified id.
    def getRobotById(self, id):
        """
        This method is used to return the robot with the specified ID.

        Args:
        id (int): The ID of the robot to look for.

        Returns:
        ComRobot object: The robot with the specified ID, or None if it is not found.
        """
        for robot in ComRobot._robot_list:
            if robot.mId == id:
                return robot
