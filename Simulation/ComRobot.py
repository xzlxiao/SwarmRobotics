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
from common import utils


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
        self.mPos = np.array(pos, dtype=np.float32)
        self.mTarget = np.array(self.mPos, dtype=np.float32)
        self.mId = ComRobot._robot_count
        self._mInformationState = 'global'       # 通信方式是global, local 或 no
        self.mCommunicationRange = 800
        self.mCommunicationRangeColor = "yellow"
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

    def setRobotType(self, robot_type='3D'):
        self.mRobotType = robot_type
        self.mProcessedInfo['Pos'] = {self.mId: self.pos}
    
    def setInformationState(self, state:str):
        """设置通信状态

        Args:
            state (str): 通信状态，global, local 或 no
        """        
        self._mInformationState = state 

    def getInformationState(self):
        return self._mInformationState

    @property
    def robot_count(self):
        return ComRobot._robot_count

    def setMaxComNum(self, num):
        self.mMaxComNum = num

    def update(self):
        """

        :return:
        """
        self.sense()
        if self.isCommunicating:
            self.processInfo()
        self.move()

    def move(self):
        super().move()

    def communicateWith(self, robot):
        if self.isCommunicating:
            robot.mInfo.append(self.mProcessedInfo)

    def getGlobalPopulations(self):
        """获得全局的所有同伴
        """        
        return getObjectByType(self.mObjectType)

    def processInfo(self):
        '''
        通信信息处理，如一致性
        '''
        ######## start 这里写一致性等通信算法
        for robot_type in self.mProcessedInfo.keys():
            dict_list = []
            if len(self.mInfo) == 0:
                return
            for item in self.mInfo:
                dict_list.append(copy.deepcopy(item[robot_type]))
            dict_list.append(copy.deepcopy(self.mProcessedInfo[robot_type]))
            neighbor_robot_available_id = ComRobot.getAllKeysInDict(dict_list)
            if len(neighbor_robot_available_id) == 0:
                continue
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
        
        self.mProcessedInfo['Pos'][self.mId] = copy.deepcopy(self.pos)
        
        for sense_obj in self.mSenseInfo:
            self.mProcessedInfo[sense_obj.type][sense_obj.id] = copy.deepcopy(sense_obj.pos)
        
        self.mSenseInfo.clear()
        self.mProcessedInfoRecorder.append(copy.deepcopy(self.mProcessedInfo))
        self.mInfo.clear()
        

    @staticmethod
    def getAllKeysInDict(dict_list: list):
        keys_list = []
        for dict_item in dict_list:
            dict_item: dict
            for key_item in dict_item.keys():
                if key_item not in keys_list:
                    keys_list.append(key_item)
        return keys_list

    def draw(self, ax):
        super().draw(ax)
        # self.drawOnFigure(ax)
        if self.isDrawCommunicationRange and self.isCommunicating:
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            x = self.mCommunicationRange * np.outer(np.cos(u), np.sin(v)) + self.mPos[0]
            y = self.mCommunicationRange * np.outer(np.sin(u), np.sin(v)) + self.mPos[1]
            z = self.mCommunicationRange * np.outer(np.ones(np.size(u)), np.cos(v)) + self.mPos[2]
            if self.mCommunicationRangeType == 0:
                ax.plot_surface(x, y, z, color=self.mCommunicationRangeColor, alpha=self.mCommunicationRangeAlpha)
            elif self.mCommunicationRangeType == 1:
                ax.plot_wireframe(x, y, z, color=self.mCommunicationRangeColor, alpha=self.mCommunicationRangeAlpha, rstride=self.mWireframeRstride, cstride=self.mWireframeCstride)
        
        if self.isShowSenseRange:
            # angle = self.mDirection
            angle_min = self.mDirection - self.mSenseAngle/2
            angle_max = self.mDirection + self.mSenseAngle/2
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
            # pt_straight = np.matmul(pt, ComObject.getRotationMat(angle))+self.mPos[0:2]
            # ax.plot((self.mPos[0], pt_straight[0]), (self.mPos[1], pt_straight[1]), (self.mPos[2], self.mPos[2]), 'r:')

            if self.mStage.mStageType == '3D':
                ax.plot((self.pos[0], pts[0][0]), (self.pos[1], pts[0][1]), (self.pos[2], self.pos[2]), 'r:')
                ax.plot((self.pos[0], pts[-1][0]), (self.pos[1], pts[-1][1]), (self.pos[2], self.pos[2]), 'r:')
                ax.plot(x, y, z, 'r:')
            else: 
                ax.plot((self.pos[0], pts[0][0]), (self.pos[1], pts[0][1]), 'r:')
                ax.plot((self.pos[0], pts[-1][0]), (self.pos[1], pts[-1][1]), 'r:')
                ax.plot(x, y, 'r:')
            
    def getObjectBySight(self):
        '''
        获得感知范围内的机器人
        '''
        obj_in_sense_length = None
        obj_in_sense_length = getObjectInRange(self.mPos, self.mSenseDistance)      # 感知距离以内的机器人
        
        # 挑选出感知夹角以内的机器人，即xy平面内的偏航角度正负self.mSenseAngle以内，与xy平面的夹角正负self.mSenseAngle以内
        for robot in obj_in_sense_length:
            angle_in_xy = ComObject.getAngleBetweenXandVector(robot.pos, self.pos, plat='xy') - self.mDirection     # xy平面内的偏航角度
            angle_with_xy = ComObject.getAngleBetweenXandVector(robot.pos, self.pos, plat='o-xy')   # 与xy平面的夹角
            if angle_in_xy > math.pi:
                angle_in_xy = 2*math.pi - angle_in_xy
            if angle_in_xy < -math.pi:
                angle_in_xy = 2*math.pi + angle_in_xy
            if angle_with_xy > math.pi:
                angle_with_xy = 2*math.pi - angle_with_xy
            if angle_with_xy < -math.pi:
                angle_with_xy = 2*math.pi + angle_with_xy

            is_robot_sensed = False
            if self.mRobotType == '3D':
                if angle_in_xy >= -self.mSenseAngle and angle_in_xy <= self.mSenseAngle:
                    if angle_with_xy >= -self.mSenseAngle and angle_with_xy <= self.mSenseAngle:
                        is_robot_sensed = True
            elif self.mRobotType == '2D':
                if angle_in_xy >= -self.mSenseAngle and angle_in_xy <= self.mSenseAngle:
                    is_robot_sensed = True

            if is_robot_sensed:
                sense_obj = ObjectInfo()
                sense_obj.id = robot.mId
                sense_obj.type = robot.mObjectType
                sense_obj.pos = robot.pos
                self.mSenseInfo.append(sense_obj)

    def sense(self):
        super().sense()
        self.getObjectBySight()
            

    def setSenseDistance(self, dist):
        self.mSenseDistance = dist

    def setCommuntcationMethod(self, com_method: CommunicateMethods):
        self.isRandomCom = False
        self.mComMethod = com_method

    def getRobotById(self, id):
        for robot in ComRobot._robot_list:
            if robot.mId == id:
                return robot