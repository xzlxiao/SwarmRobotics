# -*- coding: utf-8 -*-

import copy
import math
import random

isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Simulation.ComRobotAF import ComRobotAF
from Simulation.ComRobot import ComRobot
from Simulation.ComObjectCollection import *
from Simulation.ComObject import ComObject


class VLCMsg:
    def __init__(self, sourceId, targetId=-1, targetBox=-1, isBroadcasting=False, isFlooding=False):
        self.sourceId = sourceId

        self.targetId = targetId
        self.targetBox = targetBox
        self.linkPath = []

        self.isBroadcasting = isBroadcasting
        self.isFlooding = isFlooding
        self.seq = -1
        self.isSetTransLimit = False
        self.transLimit = -1
        self.transNum = 0

        self.payload = {}


class ComRobotAF_MAC(ComRobotAF):
    def __init__(self, pos, stage):
        super().__init__(pos)
        self.slots = set()  # 每个机器人占用的槽索引
        self.status = 'entering'  # 几种可能：online-pre-在群体中以预备槽工作，online-在群体中正常工作，entering/leaving-机器人在进入或者离开群体

        self.stage = stage  # ComStage_MAC对象

        self.in_robots = []  # 入邻居
        self.out_robots = []  # 出邻居
        self.robots_inRs = []  # 感知范围内的所有机器人

        self.idsNeedKnow = []  # 记录我们机器人需要在知道的其他机器人信息的id：[‘ALL’]表示全地图都要掌握

        self.sendMsg = []  # 发送槽信息集合,发送缓存
        self.resFlag = 0
        self.refResDict = {}

        self.hasSendReqFlag = 0  # 机器人准备接入网络时发送一次请求信息
        self.hasSendLeaFlag = 0  # 机器人离开网络发送最后消息包，0未离开，1离开未发送，2离开已发送
        self.hasSlotChangedFlag = 0  # 机器人的槽信息变化了，向网络中洪泛。

        self.globalAG = np.mat(np.zeros((50, 50)))  # 每个机器人维护一个全局的表

        self.acsInfo = []  # 接收信息存放，raw data，接收缓存
        self.p_acsInfo = {  # 处理过后的存储在机器人本地的信息
            self.mId: {'type': '', 'slots': copy.deepcopy(self.slots)},
        }
        self.p_acsInfoRecorder = []  # 记录历史值

        self.floodSeq = 0  # 记录当前已经发了多少个洪泛包了
        self.floodHistory = {}  # 记录接收到的洪泛包的源id与seq值，防止循环洪泛。

        self.mObjectType = "ComRobotAF_MAC"  # 用于标识当前物体类别

        # 以下是测试指标所需的一些变量
        self.startAccessFlag = 0  # 开始接入
        self.accessCount = 0  # 接入所花费的时间
        self.endAccessFlag = 0  # 接入成功

        # 以下变量为 布置机器人位置时所用，为了方便而写的，没啥大用
        self.initDirection = 0.0
        self.flag1 = 0

    def refresh(self):
        """
        Clears attributes related to robot communication and sensing.

        Args:
            self (object): The object calling this method.

        Returns:
            None.
        """
        
        # Clear the list of robots within communication range
        self.robots_inRs.clear()
        
        # Clear the lists of robots within sensor range and the ACS information
        self.in_robots.clear()
        self.acsInfo.clear()
        
        # Clear the list of robots that are communicating with this robot
        self.out_robots.clear()


    def communicateWith(self, robot, nowSlot):
        """
        Communicates with a specified robot during the current timeslot.

        If this robot is actively communicating with others and the current timeslot is within its specified communication slots,
        the message from this robot is appended to the specified robot's ACS information list.
        Additionally, this robot's processed information is appended to the specified robot's `mInfo` attribute.

        Args:
            self (object): The robot object calling this method.
            robot (object): The robot to communicate with.
            nowSlot (int): The current timeslot.

        Returns:
            None.
        """
        
        # Check if this robot is actively communicating and if the current timeslot is within its specified communication slots
        if self.isCommunicating and (nowSlot in self.slots):
            
            # Append this robot's message to the specified robot's ACS information
            robot.acsInfo.extend(self.sendMsg)
            
            # Append this robot's processed information to the specified robot's `mInfo` attribute
            robot.mInfo.append(self.mProcessedInfo)


    def sense(self):
        """
        Updates the agent's fitness, food, and population information based on its position and environment objects.

        Calculates robots within the communication range and robots within the sensing range.
        Selects robots within the sensing angle and updates their processed information in the `mProcessedInfo` dictionary.
        Stores the selected robots to be communicated with in the `out_robots` attribute.

        Args:
            self (object): The robot object calling this method.

        Returns:
            None.
        """
        
        # Get all robots within sensing range
        obj_in_sense_length = getObjectInRange(self.mPos, self.mCommunicationRange)
        
        # Remove this robot from the list of robots within sensing range
        obj_in_sense_length.remove(self)
        
        # Return if there are no robots within sensing range
        if len(obj_in_sense_length) == 0:
            return
        
        # Store robots within sensing range in the `robots_inRs` attribute
        self.robots_inRs = obj_in_sense_length
        
        # Determine which IDs of robots within sensing range need to be known
        if self.stage.macProtocol == 'C-CMSTDMA':
            self.idsNeedKnow = [obj.mID for obj in obj_in_sense_length]
        else:
            self.idsNeedKnow = ['ALL']
        
        # Get all robots within communication range
        obj_in_comm_length = getObjectInRange(self.mPos, self.mSenseDistance)
        
        # Remove this robot from the list of robots within communication range
        obj_in_comm_length.remove(self)
        
        # Return if there are no robots within communication range
        if len(obj_in_comm_length) == 0:
            return
        
        # Select robots within the sensing angle and update their processed information in the `mProcessedInfo` dictionary
        for robot in obj_in_comm_length:
            angle_in_xy = ComObject.getAngleBetweenXandVector(self.mPos, robot.mPos, plat='xy') - self.mDirection
            angle_with_xy = ComObject.getAngleBetweenXandVector(robot.mPos, self.mPos, plat='o-xy') 
            if angle_in_xy > math.pi:
                angle_in_xy = 2*math.pi - angle_in_xy
            if angle_in_xy < -math.pi:
                angle_in_xy = 2*math.pi + angle_in_xy
            if (-self.mSenseAngle/2) <= angle_in_xy <= (self.mSenseAngle/2) and \
                    (-self.mSenseAngle/2) <= angle_with_xy <= (self.mSenseAngle/2):
                self.mProcessedInfo[robot.mObjectType][robot.mId] = copy.deepcopy(robot.mPos)
                self.out_robots.append(robot)


    '''*********************数据接收和解析部分**************************************'''
    def getNetworkInfo(self):
        """
        Gets network information from a single input robot and updates its own network information accordingly.

        If this robot has no in-neighbor robots, returns without doing anything.
        Otherwise, gets the network information from the first in-neighbor robot.
        Deep copies the in-neighbor robot's ACS information and global adjacency matrix to its `p_acsInfo` and `globalAG` attributes, respectively.
        Updates its own `globalAG` attribute to include its in-neighbor robots.
        Finally, updates its own ACS information to include its own information.

        Args:
            self (object): The robot object calling this method.

        Returns:
            None.
        """
        
        # If this robot does not have any in-neighbor robots, cannot get network information
        if len(self.in_robots) < 1:
            return
        
        # Get network information from the first in-neighbor robot
        robot = self.in_robots[0]
        
        # Deep copy the in-neighbor robot's ACS information and global adjacency matrix to its `p_acsInfo` and `globalAG` attributes, respectively
        self.p_acsInfo = copy.deepcopy(robot.p_acsInfo)
        self.globalAG = copy.deepcopy(robot.globalAG)

        # Update its own `globalAG` attribute to include its in-neighbor robots
        self.globalAG[:, self.mId] = 0
        innbs = [rbt.mId for rbt in self.in_robots]
        for innb in innbs:
            self.globalAG[innb, self.mId] = 1

        # Update its own ACS information to include its own information
        self.p_acsInfo[self.mId] = {'type': '', 'slots': copy.deepcopy(self.slots)}


    def updateSlots(self):
        """
        Updates the robot's `slots` attribute based on its current `status`.

        If the robot is in "entering" status, returns without doing anything.
        If the robot is in "leaving" status, removes itself from all of its slots and clears the `slots` attribute.
        If the robot is in "online-pre" status:
            - If it does not have any slots and its ACS information does not have any slots, adds a random slot to its `slots`
            - If it does not have any slots but its ACS infromation does have slots, sets its `slots` attribute to equal the ACS information's `slots`, changes its status to "online", and prints a message indicating the robot has successfully accessed the network
            - If it has slots and its ACS information also has slots, updates its `slots` to equal the ACS information's `slots` and sets its status to "online"
        If the robot is in "online" status:
            - If its `slots` attribute equals its ACS information's `slots` attribute, returns without doing anything
            - If its `slots` attribute does not equal its ACS information's `slots` attribute, updates its `slots` attribute and corresponding slots in `stage.rbtID_Time`

        Args:
            self (object): The robot object calling this method.

        Returns:
            None.
        """

        # If the robot is entering, do nothing
        if self.status == 'entering':
            return

        # If the robot is leaving, remove it from all of its slots and clear its `slots` attribute
        if self.status == 'leaving':
            for s in self.slots:
                self.stage.rbtID_Time[s].remove(self.mId)
            self.slots.clear()
            return

        # If the robot is in "online-pre" status:
        if self.status == 'online-pre':

            # If the robot does not have any slots and its ACS information does not have any slots, add a random slot to its `slots`
            if not self.slots and not self.p_acsInfo[self.mId]['slots']:
                s = random.randint(1, self.stage.preSlots) + self.stage.formerSlots - 1
                self.slots.add(s)
                self.stage.rbtID_Time[s].add(self.mId)

            # If the robot does not have any slots but its ACS infromation does have slots, set its `slots` attribute to equal the ACS information's `slots`, change its status to "online", and print a message indicating the robot has successfully accessed the network
            elif not self.slots and self.p_acsInfo[self.mId]['slots']:
                for s in self.p_acsInfo[self.mId]['slots']:
                    self.slots.add(s)
                    self.stage.rbtID_Time[s].add(self.mId)
                self.status = 'online'
                self.hasSlotChangedFlag = 1

                self.endAccessFlag = 1  # 接入成功了
                self.startAccessFlag = 0
                print('*****new robot stop accessing***********')

            # If the robot has slots and its ACS information also has slots:
            elif self.slots and self.p_acsInfo[self.mId]['slots']:

                # If the robot's `slots` attribute does not only contain slots in the pre-access phase, update its attribute
                if not (len(self.slots) == 1 and list(self.slots)[0] in range(self.stage.formerSlots, self.stage.formerSlots + self.stage.preSlots)):
                    self.hasSlotChangedFlag = 1
                
                # Remove the robot from its current slots and update its `slots` attribute to equal the ACS information's `slots`
                for s in self.slots:
                    self.stage.rbtID_Time[s].remove(self.mId)
                self.slots.clear()
                for s in self.p_acsInfo[self.mId]['slots']:
                    self.slots.add(s)
                    self.stage.rbtID_Time[s].add(self.mId)

                # Change the robot's status to "online" and print a message indicating the robot has successfully accessed the network
                self.status = 'online'
                self.endAccessFlag = 1  # 接入成功了
                self.startAccessFlag = 0
                print('*****new robot stop accessing***********')

            # If the robot does not have any slots and its ACS information also does not have any slots, do nothing
            else:
                return

        # If the robot is in "online" status:
        if self.status == 'online':

            # If its `slots` attribute equals its ACS information's `slots` attribute, do nothing
            if self.slots == self.p_acsInfo[self.mId]['slots']:
                return
            
            # If its `slots` attribute does not equal its ACS information's `slots` attribute:
            else:

                # Remove the robot from all of its current slots and update its `slots` attribute to equal the ACS information's `slots`
                for s in self.slots:
                    self.stage.rbtID_Time[s].remove(self.mId)
                self.slots = copy.deepcopy(self.p_acsInfo[self.mId]['slots'])
                for s in self.slots:
                    self.stage.rbtID_Time[s].add(self.mId) 


    def updatePacsInfo(self, payload: dict):
        """This function updates the PACS (Parallel Access Channel System) information based on the payload received.
        
        Args:
            payload (dict): The payload received containing updated PACS information.
        """
        if 'REQ' in payload.keys():
            # If a request message is received
            sid = payload['REQ'][0]
            tarr = copy.deepcopy(payload['REQ'][1])
            self.resFlag = 1
            self.refResDict[sid] = tarr
            return
            
        if 'NEIGHBORS' in payload.keys():
            # If a neighbor message is received
            sid = payload['NEIGHBORS'][0]
            innbs = copy.deepcopy(payload['NEIGHBORS'][1])
            self.globalAG[:, sid] = 0
            for innb in innbs:
                self.globalAG[innb, sid] = 1
            return

        # If a slot message is received
        for id, val in payload.items():
            if val['type'] == 'slot-add':
                # Add slots to PACS information
                if id not in self.p_acsInfo.keys():
                    self.p_acsInfo[id] = {'type': '', 'slots': set()}
                for s in val['slots']:
                    self.p_acsInfo[id]['slots'].add(s)
            elif val['type'] == 'slot-rdu':
                # Remove slots from PACS information
                if id not in self.p_acsInfo.keys():
                    self.p_acsInfo[id] = {'type': '', 'slots': set()}
                for s in val['slots']:
                    self.p_acsInfo[id]['slots'].discard(s)
            elif val['type'] == 'slot-rpl':
                # Replace existing PACS information with new information
                self.p_acsInfo[id] = copy.deepcopy(val)
            else:
                # If message is not recognized
                print('VLCMsgError:: msg has no type, please check...')


    def decoderVLCMsg(self, msg, dcType):
        """This function receives messages and decodes them based on the delivery control type.
        
        Args:
            msg (_type_): The message received.
            dcType (_type_): The delivery control type.
        """
        if dcType == 'Broadcasting':
            # If delivery control is broadcasting, update PACS information
            self.updatePacsInfo(msg.payload)
            
        if dcType == 'Flooding':
            
            # Prevents forwarding messages that have exceeded their transmission limit
            if msg.isSetTransLimit and (msg.transNum > msg.transLimit):
                return
            
            # Prevents forwarding of a message if it was sent by itself
            if msg.sourceId == self.mId:
                # print('id:', self.mId, 'receive a cycle msg:', msg.payload)
                return

            if self.floodHistory.get(msg.sourceId) is None:
                # Update PACS information and add message to send buffer for forwarding
                self.floodHistory[msg.sourceId] = list(range(msg.seq + 1))
                self.updatePacsInfo(msg.payload)
                self.sendMsg.append(msg)
            else:
                '''
                Example: floodHistory[msg.sourceId] = [2,3,6]
                Explanation: 
                    1. The maximum value in the array (6) indicates that a message with seq 6 has been received from msg.sourceId
                    2. Values in the array that are less than 6 (2, 3) indicate that messages with seq 2 and 3 have not yet been received
                    3. Values below the maximum value and not in the array (0, 1, 4, 5) indicate that messages with seq 0, 1, 4, and 5 have been received
                    4. Values greater than the maximum value and not in the array (7, 8, 9...) indicate that messages with seq 7, 8, 9... have not been received.
                '''
                
                maxhist = max(self.floodHistory[msg.sourceId])
                
                if msg.seq in self.floodHistory[msg.sourceId]:       
                    # If current message has already been received
                    if msg.seq == maxhist:  
                        # Do not forward message if it has already been received to prevent flooding
                        pass
                    else:                                                
                        # Remove from flood history, update PACS information and add to send buffer for forwarding
                        self.floodHistory[msg.sourceId].remove(msg.seq)
                        self.updatePacsInfo(msg.payload)
                        if msg.isSetTransLimit:
                            if msg.transNum == msg.transLimit:
                                return
                            else:
                                msg2 = copy.deepcopy(msg)
                                msg2.transNum += 1
                                self.sendMsg.append(msg2)
                        else:
                            self.sendMsg.append(msg)
                else:
                    if msg.seq < maxhist:
                        # Do not forward message if it has already been received to prevent flooding
                        pass
                    else:
                        # Update flood history, update PACS information and add to send buffer for forwarding
                        self.floodHistory[msg.sourceId].remove(maxhist)
                        for i in range(maxhist + 1, msg.seq + 1):
                            self.floodHistory[msg.sourceId].append(i)

                        self.updatePacsInfo(msg.payload)
                        
                        if msg.isSetTransLimit:
                            if msg.transNum == msg.transLimit:
                                return
                            else:
                                msg2 = copy.deepcopy(msg)
                                msg2.transNum += 1
                                self.sendMsg.append(msg2)
                        else:
                            self.sendMsg.append(msg)


    def updateAcsInfo(self):  # Process the received data in the acsInfo buffer (VLCMsg)
        """Method to process the received VLC messages stored in the acsInfo buffer.

        """
        if self.status == 'entering' or self.status == 'leaving': # If current status is entering or leaving, return and do not process data.
            return

        for info in self.acsInfo:   # Iterate over each VLC message in acsInfo buffer.
            if info.isFlooding:     # If the message is a flooding data packet:
                self.decoderVLCMsg(info, 'Flooding')   # Decode the message, check for conditions, and forward.
            elif info.isBroadcasting:    # If the message is not a flooding packet but it's a broadcast packet intended for neighbors:
                self.decoderVLCMsg(info, 'Broadcasting')  # Decode the message and check for conditions.
            else:   # If the message is targeted:
                if info.targetId == self.mId:   # If the message is intended for this device:
                    pass    # Decode the message.
                elif self.mId == info.linkPath[0]:  # If the message is not intended for this device but has to pass through:
                    pass    # Forward the message along the link path.
                else:   # If the message was not intended for this device and has been received unintentionally:
                    pass    # Discard the message.

    '''*********************end*****************************************************'''

    '''*********************生成发送数据包部分**************************************'''
    # def collisionDetect(self):  # 有问题
    #     rept = []
    #     for i in range(len(self.in_robots)):
    #         for j in range(i, len(self.in_robots)):
    #             temp = self.in_robots[i].slots & self.in_robots[j].slots
    #             if temp:
    #                 rept.append([temp, (i, j)])
    #
    #     if len(rept) == 0:
    #         return None
    #
    #     for colin in rept:
    #         len1 = len(self.in_robots[colin[1][0]].slots)
    #         len2 = len(self.in_robots[colin[1][1]].slots)
    #         if len1 == 1 and len2 == 1:  # 都只有一个slot那就只能换
    #             if self.in_robots[colin[1][0]].mId > self.in_robots[colin[1][1]].mId:  # 谁id大就让谁换一个slot
    #                 key = self.in_robots[colin[1][0]].mId
    #                 ids = [rbt.mId for rbt in self.in_robots[colin[1][0]].out_robots]
    #             else:
    #                 key = self.in_robots[colin[1][1]].mId
    #                 ids = [rbt.mId for rbt in self.in_robots[colin[1][1]].out_robots]
    #             s = self.chooseSlot(ids)
    #
    #             chg = VLCMsg(self.mId, isFlooding=True)
    #             chg.seq = self.floodSeq
    #             chg.payload[key] = {'type': 'slot-rdu', 'slots': colin[0]}
    #             chg.payload[key] = {'type': 'slot-add', 'slots': s}
    #             self.sendMsg.append(chg)
    #             self.floodSeq += 1
    #             self.p_acsInfo[key] = {'type': '', 'slots': s}
    #
    #         else:
    #             # 如果有人是
    #             chg = VLCMsg(self.mId, isFlooding=True)
    #             chg.seq = self.floodSeq
    #             if len1 > len2:
    #                 key = self.in_robots[colin[1][0]].mId
    #                 chg.payload[key] = {'type': 'slot-rdu', 'slots': colin[0]}
    #                 s = copy.deepcopy(self.in_robots[colin[1][0]].slots) - colin[0]
    #                 self.p_acsInfo[key] = {'type': '', 'slots': s}
    #             elif len1 < len2:
    #                 key = self.in_robots[colin[1][1]].mId
    #                 chg.payload[key] = {'type': 'slot-rdu', 'slots': colin[0]}
    #                 s = copy.deepcopy(self.in_robots[colin[1][1]].slots) - colin[0]
    #                 self.p_acsInfo[key] = {'type': '', 'slots': s}
    #             else:
    #                 if self.in_robots[colin[1][0]].mId > self.in_robots[colin[1][1]].mId:  # 谁id大就让谁换一个slot
    #                     key = self.in_robots[colin[1][0]].mId
    #                     chg.payload[key] = {'type': 'slot-rdu', 'slots': colin[0]}
    #                     s = copy.deepcopy(self.in_robots[colin[1][0]].slots) - colin[0]
    #                     self.p_acsInfo[key] = {'type': '', 'slots': s}
    #                 else:
    #                     key = self.in_robots[colin[1][1]].mId
    #                     chg.payload[key] = {'type': 'slot-rdu', 'slots': colin[0]}
    #                     s = copy.deepcopy(self.in_robots[colin[1][1]].slots) - colin[0]
    #                     self.p_acsInfo[key] = {'type': '', 'slots': s}
    #
    #             self.sendMsg.append(chg)
    #             self.floodSeq += 1

    def chooseSlot(self, ids):
        """
        This function chooses a slot for a new robot to use.

        Args:
            ids (list): A list of robot ids that can be reached by the new robot.

        Returns:
            set: A set containing the chosen slot.
        """        
        csr = set(range(self.stage.formerSlots))  # Get all available slots
        sNum = {}  # Store the number of slots used by all robots and their IDs
        sFeq = {}  # Store the frequency of slot usage for all robots together
        for rid in ids:  # Traverse each rid, which represents the id of the robot that can be illuminated by the current robot.
            var1 = np.nonzero(self.globalAG[:, rid])[0]  # Get the incoming neighbors of robot with id=rid.
            for v in var1:
                if v in self.p_acsInfo:  # If robot v has already been connected, remove the slots it uses from the available slots.
                    csr -= self.p_acsInfo[v]['slots']

                    if v not in sNum:  # Record the number of slots and usage frequency used by v
                        sNum[v] = len(self.p_acsInfo[v]['slots'])
                        for ts in self.p_acsInfo[v]['slots']:
                            if ts not in sFeq:
                                sFeq[ts] = 1
                            else:
                                sFeq[ts] += 1

                else:
                    continue
        s = set()  # Initialize the return value as an empty set
        if len(csr) > 0:  # If there are available slots, randomly pick one.
            s.add(random.choice(list(csr)))
        else:  # If there are no available slots, we need to release a used slot for the new robot to use.
            slen = round(len(ids)/2)
            ids = sorted(ids)
            if self.mId not in [ids[i] for i in range(slen)]:  # Check if the current robot is in the top 50% of the robot list, return an empty set if it is not.
                return s

            tid = list(sNum.keys())[list(sNum.values()).index(max(list(sNum.values())))]  # Find the id=tid of the robot with the most slots used
            if sNum[tid] == 1:  # If the robot with id=tid only occupies one slot, it cannot be released and can only return an empty set
                print('!!!!!!!!!!!!!!!!!!!!!every robot has only one slot, new robot has no choice!!!!!!!!!!!!!!')
            else:  # If it is possible to release a slot, find the slot ts that is occupied the least in the slot of the robot with id=tid.
                tarr = [(ts, sFeq[ts]) for ts in self.p_acsInfo[tid]['slots']]
                ts = tarr[[item[1] for item in tarr].index(min([item[1] for item in tarr]))][0]
                s.add(ts)  # Add ts to the return value
                # Send information: Cancel the ts slot of the robot with id=tid and give it to the new robot
                chg = VLCMsg(self.mId, isFlooding=True)  # Initialize a VLCMsg message
                chg.seq = self.floodSeq
                for rid in ids:  # Traverse each rid, which represents the id of the robot that can be illuminated by the current robot.
                    var1 = np.nonzero(self.globalAG[:, rid])[0]  # Get the incoming neighbors of robot with id=rid.
                    for v in var1:
                        if v in self.p_acsInfo and ts in self.p_acsInfo[v]['slots']:  # If the current robot v uses the ts slot, it is necessary to cancel this slot.
                            chg.payload[v] = {'type': 'slot-rdu', 'slots': s}  # Send the change to the corresponding robot
                self.sendMsg.append(chg)  # Add the message to the list
                self.floodSeq += 1  # Increment floodSeq by one
                self.p_acsInfo[tid]['slots'].discard(ts)  # Remove ts from the slots occupied by the robot with id=tid.

                print('@@@@@@@@@@@@@@@@ reduce robot:', tid, ' with slot: ', ts, ' to guarantee new robot in @@@@@@@@@')
        return s  # Return the result set. 


    def genVLCMsg(self):
        """
        This function generates a message for VLC communication.

        Returns:
            None: If the robot status is "entering".
            VLCMsg: A message containing the current robot's ID, whether it is flooding or broadcasting, and the payload.
        """
        if self.status == 'entering':
            return

        if self.status == 'leaving':
            lea = VLCMsg(self.mId, isFlooding=True)
            lea.seq = self.floodSeq
            self.floodSeq += 1
            lea.payload['LEAVING'] = self.mId
            self.sendMsg.append(lea)
            self.hasSendLeaFlag = 1

        if self.status == 'online-pre':
            # Send a request to its outgoing neighbors using broadcasting mode.
            if not self.hasSendReqFlag:

                req1 = VLCMsg(self.mId, isBroadcasting=True)
                req1.payload['REQ'] = [self.mId, [robot.mId for robot in self.out_robots]]
                req2 = VLCMsg(self.mId, isFlooding=True)
                req2.seq = self.floodSeq
                self.floodSeq += 1
                req2.payload['NEIGHBORS'] = [self.mId, [robot.mId for robot in self.in_robots]]
                self.sendMsg.append(req1)
                self.sendMsg.append(req2)

                self.hasSendReqFlag = 1

        if self.status == 'online':
            if self.resFlag == 1:
                # For each response generated in reference, call chooseSlot to get a set of slots and append it to the sendMsg list with the corresponding key-value pairs.
                for key, ids in self.refResDict.items():
                    s = self.chooseSlot(ids)
                    if not s:
                        continue
                    res = VLCMsg(self.mId, isFlooding=True)
                    res.seq = self.floodSeq
                    res.payload[key] = {'type': 'slot-add', 'slots': s}
                    self.sendMsg.append(res)
                    self.floodSeq += 1
                    self.p_acsInfo[key] = {'type': '', 'slots': s}

                    # print('id:', self.mId, ' status: online, gen response msg for:', key, 'with slot:', s)

                self.refResDict.clear()
                self.resFlag = 0

                # Send the updated in-neighbors to out-neighbors.
                res2 = VLCMsg(self.mId, isFlooding=True)
                res2.seq = self.floodSeq
                self.floodSeq += 1
                res2.payload['NEIGHBORS'] = [self.mId, [robot.mId for robot in self.in_robots]]
                self.sendMsg.append(res2)

            if self.hasSlotChangedFlag == 1:
                scpack = VLCMsg(self.mId, isFlooding=True)
                scpack.seq = self.floodSeq
                self.floodSeq += 1
                scpack.payload[self.mId] = {'type': 'slot-rpl', 'slots': copy.deepcopy(self.slots)}
                self.sendMsg.append(scpack)
                self.hasSlotChangedFlag = 0



        # 检测有没有merging collision
        # self.collisionDetect()
    '''*********************end*****************************************************'''

    def calcTargetPos(self):
        """Calculates the target position of the robot based on its current status and location"""
            
        if self.status == 'entering':  # if the robot is entering the network
            if self.isStopping() and self.mPos[2] == 500:  # if the robot has stopped and is at the correct height
                # calculate the robot's position on the stage grid
                i = round((self.mPos[0] + self.stage.scale / 2) / self.stage.scale * self.stage.linenum - 1)
                j = round((self.mPos[1] + self.stage.scale / 2) / self.stage.scale * self.stage.linenum - 1)
                
                # if there is a clear angle for the robot to move towards, set the direction and update the status 
                if self.stage.map[i][j]['angle'] is not None:
                    self.setDirection(self.stage.map[i][j]['angle'])
                    self.status = 'online-pre'
                    self.stage.map[i][j]['status'] = 'busy'

                    self.getNetworkInfo()
                    self.startAccessFlag = 1
                    print('*****new robot start accessing***********')

                else:  # if there is no clear angle for the robot to move towards
                    # if there are at least two robots in and out of the network, update the status 
                    if len(self.out_robots) >= 1 and len(self.in_robots) >= 1:
                        self.status = 'online-pre'
                        self.stage.map[i][j]['status'] = 'busy'

                        self.getNetworkInfo()
                        self.startAccessFlag = 1
                        print('*****new robot start accessing***********')
                    
                    else:  # if there is only one robot in or out of the network
                        angle = self.mDirection + 0.54
                        if angle > math.pi:
                            angle = -math.pi
                            self.flag1 = 1
                        if angle >= self.initDirection and self.flag1 == 1:
                            self.stage.map[i][j]['status'] = 'useless'
                            self.mTarget = np.array([self.mPos[0],
                                                    self.mPos[0],
                                                    400], dtype=np.float32)
                        else:
                            self.setDirection(angle)
            else:  # if the robot is not stopped or at the correct height, update initial direction 
                self.initDirection = self.mDirection
        
        elif self.status == 'leaving':  # if the robot is leaving the network
            if self.hasSendLeaFlag == 2:  #if the robot has confirmed it is leaving
                self.mTarget = np.array([self.mPos[0],
                                        self.mPos[0],
                                        400], dtype=np.float32)  # set target position for the robot
        
        else:  # if the status is neither entering nor leaving, exit the function
            return


    def update(self):
        """Update robot state information."""
        
        # Count access time, if startAccessFlag is set
        if self.startAccessFlag:
            self.accessCount += 1

        # Update neighbor information for this robot
        self.globalAG[:, self.mId] = 0
        innbs = [rbt.mId for rbt in self.in_robots]
        for innb in innbs:
            self.globalAG[innb, self.mId] = 1

        # Process received slot messages, parse them from acsInfo buffer to p_acsInfo, and add data packets to sendMsg buffer for forwarding
        self.updateAcsInfo()

        # # Use consensus algorithm to update robot position
        # self.processInfo()

        # Calculate next target position
        self.calcTargetPos()

        # Move robot to the specified location, if necessary
        self.move()

        # Update information from p_acsInfo to self.slots and to ComStageMAC.rbtID_Time.
        self.updateSlots()

        # Generate message to be sent
        self.genVLCMsg()

        # Keep track of and save historical access counts (only executed upon ending access)
        if self.endAccessFlag:
            self.stage.accessCountHistory.append((self.mId, self.accessCount))
            self.accessCount = 0
            self.endAccessFlag = 0
            self.stage.acesSuccessFlag = 1










