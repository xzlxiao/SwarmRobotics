import sys
sys.path.append('./')
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import matplotlib.pyplot as plt
from Simulation.ComStage import ComStage
from Common.utils import *
from Common.DrKDtree import KDtree
from Simulation.ComRobot import ComRobot, CommunicateMethods


class ComStage2D(ComStage):
    def __init__(self):
        super().__init__()
        self.mStageType = '2D'

    def setEnvSize(self, _size=(1000, 1000)):
        self.mEnvSize = _size

    def initEnv(self):
        """
        初始化环境
        :return:
        """
        if self.mFig is None:
            self.mFig = plt.figure(figsize=self.mFigSize, constrained_layout=True)
        self.mAx = self.mFig.add_subplot()

        for surf in self.mSurfaceGroup:
            surf.setAx(self.mAx)

    def update(self):
        if self.mAx:
            self.mAx.cla()
            self.mAx.grid(False) 
            self.mAx.set_xlim(-self.mEnvSize[0], self.mEnvSize[0])
            self.mAx.set_ylim(-self.mEnvSize[1], self.mEnvSize[1])
        
        for surf in self.mSurfaceGroup:
            surf.update()
            surf.draw()
            

        for ind, robot in enumerate(self.mRobotList.nodes):
            # if ind == 0:
            #     print(robot.mPos)
            
            self.mRobotPosList[ind] = robot.pos
            robot.draw(self.mAx)
            robot.update()
        
        for stuff in self.mStuffList:
            
            stuff.draw(self.mAx)
            stuff.update()
        
        if self.isFixedComNet:
            if self.isComConstrained:
                self.mRobotList.clear_edges()
                for edge in self.mEdgesCopy:
                    if distance(edge[0].pos, edge[1].pos) < edge[0].mCommunicationRange:
                        self.mRobotList.add_edge(edge[0], edge[1])
        else:
        # 获得边的关系，即每个机器人与通信范围内的机器人组成的对应关系
            self.mRobotList.clear_edges()
            kd_tree = KDtree(self.mRobotPosList)
            for ind, pos in enumerate(self.mRobotPosList):
                robot = list(self.mRobotList.nodes)[ind]
                if robot.isCommunicating:
                    robots_in_range_inds, _ = kd_tree.query_radius(pos, robot.mCommunicationRange)
                    robots_in_range_inds = np.array(robots_in_range_inds[0])
                    if robots_in_range_inds[0] == ind:
                        robots_in_range_inds = robots_in_range_inds[1:]    # 排除自身ind

                    if robot.isRandomCom:
                        if len(robots_in_range_inds) > robot.mMaxComNum:
                            robots_in_range_inds = np.array(random.sample(robots_in_range_inds.tolist(), robot.mMaxComNum)) 
                    else:
                        if robot.mComMethod == CommunicateMethods.NearestCom:
                            robots_in_range_inds = robots_in_range_inds[0:robot.mMaxComNum]
                        elif robot.mComMethod == CommunicateMethods.AllCom:
                            robots_in_range_inds = robots_in_range_inds[:]
                        elif robot.mComMethod == CommunicateMethods.WS_net:
                            pass
                        elif robot.mComMethod == CommunicateMethods.D_world:
                            
                            if robot.mNetworkModule is not None:
                                robot.mNetworkModule.setAvailableComRobots(robots_in_range_inds)
                                sigma_dict = robot.mNetworkModule.update()
                                robots_in_range_inds = sigma_dict.keys()
                            else:
                                print("the robot[{}] dosen't have mNetworkModule".format(robot.mId))

                    if len(robots_in_range_inds) > 0:
                        if isCupy:
                            robots_in_range_inds = robots_in_range_inds.get()
                        for range_ind in robots_in_range_inds:
                            self.mRobotList.add_edge(robot, list(self.mRobotList.nodes)[range_ind])
        
        # 通信
        edges = self.mRobotList.edges
        for edge in edges:
            edge[0].communicateWith(edge[1])

        if self.isShowCommunicated:
            if len(edges) > 0:
                for edge in edges:
                    x = []
                    y = []
                    x.append(edge[0].pos[0])
                    x.append(edge[1].pos[0])
                    y.append(edge[0].pos[1])
                    y.append(edge[1].pos[1])
                    self.mAx.plot(x, y, 'g-.', alpha=0.3, linewidth=1)
        
        if self.isSaveFig:
            mkdir(self.mFigSaveDir + "/images")
            self.saveFig(self.mFigSaveDir + "/images")
        
        if self.isSavePos:
            mkdir(self.mPosSaveDir + "/pos")
            robot_group = self.getRobotGroup()
            stuff_group = self.getStuffGroup()
            for robot in robot_group:
                self.savePos(self.mPosSaveDir + "/pos/{}_{}.txt".format(robot.mObjectType, robot.mId), robot.pos)
            for stuff in stuff_group:
                self.savePos(self.mPosSaveDir + "/pos/{}_{}.txt".format(stuff.mObjectType, stuff.mId), stuff.pos)
        