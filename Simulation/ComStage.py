# -*- coding: utf-8 -*-

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
from matplotlib import cbook
from matplotlib import cm
from matplotlib.colors import LightSource
import networkx as net
from Common.DrKDtree import KDtree
from Common.utils import *
import Common.settings as settings 
from Simulation.ComObjectCollection import *
from Simulation.ComSurfaceBase import ComSurfaceBase
from Simulation.ComFish import ComFish
from Simulation.ComObject import ComObject
from Simulation.ComRobot import ComRobot, CommunicateMethods
import random
from Simulation.ComDataPlotBase import ComDataPlotBase
from PIL import Image
import enum
import copy

# dem = cbook.get_sample_data('jacksboro_fault_dem.npz', np_load=True)
# z = dem['elevation']
# nrows, ncols = z.shape
# x = np.linspace(dem['xmin'], dem['xmax'], ncols)
# y = np.linspace(dem['ymin'], dem['ymax'], nrows)
# x, y = np.meshgrid(x, y)

# reg = np.s_[5:50, 5:50]
# x, y, z = x[reg], y[reg], z[reg]
# x -= np.mean(x)
# x = x * 1000 / np.max(x)
# y -= np.mean(y)
# y = y * 1000 / np.max(y)
# z = (z - np.mean(z))*3 - 1000
# ls = LightSource(270, 45)
# # To use a custom hillshading mode, override the built-in shading and pass
# # in the rgb colors of the shaded surface calculated from "shade".
# rgb = ls.shade(z, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')


class ComStage:
    mCount = 0
    def __init__(self):
        self.mStageType = '3D'
        self.mRobotList = net.DiGraph()
        self.mRobotPosList = []
        self.mStuffList = []
        self.mSurfaceGroup = []
        self.mFig = None
        self.mAx = None
        self.mGraphAx = None
        self.mDataAx = None
        self.mMonitorGroupAx = None

        self.__mRuningTime = settings.CS_RUNING_TIME
        self.mEnvSize = settings.CS_ENVSIZE  # 环境大小, width, height
        self.mInterval = [settings.CS_INTERVAL]  # 两次迭代间隔的时间
        self.isShowCommunicated = True
        self.isPlotGraph = True

        self.isSavePos = False
        self.isSavePos2 = False
        self.mSavePosRound = settings.SAVE_POS_ROUND
        self.isSaveFig = False
        self.mFigSize = (18, 9)     # 画布的实际大小，w, h，英寸
        self.__ys = []

        self.mDataPlot = None       # 用于在mDataAx上绘图的对象
        self.mMonitorPlot = None    # 用于在mMonitorGroupAx上绘图的对象
        self.mCurrentTime = [0.0, self.mRuningTime]
        self.isRotating = False
        self.isFixedComNet = False  # 通信网络是否为固定
        self.isComConstrained = False   # 通信网络是否受通信范围约束
        self.mEdgesCopy = None      # 固定网络的通信连接备份

        self.mElev = 30
        self.mAzim = 45
    
    def clear(self):
        self.mRobotList = net.DiGraph()
        self.mRobotPosList = []
        self.mStuffList = []
        self.mSurfaceGroup = []
        self.__ys = []
        clearObject()
        self.count = 0
        ComRobot._robot_count = 0
        ComFish._fish_count = 0
        plt.close('all')
        del self.mFig
        self.mFig = None
        self.mAx = None
        self.mGraphAx = None
        self.mDataAx = None
        self.mMonitorGroupAx = None


    @property
    def count(self):
        return ComStage.mCount

    @count.setter
    def count(self, value):
        ComStage.mCount = value
        self.mCurrentTime[0] = ComStage.mCount * self.mInterval[0]

    def setEnvSize(self, _size=(480, 640, 500)):
        self.mEnvSize = _size

    def setFigSize(self, figsize: tuple):
        """
        画布的大小，注意与环境大小不同，是实际显示的大小
        """
        self.mFigSize = figsize

    def setDataPlot(self, data_plot: ComDataPlotBase):
        self.mDataPlot = data_plot

    def setMonitorPlot(self, monitor_plot):
        self.mMonitorPlot = monitor_plot

    def setViewAngle(self, elev=30.0, azim=60.0):  
        """视图旋转

        Args:
            elev (float, optional): The elevation angle in degrees 
            rotates the camera above the plane pierced by the vertical 
            axis, with a positive angle corresponding to a location 
            above that plane. For example, with the default vertical 
            axis of 'z', the elevation defines the angle of the camera 
            location above the x-y plane. If None, then the initial 
            value as specified in the Axes3D constructor is used. 
              Defaults to 30.0.
            azim (float, optional): The azimuthal angle in degrees 
            rotates the camera about the vertical axis, with a positive 
            angle corresponding to a right-handed rotation. For example, 
            with the default vertical axis of 'z', a positive azimuth 
            rotates the camera about the origin from its location along 
            the +x axis towards the +y axis. If None, then the initial 
            value as specified in the Axes3D constructor is used.. 
              Defaults to 60.0.
        """        
        self.mElev = elev
        self.mAzim = azim
        

    @property
    def mRuningTime(self):
        return self.__mRuningTime

    @mRuningTime.setter
    def mRuningTime(self, value):
        self.__mRuningTime = value 
        self.mCurrentTime[1] = value

    def initEnv(self):
        """
        初始化环境
        :return:
        """
        if self.mFig is None:
            self.mFig = plt.figure(figsize=self.mFigSize, constrained_layout=True)
        if self.isPlotGraph:
            gs = self.mFig.add_gridspec(8, 15)
            self.mAx = self.mFig.add_subplot(gs[:, 0:7], projection="3d")
            self.mGraphAx = self.mFig.add_subplot(gs[:4, 7:10])
            self.mDataAx = self.mFig.add_subplot(gs[:4, 10:])
            self.mMonitorGroupAx = []
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[4:6, 7:9]))
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[4:6, 9:11]))
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[4:6, 11:13]))
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[4:6, 13:]))
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[6:, 7:9]))
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[6:, 9:11]))
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[6:, 11:13]))
            self.mMonitorGroupAx.append(self.mFig.add_subplot(gs[6:, 13:]))
            
        else:
            self.mAx = self.mFig.add_subplot(projection="3d")

        for surf in self.mSurfaceGroup:
            surf.setAx(self.mAx)

        self.mAx.view_init(self.mElev, self.mAzim)

    def addRobot(self, robot):
        robot.setInterval(self.mInterval)
        self.mRobotList.add_node(robot)
        self.mRobotPosList.append(robot.pos)
        robot.setStage(self)
        robot.setAx(self.mAx)
        ObjectAppend(robot)

    def addStuff(self, stuff):
        stuff.setInterval(self.mInterval)
        self.mStuffList.append(stuff)
        stuff.setStage(self)
        ObjectAppend(stuff)

    def addSurface(self, surface: ComSurfaceBase):
        self.mSurfaceGroup.append(surface)

    def update(self):
        if self.mAx:
            self.mAx.cla()
            self.mAx.grid(False) 
            self.mAx.set_xlim(-self.mEnvSize[0], self.mEnvSize[0])
            self.mAx.set_ylim(-self.mEnvSize[1], self.mEnvSize[1])
            self.mAx.set_zlim(-self.mEnvSize[2], self.mEnvSize[2])
        if self.mGraphAx:
            self.mGraphAx.cla()
        if self.mDataAx:
            self.mDataAx.cla()
        if self.mMonitorGroupAx:
            for ax in self.mMonitorGroupAx:
                ax.set_xlim(-self.mEnvSize[0], self.mEnvSize[0])
                ax.set_ylim(-self.mEnvSize[1], self.mEnvSize[1])      
                ax.cla()

        if self.isRotating:
            self.mAx.view_init(30, self.count%360)
            
        
        # surf = self.mAx.plot_surface(x, y, z, rstride=1, cstride=1, facecolors=rgb, 
        #     linewidth=0, antialiased=False, shade=False)

        for surf in self.mSurfaceGroup:
            surf.update()
            surf.draw()
            

        for ind, robot in enumerate(self.mRobotList.nodes):
            robot.draw(self.mAx)
            robot.update()

            self.mRobotPosList[ind] = robot.pos
            
        for stuff in self.mStuffList:
            stuff.draw(self.mAx)
            stuff.update()
            
        
        if self.isFixedComNet:
            if self.isComConstrained:
                self.mRobotList.clear_edges()
                for edge in self.mEdgesCopy:
                    if distance(edge[0].pos, edge[1].pos) < edge[0].mCommunicationRange:
                        self.mRobotList.add_edge(edge[0], edge[1])
            # print(len(self.mRobotList.edges()))
        else:
        # 获得边的关系，即每个机器人与通信范围内的机器人组成的对应关系
            self.mRobotList.clear_edges()
            kd_tree = KDtree(self.mRobotPosList)
            for ind, pos in enumerate(self.mRobotPosList):
                robot = self.getRobot(ind)
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
                                # print('D_world')
                                robot.mNetworkModule.setAvailableComRobots(robots_in_range_inds)
                                sigma_dict = robot.mNetworkModule.update()
                                robots_in_range_inds = sigma_dict.keys()
                                # print(len(robots_in_range_inds))
                            else:
                                print("the robot[{}] dosen't have mNetworkModule".format(robot.mId))

                    if len(robots_in_range_inds) > 0:
                        if isCupy:
                            robots_in_range_inds = robots_in_range_inds.get()
                        for range_ind in robots_in_range_inds:
                            self.mRobotList.add_edge(robot, self.getRobot(range_ind))
        # 通信
        edges = self.mRobotList.edges
        # print("edges size: ", len(edges))
        for edge in edges:
            edge[0].communicateWith(edge[1])
        # print()

        if self.isShowCommunicated:
            if len(edges) > 0:
                for edge in edges:
                    x = []
                    y = []
                    z = []
                    x.append(edge[0].mPos[0])
                    x.append(edge[1].mPos[0])
                    y.append(edge[0].mPos[1])
                    y.append(edge[1].mPos[1])
                    z.append(edge[0].mPos[2])
                    z.append(edge[1].mPos[2])
                    self.mAx.plot(x, y, z, 'g-.', alpha=0.3, linewidth=1)
        if self.isPlotGraph and len(self.mRobotList.nodes) > 1:
            graph_pos = net.circular_layout(self.mRobotList)#布置框架
            net.draw(self.mRobotList,graph_pos,ax=self.mGraphAx, with_labels=False,node_size=30)

        if self.mDataAx is not None:
            if self.mDataPlot is None:
                # 绘制数据图
                self.mDataAx.set_ylim(-1000, 0)
                self.mDataAx.set_xlim(0, self.mRuningTime / self.mInterval[0])
                xs, ys = self.getPositionPlotData(0, 1)
                self.mDataAx.plot(xs, ys)
            else:
                self.mDataPlot.update()
                self.mDataPlot.draw(self.mDataAx)

        if self.mMonitorGroupAx is not None:
            if self.mMonitorPlot is None:
                img = Image.open("./resources/image2.jpeg")
                for ax in self.mMonitorGroupAx:
                    ax.imshow(img)
            else:
                self.mMonitorPlot.update()
                self.mMonitorPlot.draw(self.mMonitorGroupAx)
        
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
    
    def getRobot(self, ind):
        return list(self.mRobotList.nodes)[ind]

    def setFixedRegularComNet(self, k=6, isComConstrainted=False):
        """机器人的通信网络设置为固定规则网络。
        警告：机器人添加完后设置有效
        Args:
            k (int, optional): 节点平均连接数量. Defaults to 6.
            isComConstrainted (bool, optional): 是否受通信范围约束. Defaults to False.
        """
        self.isFixedComNet = True 
        self.isComConstrained = isComConstrainted
        self.mRobotList.clear_edges()
        robot_node_list = self.getRobotGroup()
        graph_regular = net.watts_strogatz_graph(len(robot_node_list), k, 0)
        for edges in graph_regular.edges():
            self.mRobotList.add_edge(robot_node_list[edges[0]], robot_node_list[edges[1]])
            self.mRobotList.add_edge(robot_node_list[edges[1]], robot_node_list[edges[0]])
        self.mEdgesCopy = self.getEdgesGroup()

    def setFixedSmallWorldComNet(self, k=6, p=0.3, isComConstrainted=False):
        """机器人的通信网络设置为小世界网络。
        警告：机器人添加完后设置有效
        Args:
            k (int, optional): 节点平均连接数量. Defaults to 6.
            p (float, optional): 随机连接的概率. Defaults to 0.3.
        """
        self.isFixedComNet = True 
        self.isComConstrained = isComConstrainted
        self.mRobotList.clear_edges()
        robot_node_list = self.getRobotGroup()
        graph_regular = net.watts_strogatz_graph(len(robot_node_list), k, p)
        for edges in graph_regular.edges():
            self.mRobotList.add_edge(robot_node_list[edges[0]], robot_node_list[edges[1]])
            self.mRobotList.add_edge(robot_node_list[edges[1]], robot_node_list[edges[0]])
        self.mEdgesCopy = self.getEdgesGroup()

    def setFixedScaledFreeComNet(self, k=6, isComConstrainted=False):
        """机器人的通信网络设置为无标度网络。
        警告：机器人添加完后设置有效
        Args:
            k (int, optional): [description]. Defaults to 6.
        """
        self.isFixedComNet = True 
        self.isComConstrained = isComConstrainted
        self.mRobotList.clear_edges()
        robot_node_list = self.getRobotGroup()
        graph_regular = net.barabasi_albert_graph(len(robot_node_list), k)
        for edges in graph_regular.edges():
            self.mRobotList.add_edge(robot_node_list[edges[0]], robot_node_list[edges[1]])
            self.mRobotList.add_edge(robot_node_list[edges[1]], robot_node_list[edges[0]])
        self.mEdgesCopy = self.getEdgesGroup()

    def setNaturalComNet(self):
        """机器人的通信网络设置为自然通信网络
        """
        self.isFixedComNet = False 
        self.mRobotList.clear_edges()

    def getPositionPlotData(self, monitor_robot_id, predict_robot_id):
        """
        获得定位信息预测绘图的数据
        @param monitor_robot_id: 作为数据记录的机器人id
        @param predict_robot_id: 作为被预测的机器人id
        @return xs ys:
        """
        robot0 = self.getRobot(monitor_robot_id)
        robot1 = self.getRobot(predict_robot_id)
        # 绘制数据图
        dict_list = []
        for item in robot0.mProcessedInfoRecorder:
            dict_list.append(item['Pos'])
        if len(robot0.mProcessedInfoRecorder) > 0:
            info_item = robot0.mProcessedInfoRecorder[-1]
            if 1 in info_item['Pos'].keys():
                dist_between_predict_real = distance(info_item['Pos'][predict_robot_id], robot1.pos)
                self.__ys.append(-dist_between_predict_real)
        xs = np.arange(len(self.__ys))
        ys_np = np.array(self.__ys)
        if isCupy:
            xs = xs.get()
            ys_np = ys_np.get()
        return xs, ys_np

    def getRobotGroup(self)->list:
        """获得机器人的list

        Returns:
            list: [description]
        """
        return list(self.mRobotList.nodes)

    def getEdgesGroup(self)->list:
        """
        获得机器人之间通信连接的列表
        """
        return list(self.mRobotList.edges)

    def getStuffGroup(self)->list:
        """获得Stuff的list

        Returns:
            list: [description]
        """
        return self.mStuffList
        
    def enableFigSave(self, dir):
        self.isSaveFig = True
        self.mFigSaveDir = dir

    def enablePosSave(self, dir):
        self.isSavePos2 = True
        self.mPosSaveDir = dir

    def saveFig(self, dir):
        if ComStage.mCount % settings.INTERVAL_SAVE == 0:
            try:
                plt.savefig(dir + '/image{0:0>5}.png'.format(ComStage.mCount))
            except:
                pass

    def savePos(self, dir, pos_list: list):
        with open(dir, 'a+') as file:
            file.write("{}".format(self.count))
            for pos in pos_list:
                file.write(', ')
                file.write(str(pos))
            file.write('\n')


    def run(self):
        self.initEnv()
        while True:
            if self.count % self.mSavePosRound == 0:
                if self.isSavePos2:
                    self.isSavePos = True
            else:
                self.isSavePos = False
            if (self.count * settings.CS_INTERVAL) >= self.mRuningTime:
                break
            self.count += 1
            ComObject.update_count = self.count
            if self.count % 1 == 0:
                print("Round: 【%d : %d】" % (self.count, self.mRuningTime/settings.CS_INTERVAL))
            self.update()
            plt.pause(settings.CS_INTERVAL)
    
    def run_once(self):
        self.initEnv()
        self.update()
        # for robot in self.mRobotList.nodes:
        #     robot.update()
        #     print(robot.mPos)
        #     robot.draw(self.mAx)
        # for stuff in self.mStuffList:
        #     stuff.update()
        #     stuff.draw(self.mAx)
        plt.show()

if __name__ == "__main__":
    a = ComStage()
    a.run()
