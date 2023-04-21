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
        self.isSaveInfo = False
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
        self.mObstacleTypeList = []     # 算作障碍物的object名称
    
    def clear(self):
        """This method clears/reset all attributes in the object and related objects.
        """
        self.mRobotList = net.DiGraph()  # Clear/reset the robot list
        self.mRobotPosList = []  # Clear/reset the robot position list
        self.mStuffList = []  # Clear/reset the stuff list
        self.mSurfaceGroup = []  # Clear/reset the surface group
        self.__ys = []  # Clear/reset private ys attribute
        clearObject()  # Clear an object in the module
        self.count = 0  # Reset the count attribute
        ComRobot._robot_count = 0  # Reset the total number of robots created
        ComFish._fish_count = 0  # Reset the total number of fishes created
        plt.close('all')  # Close all figures
        del self.mFig  # Delete the figure object
        self.mFig = None  # Set the figure to None
        self.mAx = None  # Set the axis to None
        self.mGraphAx = None  # Set the graph axis to None
        self.mDataAx = None  # Set the data axis to None
        self.mMonitorGroupAx = None  # Set the monitor group axis to None


    
    @property
    def count(self):
        """This method gets the total number of stage objects created.

        Returns:
            int: The total count of stage objects.
        """
        return ComStage.mCount  # Return the static variable mCount from the ComStage class


    @count.setter
    def count(self, value):
        """This method sets the total number of stage objects created to a specific value.

        Args:
            value (int): The new count of stage objects.
        """
        ComStage.mCount = value  # Set the static variable mCount from the ComStage class to the given value
        self.mCurrentTime[0] = ComStage.mCount * self.mInterval[0]  # Update the mCurrentTime list based on the new count and interval



    def addObstacleType(self, obstacle_type:str):
        """Add a new obstacle type to the list of obstacle types.

        Args:
            obstacle_type (str): The name of the new obstacle type.
        """
        # Append the given obstacle type to the mObstacleTypeList attribute
        self.mObstacleTypeList.append(obstacle_type)

    def setEnvSize(self, _size=(480, 640, 500)):
        """Set the size of the environment.

        Args:
            _size (tuple, optional): The size of the environment in (height, width, depth) format. Defaults to (480, 640, 500).
        """
        # Update the mEnvSize attribute with the given size tuple
        self.mEnvSize = _size

    def setFigSize(self, figsize: tuple):
        """Set the size of the figure used for plotting.
        The size of the canvas used for plotting. Note that this is different from the environment size and represents the actual display size.

        Args:
            figsize (tuple): The size of the figure.
        """

        # Update the mFigSize attribute with the given figsize tuple
        self.mFigSize = figsize

    def setDataPlot(self, data_plot: ComDataPlotBase):
        """Set the data plot object.

        Args:
            data_plot (ComDataPlotBase): An instance of the ComDataPlotBase class which handles data visualization.
        """
        # Set the mDataPlot attribute to the given data_plot object
        self.mDataPlot = data_plot

    def setMonitorPlot(self, monitor_plot):
        """Set the monitor plot.

        Args:
            monitor_plot (_type_): The monitor plot object.
        """
        # Set the mMonitorPlot attribute to the given monitor_plot object
        self.mMonitorPlot = monitor_plot


    def setViewAngle(self, elev=30.0, azim=60.0):  
        """Set the view angle of the 3D plot.

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
        """Get the current running time of the program.

        Returns:
            Time duration in seconds.
        """
        # Return the current running time value
        return self.__mRuningTime

    @mRuningTime.setter
    def mRuningTime(self, value):
        """Set the current running time of the program.

        Args:
            value: The new running time value to be set.
        """
        # Update the running time attribute with the given value
        self.__mRuningTime = value 
        # Update the second element of the mCurrentTime list with the given value
        self.mCurrentTime[1] = value


    def initEnv(self):
        """Initialize the environment.
        This function initializes the environment, setting up the figure size and layout.
        It creates an instance of a 3D axis object and optionally plots graph axes, monitor groups,
        and data axes. Additionally, it sets properties for specified surfaces and the view angle
        of the 3D axis object.

        """

        # If mFig is not None, create a new figure
        if self.mFig is None:
            self.mFig = plt.figure(figsize=self.mFigSize, constrained_layout=True)
        
        # If isPlotGraph flag is True, create a gridspec and subplot axes for graphs and monitors
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
                
        # If isPlotGraph flag is False, create a 3D axis object with no specified grid
        else:
            self.mAx = self.mFig.add_subplot(projection="3d")

        # Set the 3D axis object for all surfaces in mSurfaceGroup
        for surf in self.mSurfaceGroup:
            surf.setAx(self.mAx)

        # Set the view angle of the 3D axis object
        self.mAx.view_init(self.mElev, self.mAzim)

    def addRobot(self, robot):
        """Add a robot object to the environment.

        Args:
            robot: The robot object to be added.
        """
        # Set the update interval for the robot and add it as a node to mRobotList
        robot.setInterval(self.mInterval)
        self.mRobotList.add_node(robot)
        # Add the robot's position to the list of robot positions
        self.mRobotPosList.append(robot.pos)
        # Set the stage and 3D axis object for the robot
        robot.setStage(self)
        robot.setAx(self.mAx)
        ObjectAppend(robot)

    def addStuff(self, stuff):
        """Add a stuff object to the environment.

        Args:
            stuff: The stuff object to be added.
        """
        # Set the update interval for the stuff and add it to mStuffList
        stuff.setInterval(self.mInterval)
        self.mStuffList.append(stuff)
        # Set the stage for the stuff
        stuff.setStage(self)
        ObjectAppend(stuff)

    def addSurface(self, surface: ComSurfaceBase):
        """Add a surface object to the environment.

        Args:
            surface: The surface object to be added.
        """
        # Append the surface to mSurfaceGroup
        self.mSurfaceGroup.append(surface)

    def update(self):
        """Updates the plot and object states"""
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
        
        # 更新障碍物信息
        updateObstacle_kdtree(self.mObstacleTypeList)

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
                    self.mAx.plot(x, y, z, 'g-.', alpha=0.7, linewidth=1.5)
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
                img = Image.open("./Resource/image2.jpeg")
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

        if self.isSaveInfo:
            mkdir(self.mSaveInfoDir + '/info')
            robot_group = self.getRobotGroup()
            stuff_group = self.getStuffGroup()
            for robot in robot_group:
                self.savePos(self.mSaveInfoDir + "/info/{}_{}.txt".format(robot.mObjectType, robot.mId), robot.getMessage())
            for stuff in stuff_group:
                self.savePos(self.mSaveInfoDir + "/info/{}_{}.txt".format(stuff.mObjectType, stuff.mId), stuff.getMessage())

    
    def getRobot(self, ind):
        """
        This function returns a specific robot/node from the mRobotList in the class instance based on the index provided.

        Args:
            ind (int): The index of the robot/node to be returned from the mRobotList.

        Returns:
            list: A list converted from the nodes of the mRobotList containing the robot/node at the specified index 'ind'.
        """
        # The function returns the robot at index ind by converting the nodes of the mRobotList into a list and returning the element at the specified index.
        return list(self.mRobotList.nodes)[ind]

    def setFixedRegularComNet(self, k=6, isComConstrainted=False):
        """
        This function sets the communication network of robots to a fixed regular network. The robot network can only be set 
        after adding all the robots in the simulation.

        Args:
            k (int, optional): The average number of nodes each node is connected to. Defaults to 6.
            isComConstrainted (bool, optional): Whether or not to apply communication constraints. Defaults to False.
        """

        # Set the flag indicating that fixed communication network has been set.
        self.isFixedComNet = True 

        # Set whether or not communication constraints should be applied.
        self.isComConstrained = isComConstrainted

        # Clear any previously set edges.
        self.mRobotList.clear_edges()

        # Get the list of robot nodes to create graph from.
        robot_node_list = self.getRobotGroup()
        
        # Generate the regular graph based on provided parameters.
        graph_regular = net.watts_strogatz_graph(len(robot_node_list), k, 0)

        # Add edges between nodes in the graph.
        for edges in graph_regular.edges():
            self.mRobotList.add_edge(robot_node_list[edges[0]], robot_node_list[edges[1]])
            self.mRobotList.add_edge(robot_node_list[edges[1]], robot_node_list[edges[0]])

        # Save a copy of the edges for future reference.
        self.mEdgesCopy = self.getEdgesGroup()

    def setFixedSmallWorldComNet(self, k=6, p=0.3, isComConstrainted=False):
        """
        This function sets the communication network of robots to a small world network.

        Args:
            k (int, optional): The average number of nodes each node is connected to. Defaults to 6.
            p (float, optional): The probability of adding random edges. Defaults to 0.3.
            isComConstrainted (bool, optional): Whether or not to apply communication constraints. Defaults to False.
        """        
        # Set the flag indicating that fixed communication network has been set.
        self.isFixedComNet = True 

        # Set whether or not communication constraints should be applied.
        self.isComConstrained = isComConstrainted
        
        # Clear any previously set edges.
        self.mRobotList.clear_edges()
        
        # Get the list of robot nodes to create graph from.
        robot_node_list = self.getRobotGroup()
        
        # Generate the small world graph based on provided parameters.
        graph_regular = net.watts_strogatz_graph(len(robot_node_list), k, p)

        # Add edges between nodes in the graph.
        for edges in graph_regular.edges():
            self.mRobotList.add_edge(robot_node_list[edges[0]], robot_node_list[edges[1]])
            self.mRobotList.add_edge(robot_node_list[edges[1]], robot_node_list[edges[0]])

        # Save a copy of the edges for future reference.
        self.mEdgesCopy = self.getEdgesGroup()


    def setFixedScaledFreeComNet(self, k=6, isComConstrainted=False):
        """
        This function sets the communication network of robots to a scale-free network.

        Args:
            k (int, optional): The number of edges to attach from a new node. Defaults to 6.
            isComConstrainted (bool, optional): Whether or not to apply communication constraints. Defaults to False.
        """        
        # Set the flag indicating that fixed communication network has been set.
        self.isFixedComNet = True 

        # Set whether or not communication constraints should be applied.
        self.isComConstrained = isComConstrainted
        
        # Clear any previously set edges.
        self.mRobotList.clear_edges()
        
        # Get the list of robot nodes to create graph from.
        robot_node_list = self.getRobotGroup()
        
        # Generate the scale-free graph based on provided parameters.
        graph_regular = net.barabasi_albert_graph(len(robot_node_list), k)

        # Add edges between nodes in the graph.
        for edges in graph_regular.edges():
            self.mRobotList.add_edge(robot_node_list[edges[0]], robot_node_list[edges[1]])
            self.mRobotList.add_edge(robot_node_list[edges[1]], robot_node_list[edges[0]])

        # Save a copy of the edges for future reference.
        self.mEdgesCopy = self.getEdgesGroup()


    def setNaturalComNet(self):
        """
        This function sets the communication network of robots to a natural communication network.
        """        
        # Set the flag indicating that fixed communication network has not been set.
        self.isFixedComNet = False 

        # Clear any previously set edges.
        self.mRobotList.clear_edges()


    def getPositionPlotData(self,monitor_robot_id,predict_robot_id):
        """
        This function retrieves the data needed for plotting predicted location information.
        
        Args:
            monitor_robot_id (int): The robot ID that serves as the identifier for data recording.
            predict_robot_id (int): The robot ID to be predicted.

        Returns:
            xs, ys: This function returns two numpy arrays. 
                    First array contains x-coordinates,
                    And second array contains y-coordinates

        """        
        # Get robot instances with provided IDs.
        robot0 = self.getRobot(monitor_robot_id)
        robot1 = self.getRobot(predict_robot_id)

        # Prepare data for plotting.
        dict_list=[]
        for item in robot0.mProcessedInfoRecorder:
            dict_list.append(item['Pos'])

        # Calculate distance between predicted and actual position of robot1.
        if len(robot0.mProcessedInfoRecorder)>0:
            info_item = robot0.mProcessedInfoRecorder[-1]
            if 1 in info_item['Pos'].keys():
                dist_between_predict_real=distance(info_item['Pos'][predict_robot_id],robot1.pos)
                self.__ys.append(-dist_between_predict_real)

        # Generate x and y numpy arrays.
        xs=np.arange(len(self.__ys))
        ys_np=np.array(self.__ys)
        if isCupy:
            xs=xs.get()
            ys_np=ys_np.get()
        return xs,ys_np


    def getRobotGroup(self) -> list:
        """
        This function returns a list of all robots.

        Returns:
            list: This function returns a list of all robots.
        """
        return list(self.mRobotList.nodes)


    def getEdgesGroup(self) -> list:
        """
        This function returns a list of all edges between the robots.

        Returns:
            list: This function returns a list of all edges between the robots.
        """
        return list(self.mRobotList.edges)


    def getStuffGroup(self) -> list:
        """
        This function returns a list of all Stuff objects.

        Returns:
            list: This function returns a list of all Stuff objects.
        """
        return self.mStuffList


    def enableFigSave(self, dir):
        """
        This function enables figure saving and sets the directory where the figures will be saved.

        Args:
            dir (str): The directory where the figures will be saved.
        """
        self.isSaveFig = True
        self.mFigSaveDir = dir


    def enablePosSave(self, dir):
        """
        This function enables position data saving and sets the directory in which the position data will be saved.

        Args:
            dir (str): The directory in which the position data will be saved.
        """
        self.isSavePos2 = True
        self.mPosSaveDir = dir


    def enableInfoSave(self, dir):
        """
        This function enables info saving and sets the directory in which the info will be saved.

        Args:
            dir (str): The directory in which the info will be saved.
        """
        self.isSaveInfo = True 
        self.mSaveInfoDir = dir


    def saveFig(self, dir):
        """
        This function saves the plot figure at the specified directory.

        Args:
            dir (str): The directory where the figure will be saved.
        """
        if ComStage.mCount % settings.INTERVAL_SAVE == 0:
            try:
                plt.savefig(dir + '/image{0:0>5}.png'.format(ComStage.mCount))
            except:
                pass


    def savePos(self, dir, pos_list: list):
        """
        This function saves the position data to a file at the specified directory.

        Args:
            dir (str): The directory where the position data will be saved.
            pos_list (list): A list of position data.
        """
        with open(dir, 'a+') as file:
            file.write("{}".format(self.count))
            for pos in pos_list:
                file.write(', ')
                file.write(str(pos))
            file.write('\n')


    def saveInfo(self, dir, info:str):
        """
        This function saves the info data to a file at the specified directory.

        Args:
            dir (str): The directory where the info data will be saved.
            info (str): The info that needs to be saved.
        """
        with open(dir, 'a+') as file:
            file.write("{},".format(self.count))
            file.write(info)
            file.write('\n')



    def run(self):
        """
        This function runs the simulation continuously. It initializes the environment and updates it at regular intervals.
        """
        self.initEnv()
        while True:
            if self.count % self.mSavePosRound == 0:
                # Check if position data needs to be saved
                if self.isSavePos2:
                    self.isSavePos = True
            else:
                self.isSavePos = False
            if (self.count * settings.CS_INTERVAL) >= self.mRuningTime:
                # Check if the simulation has run for the specified time
                break
            self.count += 1
            ComObject.update_count = self.count
            if self.count % 1 == 0:
                print("Round: 【%d : %d】" % (self.count, self.mRuningTime/settings.CS_INTERVAL))
            self.update()
            plt.pause(settings.CS_INTERVAL)


    def run_once(self):
        """
        This function runs the simulation once. It initializes the environment and updates it once.
        """
        self.initEnv()
        self.update()
        plt.show()


if __name__ == "__main__":
    a = ComStage()
    a.run()
