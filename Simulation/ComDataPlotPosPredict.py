try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False

from Simulation.ComDataPlotBase import ComDataPlotBase
from Common.utils import distance

class ComDataPlotPosPredict(ComDataPlotBase):
    def __init__(self) -> None:
        super().__init__()
        self.__ys = []
        self.mMonitorRobot = None 
        self.mPredictRobot = None 

    def setRobots(self, monitor_robot, predict_robot):
        self.mMonitorRobot = monitor_robot
        self.mPredictRobot = predict_robot

    def update(self):
        super().update()
        self.getPositionPlotData()

    def draw(self, ax):
        super().draw(ax)
        ax.plot(self.mDataX, self.mDataY)

    def getPositionPlotData(self):
        """
        获得定位信息预测绘图的数据
        @return:
        """
        if self.mMonitorRobot is not None and self.mPredictRobot is not None:
            # 绘制数据图
            dict_list = []
            predict_robot_id = self.mPredictRobot.mId

            for item in self.mMonitorRobot.mProcessedInfoRecorder:
                dict_list.append(item['Pos'])
            if len(self.mMonitorRobot.mProcessedInfoRecorder) > 0:
                info_item = self.mMonitorRobot.mProcessedInfoRecorder[-1]
                if 1 in info_item['Pos'].keys():
                    dist_between_predict_real = distance(info_item['Pos'][predict_robot_id], self.mPredictRobot.mPos)
                    self.__ys.append(-dist_between_predict_real)
                else:
                    self.__ys.append(-1000)
            self.mDataX = np.arange(len(self.__ys))
            self.mDataY = np.array(self.__ys)
            if isCupy:
                self.mDataX = self.mDataX.get()
                self.mDataY = self.mDataY.get()