"""标准版小生态人工鱼群算法
"""
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random
import copy

from Simulation.ComObjectCollection import *
from Simulation.ComRobotAF_niche_markov import ComRobotAF_niche_markov

special_colors = [
    "orange",
    "yellow",
    "green",
    "cyan",
    "blue",
    "purple",
    "red",
]

R_I = 200   # 小生境范围
C_TH = 300  # 激活新种群的目标距离

class ComRobotAF_niche_markov_Global(ComRobotAF_niche_markov):
    SpecialPool = [
        False,
        False,
        False,
        False,
        False,
    ]
    def __init__(self, pos):
        super().__init__(pos)


    # def update(self):
    #     # return super().update()
    #     self.sense()
    #     self.processInfo()
    #     if self._species == -1:
    #         for food in self.mFoodAll.values():
    #             # print(food)
    #             if self.distance(food.mPos, self.mPos) < C_TH and not ComRobotAF_niche_Global.SpecialPool[food.mId]:
    #                 print("set species {}".format(food.mId))
    #                 ComRobotAF_niche_Global.SpecialPool[food.mId] = True
    #                 current_population = [(agent_id, agent_pos) for agent_id, agent_pos in self.mPopulationGroup[-1].items()]
    #                 for agent_id, agent_pos in current_population:
    #                     if self.distance(agent_pos, self.mPos) < R_I:
    #                         self.mPopulationAll[agent_id].setSpecies(food.mId)
    #                 break

    #     if self._species == -1:
    #         self.randomMove()
    #     else:
    #         self.AFfast()
    #     self.move()

    def sense(self):
        super().sense()
        # 添加更新自身fitness的算法
        
        # self.mFood = getPosByType(self.mFoodName)
        # 获取所有食物
        foods = getObjectByType(self.mFoodName)
        
        for food in foods:
            
            self.mFoodAll[food.mId] = food
        
        # 根据种群类别获取食物,更新食物位置
        self.mFood.clear()
        if self._species >= 0:
            self.mFood.append(self.mFoodAll[self._species].pos)

        # 获取所有机器人
        neighors = getObjectByType(self.mObjectType)
        for agent in neighors:
            self.mPopulationAll[agent.mId] = agent
        
        # 更新子群中机器人的坐标
        for subgroup_agent_id in self.mPopulation.keys():
            self.mPopulation[subgroup_agent_id] = self.mPopulationAll[subgroup_agent_id].pos

        self.mFitness = self.getPosFit(self.pos)


        
