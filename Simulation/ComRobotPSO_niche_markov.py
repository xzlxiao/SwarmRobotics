from Simulation.ComRobot import ComRobot 
from Simulation.ComObject import ComObject 
from Simulation.ComRobotPSO import ComRobotPSO
from Simulation.ComRobotPSO_niche import *

from Common.utils import *
import copy
import math
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from Common import settings
from Common.DrKDtree import KDtree
import Common.settings as mySettings 
import random
from Common import utils
from Simulation.ComObjectCollection import *


special_colors = [
    "cyan",
    'royalblue',
    "lightsteelblue",
    'red',
    "purple",
    'blueviolet',
    
    "yellow",
    "lightgreen",
    "orange",
    'red',
    "white",
    # 'antiquewhite',
    # 'aqua',
    
    # 'red',
    # 'white',
    
]
# R_I = 500   # 小生境范围
# R_I = 400   # 小生境范围
# C_TH = 300  # 激活新种群的目标距离
# # NICHE_aggregation_degree_threshold = 8.0      # 子群聚集度阈值
# NICHE_aggregation_degree_threshold = 5.0      # 子群聚集度阈值
# # AGG_beta = 1.0      # 聚集度参数
# AGG_beta = 0.8      # 聚集度参数
# AGG_lambda = -0.4    # 聚集度参数 
# SIG_STEEP = 0.002
# SIG_MID = 700
# target_line_len = 300.0


step_num=10000
lr=0.01
k = 20 # 目标迭代时间
# mutate_param = 0.1 # 突变率调整参数

def getTransferMat(x):
    n = len(x)
    mat = np.zeros((n, n), dtype=float)
    for i in range(n):
        mat[i, 0] = x[i]
    for i in range(n-1):
        mat[0, i+1] = (1-x[0])/(n-1)
        mat[i+1, i+1] = 1-x[i+1]
    return mat

def loss(x, S_0, k, S_t):
    """
    x: 状态参数，用于构建概率的状态转移矩阵
    S_0: 初始状态
    k: 目标迭代时间
    S_t: 目标状态
    """
    P = getTransferMat(x)
    S_p = MarkovChain(S_0, P, k)
    return mean_squared_error(S_p, S_t)

def mean_squared_error(y, t):
    return 0.5 * np.sum((y-t)**2)

def numerical_gradient(x, S_0, k, S_t):
    """
    x: 状态参数，用于构建概率的状态转移矩阵
    S_0: 初始状态
    k: 目标迭代时间
    S_t: 目标状态
    """
    h = 1e-4
    grad = np.zeros_like(x)
    

    for idx in range(len(x)):
        tmp_val = x[idx]
        # f(x+h)计算
        x[idx] = tmp_val + h 
        fxh1 = loss(x, S_0, k, S_t)

        # f(x-h)计算
        x[idx] = tmp_val - h 
        fxh2 = loss(x, S_0, k, S_t)

        grad[idx] = (fxh1 - fxh2) / (2 * h)
        x[idx] = tmp_val # 还原值

    return grad

def gradient_descent(init_x, S_0, k, S_t, lr=lr, step_num=100):
    x = np.array(init_x).astype(float) 

    for _ in range(step_num):
        grad = numerical_gradient(list(x), S_0, k, S_t)
        grad = np.array(grad).astype(float)
        x -= lr * grad
        for i in range(x.size):
            if x[i] < 0 or x[i] >= 1:
                x[i] = random.random()
    
    return x 

def MarkovChain(S_0, P, k):
    """
    S_0: 初始状态
    P: 概率的状态转移矩阵
    k: 迭代次数
    return: S_k
    """
    S_k = copy.deepcopy(np.array(S_0).astype(float))
    for _ in range(k):
        S_k = np.matmul(S_k, P)

    return S_k

class ComRobotPSO_niche_markov(ComRobotPSO_niche):
    mutate_chance_list = {-1: 1.0}
    isSaveGroupNum = False 
    GroupNumSaveDir = ""
    isFixedMutate_chance = False
    def __init__(self, pos):
        super().__init__(pos)
        self.count = 0

    def getFoodNum(self):
        '''
        当前已感知到的目标数量
        '''
        return len(ComRobotPSO_niche_markov.mutate_chance_list)-1

    def saveGroupNum(self, dir, group_num_list: list):
        with open(dir, 'a+') as file:
            file.write("{}".format(self.count))
            for group_num in group_num_list:
                file.write(', ')
                file.write(str(group_num))
            file.write('\n')
    
    @staticmethod
    def enableSaveGroupNum(dir):
        ComRobotPSO_niche_markov.isSaveGroupNum = True
        ComRobotPSO_niche_markov.GroupNumSaveDir = dir 

    
        
    def update(self):
        self.sense()
        self.processInfo()
        # print(self.getSpecies(), self.mColor)
        if self.mId == 0:
            if ComRobotPSO_niche_markov.isSaveGroupNum:
                group_num_list = []
                for i in range(-1, 10):
                    group_num_list.append(len(ComRobotPSO_niche.PopulationGroup[i]))
                self.saveGroupNum(ComRobotPSO_niche_markov.GroupNumSaveDir, group_num_list)
            
            # print(ComRobotAF_niche_markov.mutate_chance_list)
            main_pop = len(ComRobotPSO_niche.PopulationGroup[-1])
            sub_pop0 = len(ComRobotPSO_niche.PopulationGroup[0])
            sub_pop1 = len(ComRobotPSO_niche.PopulationGroup[1])
            sub_pop2 = len(ComRobotPSO_niche.PopulationGroup[2])
            sub_pop3 = len(ComRobotPSO_niche.PopulationGroup[3])
            sub_pop4 = len(ComRobotPSO_niche.PopulationGroup[4])
            sub_pop5 = len(ComRobotPSO_niche.PopulationGroup[5])
            sub_pop6 = len(ComRobotPSO_niche.PopulationGroup[6])
            sub_pop7 = len(ComRobotPSO_niche.PopulationGroup[7])
            sub_pop8 = len(ComRobotPSO_niche.PopulationGroup[8])
            sub_pop9 = len(ComRobotPSO_niche.PopulationGroup[9])
            sub_pop10 = len(ComRobotPSO_niche.PopulationGroup[10])
            sub_pop11 = len(ComRobotPSO_niche.PopulationGroup[11])
            sub_pop12 = len(ComRobotPSO_niche.PopulationGroup[12])
            sub_pop13 = len(ComRobotPSO_niche.PopulationGroup[13])
            sub_pop14 = len(ComRobotPSO_niche.PopulationGroup[14])
            sub_pop15 = len(ComRobotPSO_niche.PopulationGroup[15])
            sub_pop16 = len(ComRobotPSO_niche.PopulationGroup[16])
            sub_pop17 = len(ComRobotPSO_niche.PopulationGroup[17])
            sub_pop18 = len(ComRobotPSO_niche.PopulationGroup[18])
            sub_pop19 = len(ComRobotPSO_niche.PopulationGroup[19])
            print("""
            main pop: {}
            sub0: {}  sub1: {}  sub2: {}  sub3: {}
            sub4: {}  sub5: {}  sub6: {}  sub7: {}
            sub8: {}  sub9: {}
            sub10: {}  sub11: {}  sub12: {}  sub13: {}
            sub14: {}  sub15: {}  sub16: {}  sub17: {}
            sub18: {}  sub19: {}
            ---------------------------------
            """.format(main_pop, 
            sub_pop0, 
            sub_pop1, 
            sub_pop2, 
            sub_pop3, 
            sub_pop4, 
            sub_pop5, 
            sub_pop6, 
            sub_pop7, 
            sub_pop8, 
            sub_pop9,
            sub_pop10, 
            sub_pop11, 
            sub_pop12, 
            sub_pop13, 
            sub_pop14, 
            sub_pop15, 
            sub_pop16, 
            sub_pop17, 
            sub_pop18, 
            sub_pop19))

        if self.getUpdateCount() > 5:
            for food in self.mFoodAll.values():
                if food.isVisible:
                    if not ComRobotPSO_niche.SpecialPool[food.mId]:
                        if self.distance(food.pos, self.pos) < C_TH:
                            print("set species {}".format(food.mId))
                            ComRobotPSO_niche.SpecialPool[food.mId] = True
                            if not ComRobotPSO_niche_markov.isFixedMutate_chance:
                                ComRobotPSO_niche_markov.mutate_chance_list[food.mId] = 0.0
                            main_population = [(agent_id, agent_pos) for agent_id, agent_pos in ComRobotPSO_niche.PopulationGroup[-1].items()]
                            for agent_id, agent_pos in main_population:
                                if self.distance(agent_pos, self.pos) < R_I:
                                    self.mPopulationAll[agent_id].setSpecies(food.mId)
                            self.updateNewMutationChance2()
                            break
        
        if self._species == -1 and self.getUpdateCount() > 5:
            for food in self.mFoodAll.values():
                if food.isVisible:
                    if ComRobotPSO_niche.SpecialPool[food.mId]:
                        # 突变为新物种的概率
                        new_species_id = self.mutateToSubspecies()
                        if new_species_id != -1:
                            self.setSpecies(new_species_id)
        elif self._species!=-1:
            
            # if len(self.mFood) > 0:
            #     print(self.distance(self.pos, self.mFood[0]))
            #     print(self.getPosFit(self.pos))
                
            if self.isReturnToMainSwarm():
                print("return main swarm: {}.{} -> main".format(self._species, self.mId))
                self.randomDistancedMove()
                # with open("/media/storage1/Code/data/AFSA/20220602003/log1.txt", 'a') as file:
                #     file.write("return main swarm: {}.{} -> main\n".format(self._species, self.mId))
                self.setSpecies(-1)

        if self._species == -1:
            self.randomMove()
        else:
            if self.getPopulationNum() < 5 and len(self.mFood)>0:
                # print(self.mFood)
                self.setTarget(self.mFood)
            else:
                self.pso()
        self.move()
        self.count += 1

    def getPopulationNum(self):
        return len(self.mPopulation)

    def isReturnToMainSwarm(self):
        """
        是否突变为主群物种
        """
        
        if random.random() < self.getMainspeciesMutationChance(self._species):
            return True
        # aggregation_degree = self.nicheAggregationDegree()
        # if aggregation_degree > NICHE_aggregation_degree_threshold:
        #     # print(aggregation_degree)
        #     niche_size = len(self.mPopulation)
        #     current_time = self.getTime()
        #     rho_i = niche_size * AGG_lambda * (1.0/(1.0+math.e**(current_time*aggregation_degree/niche_size)) - 0.5)
            
        #     if rho_i > 1:
                
        #         dist = self.distance(self.pos, self.mFood[0])
        #         for agent_pos in self.mPopulation.values():
        #             if self.distance(self.mFood[0], agent_pos) > dist:
        #                 return False 
        #         return True 
        return False

    def mutateToSubspecies(self)->int:
        rand_num = random.random()
        mutate_chance = self.getSubspeciesMutationChance()
        last_mutate_chance = 0
        for _key in ComRobotPSO_niche_markov.mutate_chance_list.keys():
            if _key != -1:
                if rand_num > last_mutate_chance and rand_num < last_mutate_chance + mutate_chance:
                    return _key
                else:
                    last_mutate_chance += mutate_chance
        return -1

    def getSubspeciesMutationChance(self):
        '''
        获得突变为某个子群的概率
        '''
        if self.getFoodNum() > 0:
            return settings.AF_MUTATE_PARAM * settings.CS_INTERVAL * (1-ComRobotPSO_niche_markov.mutate_chance_list[-1])/self.getFoodNum()
        else:
            return 0

    def getMainspeciesMutationChance(self, species_id):
        '''
        获得突变为主群的概率
        '''
        # print( ComRobotAF_niche_markov.mutate_chance_list)
        return settings.AF_MUTATE_PARAM * settings.CS_INTERVAL * ComRobotPSO_niche_markov.mutate_chance_list[species_id]

    def updateNewMutationChance(self):
        '''
        更新突变率
        '''
        if not ComRobotPSO_niche_markov.isFixedMutate_chance:
            food_num = self.getFoodNum()
            main_mu = sigmoid(-food_num, -2, 0.2)
            sub_mu = (1-main_mu)/food_num
            S_0 = self.getState0()
            S_t  = [main_mu]
            for i in range(food_num):
                S_t.append(sub_mu)
            x1 = np.random.random(food_num+1)
            x2 = gradient_descent(x1, S_0, k, S_t, lr=lr, step_num=step_num)
            for i, _key in enumerate(ComRobotPSO_niche_markov.mutate_chance_list.keys()):
                ComRobotPSO_niche_markov.mutate_chance_list[_key] = x2[i]
        
    def updateNewMutationChance2(self):
        '''
        按平均值更新突变
        '''
        if not ComRobotPSO_niche_markov.isFixedMutate_chance:
            food_num = self.getFoodNum()
            main_mu = 0.3
            sub_mu = (1-main_mu)/food_num
            S_0 = self.getState0()
            S_t  = [main_mu]
            for i in range(food_num):
                S_t.append(sub_mu)
            x1 = np.random.random(food_num+1)
            x2 = gradient_descent(x1, S_0, k, S_t, lr=lr, step_num=step_num)
            for i, _key in enumerate(ComRobotPSO_niche_markov.mutate_chance_list.keys()):
                ComRobotPSO_niche_markov.mutate_chance_list[_key] = x2[i]

    def getState0(self):
        '''
        获得当前状态（初始状态）
        '''
        ret = []
        robots_num = self.getAllRobotsNum()
        for species_id in ComRobotPSO_niche_markov.mutate_chance_list.keys():
            robots_num_by_species = self.getRobotsNumBySpecies(species_id)
            ret.append(robots_num_by_species / robots_num)
        return ret

    def randomDistancedMove(self):
        '''
        随机移动，越远的越
        '''
        # self.chooseRandomTarget()
        self.chooseRandomDistancedTarget()
        # self.isDistancedTravel = True