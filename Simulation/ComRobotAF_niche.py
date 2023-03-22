isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
import random
import copy
import math
from Common.utils import *
from Simulation.ComObjectCollection import *
from Simulation.ComRobotAFfast import ComRobotAFfast

# special_colors = [
#     "red",
#     'red',
#     "red",
#     'red',
#     "red",
#     'red',
    
#     "red",
#     "red",
#     "red",
#     'red',
#     "white",
#     # 'antiquewhite',
#     # 'aqua',
    
#     # 'red',
#     # 'white',
    
# ]

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

R_I = 200   # 小生境范围
C_TH = 600  # 激活新种群的目标距离
NICHE_aggregation_degree_threshold = 8.0      # 子群聚集度阈值
AGG_beta = 1.0      # 聚集度参数
AGG_lambda = -0.4    # 聚集度参数 
SIG_STEEP = 0.002
SIG_MID = 700

class ComRobotAF_niche(ComRobotAFfast):
    SpecialPool = []
    PopulationGroup = []      # 子群集合

    def __init__(self, pos):
        super().__init__(pos)
        if len(ComRobotAF_niche.SpecialPool) == 0:
            ComRobotAF_niche.SpecialPool = [[] for _ in range(10000)]
        if len(ComRobotAF_niche.PopulationGroup) == 0:
            ComRobotAF_niche.PopulationGroup = [{} for _ in range(10000) ]
        self.setColor("blue")
        self.mFoodAll = {}              # 所有食物
        self.mPopulationAll = {}        # 所有个体
        
        self._species = -1
        self.setSpecies(-1)

    def getAllRobotsNum(self):
        '''
        获得所有机器人的数量
        '''
        return len(self.mPopulationAll)

    def getRobotsNumBySpecies(self, species_id):
        '''
        获得某个种群机器人的数量
        '''
        return len(ComRobotAF_niche.PopulationGroup[species_id])

    def getSpecies(self):
        return self._species
        
    def update(self):
        # return super().update()
        # if self.mId == 0:
        #     print(self._species)
        #     print(self.nicheAggregationDegree())
        #     with open("/media/storage1/Code/data/AFSA/20220602002/log.txt", 'a') as file:
        #         file.write("{} | {}\n".format(self._species, self.nicheAggregationDegree()))

        # if self.nicheAggregationDegree()> 5:
        #     print(self.nicheAggregationDegree())
        if self.mId == 0:
            main_pop = len(ComRobotAF_niche.PopulationGroup[-1])
            sub_pop0 = len(ComRobotAF_niche.PopulationGroup[0])
            sub_pop1 = len(ComRobotAF_niche.PopulationGroup[1])
            sub_pop2 = len(ComRobotAF_niche.PopulationGroup[2])
            sub_pop3 = len(ComRobotAF_niche.PopulationGroup[3])
            sub_pop4 = len(ComRobotAF_niche.PopulationGroup[4])
            print("""
            main pop: {}
            sub0: {}
            sub1: {}
            sub2: {}
            sub3: {}
            sub4: {}
            ---------------------------------
            """.format(main_pop, sub_pop0, sub_pop1, sub_pop2, sub_pop3, sub_pop4))
            # with open("/media/storage1/Code/data/AFSA/20220602003/log2.txt", 'a') as file:
            #     file.write("""
            #         main pop: {}
            #         sub0: {}
            #         sub1: {}
            #         sub2: {}
            #         sub3: {}
            #         sub4: {}
            #         ---------------------------------
            #         """.format(main_pop, sub_pop0, sub_pop1, sub_pop2, sub_pop3, sub_pop4))

        self.sense()
        self.processInfo()
        if self._species == -1:
            for food in self.mFoodAll.values():
                # print(food)
                if not ComRobotAF_niche.SpecialPool[food.mId]:
                    if self.distance(food.pos, self.pos) < C_TH:
                        print("set species {}".format(food.mId))
                        ComRobotAF_niche.SpecialPool[food.mId] = True
                        current_population = [(agent_id, agent_pos) for agent_id, agent_pos in ComRobotAF_niche.PopulationGroup[-1].items()]
                        for agent_id, agent_pos in current_population:
                            if self.distance(agent_pos, self.pos) < R_I:
                                self.mPopulationAll[agent_id].setSpecies(food.mId)
                        break
                else:
                    if self.distance(food.pos, self.pos) < R_I:
                        self.mPopulationAll[self.mId].setSpecies(food.mId)
        else:
            if self.isReturnToMainSwarm():
                print("return main swarm: {}.{} -> main".format(self._species, self.mId))
                # with open("/media/storage1/Code/data/AFSA/20220602003/log1.txt", 'a') as file:
                #     file.write("return main swarm: {}.{} -> main\n".format(self._species, self.mId))
                self.setSpecies(-1)
            # print(len(self.mPopulation))

        if self._species == -1:
            self.randomMove()
        else:
            self.AFfast()
        self.move()

    def randomMove(self):
        if self.isStopping():
            self.chooseRandomTarget()

    def sense(self):
        super().sense()

        # # 获取所有食物
        # foods = getObjectByType(self.mFoodName)
        
        # for food in foods:
            
        #     self.mFoodAll[food.mId] = food
        
        # # 根据种群类别获取食物,更新食物位置
        # self.mFood.clear()
        # if self._species >= 0:
        #     self.mFood.append(self.mFoodAll[self._species].pos)

        # # 获取所有机器人
        # neighors = getObjectByType(self.mObjectType)
        # for agent in neighors:
        #     self.mPopulationAll[agent.mId] = agent
        
        # # 更新子群中机器人的坐标
        # for subgroup_agent_id in self.mPopulation.keys():
        #     self.mPopulation[subgroup_agent_id] = self.mPopulationAll[subgroup_agent_id].pos

        # self.mFitness = self.getPosFit(self.pos)

    def setSpecies(self, species_id):
        self._species = species_id
        self.setColor(special_colors[species_id%len(special_colors)])
        if self.mId in self.mPopulation.keys():
            self.mPopulation.pop(self.mId)
        self.mPopulation = ComRobotAF_niche.PopulationGroup[self._species]
        self.mPopulation[self.mId] = self.pos
        
    def nicheAggregationDegree(self):
        """
        计算子群的聚集度
        """
        aggregation_degree = 0
        if len(self.mFood) > 0:
            for agent_pos in self.mPopulation.values():
                aggregation_degree += 1 - (self.distance(self.mFood[0], agent_pos) / R_I)**AGG_beta
        return aggregation_degree

    def isReturnToMainSwarm(self):
        """
        如果子群聚集度大于阈值，计算是否需要回到主群
        """
        
        aggregation_degree = self.nicheAggregationDegree()
        if aggregation_degree > NICHE_aggregation_degree_threshold:
            # print(aggregation_degree)
            niche_size = len(self.mPopulation)
            current_time = self.getTime()
            rho_i = niche_size * AGG_lambda * (1.0/(1.0+math.e**(current_time*aggregation_degree/niche_size)) - 0.5)
            
            if rho_i > 1:
                
                dist = self.distance(self.pos, self.mFood[0])
                for agent_pos in self.mPopulation.values():
                    if self.distance(self.mFood[0], agent_pos) > dist:
                        return False 
                return True 
        return False

    def chooseRandomDistancedTarget(self):
        """
        距离越远的概率越大
        """
        x = 0
        y = 0
        z = 0
        while True:
            range_x = (-100, 100)
            range_y = (-100, 100)
            range_z = (-100, 100)
            if self.mStage is not None:
                range_x = (-self.mStage.mEnvSize[0], self.mStage.mEnvSize[0])
                range_y = (-self.mStage.mEnvSize[1], self.mStage.mEnvSize[1])
                range_z = (-self.mStage.mEnvSize[2], self.mStage.mEnvSize[2])
            x = random.uniform(range_x[0], range_x[1])
            y = random.uniform(range_y[0], range_y[1])
            z = random.uniform(range_z[0], range_z[1])
            dist = self.distance(self.pos, (x, y, z))
            posibility = sigmoid(dist, SIG_MID, SIG_STEEP)
            if random.random() < posibility:
                break

        self.setTarget((x, y, z))