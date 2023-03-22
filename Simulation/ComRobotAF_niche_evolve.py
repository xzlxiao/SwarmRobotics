"""进化版小生态人工鱼群算法
"""
isCupy = False
try:
    import cupy as np
    isCupy = True
except:
    import numpy as np
    isCupy = False
from ast import Num
import random
import copy
import math
from Common.utils import *
from Common import settings

from Simulation.ComObjectCollection import *
from Simulation.ComRobotAF_niche import *



class ComRobotAF_niche_evolve(ComRobotAF_niche):
    
    def __init__(self, pos):
        super().__init__(pos)
        self.isDistancedTravel = False # 子群突变会主群，进行远距离移动，该值为True，到达目的地后变为False
        self.mKeepSpeciesTime = 0.0    # 维持种群的时间
    def update(self):
        self.mKeepSpeciesTime += settings.CS_INTERVAL
        if self.mId == 0:
            main_pop = len(ComRobotAF_niche.PopulationGroup[-1])
            sub_pop0 = len(ComRobotAF_niche.PopulationGroup[0])
            sub_pop1 = len(ComRobotAF_niche.PopulationGroup[1])
            sub_pop2 = len(ComRobotAF_niche.PopulationGroup[2])
            sub_pop3 = len(ComRobotAF_niche.PopulationGroup[3])
            sub_pop4 = len(ComRobotAF_niche.PopulationGroup[4])
            sub_pop5 = len(ComRobotAF_niche.PopulationGroup[5])
            sub_pop6 = len(ComRobotAF_niche.PopulationGroup[6])
            sub_pop7 = len(ComRobotAF_niche.PopulationGroup[7])
            sub_pop8 = len(ComRobotAF_niche.PopulationGroup[8])
            sub_pop9 = len(ComRobotAF_niche.PopulationGroup[9])
            print("""
            main pop: {}
            sub0: {}
            sub1: {}
            sub2: {}
            sub3: {}
            sub4: {}
            sub5: {}
            sub6: {}
            sub7: {}
            sub8: {}
            sub9: {}
            ---------------------------------
            """.format(main_pop, sub_pop0, sub_pop1, sub_pop2, sub_pop3, sub_pop4, sub_pop5, sub_pop6, sub_pop7, sub_pop8, sub_pop9))

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
                    if not self.isDistancedTravel:
                        if self.distance(food.pos, self.pos) < R_I:
                            self.mPopulationAll[self.mId].setSpecies(food.mId)
                        else:
                            # 突变为新物种的概率
                            new_species_id = self.mutateToSubspecies()
                            if new_species_id != -1:
                                self.setSpecies(new_species_id)
        else:
            
            # if len(self.mFood) > 0:
            #     print(self.distance(self.mPos, self.mFood[0]))
            #     print(self.getPosFit(self.mPos))
                
            if self.isReturnToMainSwarm():
                print("return main swarm: {}.{} -> main".format(self._species, self.mId))
                self.randomDistancedMove()
                # with open("/media/storage1/Code/data/AFSA/20220602003/log1.txt", 'a') as file:
                #     file.write("return main swarm: {}.{} -> main\n".format(self._species, self.mId))
                self.setSpecies(-1)

        if self._species == -1:
            self.randomMove()
        else:
            self.AFfast()
        self.move()

    def setSpecies(self, species_id):
        super().setSpecies(species_id)
        self.mKeepSpeciesTime = 0.0

    def isReturnToMainSwarm(self):
        """
        是否突变为主群物种
        """
        if random.random() < self.getMainspeciesMutationChance():
            return True
        # aggregation_degree = self.nicheAggregationDegree()
        # if aggregation_degree > NICHE_aggregation_degree_threshold:
        #     # print(aggregation_degree)
        #     niche_size = len(self.mPopulation)
        #     current_time = self.getTime()
        #     rho_i = niche_size * AGG_lambda * (1.0/(1.0+math.e**(current_time*aggregation_degree/niche_size)) - 0.5)
            
        #     if rho_i > 1:
                
        #         dist = self.distance(self.mPos, self.mFood[0])
        #         for agent_pos in self.mPopulation.values():
        #             if self.distance(self.mFood[0], agent_pos) > dist:
        #                 return False 
        #         return True 
        return False

    def mutateToSubspecies(self)->int:
        for species_id, is_species_init in enumerate(ComRobotAF_niche.SpecialPool):
            if is_species_init:
                if random.random() < self.getSubspeciesMutationChance(ComRobotAF_niche.PopulationGroup[species_id]):
                    return species_id
        return -1

    def getSubspeciesMutationChance(self, population)->Num:
        '''
        获得突变为某个子群的概率，距离越近，应该突变概率越高
        '''
        return 1.0 * len(self.mPopulation)/len(self.mPopulationAll) * settings.CS_INTERVAL

    def getMainspeciesMutationChance(self)->Num:
        '''
        获得突变为主群的概率
        '''
        return 1.0 * len(self.mPopulation)/len(self.mPopulationAll) * settings.CS_INTERVAL

    def randomDistancedMove(self):
        '''
        随机移动，越远的越
        '''
        # self.chooseRandomTarget()
        self.chooseRandomDistancedTarget()
        self.isDistancedTravel = True

    

    def move(self):
        super().move()
        if self.isStopping():
            self.isDistancedTravel = False

    def getPosFit(self, position):
        """
        适应度计算
        :param position:
        :return: 适应度
        """
        fitness = 0.0
        
        if len(self.mFood) > 0:
            for food_pos in self.mFood:
                fitness_tmp = 1 - (np.linalg.norm(np.array(position, dtype=np.float32) - food_pos, ord=2) / self.mSenseDistance)
                # fitness_tmp = 1 / (np.linalg.norm(np.array(position, dtype=np.float32) - food_pos, ord=2) + 0.000000000000001)
                if fitness_tmp > fitness:
                    fitness = fitness_tmp
        # fitness = sigmoid(fitness, 0.5, 1)
        return fitness