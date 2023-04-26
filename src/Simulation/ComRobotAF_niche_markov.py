"""自监督学习马尔可夫链小生态人工鱼群算法
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

from Common.utils import sigmoid

step_num=10000
lr=0.01
k = 20 # 目标迭代时间
# mutate_param = 0.1 # 突变率调整参数

def getTransferMat(x):
    """
    This function takes in a list x and returns a transfer matrix.

    Args:
        x (list): A list of floats.

    Returns:
        numpy.ndarray: A 2D numpy array of shape (n, n) where n is the length of x.
    """
    # Get length of input list
    n = len(x)

    # Create an empty n x n matrix with float data type
    mat = np.zeros((n, n), dtype=float)

    # Fill first column with values from input list
    for i in range(n):
        mat[i, 0] = x[i]

    # Fill remaining columns using formulae based on input list
    for i in range(n-1):
        mat[0, i+1] = (1-x[0])/(n-1)
        mat[i+1, i+1] = 1-x[i+1]

    # Return the final transfer matrix
    return mat


def loss(x, S_0, k, S_t):
    """
    This function takes in state parameters x, starting state S_0, target iteration number k and target state S_t,
    and returns the mean squared error between the actual and target states after Markov Chain iteration of k.

    Args:
        x (list): A list of floats representing state parameters used to build a probability transition matrix.
        S_0 (list/numpy.ndarray): A list or array of floats representing the starting state.
        k (int): An integer representing the target iteration number for updating the state according to the
                  transition matrix.
        S_t (list/numpy.ndarray): A list or array of floats representing the target state.

    Returns:
        float: The mean squared error between the resulting and target states after Markov Chain iteration of k.
          'getTransferMat' and 'MarkovChain' functions should be imported before calling this function.
    """    

    P = getTransferMat(x) # Get probability transition matrix
    S_p = MarkovChain(S_0, P, k) # Get final state after iteration
    return mean_squared_error(S_p, S_t) # Return mean squared error between target and resulting states

def mean_squared_error(y, t):
    """
    This function takes two arrays y and t and computes the mean squared error between them.

    Args:
        y (list/numpy.ndarray): A list or array of floats.
        t (list/numpy.ndarray): A list or array of floats.

    Returns:
        float: The mean squared error between y and t.
    """ 
    return 0.5 * np.sum((y-t)**2) # Compute the mean squared error between y and t

def numerical_gradient(x, S_0, k, S_t):
    """
    This function takes in state parameters x, starting state S_0, target iteration number k and target state S_t,
    and returns its numerical gradient.

    Args:
        x (list): A list of floats representing state parameters used to build a probability transition matrix.
        S_0 (list/numpy.ndarray): A list or array of floats representing the starting state.
        k (int): An integer representing the target iteration number for updating the state according to the
                  transition matrix.
        S_t (list/numpy.ndarray): A list or array of floats representing the target state.

    Returns:
        numpy.ndarray: The numerical gradient of state parameters x.
    """
    h = 1e-4
    grad = np.zeros_like(x)

    for idx in range(len(x)):
        tmp_val = x[idx]

        # f(x+h)计算 compute f(x+h)
        x[idx] = tmp_val + h 
        fxh1 = loss(x, S_0, k, S_t) 

        # f(x-h)计算 compute f(x-h)
        x[idx] = tmp_val - h 
        fxh2 = loss(x, S_0, k, S_t)

        grad[idx] = (fxh1 - fxh2) / (2 * h) # calculate the slope
        x[idx] = tmp_val # restore value

    return grad

def gradient_descent(init_x, S_0, k, S_t, lr=0.01, step_num=100):
    """
    This function performs gradient descent optimization to find the optimal state parameters x.

    Args:
        init_x (list/numpy.ndarray): Initial values of the state parameters.
        S_0 (list/numpy.ndarray): A list or array of floats representing the starting state.
        k (int): An integer representing the target iteration number for updating the state according to the
                  transition matrix.
        S_t (list/numpy.ndarray): A list or array of floats representing the target state.
        lr (float): Learning rate of gradient descent. Default value is set to 0.01.
        step_num (int): Number of iterations. Default value is set to 100.

    Returns:
        numpy.ndarray: The optimized state parameters x.
    """
    x = np.array(init_x).astype(float) # convert initial parameters to float type

    for _ in range(step_num):
        grad = numerical_gradient(list(x), S_0, k, S_t) # calculate gradient
        grad = np.array(grad).astype(float)
        x -= lr * grad # update parameters using gradient descent
        for i in range(x.size):
            if x[i] < 0 or x[i] >= 1:
                x[i] = random.random() # randomize parameters that are out of bounds
    
    return x 

def MarkovChain(S_0, P, k):
    """
    This function takes in a starting state S_0, a probability transition matrix P and a target iteration number k,
    and returns the resulting state after Markov Chain iteration of k.

    Args:
        S_0 (list/numpy.ndarray): A list or array of floats representing the starting state.
        P (numpy.ndarray): A 2D numpy array representing the probability transition matrix.
        k (int): An integer representing the target iteration number for updating the state according to the
                  transition matrix.

    Returns:
        numpy.ndarray: The resulting state after Markov Chain iteration of k.
    """
    S_k = copy.deepcopy(np.array(S_0).astype(float)) # convert starting state to float type
    for _ in range(k):
        S_k = np.matmul(S_k, P) # perform Markov Chain iteration

    return S_k 


class ComRobotAF_niche_markov(ComRobotAF_niche):
    mutate_chance_list = {-1: 1.0}
    isSaveGroupNum = False 
    GroupNumSaveDir = ""
    isFixedMutate_chance = False
    def __init__(self, pos):
        super().__init__(pos)
        self.count = 0

    def getFoodNum(self):
        """Returns the current number of perceived targets.
        
        This method calculates the current number of perceived targets
        by subtracting 1 from the length of the `mutate_chance_list` attribute
        in the `ComRobotAF_niche_markov` object.

        Returns:
            int: The current number of perceived targets.
        """        

        return len(ComRobotAF_niche_markov.mutate_chance_list)-1


    def saveGroupNum(self, dir, group_num_list: list):
        """Saves the current count and a list of group numbers to a file.
        
        This method takes in a directory path and a list of group numbers
        and appends them to a file. The first item written is the self.count
        attribute followed by each item in the group_num_list separated by a comma.

        Args:
            dir (str): The directory path of the file to write to.
            group_num_list (list): A list of integers representing group numbers.
        """
        # Open the file in append mode
        with open(dir, 'a+') as file:
            # Write the current count to the file
            file.write("{}".format(self.count))
            
            # Loop through the list of group numbers and write them to the file separated by commas
            for group_num in group_num_list:
                file.write(', ')
                file.write(str(group_num))
            
            # Write a newline character to the file to separate entries
            file.write('\n')

    
    @staticmethod
    def enableSaveGroupNum(dir):
        """Enables saving of group numbers.

        This function sets the isSaveGroupNum attribute to True and sets
        the GroupNumSaveDir attribute to the provided directory path.

        Args:
            dir (str): The directory path where the file will be saved.
        """
        # Set the isSaveGroupNum attribute to True, indicating that group numbers should be saved
        ComRobotAF_niche_markov.isSaveGroupNum = True

        # Set the GroupNumSaveDir attribute to the provided directory path, indicating where the file should be saved
        ComRobotAF_niche_markov.GroupNumSaveDir = dir 


    def update(self):
        """Updates the robot's behavior and state.

        This function is called every iteration of the simulation to 
        update the robot's behavior and state based on its surroundings.
        """
        self.sense()
        self.processInfo()

        # If this robot has ID 0, check if group numbers should be saved
        if self.mId == 0:
            if ComRobotAF_niche_markov.isSaveGroupNum:
                # Get group numbers for all population groups and save to file
                group_num_list = []
                for i in range(-1, 10):
                    group_num_list.append(len(ComRobotAF_niche.PopulationGroup[i]))
                self.saveGroupNum(ComRobotAF_niche_markov.GroupNumSaveDir, group_num_list)

            # Print population numbers for each subspecies
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

        # Check if update count is greater than 5
        if self.getUpdateCount() > 5:
            for food in self.mFoodAll.values():
                if food.isVisible:
                    if not ComRobotAF_niche.SpecialPool[food.mId]:
                        # If there is nearby food that has not been claimed by a subspecies robot yet, claim it
                        if self.distance(food.pos, self.pos) < C_TH:
                            print("set species {}".format(food.mId))
                            ComRobotAF_niche.SpecialPool[food.mId] = True
                            if not ComRobotAF_niche_markov.isFixedMutate_chance:
                                ComRobotAF_niche_markov.mutate_chance_list[food.mId] = 0.0
                            main_population = [(agent_id, agent_pos) for agent_id, agent_pos in ComRobotAF_niche.PopulationGroup[-1].items()]
                            for agent_id, agent_pos in main_population:
                                if self.distance(agent_pos, self.pos) < R_I:
                                    self.mPopulationAll[agent_id].setSpecies(food.mId)
                            self.updateNewMutationChance()
                            break

        # Check if robot is currently unassigned to a subspecies
        if self._species == -1 and self.getUpdateCount() > 5:
            for food in self.mFoodAll.values():
                if food.isVisible:
                    if ComRobotAF_niche.SpecialPool[food.mId]:
                        # If there is nearby food claimed by a subspecies robot, mutate to that subspecies
                        new_species_id = self.mutateToSubspecies()
                        if new_species_id != -1:
                            self.setSpecies(new_species_id)
        elif self._species!=-1:

            # Check if robot should return to the main swarm
            if self.isReturnToMainSwarm():
                print("return main swarm: {}.{} -> main".format(self._species, self.mId))
                self.randomDistancedMove()
                self.setSpecies(-1)

        # If the robot has no subspecies assignment, move randomly
        if self._species == -1:
            self.randomMove()
        else:
            self.AFfast()
        self.move()
        self.count += 1


    def isReturnToMainSwarm(self):
        """
        Checks whether the agent will mutate into the main species based on a certain probability.
        If true, returns True, otherwise False.

        Returns:
            bool: True if the agent should mutate into the main swarm, False otherwise
        """        
        
        if random.random() < self.getMainspeciesMutationChance(self._species):
            return True
        return False


    def mutateToSubspecies(self) -> int:
        """
        Returns a random subspecies ID number based on set probabilities.
        If the random number does not fall within any probability range,
        returns -1 by default.

        Returns:
            int: _description_
        """        
        rand_num = random.random()
        mutate_chance = self.getSubspeciesMutationChance()
        last_mutate_chance = 0
        for _key in ComRobotAF_niche_markov.mutate_chance_list.keys():
            if _key != -1:
                if rand_num > last_mutate_chance and rand_num < last_mutate_chance + mutate_chance:
                    return _key
                else:
                    last_mutate_chance += mutate_chance
        return -1


    def getSubspeciesMutationChance(self) -> Num:
        """
        Calculates the mutation chance for a given subspecies based on the number of food sources available.
        If there are no available food sources, returns 0 by default.

        Returns:
            Num: Returns a float value representing the mutation chance for a given subspecies.
            If no food is available, returns 0. 
        """        

        if self.getFoodNum() > 0:
            return settings.AF_MUTATE_PARAM * settings.CS_INTERVAL * (1-ComRobotAF_niche_markov.mutate_chance_list[-1])/self.getFoodNum()
        else:
            return 0


    def getMainspeciesMutationChance(self, species_id)->Num:
        """
        Calculates the mutation chance for a given species to become the main species. 
        The mutation chance is calculated according to this formula:
        Args:
            species_id (_type_): An integer representing the ID of the species.

        Returns:
            Num: Returns a float value representing the mutation chance for the given species to become the main species.
        """
      
        # print( ComRobotAF_niche_markov.mutate_chance_list)
        return settings.AF_MUTATE_PARAM * settings.CS_INTERVAL * ComRobotAF_niche_markov.mutate_chance_list[species_id]

    def updateNewMutationChance(self):
        """
        Updates the mutation chance for each subspecies based on the current state and food availability.
        
        """        

        # Check if mutation chance is fixed before updating.
        if not ComRobotAF_niche_markov.isFixedMutate_chance:
            
            # Calculate main_mu using the sigmoid function and the number of available foods.
            food_num = self.getFoodNum()
            main_mu = sigmoid(-food_num, -2, 0.2)
            
            # Calculate sub_mu for each subspecies.
            sub_mu = (1-main_mu) / food_num
            
            # Get the current state S_0 using getState0() and generate S_t.
            S_0 = self.getState0()
            S_t  = [main_mu]
            for i in range(food_num):
                S_t.append(sub_mu)
            
            # Use gradient descent to calculate the optimal x2 values that minimize the difference between S_t and S_0.
            x1 = np.random.random(food_num+1)
            x2 = gradient_descent(x1, S_0, k, S_t, lr=lr, step_num=step_num)
            
            # Update the mutate_chance_list dictionary with the new mutation probabilities.
            for i, _key in enumerate(ComRobotAF_niche_markov.mutate_chance_list.keys()):
                ComRobotAF_niche_markov.mutate_chance_list[_key] = x2[i]

    def getState0(self):
        """
        Calculates the current state (initial state) based on the number of robots for each subspecies.

        Returns:
            A list of floats representing the current state for each subspecies.
        """        

        # Initialize empty list to store results.
        ret = []

        # Get the total number of robots across all subspecies.
        robots_num = self.getAllRobotsNum()

        # Calculate the ratio of robots for each subspecies to the total number of robots.
        for species_id in ComRobotAF_niche_markov.mutate_chance_list.keys():
            robots_num_by_species = self.getRobotsNumBySpecies(species_id)
            ret.append(robots_num_by_species / robots_num)

        # Return the list of ratios as the current state.
        return ret

    def randomDistancedMove(self):
        """ 
        Randomly selects a new target for the robot to move towards, with a bias towards farther targets.

        """        
        # self.chooseRandomTarget()
        self.chooseRandomDistancedTarget()
        # self.isDistancedTravel = True