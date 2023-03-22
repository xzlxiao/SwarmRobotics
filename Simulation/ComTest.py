import sys
sys.path.append('./')
from Simulation.ComApi import *


def testGetAllKeysInDict():
    dict_list = []
    dict_list.append({'11': 3, '34': 4})
    dict_list.append({'15': 8, '123': 3})
    dict_list.append({'765': 8, '423': 3})
    dict_list.append({})
    ret = ComRobot.getAllKeysInDict(dict_list)
    print(ret)


if __name__ == '__main__':
    testGetAllKeysInDict()