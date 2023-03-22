# -*- coding: utf-8 -*-
"""
       .==.        .==.
      //`^\\      //^`\\
     // ^ ^\(\__/)/^ ^^\\PIXEL_SCALE
    //^ ^^ ^/+  0\ ^^ ^ \\
   //^ ^^ ^/( >< )\^ ^ ^ \\
  // ^^ ^/\| v''v |/\^ ^ ^\\
 // ^^/\/ /  `~~`  \ \/\^ ^\\
 ----------------------------
BE CAREFULL! THERE IS A DRAGON.

功能：settings
备注：

案例：

模块：
(c) 肖镇龙 2019
依赖：
"""
import sympy
usingCuda = False 
isTimeDisp = True
RUNING_TIME = 100    # 运行时间，秒
INTERVAL_SAVE = 1   # 每间隔多少张图片保存一张图片
SAVE_POS_ROUND = 1

RB_GAMMA_J = 100
RB_GAMMA_J2 = 1000
RB_TIME = 3000
RB_DIFFUSION_COEFFICIENT = 0.02
RB_GAMMA_J3 = 100
RB_TIME3 = 1000
RB_DIFFUSION_COEFFICIENT3 = 0.03
theta1 = 3
theta2 = 10
theta3 = 2.1
theta4 = 7
k = 2
RB_SPEED = 500.0

# RB_C = 0.1
# RB_B = 0.3
# RB_M = 0.03
# RB_A = 2
# RB_R = 0.1

RB_C = 0.1
RB_B = 0.3
RB_M = 0.03
RB_A = 2
RB_R = 5

# RB_C = 0.004
# RB_B = 0.04
# RB_M = 0.0012
# RB_A = 0.2
# RB_R = 0.4

TIME_INTERVAL = 0.05   # 两次迭代间隔的时间

## 人工鱼群
AF_SPEED = 1            # 每次移动距离
AF_MAXPREYNUM = 50     # 每次最大进行觅食尝试的次数
AF_POPULATIONNUM = 20   # 人工鱼数量
AF_FOODSIZE = 5         # 最大食物数量
AF_MAXITERNUM = 1000    # 最大迭代次数
AF_ENVSIZE = [50, 50]   # 环境大小, width, height
AF_INTERVAL = TIME_INTERVAL      # 两次迭代间隔的时间
AF_SENSEDIST = 50       # 感知距离
AF_MAXCROWDED = 1 / 10  # 拥挤度因子, 分母代表人工鱼的数目
AF_GETFOODDIST = 1      # 找到食物的最小距离
AF_TITLE = "Artificial Fish School Algorithm"
AF_REACHFOODSTOP = False    # 找到食物后是否停止

xn = sympy.Symbol('xn')
AF_LOG = 1.05
AF_DIVID = 1
AF_FLEETDIST = sympy.solve([sympy.log(xn, AF_LOG)-AF_DIVID/xn], xn)    # 列队时各鱼的距离


## 一致性
CS_ENVSIZE = [1000, 1000, 1000]   # 环境大小, x, y, z
CS_AGENT_ENVSIZ = [(-1000, 1000), (-1000, 1000), (-500, 1000)]    # 机器人的实际运动环境大小
CS_INTERVAL = TIME_INTERVAL      # 两次迭代间隔的时间
CS_TITLE = "Gene Regulatory Network"
CS_RUNING_TIME = 30
CS_CROWDEDRANGE = 100   # 拥挤度计算范围

AF_MUTATE_PARAM = 10.0

__param_path = 1
PATH_FOLLOWING_K_RHO = 9 * __param_path
PATH_FOLLOWING_K_ALPHA = 15 * __param_path
PATH_FOLLOWING_K_BETA = 3 * __param_path