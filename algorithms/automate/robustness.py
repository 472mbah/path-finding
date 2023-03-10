import math
import random
# random.seed(0.24975077935850032)
# random.seed(0.0329938859085428)
# print('Current seed - robustness', random.random())
"""
position 19,30 could be avoided with areaSize parameter and fitness function
Seed: 0.24975077935850032
This seed does a good job of exaggerating the difference between single iteration vs two iterations
"""

#last: 25
# 1, 6, 21 -> No path possible
# 2 -> visits overlay
# 4, 11, 22, 14 (worst i have seen) -> visits potentially overlay (when visited shared)
# 12 -> visits potentially overlay (when visited not shared)
# 9 is really good

def getBorder (d, existing={}, blockMode=True):
    # do top
    for i in range(d):
        existing[ f'-1:{i}' if not blockMode else (-1, i) ] = None

    for i in range(d):
        existing[ f'{d}:{i}' if not blockMode else (d, i) ] = None

    for i in range(-1,d+1):
        existing[ f'{i}:{-1}' if not blockMode else (i, -1) ] = None 
    for i in range(-1,d+1):
        existing[ f'{i}:{d}' if not blockMode else (i, d) ] = None                
    
    return existing

def produceRandomMaze (percentage, dimension):
    start = (0, 0)
    end = (dimension-1, dimension-1)
    obstacleFrequency = math.floor(math.pow(dimension, 2) * percentage/100)
    obstacles = {}
    while obstacleFrequency > 0:
        cords = (random.randint(0,dimension) ,random.randint(0,dimension))
        if cords == start or cords == end or cords in obstacles:
            continue
        obstacles[cords] = None
        obstacleFrequency-=1
    return (start, end, getBorder(dimension, obstacles))
