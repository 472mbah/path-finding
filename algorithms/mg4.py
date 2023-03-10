from utility.turn import getDistance
from utility.util import addNode, inBucket, sendRaw
import sys
import time
# import rospy
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")

"""
    MG4 can find a path between two nodes, with heuristic
"""

queue = []
visited = []
obstacles = {}

def inBucket (cords, visited):
    return cords in visited

def filterOptions(cords):
    if inBucket(cords, visited):
        return False
    if inBucket(cords, obstacles):
        return False
    return True

def keysVersion (arr, appen=False):
    obj = {} if not appen else []
    for x in arr:
        if appen:
            obj.append(f'{x[0]}:{x[1]}')
        else:
            obj[f'{x[0]}:{x[1]}']=None
    return obj

def extractPath (end, nodes):
    currentlyAt = end
    path = [end]
    while currentlyAt in nodes and nodes[currentlyAt] != None:
        path.append(nodes[currentlyAt])
        currentlyAt = nodes[currentlyAt]

    return path[::-1]

def heuristic(start, end, barriers, maxIterations = 1000): #function for something else
    global obstacles
    global queue
    global visited
    global queue
    
    queue = []
    visited = []
    visitedTrace = {}
    reserves = []

    obstacles = barriers
    visited.append(start)
    queue.append(start)
    current = start
    visitedTrace = {start:None}
    reserves = []
    
    while queue and current != end and maxIterations >= 0:
        current = queue.pop(len(queue)-1) 
        options = [
          (current[0]-1, current[1]),  # top
          (current[0], current[1]+1),  # right
          (current[0]+1, current[1]),  # bottom
          (current[0], current[1]-1),  # left
        ]

        options = list(filter(filterOptions, options))

        if not len(options) and len(reserves):
            # trying to find a new route
            (parent, nextRes) = reserves.pop(0)
            visited.append(nextRes)
            visitedTrace[nextRes] = parent
            queue.append(nextRes) 
            continue
        elif not len(options):
            # print('No path possible!')
            return ([], {})         

        bestFitness = float('inf')
        bestCord = None

        for opt in options:
            fitness = abs(opt[0] - end[0]) + abs(opt[1] - end[1])
            if fitness < bestFitness:
                bestFitness = fitness
                bestCord = opt
        
        visited.append(bestCord)
        visitedTrace[bestCord] = current
        queue.append(bestCord)

        for opt in options:
            if opt != bestCord:
                reserves.append((current, opt))
        maxIterations -= 1
        # path = extractPath(current, visitedTrace)
        # stepDetails = { 'steps':path, 'visited':keysVersion(visited), 'end':end }
        # sendRaw(stepDetails, (0,0,0,(25,25)), obstacles, True)
        # time.sleep(.01)

    if maxIterations <= 0:
        return ([], [])

    return (extractPath(end, visitedTrace), visited)

    # for option in options:
    #   if option not in visited:
    
