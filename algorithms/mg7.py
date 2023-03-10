from utility.turn import getDistance
from utility.util import addNode, inBucket, sendRaw
import sys
import time
# import rospy
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")

"""
    MG5 can find a path between two nodes, with heuristic, with the magnetism enabled 
    Similar to mg5, however target best parent instead of a single parent by updating nodes!
"""

queue = ([], [])
visited = []
obstacles = {}
traces = ({}, {})
reserves = ([], [])

def inBucket (cords, visited):
    return cords in visited

def filterOptions(cords, visited, obstacles):
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

def extractPath (end, nodes, reverse=True):
    currentlyAt = end
    path = [end]
    while currentlyAt in nodes and nodes[currentlyAt] != None:
        path.append(nodes[currentlyAt])
        currentlyAt = nodes[currentlyAt]

    return path[::-1] if reverse else path

def runSingleIteration (currentInfo, queue, reserves, visitedTrace, visited, vistedNext, end, obstacles):
    currentInfo['value'] = queue.pop(0) 
    current = currentInfo['value']
    thoughOptions = [
      (current[0]-1, current[1]),  # top
      (current[0], current[1]+1),  # right
      (current[0]+1, current[1]),  # bottom
      (current[0], current[1]-1),  # left
    ]
    
    options = []
    for opt in thoughOptions:
        include = filterOptions(opt, visited, obstacles)
        if include:
            options.append(opt)

    if not len(options) and len(reserves):
        # this is when we know we have rerouted since we are no longer taking the best option
        (parent, nextRes) = reserves.pop(0)
        visited.append(nextRes)
        visitedTrace[nextRes] = parent
        if nextRes in vistedNext:
            return nextRes
        queue.append(nextRes)
        return 1
    elif not len(options):
        print('No path possible!')
        return -1   

    bestFitness = float('inf')
    bestCord = None

    for opt in options:
        fitness = abs(opt[0] - end[0]) + abs(opt[1] - end[1]) 
        # + min ( abs(opt[0] - end[0]), abs(opt[1] - end[1]) )
        if fitness < bestFitness:
            bestFitness = fitness
            bestCord = opt

    visited.append(bestCord)
    visitedTrace[bestCord] = current
    if bestCord in vistedNext:
        return bestCord
    queue.append(bestCord)

    for opt in options:
        if opt != bestCord:
            reserves.append((current, opt))    

    return True
def distanceApart (cordA, cordB):
    return abs(cordA[0]-cordB[0]) + abs(cordA[1]-cordB[1])

def heuristic(start, end, barriers): #function for something else
    global obstacles
    obstacles = barriers
    queue = ([start], [end])
    visited = ([start], [end])
    traces = ({start:None}, {end:None})
    reserves = ([], [])
    currents = [{'value':start}, {'value':end}]
    index = 0
    # add extra condition where visits overlay
    mergePoint = None
    while len(queue[0]) and len(queue[1]) and distanceApart(currents[0]['value'], currents[1]['value']) > 1:
        try:
            nextIndex = (index+1) % 2
            status = runSingleIteration (currents[index], queue[index], reserves[index], traces[index], visited[index], visited[nextIndex], currents[nextIndex]['value'], obstacles)
            if status == -1:
                return (-1, -1)
            if type(status) == tuple:
                mergePoint = status
                break
            # path = extractPath(currents[index]['value'], traces[index]) + extractPath(currents[nextIndex]['value'], traces[nextIndex], False)
            # stepDetails = { 'steps':path, 'visited':keysVersion(visited[0]+visited[1]), 'end':end }
            # sendRaw(stepDetails, (0,0,0,(25,25)), obstacles, True)
            # time.sleep(.01)
            index = nextIndex
        except:
            return -1

    if mergePoint != None:
        path = extractPath(mergePoint, traces[0]) + extractPath(mergePoint, traces[1], False)
        # stepDetails = { 'steps':path, 'visited':keysVersion(visited[0]+visited[1]), 'end':end }
        # sendRaw(stepDetails, (0,0,0,(25,25)), obstacles, True)        
        return (path, visited[0]+visited[1])

    # return (extractPath(end, visitedTrace), visited)
    return (extractPath(currents[0]['value'], traces[0]) + extractPath(currents[1]['value'], traces[1], False), visited[0]+visited[1])
    
