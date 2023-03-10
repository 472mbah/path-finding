from utility.turn import getDistance
from utility.util import addNode, inBucket as inBucketArea, sendRaw
import sys
import math
import time
# import rospy
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")

"""
    MG5 can find a path between two nodes, with heuristic, with the magnetism enabled 
    This should enable area skipping too with 
"""

queue = ([], [])
visited = []
obstacles = {}
traces = ({}, {})
reserves = ([], [])

def skipTraversal (current, end, areaSize, blockSize, obstacles, visited, queueGiven, visitedTraceGiven, vistedNextGiven, iterations):
    parentCords = ( math.floor(current[0]*blockSize / areaSize), math.floor(current[1]*blockSize / areaSize) )

    queue = queueGiven.copy(), 
    visitedTrace = visitedTraceGiven.copy(), 
    vistedNext = vistedNextGiven.copy(), 

    xAdd = -areaSize if parentCords[0] < 0 else areaSize
    yAdd = -areaSize if parentCords[1] < 0 else areaSize

    thoughOptions = [   
        (parentCords[0]*(areaSize), parentCords[1]*(areaSize)),
        (parentCords[0]*(areaSize) + xAdd, parentCords[1]*(areaSize)),
        (parentCords[0]*(areaSize), parentCords[1]*(areaSize) + yAdd),
        (parentCords[0]*(areaSize) + xAdd, parentCords[1]*(areaSize) + yAdd),
    ]

    options = []
    for opt in thoughOptions:
        include = filterOptions(opt, visited, obstacles, areaSize, blockSize)
        if include:
            options.append(opt)

    bestFitness = float('inf')
    bestCord = None

    for opt in options:
        fitness = abs(opt[0] - end[0]) + abs(opt[1] - end[1]) 
        if fitness < bestFitness:
            bestFitness = fitness
            bestCord = opt
    
    if bestCord == None:
        return (False, None)

    xDistance = int(bestCord[0] - current[0])
    yDistance = int(bestCord[1] - current[1])

    xIt = 0
    if xDistance != 0:
        xIt = xDistance / abs(xDistance)

    yIt = 0
    if yDistance != 0:
        yIt = yDistance / abs(yDistance)    

    xIndex = current[0]
    yIndex = current[1]
    prev = None if iterations == 0 else current
    stop = False

    tempQueue = []

    seenFirst = False

    if xIt != 0:
        while not stop:
            if seenFirst:
                target = ( xIndex, current[1] )
                visited.append(target)
                visitedTrace[target] = prev
                prev = target
                if target in vistedNext:
                    return (True, target)
                tempQueue.append(target)            
            else:
                seenFirst = True
            xIndex += xIt
            if xIndex == bestCord[0]:
                stop = True

    stop = False

    if yIt != 0:
        while not stop:
            if seenFirst:
                target = ( xIndex, yIndex )
                visited.append(target)
                visitedTrace[target] = prev
                prev = target
                if target in vistedNext:
                    return (True, target)
                tempQueue.append(target)
            else:
                seenFirst = True
            yIndex += yIt
            if yIndex == bestCord[1]:
                stop = True

    for k in range(len(tempQueue)):
        queue.insert(0, tempQueue[k])

    """
        return format: Successfully found path, cordinates for linking or None
    """
    return (True, None)

def inBucket (cords, visited):
    return cords in visited

def filterOptions(cords, visited, obstacles, areaSize, blockSize):
    if inBucket(cords, visited):
        return False
    if inBucketArea(cords, obstacles, areaSize, blockSize):
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

def extractPathShorter (end, nodes, reverse=True):
    currentlyAt = end
    path = [end]
    index = 0
    while currentlyAt in nodes and nodes[currentlyAt] != None:
        size = len(path)
        if size > 2:
            curr = nodes[currentlyAt]
            rightAngle = path[size - 1]
            hasBlockedNeighbour = False
            thoughOptions = [
                (rightAngle[0]-1, rightAngle[1]-1),  
                (rightAngle[0]-1, rightAngle[1]+1),  
                (rightAngle[0]+1, rightAngle[1]+1),  
                (rightAngle[0]+1, rightAngle[1]-1),  
            ]

            for opt in thoughOptions:
                if opt in obstacles:
                    hasBlockedNeighbour = True
                    break
            if not hasBlockedNeighbour:
                path[size - 1] = curr
            else:
                path.append(curr)
            
            currentlyAt = nodes[currentlyAt]
            continue

        path.append(nodes[currentlyAt])
        # if 

        currentlyAt = nodes[currentlyAt]
        index+=1
    return path[::-1] if reverse else path    

def runSingleIteration (currentInfo, queue, reserves, visitedTrace, visited, vistedNext, end, obstacles, areaSize, blockSize, destArea, iterations):
    currentInfo['value'] = queue.pop(0) 
    current = currentInfo['value']
    
    # emptyArea = inBucketArea(current, obstacles, areaSize, blockSize, True, True)
    # if type(emptyArea) == tuple:
    #     """
    #         Only continue if empty space is not in the same space as the destination
    #     """
    #     if destArea != emptyArea[1]:
    #         (successful, linkingNode) =  skipTraversal (current, end, areaSize, blockSize, obstacles, visited, queue, visitedTrace, vistedNext, iterations)
            
    #         if linkingNode!=None:
    #             return linkingNode
    #         if successful:
    #             return
    
    thoughOptions = [
      (current[0]-1, current[1]),  # top
      (current[0], current[1]+1),  # right
      (current[0]+1, current[1]),  # bottom
      (current[0], current[1]-1),  # left
    ]
    
    options = []
    for opt in thoughOptions:
        include = filterOptions(opt, visited, obstacles, areaSize, blockSize)
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

def heuristic(start, end, barriers, areaSize, blockSize): #function for BFS
    global obstacles
    obstacles = barriers.copy()
    destAreas = (inBucketArea(end, barriers, areaSize, blockSize, True, True), inBucketArea(start, barriers, areaSize, blockSize, True, True))
    queue = ([start], [end])
    visited = ([start], [end])
    traces = ({start:None}, {end:None})
    reserves = ([], [])
    currents = [{'value':start}, {'value':end}]
    index = 0
    iterations = [0, 0]
    # add extra condition where visits overlay
    mergePoint = None
    while len(queue[0]) and len(queue[1]) and distanceApart(currents[0]['value'], currents[1]['value']) > 1:
        try:
            nextIndex = (index+1) % 2
            status = runSingleIteration (currents[index], queue[index], reserves[index], traces[index], visited[index], visited[nextIndex], currents[nextIndex]['value'], obstacles, areaSize, blockSize, destAreas[index], iterations[index])
            if status == -1:
                return (-1, -1)
            if type(status) == tuple:
                mergePoint = status
                break
            path = extractPath(currents[index]['value'], traces[index]) + extractPath(currents[nextIndex]['value'], traces[nextIndex], False)
            stepDetails = { 'steps':path, 'visited':keysVersion(visited[0]+visited[1]), 'end':end }
            sendRaw(stepDetails, (0,0,0,(25,25)), obstacles, True)
            time.sleep(.1)
            iterations[index] += 1
            index = nextIndex
        except:
            return (-1, -1)

    if mergePoint != None:
        path = extractPath(mergePoint, traces[0]) + extractPath(mergePoint, traces[1], False)
        stepDetails = { 'steps':path, 'visited':keysVersion(visited[0]+visited[1]), 'end':end }
        sendRaw(stepDetails, (0,0,0,(25,25)), obstacles, True)        
        return (path, visited[0]+visited[1])

    # return (extractPath(end, visitedTrace), visited)
    return (extractPath(currents[0]['value'], traces[0]) + extractPath(currents[1]['value'], traces[1], False), visited[0]+visited[1])
    
