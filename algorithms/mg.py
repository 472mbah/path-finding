from functools import reduce
from utility.util import inBucket, addNode, fuseObjects, send, getParentCords
from utility.turn import getDistance
import math
import time
import sys
sys.path.append("/Users/momodoubah/research/catkin_ws/src/mitad/src")

def getOptions (current):
    return [
        (current[0]-1, current[1]-1),  # top
        (current[0]+1, current[1]+1),  # right
        (current[0]+1, current[1]-1),  # bottom
        (current[0]-1, current[1]+1),  # left

        (current[0]-1, current[1]),  # top
        (current[0], current[1]+1),  # right
        (current[0]+1, current[1]),  # bottom
        (current[0], current[1]-1),  # left    
    ] 

def addExpansionDistance (blockers, areaSize, blockSize):
    appendNodes = []
    for area in blockers:
        for block in blockers[area]:
            options = getOptions(block)
            for opt in options:
                if not inBucket(opt, blockers, areaSize, blockSize):
                    appendNodes.append(opt)
    [ addNode(opt, blockers, areaSize, blockSize) for opt in appendNodes ]
   

def othersInProximity (areaCords, blockers):
    options = [ 
        (areaCords[0]+1, areaCords[1]), 
        (areaCords[0]-1, areaCords[1]),   
        (areaCords[0], areaCords[1]+1), 
        (areaCords[0], areaCords[1]-1),   
        (areaCords[0]+1, areaCords[1]+1),   
        (areaCords[0]+1, areaCords[1]-1),   
        (areaCords[0]-1, areaCords[1]+1),   
        (areaCords[0]-1, areaCords[1]-1),   
    ]
    for option in options:
        if option in blockers:
            return False
    return True

def skipTraversal (current, end, areaSize, blockSize, path, blockers, visited):
    parentCords = ( math.floor(current[0]*blockSize / areaSize), math.floor(current[1]*blockSize / areaSize) )
    consecutive = othersInProximity(parentCords, blockers)
    if not consecutive:
        return ([], None, False)
    xAdd = -areaSize if parentCords[0] < 0 else areaSize
    yAdd = -areaSize if parentCords[1] < 0 else areaSize

    options = filterOptionsArray ([   
        (parentCords[0]*(areaSize), parentCords[1]*(areaSize)),
        (parentCords[0]*(areaSize) + xAdd, parentCords[1]*(areaSize)),
        (parentCords[0]*(areaSize), parentCords[1]*(areaSize) + yAdd),
        (parentCords[0]*(areaSize) + xAdd, parentCords[1]*(areaSize) + yAdd),
    ], visited, blockers, path, areaSize, blockSize)

    foundEnd = False

    next = getBestOption(options, end, blockers, blockSize, areaSize)

    xDistance = int(next[0] - current[0])
    yDistance = int(next[1] - current[1])

    xIt = 0
    if xDistance != 0:
        xIt = xDistance / abs(xDistance)

    yIt = 0
    if yDistance != 0:
        yIt = yDistance / abs(yDistance)    


    xIndex = current[0]
    yIndex = current[1]
    appendPath = []

    stop = False
    if xIt != 0:
        while not stop:
            appendPath.append(( xIndex, current[1] ))
            xIndex += xIt
            if xIndex == next[0]:
                stop = True

    stop = False

    if yIt != 0:
        while not stop:
            appendPath.append(( xIndex, yIndex ))
            yIndex += yIt
            if yIndex == next[1]:
                stop = True
    
    
    return (appendPath, (xIndex, yIndex), foundEnd )


    # for x in range(current[0], current[0]+xDistance, 1 if xDistance > 0 else -1):
    #     appendPath.append((current[0]+x, current[1]))

    # for y in range(current[1], current[1]+yDistance, 1 if yDistance > 0 else -1):
    #     appendPath.append((current[0], current[1]+y))        

    # print('skip path found path')
    # print(appendPath)

def filterOptions(cords, visited, blockers, path, areaSize, blockSize):
    if cords in path:
        return False
    if inBucket(cords, blockers, areaSize, blockSize):
        return False
    if inBucket(cords, visited, areaSize, blockSize):
        return False
    return True

def filterOptionsArray (options, visited, blockers, path, areaSize, blockSize):
    
    filteredOptions = []
    
    for cords in options:
        valid = filterOptions(cords, visited, blockers, path, areaSize, blockSize)  
        if valid:
            filteredOptions.append(cords) 
    
    return filteredOptions

def calculateFitness (cords, end, areaBlock=0):
    return getDistance(cords, end, False) + min( abs(cords[0]-end[0]), abs(cords[1]-end[1]) ) + areaBlock

def getBestOption (options, end, blockers={}, areaSize=2, blockSize=1):
    distances = {}
    """
        distance (number type): cordinates ([number, number] type)
    """
    for cords in options:
        # parents = getParentCords(cords, areaSize, blockSize)
        # numberOfBarriers = 0 if parents not in blockers else len(blockers[parents])
        distance = calculateFitness(cords, end, 0)
        distances[distance] = cords               
        
    lowest = min(distances)
    return distances[lowest]

def runSingleIteration (path, current, start, end, endTarget, areaSize, blockSize, visited, blockers, lastDistance):

    if current[0] == end[0] and current[1] == end[1]:
        return True

    suggestedPath = None
    suggestNewCurrent = None
    # lastDistance!=None and lastDistance > areaSize*2 and
    if not inBucket(current, blockers, areaSize, True):
        pass
        """
        (proposed, newCurrent, endFound) = skipTraversal (current, endTarget, areaSize, blockSize, path, blockers, visited)
        if len(proposed) > 1:
            path += proposed
            current = newCurrent
        if endFound:
            return 1
        # """

    options = filterOptionsArray ([   
        # (current[0]-1, current[1]-1),  # top
        # (current[0]+1, current[1]+1),  # right
        # (current[0]+1, current[1]-1),  # bottom
        # (current[0]-1, current[1]+1),  # left

        (current[0]-1, current[1]),  # top
        (current[0], current[1]+1),  # right
        (current[0]+1, current[1]),  # bottom
        (current[0], current[1]-1),  # left
    ], visited, blockers, path, areaSize, blockSize)

    optionsSize = len(options)

    if not optionsSize:
        if current[0] == start[0] and current[1] == start[1]:
            """
                Then we have no options and we are at the start node
                Therefore impossible to get to the target
            """
            return (False, None)
        else:
            """
                backtrack to the previous node in the path and see if 
                they have any relevant children
            """
            addNode(current, visited, areaSize, blockSize)
            parent = path.pop(len(path)-1)            
            return (True, parent)
    
    size = len(path)
    next = getBestOption(options, endTarget, blockers, areaSize, blockSize)

    if size==0 or (size > 0 and not (path[size-1][0] == current[0] and path[size-1][1] == current[1])):
        path.append(current)
        addNode(current, visited, areaSize, blockSize)

    return ( True, next )

def runMagnetic(start, end, blockers, areaSize=4, blockSize=1, sendImage=False):

    addExpansionDistance(blockers, areaSize, blockSize)

    if inBucket(start, blockers, areaSize, blockSize):
        print("This route not possible since start is in blockers")
        return -1
    if inBucket(end, blockers, areaSize, blockSize):
        print("This route not possible since end is in blockers")
        return -1

    currents = [(start[0], start[1]), (end[0], end[1])]
    paths = ([], [])
    visitedItems = ({}, {})
    impossible = False 
    index = 0 

    useSinglePathIndex = None
    distance = None

    issue = False

    while True:
        if not issue:
            nextIndex = (index+1)%2
        response = runSingleIteration(paths[index], currents[index], start if index==0 else end, end if index==0 else start, currents[nextIndex], areaSize, blockSize, visitedItems[index], blockers, distance)
        if response == True:
            break
        if response == 1:
            break
        (possible, next) = response
        if possible:
            currents[index] = next
            index = (index+1)%2
        else:
            impossible = True
            break
        distance = getDistance(currents[0], currents[1], False)
        if distance <= 1:
            paths[index].append(next)
            paths[nextIndex].append(currents[nextIndex])
            break
        if next == end:
            useSinglePathIndex = index
            break
        issue = next[0] == currents[nextIndex][0] or next[1] == currents[nextIndex][1]
        if sendImage:
            stepDetails = { 'steps':paths[0]+paths[1], 'visited':{}, 'end':end }
            send(stepDetails, (0,0,0,(0,0)), blockers)
            time.sleep(.1)
        
    path  = []
    if impossible:
        print("Path not possible")
        return -1
    elif useSinglePathIndex!=None:
        path = paths[useSinglePathIndex]
    else:
        path = paths[0] + paths[1][::-1]

    return (path, fuseObjects(visitedItems[0], visitedItems[1]))