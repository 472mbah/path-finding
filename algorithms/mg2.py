from functools import reduce
from utility.util import inBucket, addNode, fuseObjects, send, sendRaw, getParentCords
from utility.turn import getDistance, convertVectorToAngle, findVectorDifference, findAngle
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

def skipTraversal (current, end, areaSize, blockSize, path, blockers, visited, retra):
    parentCords = ( math.floor(current[0]*blockSize / areaSize), math.floor(current[1]*blockSize / areaSize) )
    consecutive = othersInProximity(parentCords, blockers)
    if not consecutive:
        return ([], None, False)
    xAdd = -areaSize if parentCords[0] < 0 else areaSize
    yAdd = -areaSize if parentCords[1] < 0 else areaSize

    (options, retraceOptions) = filterOptionsArray ([   
        (parentCords[0]*(areaSize), parentCords[1]*(areaSize)),
        (parentCords[0]*(areaSize) + xAdd, parentCords[1]*(areaSize)),
        (parentCords[0]*(areaSize), parentCords[1]*(areaSize) + yAdd),
        (parentCords[0]*(areaSize) + xAdd, parentCords[1]*(areaSize) + yAdd),
    ], visited, blockers, path, areaSize, blockSize, current, retra)

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
        return 2
    if inBucket(cords, blockers, areaSize, blockSize):
        return False
    if inBucket(cords, visited, areaSize, blockSize):
        return False
    return True

def filterOptionsArray (options, visited, blockers, path, areaSize, blockSize, current, retracedPathsGlobal=[]):
    
    filteredOptions = []
    inPathSuggested = []

    for cords in options:
        valid = filterOptions(cords, visited, blockers, path, areaSize, blockSize)  
        if valid == 2 and cords != current and cords not in retracedPathsGlobal:
            inPathSuggested.append(cords)
        if valid and cords not in retracedPathsGlobal:
            filteredOptions.append(cords) 

    return (filteredOptions, inPathSuggested)

def calculateFitness (cords, end, areaBlock=0):
    return getDistance(cords, end, False) + min( abs(cords[0]-end[0]), abs(cords[1]-end[1]) ) + areaBlock

def getBestOption (options, end, current, blockers={}, areaSize=2, blockSize=1):
    # add a penalty if we have already traversed this node
    distances = {}
    """
        distance (number type): cordinates ([number, number] type)
    """

    # targetVector = findVectorDifference(end, current)

    for cords in options:
        # directionVector = findVectorDifference(cords, current)
        # angleDistance = abs(findAngle(convertVectorToAngle(directionVector), targetVector))
        # parents = getParentCords(cords, areaSize, blockSize)
        # numberOfBarriers = 0 if parents not in blockers else len(blockers[parents])
        distance = calculateFitness(cords, end, 0)
        distances[distance] = cords               
    
    # if not len(list(distances)):
    #     print(True)
    #     stepDetails = { 'steps':[], 'visited':{}, 'end':end }
    #     send(stepDetails, (0,0,0,(25,25)), blockers)


    lowest = min(distances)
    return distances[lowest]

def runSingleIteration (path, current, start, end, endTarget, areaSize, blockSize, visited, blockers, retracedPathsGlobal):

    if current[0] == end[0] and current[1] == end[1]:
        return True

    suggestedPath = None
    suggestNewCurrent = None
    # lastDistance!=None and lastDistance > areaSize*2 and
    if not inBucket(current, blockers, areaSize, True):
        # """
        (proposed, newCurrent, endFound) = skipTraversal (current, endTarget, areaSize, blockSize, path, blockers, visited, retracedPathsGlobal)
        if len(proposed) > 1:
            path += proposed
            current = newCurrent
        if endFound:
            return 1
        # """

    (options, retraceOptions) = filterOptionsArray ([   
        # (current[0]-1, current[1]-1),  # top
        # (current[0]+1, current[1]+1),  # right
        # (current[0]+1, current[1]-1),  # bottom
        # (current[0]-1, current[1]+1),  # left

        (current[0]-1, current[1]),  # top
        (current[0], current[1]+1),  # right
        (current[0]+1, current[1]),  # bottom
        (current[0], current[1]-1),  # left
    ], visited, blockers, path, areaSize, blockSize, current, retracedPathsGlobal)

    optionsSize = len(options)

    if not optionsSize:
        if current[0] == start[0] and current[1] == start[1]:
            """
                Then we have no options and we are at the start node
                Therefore impossible to get to the target
            """
            return (False, None, None, False)
        else:
            """
                backtrack to the previous node in the path and see if 
                they have any relevant children
            """
            addNode(current, visited, areaSize, blockSize)
            parent = path.pop(len(path)-1)            
            return (True, parent, None, False)
    
    size = len(path)
    next1 = getBestOption(options, end, current, blockers, areaSize, blockSize)
    next2 = getBestOption(options, endTarget, current, blockers, areaSize, blockSize)
    next = getBestOption([next1, next2], endTarget, current, blockers, areaSize, blockSize)

    if next in retraceOptions and next not in retracedPathsGlobal:
        optionIndex = path.index(next)
        origial = path[optionIndex]
        retracedPathsGlobal.append(origial)
        path = path[:optionIndex]
        if len(path):
            retracedPathsGlobal.append(path[len(path)-1])
        return ( True, origial, path, True )
        # print('better option found with index of path', optionIndex, path, origial)
    

    if size==0 or (size > 0 and not (path[size-1][0] == current[0] and path[size-1][1] == current[1])):
        path.append(current)
        # addNode(current, visited, areaSize, blockSize)

    return ( True, next, None, False )

def runMagnetic(start, end, blockers, areaSize=4, blockSize=1, sendImage=False):

    # addExpansionDistance(blockers, areaSize, blockSize)

    if inBucket(start, blockers, areaSize, blockSize):
        print("This route not possible since start is in blockers", start)
        return ([], {})
    if inBucket(end, blockers, areaSize, blockSize):
        print("This route not possible since end is in blockers")
        return ([], {})

    currents = [(start[0], start[1]), (end[0], end[1])]
    paths = [[], []]
    visitedItems = ({}, {})
    impossible = False 
    index = 0 

    useSinglePathIndex = None
    distance = None

    issue = False
    retracedPaths = []

    maxIterations = calculateFitness(start, end, False) 

    backtracks = [0, 0]
    stopCheckingBackracks = False

    switchFrequence = 5
    # toggleOn = False

    while True:
        nextIndex = (index+1)%2
        response = runSingleIteration(paths[index], currents[index], start if index==0 else end, end if index==0 else start, currents[nextIndex], areaSize, blockSize, visitedItems[index], blockers, retracedPaths)
        if response == True:
            break
        if response == 1:
            break
        (possible, next, fixedPath, didBackTrack ) = response
        
        if didBackTrack and not stopCheckingBackracks:
            backtracks[index] += 1
            stopCheckingBackracks = True

        if possible:
            if fixedPath!=None:
                paths[index] = fixedPath
            currents[index] = next

            # if maxIterations > 0:
            # index = (index+1)%2
            # else:
            #     if backtracks[0] < backtracks[1]:
            #         index = 0
            #     else:
            #         index = 1

        else:
            impossible = True
            break
        distance = getDistance(currents[0], currents[1], False)
        if distance <= 1 or next in paths[nextIndex]:
            paths[index].append(next)
            paths[nextIndex].append(currents[nextIndex])
            break
        if next == end:
            useSinglePathIndex = index
            break
        # issue = 
        maxIterations -= 1
        maxIterations += 1
        # if sendImage:
        #     stepDetails = { 'steps':paths[0]+paths[1], 'visited':{}, 'end':end }
        #     send(stepDetails, (0,0,0,(20,20)), blockers)
        #     time.sleep(.1)
        
    path  = []
    if impossible:
        print("Path not possible")
        return -1
    elif useSinglePathIndex!=None:
        path = paths[useSinglePathIndex]
    else:
        path = paths[0] + paths[1][::-1]

    return (path, fuseObjects(visitedItems[0], visitedItems[1]))