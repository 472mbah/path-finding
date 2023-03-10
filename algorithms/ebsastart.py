import math
from functools import reduce
import requests
from timeit import default_timer as timer


def pythagoras(start, end):
    return math.sqrt((end[0]-start[0])**2 + (end[1] - start[1])**2)

def build2DMap (size):
    nodes  = {}
    for i in range(size):
        for j in range(size):
            children = []
            # top
            if i - 1 >= 0:
                children.append(f'{i-1}:{j}')
            # bottom
            if i + 1 < size:
                children.append(f'{i+1}:{j}')  
            # left
            if j + 1 < size:
                children.append(f'{i}:{j+1}')                                
            # right
            if j - 1 >= 0:
                children.append(f'{i}:{j-1}')

            nodes[f'{i}:{j}'] = { 'cords':[i, j], 'children':children, 'parent':None, 'distance':None, 'direction':None }

    return nodes

def findNextBestNode (nodes, completed):
    bestFit = float('inf')
    nextNode = None
    for node in nodes:
        if node in completed:
            continue
        if nodes[node]['distance'] == None:
            continue
        if nodes[node]['distance'] < bestFit:
            nextNode = node
    return nextNode


def addExpansion (blockers):
    for id in blockers:
        cords = [ int(x.split(':')) for x in id ]
        options = [ 
            [ cords[0]-1, cords[1]-1 ],
            [ cords[0]-1, cords[1]+1 ], 
            [ cords[0]+1, cords[1]+1 ], 
            [ cords[0]+1, cords[1]-1 ], 
            [ cords[0]-1, cords[1] ],
            [ cords[0]+1, cords[1] ],
            [ cords[0], cords[1]-1 ], 
            [ cords[0], cords[1]+1 ] 
        ]

        for opt in options:
            id = f'{opt[0]}:{opt[1]}'
            if id not in blockers:
                blockers[id] = -1

def findLengthOfVector(vector):
    sqaures = [math.pow(x, 2) for x in vector]
    sums = reduce(lambda a, b: a + b, sqaures)
    return math.sqrt(sums)

def findDeltaAngle(currentVector, targetVector):
    dotProduct = (currentVector[0] * targetVector[0]) + (currentVector[1] * targetVector[1])
    p1 = findLengthOfVector(currentVector)
    p2 = findLengthOfVector(targetVector)
    if p1 == 0 and p2 == 0:
        return 0
    elif p1 == 0:
        return math.acos(targetVector[0] / targetVector[1])
    elif p2 == 0:
        return 0

    denomiator = p1 * p2

    division = dotProduct / denomiator

    if abs(division) > 1:
        if abs(division) != division:
            division = -1
        else:
            division = 1
    return math.acos(division)


def smooth (path, nodes):
    indexesToOmit = {}
    for x in range(2, len(path)):
        y = x - 2
        
        # changeInX = abs(nodes[path[x]]['cords'][0] - nodes[path[y]]['cords'][0])
        # changeInY = abs(nodes[path[x]]['cords'][1] - nodes[path[y]]['cords'][1])


        angle = findDeltaAngle(nodes[path[x]]['cords'], nodes[path[y]]['cords'])

        # print('angle between ', nodes[path[x]]['cords'], 'and', nodes[path[y]]['cords'], 'is', angle)
        if angle > 0:
            indexesToOmit[x-1] = None
    # print(indexesToOmit)
    newPaths = []
    for index in range(len(path)):
        if index not in indexesToOmit:
            newPaths.append(path[index])

    return newPaths

def checkOpposingNode (nodes, checking, compareNode):
    cords = nodes[checking]['cords']
    options = [ [cords[0]-1, cords[1]], [cords[0]+1, cords[1]], [cords[0], cords[1]-1], [cords[0], cords[1]+1] ]
    for opt in options:
        id = f'{opt[0]}:{opt[1]}'
        if id not in nodes:
            continue
        if nodes[id]['parent'] == None:
            continue
        if compareNode[id]['parent'] == None:
            continue
        return id
        
    return None

def runSingleAstar (currentlyAt, endSplit, nodes, completed, compareNode):
    currentSplit = [int(x) for x in currentlyAt.split(':')]
    newDistance = nodes[currentlyAt]['distance'] + 1 + pythagoras(endSplit, currentSplit)
    
    foundOpposing = None
    comparableChild = None

    for childName in nodes[currentlyAt]['children']:
        if nodes[childName]['distance'] == None or nodes[childName]['distance'] > newDistance:
            nodes[childName]['distance'] = newDistance
            nodes[childName]['parent'] = currentlyAt
            if foundOpposing==None:
                foundOpposing = checkOpposingNode (nodes, childName, compareNode)
                comparableChild = childName

    completed[currentlyAt] = None
    return (findNextBestNode(nodes, completed), foundOpposing, comparableChild)  # return string and array versions
                                                               # "5:5" -> [5, 5] to reduce computation 

def retrieveVisitors (path, nodesA):
    visited = {}
    for x in nodesA:
        if x not in path and nodesA[x]['parent']!=None:
            visited[x] = True
    return visited


def ebsastar (start, end, graphSize=10):
    currentlyAtForward = start
    currentlyAtBackward = end

    nodes=build2DMap(graphSize)
    nodesBackward=build2DMap(graphSize)
    start = timer()
    completedForward={}
    completedBackward={}

    nodesBackward[currentlyAtBackward]['distance'] = 0
    nodes[currentlyAtForward]['distance'] = 0

    endSplit = [int(x) for x in end.split(':')]
    startSplit = [int(x) for x in end.split(':')]
    
    intermediateForwardEnd = None
    intermediateBackwardEnd = None
    
    completePath = None

    while completePath==None and currentlyAtForward != end and currentlyAtForward != None and currentlyAtBackward != start and currentlyAtBackward != None and intermediateForwardEnd==None and intermediateBackwardEnd==None:
        (currentlyAtForward, forwardCords, comparable) = runSingleAstar (currentlyAtForward, endSplit, nodes, completedForward, nodesBackward)
        (currentlyAtBackward, backwardCords, backwardComparable) = runSingleAstar (currentlyAtBackward, startSplit, nodesBackward, completedBackward, nodes)
        if forwardCords!=None:
            completePath = extractPath(forwardCords, nodes) + extractPath(comparable, nodesBackward, False)
        if backwardCords!=None:
            completePath = extractPath(backwardCords, nodes) + extractPath(backwardComparable, nodesBackward , False)
    # smoothedOut = smooth(completePath, nodes)
    # print('len of original', len(completePath), 'but new has ', len(smoothedOut))
    duration = timer() - start
    return ([nodes[x]['cords'] for x in completePath], retrieveVisitors (completePath, nodes), duration)

def extractPath (end, nodes, reverse=True):
    currentlyAt = end
    path = [end]
    while currentlyAt in nodes and nodes[currentlyAt]['parent'] != None:
        path.append(nodes[currentlyAt]['parent'])
        currentlyAt = nodes[currentlyAt]['parent']

    return path[::-1] if reverse else path

# start = '0:0'
# end = '4:16'
# (path, visits, duration) = ebsastar(start, end, 20)
# print(path, duration)
# # path = extractPath('2:0', nodes)
# # print(path)
# LOCAL_URL="http://localhost:9000/"

# def send():
#     dataToSend = {
#         'blockers':{},
#         'visits':visits,
#         'path':path,
#         'facingAngle':0,
#         'targetAngle':0,
#         'robotPosition':[8, 4]
#     }
#     try:
#         requests.post(LOCAL_URL, json=dataToSend)
#     # rospy.loginfo(f"data sent! {VECTOR_POSITION}")
#     except:
#         # rospy.loginfo(f'{x}')
#         pass

# send()